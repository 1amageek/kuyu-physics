import Foundation
import KuyuCore

public struct SwappableSensorField<Base: SensorField>: SensorField {
    public var base: Base
    public let swapEvents: [SwapEvent]
    public let hfEvents: [HFStressEvent]
    public let baseNoise: IMU6NoiseConfig
    public let seed: UInt64
    public let stateChannelStore: ReferenceQuadrotorWorldStore?

    private var delayBuffer: SampleDelayBuffer
    private var currentDelay: UInt64
    private var noiseModels: [GaussianNoise]
    /// Per-channel first-order low-pass state for bandwidth modeling. `nil` until a
    /// channel produces its first post-modifier sample.
    private var filterState: [Double?]

    public init(
        base: Base,
        swapEvents: [SwapEvent],
        hfEvents: [HFStressEvent],
        baseNoise: IMU6NoiseConfig,
        seed: UInt64,
        stateChannelStore: ReferenceQuadrotorWorldStore? = nil
    ) {
        self.base = base
        self.swapEvents = swapEvents
        self.hfEvents = hfEvents
        self.baseNoise = baseNoise
        self.seed = seed
        self.stateChannelStore = stateChannelStore
        self.currentDelay = 0
        self.delayBuffer = SampleDelayBuffer(delaySteps: 0)
        let channelCount = stateChannelStore == nil ? 6 : 8
        self.noiseModels = (0..<channelCount).map { GaussianNoise(stdDev: 1.0, seed: seed &+ UInt64($0 + 1)) }
        self.filterState = Array(repeating: nil, count: channelCount)
    }

    public mutating func sample(time: WorldTime) throws -> [ChannelSample] {
        let baseSamples = try base.sample(time: time)
        if baseSamples.isEmpty {
            return []
        }

        var rawSamples = baseSamples
        rawSamples.reserveCapacity(baseSamples.count + (stateChannelStore == nil ? 0 : 2))
        if let stateChannelStore {
            rawSamples.append(try ChannelSample(
                channelIndex: 6,
                value: stateChannelStore.state.position.z,
                timestamp: time.time
            ))
            rawSamples.append(try ChannelSample(
                channelIndex: 7,
                value: stateChannelStore.state.velocity.z,
                timestamp: time.time
            ))
        }

        let channelCount = rawSamples.reduce(0) { max($0, Int($1.channelIndex) + 1) }
        let modifiers = modifiersForTime(time, channelCount: channelCount)
        if modifiers.delaySteps != currentDelay {
            currentDelay = modifiers.delaySteps
            delayBuffer = SampleDelayBuffer(delaySteps: currentDelay)
        }

        var updated: [ChannelSample] = []
        updated.reserveCapacity(rawSamples.count)

        for sample in rawSamples {
            let idx = Int(sample.channelIndex)
            guard idx >= 0, idx < modifiers.channel.count else { continue }

            let channelMods = modifiers.channel[idx]
            if channelMods.dropoutProbability > 0 {
                let rand = deterministicUniform(time: time.stepIndex, channelIndex: sample.channelIndex)
                if rand < channelMods.dropoutProbability {
                    continue
                }
            }

            let extraStd = extraNoiseStdDev(
                channelIndex: idx,
                noiseScale: channelMods.noiseScale
            )
            var value = sample.value
            value = value * channelMods.gainScale + channelMods.biasShift
            if extraStd > 0 {
                if idx >= noiseModels.count {
                    noiseModels.append(contentsOf: (noiseModels.count...idx).map {
                        GaussianNoise(stdDev: 1.0, seed: seed &+ UInt64($0 + 1))
                    })
                }
                noiseModels[idx].stdDev = extraStd
                value += noiseModels[idx].sample()
            }
            value = applyBandwidth(channelIndex: idx, value: value, bandwidthScale: channelMods.bandwidthScale)

            updated.append(try ChannelSample(channelIndex: sample.channelIndex, value: value, timestamp: sample.timestamp))
        }

        return delayBuffer.push(updated)
    }

    /// First-order low-pass: `y[n] = y[n-1] + alpha * (x[n] - y[n-1])` with `alpha = bandwidthScale`.
    /// `bandwidthScale == 1.0` passes the signal through unchanged.
    private mutating func applyBandwidth(channelIndex idx: Int, value: Double, bandwidthScale: Double) -> Double {
        if idx >= filterState.count {
            filterState.append(contentsOf: Array(repeating: Double?.none, count: idx - filterState.count + 1))
        }
        if bandwidthScale >= 1.0 {
            filterState[idx] = value
            return value
        }
        let previous = filterState[idx] ?? value
        let filtered = previous + bandwidthScale * (value - previous)
        filterState[idx] = filtered
        return filtered
    }

    private func extraNoiseStdDev(channelIndex: Int, noiseScale: Double) -> Double {
        let scale = max(0.0, noiseScale - 1.0)
        let baseStd: Double
        if channelIndex < 3 {
            baseStd = baseNoise.gyroNoiseStdDev
        } else {
            baseStd = baseNoise.accelNoiseStdDev
        }
        return baseStd * scale
    }

    private func deterministicUniform(time: UInt64, channelIndex: UInt32) -> Double {
        var rng = SplitMix64(seed: seed &+ time &+ UInt64(channelIndex))
        return rng.nextDouble()
    }

    private func modifiersForTime(_ time: WorldTime, channelCount: Int) -> Modifiers {
        var modifiers = Modifiers(channelCount: channelCount)
        let now = time.time

        for event in swapEvents {
            guard case .sensor(let sensor) = event else { continue }
            guard now >= sensor.startTime && now <= sensor.startTime + sensor.duration else { continue }
            for channel in sensor.targetChannels {
                let idx = Int(channel)
                guard idx >= 0, idx < modifiers.channel.count else { continue }
                modifiers.channel[idx].gainScale *= sensor.gainScale
                modifiers.channel[idx].biasShift += sensor.biasShift
                modifiers.channel[idx].noiseScale *= sensor.noiseScale
                modifiers.channel[idx].bandwidthScale *= sensor.bandwidthScale
                modifiers.channel[idx].dropoutProbability = combineDropout(
                    modifiers.channel[idx].dropoutProbability,
                    sensor.dropoutProbability
                )
            }
            modifiers.delaySteps = addDelay(modifiers.delaySteps, sensor.delayShiftSteps)
        }

        for event in hfEvents {
            guard now >= event.startTime && now <= event.startTime + event.duration else { continue }
            switch event.kind {
            case .sensorGlitch:
                for idx in 0..<modifiers.channel.count {
                    let sign = deterministicUniform(time: time.stepIndex &+ UInt64(idx), channelIndex: UInt32(idx)) < 0.5 ? -1.0 : 1.0
                    modifiers.channel[idx].biasShift += sign * event.magnitude
                }
            case .latencySpike:
                let extra = max(1, Int(event.magnitude))
                modifiers.delaySteps = addDelay(modifiers.delaySteps, extra)
            default:
                break
            }
        }

        return modifiers
    }

    private func combineDropout(_ a: Double, _ b: Double) -> Double {
        let clampedA = min(max(a, 0.0), 1.0)
        let clampedB = min(max(b, 0.0), 1.0)
        return 1.0 - ((1.0 - clampedA) * (1.0 - clampedB))
    }

    private func addDelay(_ base: UInt64, _ shift: Int) -> UInt64 {
        if shift == 0 { return base }
        let next = Int(base) + shift
        return UInt64(max(0, next))
    }

    private struct ChannelModifiers {
        var gainScale: Double = 1.0
        var biasShift: Double = 0.0
        var noiseScale: Double = 1.0
        var dropoutProbability: Double = 0.0
        var bandwidthScale: Double = 1.0
    }

    private struct Modifiers {
        var channel: [ChannelModifiers]
        var delaySteps: UInt64 = 0

        init(channelCount: Int) {
            self.channel = Array(repeating: ChannelModifiers(), count: max(0, channelCount))
        }
    }
}
