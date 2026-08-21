# kuyu-physics

Physics engines and analytical models for the Kuyu simulation environment.

## Overview

kuyu-physics provides concrete physics implementations that conform to kuyu-core protocols. It contains ODE-based rigid body dynamics, sensor models, actuator models, and the quadrotor reference plant.

## Reliability Contract

Package-local reliability milestones are defined in `RELIABILITY_MILESTONES.md`,
with verification evidence recorded in `RELIABILITY_EVIDENCE.md`. The root
individual reliability contract in `../INDIVIDUAL_RELIABILITY_MILESTONES.md`
requires this package to remain independently verifiable through
`TEST_TIMEOUT_SECONDS=180 ../scripts/test.sh kuyu-physics` before downstream
scenario, training, MLX, or app work treats physics readiness as stable.

### Physics Engines

- **`ReferenceQuadrotorPlantEngine`** — 6-DOF rigid body dynamics evaluated from the validated canonical operation program with declared RK4 integration, motor mixing, aerodynamics, drag, gravity, and gyroscopic effects.
- **`SinglePropPlantEngine`** — Vertical constrained fidelity over the same canonical quadrotor program.
- **`ArticulatedRigidBodySimulator`** — Descriptor-driven articulated rigid body simulation for manipulator-style bodies.

## Canonical Dynamics Program

The reference quadrotor's force terms, state derivative, IMU observables,
fidelity partitions, constraint projections, and RK4 stage contract are one
closed SSA program. Program construction validates operand order, shapes,
physical dimensions, differentiability propagation, output layouts, fidelity
partitions, and explicit integration support before computing a canonical
SHA-256 digest.

```text
CanonicalDynamicsProgram
  -> ReferenceQuadrotorScalarDynamicsExecutor (Swift Float64 reference)
      -> ReferenceQuadrotorPhysicsModel
          -> ReferenceQuadrotorPlantEngine / SinglePropPlantEngine
          -> IMU6SensorField
```

The current reference digest is
`6c6773c5a824508fd683390aa7a4acdc1636e8c8483f6ac9ee9667bf62d54310`.
Closure-backed force terms and duplicated sensor equations are not fallback
paths. Future Mojo CPU, Metal, and CUDA executors must consume this program and
qualify against the same digest and golden traces.

## Articulated Simulator Contracts

The articulated simulator is treated as a deterministic virtualization substrate. It fails closed instead of silently approximating unsupported physical modes.

| Contract | Enforced invariant |
|---|---|
| Time discretization | `duration / dt` must be integral; no hidden truncation is allowed. |
| World time | `request.timeStep.delta` must match `world.time.fixedStepSeconds` within `1e-12`. |
| Numerical stability | `dt / substeps <= actuator.timeConstantSeconds` for every bound actuator. Declared time constants are used directly; no hidden minimum time-constant approximation is applied. |
| Signal ordering | The authoritative joint order is actuator signal index order, not body file order. |
| Actuator mapping | `ActuatorAttachment` transmission ratio, command direction, actuator zero, and joint zero map actuator position/velocity before joint dynamics. Actuator torque telemetry maps joint torque through `mechanicalReductionRatio * efficiency`. |
| Effort limits | A declared joint effort limit of `0` remains zero; the simulator must not lift it to an epsilon effort. |
| Prismatic inertia | Prismatic effective inertia is descendant mass plus reflected actuator inertia, not rotational inertia. |
| Joint range | Provider context receives mapped `actuatorLimits ∩ jointLimits`; invalid transformed envelopes fail readiness. |
| Contact world | Disabled contact uses `disabledContact`; penalty/constraint contact requires `deterministicConstraint`. |
| Contact support | Static world box surfaces in the `world` frame are supported against link sphere, box, and cylinder collision geometry. Surface pose, geometry pose, and geometry scale are composed for unrotated boxes. Mesh contact and rotated world surfaces fail fast. |
| Contact material readiness | `contactTraining` requires every collidable link material to define static friction, dynamic friction, and restitution. Missing coefficients fail readiness and runtime contact setup. |
| Contact logging | Contact runs emit `contact.active.count`, `contact.penetration.max`, `contact.normalImpulse.max`, `contact.normalForce.max`, and `contact.solver.iterations` in `PlantStateSnapshot.scalars`. |
| Constraint residual | Deterministic contact constraint projection is inverse-inertia weighted and must finish with residual penetration `<= world.solver.tolerance` or fail with `unresolvedContact`. |
| Descriptor corpus acceptance | `DescriptorCorpusAcceptanceService` accepts typed robot descriptor bundles, validates readiness, performs deterministic replay for articulated, contact-training, and rigid actuator descriptors, records sorted-JSON byte stability, stores contact replay evidence for contact-training descriptors, and reports hardware-parity readiness gaps as typed evidence instead of hiding them in logs. Hardware-parity summaries persist calibration report ID, source, coverage counts, and canonical report hash. `DescriptorCorpusAcceptanceArtifactStore` persists and reloads accepted summaries through the same validation boundary, and `kuyu-training` project evidence packs can reference those summaries only when the saved physics artifact reloads to the same evidence. |
| Unsupported physics | RK4 articulated worlds and unsupported contact frames/geometries fail fast. |
| Determinism | Equal request, seed, and drive provider must produce equal `SimulationLog` values and sorted JSON bytes. |

The current verification suite includes 12-axis and 48-axis articulated chains with deliberately reversed body joint order. The 48-axis fixture mixes revolute and prismatic joints and starts from non-zero home positions. Snapshot conformance covers unordered revolute, prismatic, and fixed chains plus a branched unordered graph with independent sibling links. Contact verification covers descriptor-driven vertical prismatic bodies for penalty force and constraint projection, inverse-inertia weighted projection on a two-axis contact chain, 100 byte-stable contact replays, and a 48-axis contact constraint budget. Descriptor-to-dynamics verification includes non-identity actuator attachment mapping, mechanical-reduction torque telemetry, required contact material coefficients, readiness rejection for transformed actuator limits, and high-dimensional readiness rejection for missing actuator dynamics. The suite checks signal contracts, finite state, deterministic replay, contact residuals, contact force emission, and axis-step throughput for higher-dimensional use.

| Verification | Current automated budget |
|---|---:|
| 2-axis articulated replay | 2,000 steps, at least 2,000 steps/s under `xcodebuild test` |
| 12-axis articulated replay | 12,000 axis-steps, at least 10,000 axis-steps/s under `xcodebuild test` |
| 48-axis mixed articulated replay | 9,600 axis-steps, at least 20,000 axis-steps/s under `xcodebuild test` |
| Branched unordered articulated snapshot | A sibling graph with revolute, prismatic, and fixed joints must compose parent transforms without relying on descriptor joint order |
| 48-axis readiness negative gate | A missing actuator dynamics declaration must fail `dynamicSimulation` readiness before simulation |
| Run config identity | `configHash` is a 16-character `ConfigHash` value over body, world, embodiment, readiness, duration, timestep, seed, and drive provider; equal requests match and descriptor/provider changes diverge |
| Non-identity actuator mapping replay | `transmissionRatio`, `commandDirection`, `actuatorZeroOffset`, and `jointZeroOffset` must map actuator `-0.10` to joint `0.15` and preserve actuator telemetry separately |
| Mechanical-reduction torque replay | `torque_<actuatorSignal>` must map from joint torque through `mechanicalReductionRatio * efficiency`, independent of position `transmissionRatio` |
| Zero effort replay | A driven prismatic joint with `effortLimit = 0` must log `torque = 0` and no position change over one step |
| Prismatic inertia replay | A `0.2 kg` prismatic descendant with `1 N` saturated effort over `0.01 s` must produce `0.05 m/s` velocity and `0.0005 m` displacement |
| Declared actuator time constant | `dt/substeps = 0.001` with `timeConstantSeconds = 0.0005` must fail with `unstableTimeStep` instead of being rounded up |
| Contact material readiness | Missing body static friction, dynamic friction, or restitution fails `contactTraining` |
| 1-axis penalty contact replay | Descriptor stiffness/damping must emit finite positive normal force and impulse and move the joint out of penetration |
| 1-axis contact constraint replay | Residual penetration `<= 1e-9 m` with descriptor body/world/material/collision inputs |
| 2-axis contact projection replay | Constraint correction is inverse-inertia weighted: a high-reflected-inertia axis remains under `0.001 m` correction while the low-inertia axis absorbs `> 0.14 m` |
| 1-axis contact surface pose replay | World surface pose plus geometry pose must compose to the contact plane used by constraint projection |
| 1-axis contact replay determinism | 100 repeated runs produce equal `SimulationLog` values and sorted JSON bytes |
| 48-axis contact constraint replay | 2,400 contact axis-steps, at least 5,000 axis-steps/s under `xcodebuild test`, residual penetration `<= 1e-9 m` |
| Descriptor corpus acceptance | RoArm M1-like articulated descriptors, contact/material training descriptors, 24-axis contact/material descriptors, and rigid actuator descriptors must pass readiness, deterministic replay, and sorted JSON byte stability while recording hardware-parity gaps when measured evidence is below `.hardwareParity`; a valid hardware-parity report must promote the same articulated descriptor to `.hardwareParity` and persist typed report provenance evidence; missing contact material coefficients must fail readiness before replay; contact-training summaries must persist active-contact replay evidence; saved acceptance summaries must round-trip through artifact validation while rejecting tampered replay or missing hardware-parity evidence. |

The simulator precomputes joint dynamics invariants and snapshot/contact topology before stepping: mapped actuator envelopes, effective inertia, effort limits, servo stiffness/damping, friction, gravity lever terms, root links, deterministic joint traversal order, surface contact constraints, material pairs, and link ancestor paths. Runtime stepping then updates only the evolving state vectors, contact projections, and deterministic logs. The articulated runtime is split into request, public error shell, run loop, validation, descriptor binding, contact setup, logging, topology, and config-hash files so physics review can follow the same contracts the tests exercise. The contact solver is also split into setup, penalty force, constraint projection, contact evaluation, Jacobian mapping, geometry witness, material/surface validation, numeric helpers, and support types.

### Fusion Adapters

- **`QuadrotorAnalyticalModel`** — Wraps `ReferenceQuadrotorPlantEngine` to conform to `AnalyticalModel`. Provides the physics prediction that the world model corrects.
- **`QuadrotorAnalyticalState`** — 13-dimensional state: position(3) + velocity(3) + quaternion(4) + angular velocity(3).

### Sensors

- **`IMU6SensorField`** — 6-axis IMU with configurable noise, bias, delay, and bandwidth.
- **`SwappableSensorField`** — Runtime-swappable sensor parameters for robustness testing.

### Actuators

- **`ReferenceQuadrotorActuatorEngine`** — Motor dynamics with time constants and max thrust clamping.
- **`SwappableActuatorEngine`** — Runtime-swappable actuator parameters.

### Controllers

- **`ImuRateDampingCut`** — Baseline PID controller for quadrotor attitude stabilization.
- **`SinglePropHoverCut`** / **`SinglePropLiftCut`** — Baseline controllers for single-propeller platforms.

## Package Structure

| Module | Dependencies | Description |
|--------|-------------|-------------|
| **KuyuPhysics** | KuyuCore | All physics implementations |

## Requirements

- Swift 6.2+
- macOS 26+

## Dependency Graph

```
KuyuCore
  |
  +-- KuyuPhysics (this package)
        |
        +-- kuyu-scenarios (uses physics for evaluation)
        +-- kuyu-training  (uses physics for data collection)
        +-- kuyu           (assembles fused environment)
```

## License

See repository for license information.
