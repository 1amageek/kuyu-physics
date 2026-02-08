# kuyu-physics

Physics engines and analytical models for the Kuyu simulation environment.

## Overview

kuyu-physics provides concrete physics implementations that conform to kuyu-core protocols. It contains ODE-based rigid body dynamics, sensor models, actuator models, and the quadrotor reference plant.

### Physics Engines

- **`ReferenceQuadrotorPlantEngine`** — 6-DOF rigid body dynamics with RK4 integration, motor model, aerodynamics, drag, gravity, and gyroscopic effects.
- **`SinglePropPlantEngine`** — Simplified single-propeller dynamics for lift control.

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
