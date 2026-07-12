# Admittance vs. Impedance Control

**Compliance** is the umbrella term: a robot that yields to external force instead of
fighting it rigidly. Impedance and admittance are the two ways to *implement*
compliance. They're duals of each other — the difference is which direction the
causality runs.

## Impedance control

- **Input:** motion (position/velocity error)
- **Output:** force/torque
- Computes a spring-damper law directly — `F = K·x + B·ẋ` — and commands that
  force/torque to the actuator.
- "Given how far you've been pushed off target, apply this much resisting force."
- Natural fit for robots with **direct torque control** at the joint (e.g. an arm
  with torque-sensing/torque-controllable motors).

## Admittance control

- **Input:** force/torque (measured, usually from an F/T sensor)
- **Output:** motion (position/velocity command)
- Inverts the spring-damper law to solve for `x` given a measured `F`, then feeds
  that as a setpoint to an inner position controller.
- "Given how hard you're being pushed, here's how much you should move."
- Natural fit for robots that are **position-controlled** at the joint (most
  hobby/industrial servo setups), since it wraps an outer loop around position
  control you already have — no torque-controllable actuators required.

## Comparison

| | Impedance | Admittance |
|---|---|---|
| Senses | position/motion | force/torque |
| Outputs | force/torque | position/motion |
| Needs | torque-controllable actuators | force/torque sensor + inner position loop |

## Where this fits WALL-E

The gimbal (`uno_bridge.cpp`) and drive motors are position/velocity-commanded,
not torque-commanded, and there's no force/torque sensor anywhere on the robot —
only an IMU (MPU6050). That makes **admittance** the only realistically
implementable option without new hardware, and even then "force" would have to be
*inferred* rather than measured directly:

- Gimbal: IMU jolt/acceleration spikes as a proxy for an external push
- Drive base: motor current draw as a proxy for collision/obstruction force

This is sometimes called **"sensorless" or "virtual" admittance** — noisier and
less precise than a real F/T-sensor-based implementation, but usable without
adding hardware.

**Status:** notes only — no admittance controller implemented yet. If/when this
gets built, it would sit as an outer loop around the existing gimbal PID
(`uno_bridge.cpp`) or drive commands (`state_machine.cpp`), not replace them.
