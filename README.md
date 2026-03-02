# Horatio

- WIP Control system for propulsive landing - to be paired with controller physics model repo.

Current control architecture
-
- Multiphase Successive convexification MPC -> kalman disturbance filtering tracking MPC -> PID actuators

Current relight method
-
- To allow optimal relight solving whilst remaining a convex optimisation process a multiphase temporal dialation approach has been taken.

Next changes
-
- Implementing a finite state machine for KDF MPC layer to allow parameter changes across phases

- Integrate dynamics model into MPC controllers, can only be done when the dynamics model is complete

- Cleaning code and optimisation in heavy areas

- integrate sensor models and EKF built by electronics into the loop

- Allow interfacing with real hardware (PWM signals for servos)

- Implement control logic into simulink for testing