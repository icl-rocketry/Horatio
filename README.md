WIP Control system for propulsive landing - to be paired with controller physics model repo.

Current control architecture
Successive convexification MPC -> kalman disturbance filtering tracking MPC -> PID actuators

Next changes:
-Adding state machine to adjust control authority and cost matrices at engine relight and prevent kalman updates during relight period
-Integrate dynamics model into MPC controllers, can only be done when the dynamics model is complete
-Decide optimal relight method - specified altitude or decision based and if so how is this nonlinearity accounted for in the dynamics linearisation