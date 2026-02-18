WIP Control system for propulsive landing - to be paired with controller physics model repo.

Current control architecture
Multiphase Successive convexification MPC -> kalman disturbance filtering tracking MPC -> PID actuators

Current relight method
-To allow optimal relight solving whilst remaining a convex optimisation process a multiphase temporal dialation approach has been taken.

Next changes:
-Adding state machine to adjust control authority and cost matrices at engine relight and prevent kalman updates during relight period
-Integrate dynamics model into MPC controllers, can only be done when the dynamics model is complete
-Finding robust method to handle relight and spooling period where relight takes longer than predicted