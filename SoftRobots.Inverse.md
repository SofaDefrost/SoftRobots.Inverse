# InverseSoftRobotics

 - SoftRobots.Inverse is a private (closed-source) extension to the SoftRobots plugin. The SoftRobots plugin is extended mainly by adding the capability of solving the Inverse Problem. This means the user has the capability of defining an Effector and a Goal so the Inverse Solver will determine the required actuation to minimize the distance between the Effector and the Goal. More information about the methods can be found here [Constraint Resolution Method](https://project.inria.fr/softrobot/documentation/constraint-resolution-method/) and [Real-Time Inverse Simulation Method](https://project.inria.fr/softrobot/documentation/real-time-inverse-simulation-method/)

## TUTORIALS :

In addition to the tutorial provided with the SoftRobots plugin, in the following we will introduce the examples that show the functionality that only works with the SoftRobots.Inverse plugin.

Steps 1-4 of the Tutorial are provided in the SoftRobots plugin. If you haven't seen them we recommend you to look at them first by loading the SoftRobots.pyscn scene in your SoftRobots plugin directory.

Here are the links, which will load each example with respective description. Please use the 'back'-button of the browser to return to this page.

[Tutorial 5: Actuator Limits](./examples/Tutorial/step5-ActuatorLimits/Finger.pyscn)
[Tutorial 6: Direct Control of Underactuated Multi-Finger Gripper](./examples/Tutorial/step6-SoftGripper/SoftGripper_directControl.pyscn)
[Tutorial 7: Grasping Simulation with Multi-Finger Gripper](./examples/Tutorial/step7-SoftGripperGraspingSimulation/SoftGripper.pyscn)
[Tutorial 8: Enabiling Self-Collision Regions](./examples/Tutorial/step8-EnablingSelfCollisionRegions/Finger.pyscn)

## OTHER COMMENTS :

## Usage:
<RequiredPlugin name="SoftRobots">
<RequiredPlugin name="SoftRobots.Inverse">
