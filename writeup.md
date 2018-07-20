## Project: Building a Controller
For this project, you will be building a controller in C++. You will be implementing and tuning this controller in several steps.
---

# Required Steps for a Passing Submission:
1. Body rate and roll/pitch control (scenario 2)
2. Position/velocity and yaw angle control (scenario 3)
3. Non-idealities and robustness (scenario 4)
4. Tracking trajectories (scenario 5)

## [Rubric](https://review.udacity.com/#!/rubrics/1643/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Implemented Controller

#### 1. Explain the implemented body rate control in C++.
The controller should be a proportional controller on body rates to commanded moments. The controller should take into account the moments of inertia of the drone when calculating the commanded moments.

It calculates a desired 3-axis moment given a desired and current body rate.

type|parameter|description
---|---|---
Input|pqrCmd| desired body rates [rad/s]
Input|pqr| current or estimated body rates
Output|momentCmd|a V3F containing the desired moments for each of the 3 axes

momentCmd = Inertia x kpPQR x (pqrCmd - pqr)

#### 2. Explain the implemented roll pitch control in C++.
The controller should use the acceleration and thrust commands, in addition to the vehicle attitude to output a body rate command. The controller should account for the non-linear transformation from local accelerations to body rates. Note that the drone's mass should be accounted for when calculating the target angles.

It calculates a desired pitch and roll angle rates based on a desired global lateral acceleration, the current attitude of the quad, and desired collective thrust command.

type|parameter|description
---|---|---
Input|accelCmd| desired acceleration in global XY coordinates [m/s2]
Input|attitude| current or estimated attitude of the vehicle
Input|collThrustCmd| desired collective thrust of the quad [N]
Output|pqrCmd|a V3F containing the desired pitch and roll rates. The Z element of the V3F should be left at its default value (0)

- bx_c_dot = kpBank x (bx_c - bx_a)
- by_c_dot = kpBank x (by_c - by_a)
- pqrCmd.x = (1/R(2,2)) x (R(1,0) x bx_c_dot - R(0,0) x by_c_dot)
- pqrCmd.y = (1/R(2,2)) x (R(1,1) x bx_c_dot - R(0,1) x by_c_dot)

#### 3. Explain the implemented altitude controller in C++.
The controller should use both the down position and the down velocity to command thrust. Ensure that the output value is indeed thrust (the drone's mass needs to be accounted for) and that the thrust includes the non-linear effects from non-zero roll/pitch angles.

Additionally, the C++ altitude controller should contain an integrator to handle the weight non-idealities presented in scenario 4.

It calcuates desired quad thrust based on altitude setpoint, actual altitude, vertical velocity setpoint, actual vertical velocity, and a vertical acceleration feed-forward command

type|parameter|description
---|---|---
Input|posZCmd, velZCmd| desired vertical position and velocity in NED [m]
Input|posZ, velZ| current vertical position and velocity in NED [m]
Input|accelZCmd| feed-forward vertical acceleration in NED [m/s2]
Input|dt|the time step of the measurements [sec]
Output|thrust|a collective thrust command in [N]

- integratedAltitudeError += (posZCmd - posZ) * dt
- u1_bar = kpPosZ x (posZCmd - posZ) + kpVelZ x (velZCmd - velZ) + KiPosZ x integratedAltitudeError + accelZCmd
- accel = (u1_bar - gravity) / R(2,2)
- thrust = - mass * accel

#### 4. Explain the implemented lateral position control in C++.
The controller should use the local NE position and velocity to generate a commanded local acceleration.

It calculates a desired horizontal acceleration based on desired lateral position / velocity / acceleration and current pose

type|parameter|description
---|---|---
Input|posCmd| desired position in NED [m]
Input|velCmd| desired velocity in NED [m/s]
Input|pos| current position in NED [m]
Input|vel| current velocity in NDE [m/s]
Input|accelCmdFF| feed-forward acceleration in NED [m/s2]
Output|accelCmd|a V3F with desired horizontal accelerations. The Z component should be 0.

- accelCmd = kpPosXYZ x (posCmd - pos) + kpVelXYZ x (velCmd - vel) + accelCmdFF

#### 5. Explain the implemented yaw control in C++.
The controller can be a linear/proportional heading controller to yaw rate commands (non-linear transformation not required).

It calculates a desired yaw rate to control yaw to yawCmd.

type|parameter|description
---|---|---
Input|yawCmd| commanded yaw [rad]
Input|yaw| current yaw [rad]
Output|yawRateCmd|a desired yaw rate [rad/s]

- yawCmd_unwrap : unwrap a radian angle measure float to range [0,2xPI]
- yawRateCmd = kpYaw x (yawCmd_unwrap - yaw)

#### 6. Explain the implemented calculation of the motor commands given commanded thrust and moments in C++.
The thrust and moments should be converted to the appropriate 4 different desired thrust forces for the moments. Ensure that the dimensions of the drone are properly accounted for when calculating thrust from moments.

It converts a desired 3-axis moment and collective thrust command to individual motor thrust commands.

type|parameter|description
---|---|---
Input|collThrustCmd| desired collective thrust [N]
Input|momentCmd| desired rotation moment about each axis [N m]
Output|cmd| set class member variable cmd where cmd.desiredThrustsN[0..3]: motor commands in [N]

- l = L / sqrt(2)
- f_x = momentCmd.x / l
- f_y = momentCmd.y / l
- f_z = -momentCmd.z / kappa
- f_tot = collThrustCmd
- cmd.desiredThrustsN[0] = (f_x + f_y + f_z + f_tot) / 4
- cmd.desiredThrustsN[1] = (-f_x + f_y - f_z + f_tot) / 4
- cmd.desiredThrustsN[2] = (f_x - f_y - f_z + f_tot) / 4
- cmd.desiredThrustsN[3] = (-f_x - f_y + f_z + f_tot) / 4

### Flight Evaluation
Your C++ controller is successfully able to fly the provided test trajectory and visually passes inspection of the scenarios leading up to the test trajectory.

Ensure that in each scenario the drone looks stable and performs the required task. Specifically check that the student's controller is able to handle the non-linearities of scenario 4 (all three drones in the scenario should be able to perform the required task with the same control gains used).

The controller can fly the provided test trajectory and visually passes inspection of the scenarios leading up to the test trajectory.
