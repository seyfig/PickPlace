## Project: Kinematics Pick & Place

---


**Steps to complete the project:**


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.


[//]: # (Image References)

[image1]: ./misc_images/l21-l-inverse-kinematics-01.png
[image2]: ./misc_images/l21-l-inverse-kinematics-new-design-fixed.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

From the URDF file the relative positions are given in the table below.

Joint Name | Parent Link | Child Link | x(m) | y(m) | z(m)
--- | --- | --- | --- | --- | ---
joint_1 | base_link | link_1 | 0 | 0 | 0.33
joint_2 | link_1 | link_2 | 0.35 | 0 | 0.42
joint_3 | link_2 | link_3 | 0 | 0 | 1.25
joint_4 | link_3 | link_4 | 0.96 | 0 | -0.054
joint_5 | link_4 | link_5 | 0.54 | 0 | 0
joint_6 | link_5 | link_6 | 0.193 | 0 | 0
gripper-joint | link_6 | gripper_link | 0.11 | 0 | 0


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 1.25 | q3
3->4 | - pi/2 | -0.54 | 1.50 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Joint angles can be seen from the diagrams.

![alt text][image1]

From the image above,

```
theta1 = atan2(yc, xc)
```

![alt text][image2]

From the image above,

```
theta2 = pi/2 - a - atan(WCz, WCxy)
theta3 = pi/2 - (b + 0.036)
```

The following joint angles are calculated by finding the Euler Angles from the rotation matrix from 3rd frame to 6th frame (R3_6).

```
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(sqsum(R3_6[0,2], R3_6[2,2])), R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```



### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

##### 1. Additional Functions
** homogeneous_transform function **
It returns the homogeneous_transform matrix for given alpha_n_1, a_n_1, q_n and d_n. This function was used to obtain 7 matrices for each link.

** sqsum function **
It returns to sum of squares of the given two parameters.

##### 2. Transformer Class
It creates the required matrices, when the class is created.
* Matrix of Composition of Homogeneous Transforms (T0_G)
* Rot_EE
```
Rrpy = Rot(Z, yaw) * Rot(Y, pitch) * Rot(X, roll) * R_corr
```
* Rotation Matrix from base frame to 3rd frame (R0_3).

Since they are required symbols; q1, q2, and q3 are properties of this class.

##### 3. handle_calculate_IK function
For each pose received, this function obtains the position and orientation of the end effector. Using the Transformer class instance, it calculates the position of the wrist center. Then calculates the joint angles.

##### 4. IK_server function
It creates transformer instance of Transformer class. Since, it makes calculations in advance, it takes some time to complete and init the IK_server ros node.

This implementation is based on the KR210 Forward Kinematics lessos and the Project Walkthrough Video.




