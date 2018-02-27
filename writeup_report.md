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

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/Illustration_DH_drawing.png "DH Image"
[image3]: ./misc_images/Illustration_URDF_drawing.png "URDF Image"
[image4]: ./misc_images/l21-l-inverse-kinematics-01.png
[image5]: ./misc_images/l21-l-inverse-kinematics-new-design-fixed.png
[image6]: ./misc_images/scrot1.png
[image7]: ./misc_images/scrot2.png
[image8]: ./misc_images/scrot3.png
[image9]: ./misc_images/scrot4.png



## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.


The image from the demo is given below.


![alt text][image1]


The table below shows the relative locations of joint (i-1) to joint (i) from the URDF file.


Joint Symbol  |    Joint Name | Parent Link |   Child Link | x(m)  | y(m) | z(m)
          --- |           --- |         --- |          --- |  ---  |  --- | ---
J<sub>1</sub> |       joint_1 |   base_link |       link_1 | 0     |    0 | 0.33
J<sub>2</sub> |       joint_2 |      link_1 |       link_2 | 0.35  |    0 | 0.42
J<sub>3</sub> |       joint_3 |      link_2 |       link_3 | 0     |    0 | 1.25
J<sub>4</sub> |       joint_4 |      link_3 |       link_4 | 0.96  |    0 | -0.054
J<sub>5</sub> |       joint_5 |      link_4 |       link_5 | 0.54  |    0 | 0
J<sub>6</sub> |       joint_6 |      link_5 |       link_6 | 0.193 |    0 | 0
J<sub>G</sub> | gripper-joint |      link_6 | gripper_link | 0.11  |    0 | 0


The illustrations of the reference frames are given in the following images.
The DH image shows the reference frames selected with DH algorithm.


![alt text][image2]


The URDF image shows reference frame assignments in URDF file.


![alt text][image3]


Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | alpha0 | a0 | d1 | q1
1->2 | alpha1 | a1 | d2 | q2
2->3 | alpha2 | a2 | d3 | q3
3->4 | alpha3 | a3 | d4 | q4
4->5 | alpha4 | a4 | d5 | q5
5->6 | alpha5 | a5 | d6 | q6
6->G |alpha6 | a6 | d7 | q7

Alpha is the twist angle between Z<sub>i-1</sub> and Z<sub>i</sub> measured about the X<sub>i-1</sub> axis, according to the right hand rule.
If the two joint axes are parallel, alpha equals zero.
a<sub>i-1</sub> is the distance from Z<sub>i-1</sub> to Z<sub>i</sub> measured along the X<sub>i-1</sub> axis.
d<sub>i</sub> is the signed distance between X<sub>i-1</sub> and X<sub>i</sub> measured along the Z<sub>i</sub> axis.
theta<sub>i</sub> is the angle angle between X<sub>i-1</sub> and x<sub>i</sub> measured about the Z<sub>i</sub> axis in a right hand sense.
Since six revolutes, only the theta terms are time variables. Other than Joint2 there are no offsets. Since X<sub>1</sub> is not parallel to X<sub>2</sub> when theta<sub>2</sub> equals zero, there is a constant offset of -90 degrees.


Please note that subscripts are written with a '\_' in the following equations.
For example:  Z<sub>0</sub> is writen as Z_0.
Further, coordinates are separated with '\_' again.
For example: z coordinate of O<sub>0</sub> is shown as O_1_z
O_i      O<sub>i</sub>      symbols can seen from DH image.
O_jointi O<sub>jointi</sub> symbols can seen from URDF image.

```
Since Z_0 and Z_1 are coincident, alpha0 and a0 are zero.
d1 is the distance between X_0 and X_1 measured along Z_1
d1 = O_1_z - O_0_z
   = (O_joint1_z - O_0_z) + (O_joint2_z - O_joint_1_z)
   = J_1_z + J_2_z
   = 0.33 + 0.42
   = 0.75
q1 is the joint angle variable.
```
```
Z_1 || Z_2, when measured about X_1, Z_2 is rotated clockwise, therefore alpha1 = -90.
a1 is the distance from Z1 to Z2 measured along the X_1 axis.
a1 = O_2_x - O_1_x
   = O_joint2_x - O_joint_1_x
   = 0.35
d2 is the distance between X_1 and X_2 measured along Z_2
d2 equals to 0, since X_1 and X_2 intersects.
q2 has a -90 degrees offset, since X_1 and X_2 are not parallel.
q2 = q2 - pi/2
```

```
Z_2 and Z_3 are parallel, therefore, alpha2 equals to zero.
a2 is the distance from Z2 to Z3 measured along the X_2 axis.
a2 = O_3_x - O_2_x
   = O_joint3_z - O_joint2_z
   = 1.25
d3 is the distance between X_2 and X_3 measured along Z_3
d3 equals to 0, since X_2 and X_3 are coincident.
q3 is the joint angle variable.
```

```
Z_3 || Z_4, when measured about X_3, Z_4 is rotated clockwise, therefore alpha3 = -90.
a3 is the distance from Z3 to Z4 measured along the X_3 axis.
a3 = O_4_x - O_3_x
   = O_joint4_z - O_joint3_z
   = -0.054
d4 is the distance between X_3 and X_4 measured along Z_4
d4 = O_5_z - O_3_z
   = (O_joint4_x - O_joint3_x) + (O_joint5_x - O_joint4_x)
   = 0.96  + 0.54
   = 1.5
q4 is the joint angle variable.
```

```
Z_4 || Z_5, when measured about X_4, Z_5 is rotated counter-clockwise, therefore alpha3 = 90.
Since O_4 and O_5 are coincident, a4 and d5 are equal to 0. Z_4 and Z_5 intersects, and X_4 and X_5 are coincident.
q5 is the joint angle variable.
```

```
Z_5 || Z_6, when measured about X_5, Z_6 is rotated clockwise, therefore alpha3 = -90.
Since O_5 and O_6 are coincident, a5 and d6 are equal to 0. Z_5 and Z_6 intersects, and X_5 and X_6 are coincident.
q6 is the joint angle variable.
```

```
Z_6 and Z_G are councidet, alpha6 and a6 are zero.
d7 is the distance between X_6 and X_G measured along Z_G.
d7 = O_G_z - O_6_z
   = (O_joint_6_x - O_joint_5_x) + (O_gripperlink_x - O_joint_6_x)
   = 0.193 + 0.11
   = 0.303
q7 equals to 0 since it is a fixed joint.
```

The resulting DH parameter table is given in the table below.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
---   |        --- |    --- |    --- | ---
0->1  |          0 |  0     |   0.75 |         q1
1->2  |     - pi/2 |  0.35  |      0 | -pi/2 + q2
2->3  |          0 |  1.25  |      0 |         q3
3->4  |     - pi/2 | -0.054 |   1.50 |         q4
4->5  |       pi/2 |  0     |      0 |         q5
5->6  |     - pi/2 |  0     |      0 |         q6
6->G  |          0 |  0     |  0.303 |          0


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

From the values of the DH parameter table, the individual transformation matrices was constructed. The general form of the transformation matrix from frame i-1 to frame i is as follows:

|cos(theta<sub>i</sub>) 				     	   |        -sin(theta<sub>i</sub>)                    |        0 				   |          a<sub>i-1</sub>			     |
|:------------------------------------------------:|:-------------------------------------------------:|:-------------------------:|:---------------------------------------:|
|sin(theta<sub>i</sub>) * cos(alpha<sub>i-1</sub>) | cos(theta<sub>i</sub>) * cos(alpha<sub>i-1</sub>) | -sin(alpha<sub>i-1</sub>) | -sin(alpha<sub>i-1</sub>) * d<sub>n</sub|
|sin(theta<sub>i</sub>) * sin(alpha<sub>i-1</sub>) | cos(theta<sub>i</sub>) * sin(alpha<sub>i-1</sub>) |  cos(alpha<sub>i-1</sub>) |  cos(alpha<sub>i-1</sub>) * d<sub>n</sub|
|            0 									   | 									            0  |        0 				   |              1							 |

##### Transform Matrix From Base Link to Joint 1


|cos(q1) 		  | -sin(q1)         |    0    |     0         |
|:---------------:|:----------------:|:-------:|:-------------:|
|sin(q1) * cos(0) | cos(q1) * cos(0) | -sin(0) | -sin(0) * 0.75|
|sin(q1) * sin(0) | cos(q1) * sin(0) |  cos(0) |  cos(0) * 0.75|
|            0 	  | 		      0  |    0    |     1         |


|  cos(q1) | -sin(q1) |     0    |     0    |
|:--------:|:--------:|:--------:|:--------:|
|  sin(q1) | cos(q1)  |     0    |     0    |
|   0      |    0     |     1    |     0.75 |
|   0      | 	0     |     0    |     1    |


##### Transform Matrix From Joint 1 to Joint 2
|cos(q2-pi/2) 		       | -sin(q2-pi/2)             |    0        |     0.35       |
|:------------------------:|:-------------------------:|:-----------:|:--------------:|
|sin(q2-pi/2) * cos(-pi/2) | cos(q2-pi/2) * cos(-pi/2) | -sin(-pi/2) | -sin(-pi/2) * 0|
|sin(q2-pi/2) * sin(-pi/2) | cos(q2-pi/2) * sin(-pi/2) |  cos(-pi/2) |  cos(-pi/2) * 0|
|            0 	           | 		      0            |    0        |     1          |

|  sin(q2) |  cos(q2) |     0    |     0.35 |
|:--------:|:--------:|:--------:|:--------:|
|   0      |    0     |     1    |     0    |
|  cos(q2) | -sin(q2) |     0    |     0    |
|   0      |    0     |     0    |     1    |



##### Transform Matrix From Joint 2 to Joint 3
|cos(q3) 	      | -sin(q3)         |    0        |     1.25    |
|:---------------:|:----------------:|:-----------:|:-----------:|
|sin(q3) * cos(0) | cos(q3) * cos(0) | -sin(0)     | -sin(0) * 0 |
|sin(q3) * sin(0) | cos(q3) * sin(0) |  cos(0)     |  cos(0) * 0 |
|            0    | 		      0  |    0        |     1       |

|  cos(q3) | -sin(q3) |     0    |     1.25 |
|:--------:|:--------:|:--------:|:--------:|
|  sin(q3) |  cos(q3) |     0    |     0    |
|   0      |    0     |     1    |     0    |
|   0      |    0     |     0    |     1    |


##### Transform Matrix From Joint 3 to Joint 4
|cos(q4) 		      | -sin(q4)             |    0        |    -0.054          |
|:-------------------:|:--------------------:|:-----------:|:------------------:|
|sin(q4) * cos(-pi/2) | cos(q4) * cos(-pi/2) | -sin(-pi/2) | -sin(-pi/2) * 1.50 |
|sin(q4) * sin(-pi/2) | cos(q4) * sin(-pi/2) |  cos(-pi/2) |  cos(-pi/2) * 1.50 |
|            0 	      | 		      0      |    0        |     1              |

|  cos(q4) | -sin(q4) |     0    |   -0.054 |
|:--------:|:--------:|:--------:|:--------:|
|   0      |    0     |     1    |    1.50  |
| -sin(q4) | -cos(q4) |     0    |    0     |
|   0      |    0     |     0    |    1     |


##### Transform Matrix From Joint 4 to Joint 5
|cos(q5) 		      | -sin(q5)             |    0        |     0              |
|:-------------------:|:--------------------:|:-----------:|:------------------:|
|sin(q5) * cos(pi/2)  | cos(q5) * cos(pi/2)  | -sin(pi/2)  | -sin(pi/2)  * 0    |
|sin(q5) * sin(pi/2)  | cos(q5) * sin(pi/2)  |  cos(pi/2)  |  cos(pi/2)  * 0    |
|            0 	      | 		      0      |    0        |     1              |

|  cos(q5) | -sin(q5) |     0    |    0     |
|:--------:|:--------:|:--------:|:--------:|
|   0      |    0     |    -1    |    0     |
|  sin(q5) |  cos(q5) |     0    |    0     |
|   0      |    0     |     0    |    1     |


##### Transform Matrix From Joint 5 to Joint 6
|cos(q6) 		      | -sin(q6)             |    0        |     0              |
|:-------------------:|:--------------------:|:-----------:|:------------------:|
|sin(q6) * cos(-pi/2) | cos(q6) * cos(-pi/2) | -sin(-pi/2) | -sin(-pi/2) * 0    |
|sin(q6) * sin(-pi/2) | cos(q6) * sin(-pi/2) |  cos(-pi/2) |  cos(-pi/2) * 0    |
|            0 	      | 		      0      |    0        |     1              |

|  cos(q6) | -sin(q6) |     0    |    0     |
|:--------:|:--------:|:--------:|:--------:|
|   0      |    0     |     1    |    0     |
| -sin(q6) | -cos(q6) |     0    |    0     |
|   0      |    0     |     0    |    1     |


##### Transform Matrix From Joint 6 to Gripper Link


|cos(0) 		  | -sin(0)         |    0    |     0           |
|:---------------:|:---------------:|:-------:|:---------------:|
|sin(0) * cos(0)  | cos(0) * cos(0) | -sin(0) | -sin(0) * 0.303 |
|sin(0) * sin(0)  | cos(0) * sin(0) |  cos(0) |  cos(0) * 0.303 |
|            0 	  | 		     0  |    0    |     1           |


|   1      |    0     |     0    |    0     |
|:--------:|:--------:|:--------:|:--------:|
|   0      |    1     |     0    |    0     |
|   0      |    0     |     1    |    0.303 |
|   0      | 	0     |     0    |    1     |

##### Generalized Homogeneous Transform Between base_link and gripper_link (using only end-effector(gripper) pose)


The overall homogeneous transform between the base and end effector:
T0_EE


| R0_6  | 0<sub>r<sub>EE/0</sub></sub> |
|:-----:|:----------------------------:|
| 0 0 0 | 1                            |


| r<sub>11</sub> | r<sub>12</sub> | r<sub>13</sub> | p<sub>x</sub> |
|:--------------:|:--------------:|:--------------:|:-------------:|
| r<sub>21</sub> | r<sub>22</sub> | r<sub>23</sub> | p<sub>y</sub> |
| r<sub>31</sub> | r<sub>32</sub> | r<sub>33</sub> | p<sub>z</sub> |
|         0      | 	        0     |           0    |         1     |


The R0_6 can be obtained as follows:


R0_6 = Rot_z * Rot_y * Rot_x
Where

Rot_x is


|      1 |       0 |       0 |
|:------:|:-------:|:-------:|
|      0 |  cos(r) | -sin(r) |
|      0 |  sin(r) |  cos(r) |


Rot_y is


| cos(p) |       0 | -sin(p) |
|:------:|:-------:|:-------:|
|      0 |       1 |       0 |
|-sin(p) |       0 |  cos(p) |


Rot_z is


| cos(y) | -sin(y) |       0 |
|:------:|:-------:|:-------:|
| sin(y) |  cos(y) |       0 |
|      0 |       0 |       1 |


Therefore, R0_6 is


|cos(p)⋅cos(y) | sin(p)⋅sin(r)⋅cos(y) - sin(y)⋅cos(r) | sin(p)⋅cos(r)⋅cos(y) + sin(r)⋅sin(y)|
|:------------:|:------------------------------------:|:-----------------------------------:|
|sin(y)⋅cos(p) | sin(p)⋅sin(r)⋅sin(y) + cos(r)⋅cos(y) | sin(p)⋅sin(y)⋅cos(r) - sin(r)⋅cos(y)|
|   -sin(p)    |            sin(r)⋅cos(p)             |            cos(p)⋅cos(r)            |


There is also the rotation error (Rot_Error), which can be calculated by R0_6 when y equals pi and p equals -pi/2. It is equal to


| 0 | 0  | 1 |
|:-:|:--:|:-:|
| 0 | -1 | 0 |
| 1 | 0  | 0 |


The R0_6 becomes R0_6 * Rot_Error:


|sin(p)⋅cos(r)⋅cos(y) + sin(r)⋅sin(y) | -sin(p)⋅sin(r)⋅cos(y) + sin(y)⋅cos(r) | cos(p)⋅cos(y) |
|:-----------------------------------:|:-------------------------------------:|:-------------:|
|sin(p)⋅sin(y)⋅cos(r) - sin(r)⋅cos(y) | -sin(p)⋅sin(r)⋅sin(y) - cos(r)⋅cos(y) | sin(y)⋅cos(p) |
|           cos(p)⋅cos(r)             |            -sin(r)⋅cos(p)             |    -sin(p)    |


The 0<sub>r<sub>EE/0</sub></sub> is a vector of 3x1 [[px], [py], [pz]]
Combining R0_6 and 0<sub>r<sub>EE/0</sub></sub> as mentioned in the beginning of this section results as


T0_EE:
|sin(p)⋅cos(r)⋅cos(y) + sin(r)⋅sin(y) | -sin(p)⋅sin(r)⋅cos(y) + sin(y)⋅cos(r) | cos(p)⋅cos(y) | px|
|:-----------------------------------:|:-------------------------------------:|:-------------:|:-:|
|sin(p)⋅sin(y)⋅cos(r) - sin(r)⋅cos(y) | -sin(p)⋅sin(r)⋅sin(y) - cos(r)⋅cos(y) | sin(y)⋅cos(p) | py|
|           cos(p)⋅cos(r)             |            -sin(r)⋅cos(p)             |    -sin(p)    | pz|
|                 0                   |                   0                   |       0       | 1 |





#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Joint angles can be seen from the diagrams. WC is the center of the Wrist Center, in the DH steps, WC was accepted to be coincident with O<sub>5</sub>.

![alt text][image4]

From the image above, theta<sub>1</sub> is the atan2 of the y coordinate of the WC devided by the x coordinate of the WC.

```
theta1 = atan2(yc, xc)
```

![alt text][image5]

From the image above,
* WC is the center of the Wrist Center
* 2 and 3 are the Joint2 and Joint3.

In order to calculate theta2 and theta3, we need to construct a triangle, with 2,3, and WC are corners. The length of each side (A, B, C), in other words, the distance between each joint should be calculated.

```
A = | O_joint5 - O_joint3 |
  = | O_3 - O-4 |
  = sqrt((-0.054)^2 + (1.50)^2)
  = 1.500971685275908
```

```
C = | O_joint3 - O_joint2 |
  = 1.25
```


We know the distance between joint2 and joint1 on the XY plane, and also along z axis. Therefore, we need to calculate the distance between WC and joint1 on the XY plane, which can be calculated by sqrt((WC<sub>x</sub>)<sup>2</sup> + (WC<sub>y</sub>)<sup>2</sup>). The distance between WC and joint1 along z axis is the z component of WC, that is WC<sub>z</sub>.

```
WC_xy = sqrt(WC_x ^ 2 + WC_y ^ 2)
WC_z = WC_z
```
```
B = | WC - O_joint2 |
  = sqrt((WCxy - O_2_x) ^ 2 +  (WCz - O_2_z) ^ 2)
  = sqrt((WCxy - 0.35) ^ 2 +  (WCz - 0.75) ^ 2)
```


The angle values were calculated, from the side lengths.
```
a = acos((B^2 + C^2 - A^2) / (2 * B * C))
b = acos((A^2 + C^2 - B^2) / (2 * A * C))
c = acos((A^2 + B^2 - C^2) / (2 * A * B))
```


```
theta2 + a + a' = pi/2
a' = atan(WC_z, WC_xy)
theta2 = pi/2 - a - a'

theta3 + (b + sag_angle) = pi/2
sag_angle = atan2(-0.054, A)
# sag_angle ~= 0.036 from link4 of -0.054m
theta3 = pi/2 - (b + sag_angle)
```

The following joint angles are calculated by finding the Euler Angles from the rotation matrix from 3rd frame to 6th frame (R3_6). Calculations to obtain the R3_6 are given below. R0_6 (Rot_EE) was obtained in the last part of second part of the Kinematic Analysis section. R0_3 is the rotation matrix from base frame to 3rd frame. R0_1 can be obtained by taking the 3x3 matrix from the upper left corner of the T0_1 matrix. Same rule applies for R1_2 and R2_3 too.

```
R3_6 = inv(R0_3) * R0_6
R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
```

Once we obtain the R3_6 matrix, the Euler Angles can be obtained from that matrix.
<sup>A</sup><sub>B</sub>R<sub>XYZ</sub> = R<sub>Z</sub>(alpha)R<sub>Y</sub>(beta)R<sub>X</sub>(gamma) = R<sub>Z</sub>(a)R<sub>Y</sub>(b)R<sub>X</sub>(g)

| r<sub>11</sub> | r<sub>12</sub> | r<sub>13</sub> |
|:--------------:|:--------------:|:--------------:|
| r<sub>21</sub> | r<sub>22</sub> | r<sub>23</sub> |
| r<sub>31</sub> | r<sub>32</sub> | r<sub>33</sub> |


| -sin(q₄)⋅sin(q₆) + cos(q₄)⋅cos(q₅)⋅cos(q₆) | -sin(q₄)⋅cos(q₆) - sin(q₆)⋅cos(q₄)⋅cos(q₅) | -sin(q₅)⋅cos(q₄)|
|:------------------------------------------:|:------------------------------------------:|:---------------:|
|              sin(q₅)⋅cos(q₆)               |              -sin(q₅)⋅sin(q₆)              |     cos(q₅)     |
| -sin(q₄)⋅cos(q₅)⋅cos(q₆) - sin(q₆)⋅cos(q₄) | sin(q₄)⋅sin(q₆)⋅cos(q₅) - cos(q₄)⋅cos(q₆)  | sin(q₄)⋅sin(q₅) |


```
q₅ = atan2(sqrt((-sin(q₅)⋅cos(q₄))^2 + (sin(q₄)⋅sin(q₅))^2), cos(q₅))
   = atan2(sqrt((r_13)^2 + (r_33)^2), r_23)

q₄ = atan2(sin(q₄)⋅sin(q₅), -(-sin(q₅)⋅cos(q₄)))
   = atan2(r_33, -r_13) if sin(q₅) >= 0
   = atan2(-r_33, r_13) if sin(q₅) < 0

q₆ = atan2(-(-sin(q₅)⋅sin(q₆)), sin(q₅)⋅cos(q₆))
   = atan2(-r_22, r_21) if sin(q₅) >= 0
   = atan2(r_22, -r_21) if sin(q₅) < 0
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

#### 2. Screenshots


![alt text][image6]


First screenshot shows eight cylinders in the bin. There are two other cylinders, one of them is in front of the shelf, the other is on the left bottom corner of the shelf.


![alt text][image7]


The second screenshot shows the 11th cylinder on the upper right corner of the shelf. The other two cylinders that the Arm failed to place in the bin can be seen. One of them is in front of the shelf, the other is on the left bottom corner of the shelf. It can be seen that there are 11 "target_model"s from the World tab.


![alt text][image8]


The third screenshot shows the 11th cylinder is just on top of the bin.


![alt text][image9]


The fourth screenshot shows the 11th cylinder is succesfully placed in to the bin. It makes 9 successful placements out of 11. In addition, the 12th cylinder is on the bottom middle cell of the shelf.


#### 3. Results


The code can complete 8/10, further, it increased by on with 9/11. However, it is not consistent. It cannot always place eight cylinders out of ten. Mostly, the arm fails to grasp the cylinders. The algorithm is slow since it takes more than one minute to place a cylinder. If the error is calculated, then the process time increases significantly.
