## Project: Kinematics Pick & Place

---

[//]: # (Image References)

[image1]: ./misc_images/IMG_1432.jpeg
[image2]: ./misc_images/IMG_1434.jpg
[image3]: ./misc_images/IMG_1435.jpg
[image4]: ./misc_images/1.png
[image5]: ./misc_images/2.png
[image6]: ./misc_images/3.png
[image7]: ./misc_images/4.png
[image8]: ./misc_images/5.png
[image9]: ./misc_images/6.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Kinematic Analysis

#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Using the URDF file, specifically starting from line 316:

```xml
<!-- joints -->
  <joint name="fixed_base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <joint name="joint_1" type="revolute">
    <origin xyz="0 0 0.33" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="link_1"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-185*deg}" upper="${185*deg}" effort="300" velocity="${123*deg}"/>
  </joint>
  <joint name="joint_2" type="revolute">
    <origin xyz="0.35 0 0.42" rpy="0 0 0"/>
    <parent link="link_1"/>
    <child link="link_2"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-45*deg}" upper="${85*deg}" effort="300" velocity="${115*deg}"/>
  </joint>
  <joint name="joint_3" type="revolute">
    <origin xyz="0 0 1.25" rpy="0 0 0"/>
    <parent link="link_2"/>
    <child link="link_3"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-210*deg}" upper="${(155-90)*deg}" effort="300" velocity="${112*deg}"/>
  </joint>
  <joint name="joint_4" type="revolute">
    <origin xyz="0.96 0 -0.054" rpy="0 0 0"/>
    <parent link="link_3"/>
    <child link="link_4"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${179*deg}"/>
  </joint>
  <joint name="joint_5" type="revolute">
    <origin xyz="0.54 0 0" rpy="0 0 0"/>
    <parent link="link_4"/>
    <child link="link_5"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-125*deg}" upper="${125*deg}" effort="300" velocity="${172*deg}"/>
  </joint>
  <joint name="joint_6" type="revolute">
    <origin xyz="0.193 0 0" rpy="0 0 0"/>
    <parent link="link_5"/>
    <child link="link_6"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${219*deg}"/>
  </joint>
```

I calculated the parameters drawn below. For example, 
```math
d_4=0.96+0.54=1.5
```
from URDF file above.

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Here is the DH table I calculated:

```python
s = {alpha0:     0,  a0:      0, d1:  0.75,
     alpha1: -pi/2,  a1:   0.35, d2:     0, q2: q2-pi/2,
     alpha2:     0,  a2:   1.25, d3:     0,
     alpha3: -pi/2,  a3: -0.054, d4:   1.5,
     alpha4:  pi/2,  a4:      0, d5:     0,
     alpha5: -pi/2,  a5:      0, d6:     0,
     alpha6:     0,  a6:      0, d7: 0.303, q7: 0}
```

And the following are the transformation matrices:

```python
T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
               [                   0,                   0,            0,               1]])
T0_1 = T0_1.subs(s)

T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
               [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
               [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
               [                   0,                   0,            0,               1]])
T1_2 = T1_2.subs(s)

T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
               [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
               [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
               [                   0,                   0,            0,               1]])
T2_3 = T2_3.subs(s)

T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
               [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
               [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
               [                   0,                   0,            0,               1]])
T3_4 = T3_4.subs(s)

T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
               [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
               [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
               [                   0,                   0,            0,               1]])
T4_5 = T4_5.subs(s)

T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
               [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
               [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
               [                   0,                   0,            0,               1]])
T5_6 = T5_6.subs(s)

T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
               [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
               [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
               [                   0,                   0,            0,               1]])
T6_G = T6_G.subs(s)
```

The homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose, can be computed as follows:

```python
R_roll  = Matrix([[           1,         0,          0],
                  [           0, cos(roll), -sin(roll)],
                  [           0, sin(roll),  cos(roll)]])

R_pitch = Matrix([[  cos(pitch),         0, sin(pitch)],
                  [           0,         1,          0],
                  [ -sin(pitch),         0, cos(pitch)]])

R_yaw   = Matrix([[    cos(yaw), -sin(yaw),          0],
                  [    sin(yaw),  cos(yaw),          0],
                  [           0,         0,          1]])

R_z = Matrix([[     cos(pi), -sin(pi),          0, 0],
              [     sin(pi),  cos(pi),          0, 0],
              [           0,        0,          1, 0],
              [           0,        0,          0, 1]])

R_y = Matrix([[  cos(-pi/2),        0, sin(-pi/2), 0],
              [           0,        1,          0, 0],
              [ -sin(-pi/2),        0, cos(-pi/2), 0],
              [           0,        0,          0, 1]])
R_corr = R_z * R_y

R0_6 = R_yaw * R_pitch * R_roll * R_corr[:3, :3]
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

After computing the position of Wrist Center (WC) using 

```python 
WC = Matrix([[px], [py], [pz]]) - 0.303 * R0_6[:, 2]
```
we can compute the theta (1, 2, 3) angles. Here is a sketch of my calculation for the derivation of theta angles. 

![alt text][image2]
![alt text][image3]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

Attached `IK_server.py` completed 10/10 successful jobs. Here are some of the screenshots after the completion of 10 pick and place jobs.

In `IK_server.py` the reduced the running time by moving the `R_corr` calculation outside of the `for` loop. Other than that I played with changing the `sympy`'s `inv` algorithm from Gaussian Elimination to LU but that increases the running time. 

![alt text][image4]
![alt text][image5]
![alt text][image6]
![alt text][image7]
![alt text][image8]
![alt text][image9]

