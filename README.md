### VERSA-BOT V 1.0 – A SHOP -FLOOR MOBILE MANIPULATOR 

## Goals of the Project:

- Modelling the Versa-BOT in the SolidWorks software with 7 degrees of freedom.
- Exporting the 3-D model into URDF.
- Moving the mobile manipulator URDF to the Ubuntu OS to work with ROS 2.
- Spawning the robot in the Gazebo environment in an empty world like did in project 1.
- Including LIDAR in the robot and RViz visualization of the sensor.
- Creating an algorithm for controlling the robot in the gazebo environment.

### 3.2. Dimensions of the Robot

**MOBILE BASE**

| Property   | Measurement |
|------------|-------------|
| Length     | 762.00 mm   |
| Width      | 381.00 mm   |
| Height     | 157.85 mm   |
| Wheelbase  | 304.80 mm   |
| Wheel Radius | 108.00 mm |

**MANIPULATOR**

| Property       | Measurement |
|----------------|-------------|
| Arm Base Length  | 100.00 mm   |
| First Link Length | 190.50 mm   |
| Second Link Length | 190.50 mm   |
| Third Link Length | 328.93 mm   |
| Radius of the Link | 38.00 mm    |
| End-Effector    | Vacuum Gripper |

![568acdeb-eeaa-4383-bb7b-baf9fdfe6db5](https://github.com/cravotics/Mobile-Manipulator/assets/90138418/f3cd257b-cadc-4c59-9f25-4d34466b88f4)

## 9.2 Control Method - Open-loop Controller 

The control method employed for robot operation is an **open-loop controller**. This type of controller facilitates the operation in the Gazebo environment, enabling the robot's vacuum gripper—the end-effector—to pick up a can effectively.

### Control Flow
The sequence of operations is as follows:

- **Initial Movement:**
  - For the initial **5 seconds**, the robot is set to a velocity of `3m/s`.
  - Then, it accelerates to `6 m/s` for the subsequent **3 seconds**.
  
- **Arm Articulation:**
  - Post linear motion, the robotic arm executes a **predefined joint angle trajectory**, tracing the path of an arc.
  - A `time function` halts the arm after **2 seconds**, ensuring a smooth transition.

- **Gripper Engagement:**
  - As the arm ceases motion, the **vacuum gripper** activates to secure a coke can.

- **Reverse Motion:**
  - The robot then reverses at a speed of `3 m/s` for **5 seconds**, followed by `6 m/s` for the final **3 seconds**.
  - During this time, the **gripper maintains its hold** on the Coke can.

- **Gripper Disengagement:**
  - The grip is released after the reversal is complete, marking the end of the operation.

This systematic approach, merging articulated manipulation with linear movement, showcases the robot's capability to execute a **pick and place** task with precision and efficiency.
