#!/usr/bin/env python3

from sympy import *
from matplotlib import pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
import time as tm
init_printing(use_unicode=False, wrap_line=False)

# Symbols definition
alpha, a, d, theta = symbols('alpha a d theta')
theta1, theta2, theta3, theta4 = symbols('theta1 theta2 theta3 theta4')

class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.arm_joint_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.gripper_service = self.create_client(SetBool, '/switch')

    def publish_joint_angles(self, angles):
        msg = Float64MultiArray()
        msg.data = angles
        self.arm_joint_pub.publish(msg)
    def publish_wheel_velocities(self, velocities):
        msg = Float64MultiArray()
        msg.data = velocities
        self.wheel_velocities_pub.publish(msg)


q1_min, q1_max = -3.14, 3.14
q2_min, q2_max = -0.5, 1.0
q3_min, q3_max = -2.9, 2.5
q4_min, q4_max = -3.2, 2

#dh parameters
alpha_array = [pi/2, 0, 0, 0]
theta_array = [theta1, -(pi/2)+theta2, theta3+pi, theta4]
d_array = [0.0381, -0.0706, 0, 0]  # m
a_array = [0, -0.1905, -0.1907, 0.32893]  # m

#calculating the trasnformation matrix
Rz = Matrix([[cos(theta), -sin(theta), 0, 0],
             [sin(theta),  cos(theta), 0, 0],
             [0,        0, 1, 0],
             [0,        0, 0, 1]])

Tz = Matrix([[1,  0,  0,  0],
             [0,  1,  0,  0],
             [0,  0,  1,  d],
             [0,  0,  0,  1]])

Tx = Matrix([[1,  0,  0,  a],
             [0,  1,  0,  0],
             [0,  0,  1,  0],
             [0,  0,  0,  1]])

Rx = Matrix([[1,        0,        0,  0],
             [0,  cos(alpha), -sin(alpha),  0],
             [0,  sin(alpha),  cos(alpha),  0],
             [0,        0,        0,  1]])

T1_gen = Rz*Tz*Tx*Rx
T2_gen = Rz*Tz*Tx*Rx
T3_gen = Rz*Tz*Tx*Rx
T4_gen = Rz*Tz*Tx*Rx

#Substituting Params into Each Transformation Matrix
T1 = T1_gen.subs(alpha, alpha_array[0]).subs(
    theta, theta_array[0]).subs(d, d_array[0]).subs(a, a_array[0])
T2 = T2_gen.subs(alpha, alpha_array[1]).subs(
    theta, theta_array[1]).subs(d, d_array[1]).subs(a, a_array[1])
T3 = T3_gen.subs(alpha, alpha_array[2]).subs(
    theta, theta_array[2]).subs(d, d_array[2]).subs(a, a_array[2])
T4 = T4_gen.subs(alpha, alpha_array[3]).subs(
    theta, theta_array[3]).subs(d, d_array[3]).subs(a, a_array[3])


#calculating the jacobian matrix by calculating the individual conponenets 
H0_1 = T1
H0_2 = T1*T2
H0_3 = T1*T2*T3
H0_4 = T1*T2*T3*T4

P = Matrix([[H0_4[3]], [H0_4[7]], [H0_4[11]]])

Par1 = diff(P, theta1)
Par2 = diff(P, theta2)
Par3 = diff(P, theta3)
Par4 = diff(P, theta4)

Z0_1 = Matrix([[H0_1[2]], [H0_1[6]], [H0_1[10]]])
Z0_2 = Matrix([[H0_2[2]], [H0_2[6]], [H0_2[10]]])
Z0_3 = Matrix([[H0_3[2]], [H0_3[6]], [H0_3[10]]])
Z0_4 = Matrix([[H0_4[2]], [H0_4[6]], [H0_4[10]]])

J1 = Matrix([[Par1], [Z0_1]])
J2 = Matrix([[Par2], [Z0_2]])
J3 = Matrix([[Par3], [Z0_3]])
J4 = Matrix([[Par4], [Z0_4]])


#Jacobian
J = Matrix([[J1, J2, J3, J4]])

r = 0.11  # m

q1 = 0.0
q2 = 0.0
q3 = 0.0
q4 = 0.0



def clamp_angle(angle, min_limit, max_limit):
    if angle < min_limit:
        return min_limit
    elif angle > max_limit:
        return max_limit
    return angle


rclpy.init(args=None)
arm_controller_node = ArmControllerNode()
wheel_velocities = Float64MultiArray()
wheel_velocities.data = [0.0, 0.0, 0.0, 0.0]  # Initialize all wheel velocities to zero

wheel_velocities = [0.0, 0.0, 0.0, 0.0]

#moving the bot forward
timer = tm.time()
run_duration = 8.0

while True:
    current_time = tm.time()
    # Setting wheel velocity to zero
    if current_time - timer >= run_duration:

        wheel_velocities = [0.0, 0.0, 0.0, 0.0]
        arm_controller_node.publish_wheel_velocities(wheel_velocities)
        break
    
    if current_time - timer <= 5.0:
        wheel_velocities = [3.0, -3.0, 3.0, -3.0]
    elif 5.0 < current_time - timer <= 8.0:
        wheel_velocities = [6.0, -6.0, 6.0, -6.0]
    arm_controller_node.publish_wheel_velocities(wheel_velocities)
    tm.sleep(0.1)

last_publish_time = tm.time()
time = np.linspace(0, 200, num=400)
dt = 200 / 400

x_1, y_1, z_1 = 0.0, 0.0, 0.0
x_2, y_2, z_2 = 0.0, 0.0, 1.0

# Calculating the individual joint angles
total_distance = np.sqrt((x_2 - x_1)*2 + (y_2 - y_1)*2 + (z_2 - z_1)*2)
velocity = total_distance / 20 
timer_3 = tm.time()
for t in time:
    current_time_3 = tm.time()
    v_x = -((2*pi*r)/200)*sin((2*pi*t)/200)
    v_y = 0.0
    v_z = ((2*pi*r)/200)*cos((2*pi*t)/200)
    w_x = 0.0
    w_y = 0.0
    w_z = 0.0
    X_dot = Matrix([[v_x], [v_y], [v_z], [w_x], [w_y], [w_z]])
    J_Applied = J.subs({theta1: q1, theta2: q2, theta3: q3, theta4: q4})
    J_T = J_Applied.transpose()
    q_dot = (J_Applied.pinv() * X_dot).evalf()

  
    q1_dot = q_dot[0]
    q2_dot = q_dot[1]
    q3_dot = q_dot[2]
    q4_dot = q_dot[3]
    
    q1 = clamp_angle(q1 + q1_dot * dt, q1_min, q1_max)
    q2 = clamp_angle(q2 + q2_dot * dt, q2_min, q2_max)
    q3 = clamp_angle(q3 + q3_dot * dt, q3_min, q3_max)
    q4 = clamp_angle(q4 + q4_dot * dt, q4_min, q4_max)

    #timer to breake the arm movement
    if current_time_3 - timer_3 >= 2:
        break

    current_time = tm.time()
    if current_time - last_publish_time >= 0.001:
        q1 = float(q1)
        q2 = float(q2)
        q3 = float(q3)
        q4 = float(q4)
        print(q1, q2, q3, q4)
        arm_controller_node.publish_joint_angles([q1, q2, q3, q4, 0.0])
        last_publish_time = current_time

#vacuum gripper
arm_controller_node.gripper_service.wait_for_service()
request = SetBool.Request()
request.data = True
arm_controller_node.gripper_service.call_async(request)

#moving the bot back
timer_3 = tm.time()
while True:
    current_time_3 = tm.time()
    #setting wheel velocity to 0 before going to the next process
    if current_time_3 - timer_3 >= run_duration:
        wheel_velocities = [0.0, 0.0, 0.0, 0.0]
        arm_controller_node.publish_wheel_velocities(wheel_velocities)
        break
    
    if current_time_3 - timer_3 <= 5.0:
        wheel_velocities = [-3.0, 3.0, -3.0, 3.0]
    elif 5.0 < current_time_3 - timer_3 <= 4.0:
        wheel_velocities = [-6.0, 6.0, -6.0, 6.0]
    arm_controller_node.publish_wheel_velocities(wheel_velocities)
    tm.sleep(0.1)
    
arm_controller_node.gripper_service.wait_for_service()
request = SetBool.Request()
request.data = False
arm_controller_node.gripper_service.call_async(request)
rclpy.spin_once(arm_controller_node, timeout_sec=0)
rclpy.shutdown()