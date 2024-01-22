#!/usr/bin/env python3

import rclpy
from sympy import *
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
from matplotlib import pyplot as plt
import time as tm

init_printing(use_unicode=False, wrap_line=False)
matplotlib.use('TkAgg')

class ArmControllerNode(Node):
    def init(self):
        super().init('arm_controller')
        self.arm_joint_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
    def publish_joint_angles(self, angles):
        msg = Float64MultiArray()
        msg.data = angles
        self.arm_joint_pub.publish(msg)
    def publish_wheel_velocities(self, velocities):
        msg = Float64MultiArray()
        msg.data = velocities
        self.wheel_velocities_pub.publish(msg)

alpha, a, d, theta = symbols('alpha a d theta')

#defining the symbols
theta1, theta2, theta3, theta4 = symbols('theta1 theta2 theta3 theta4')

q1_min, q1_max = 0, 6.3
q2_min, q2_max = -3.14, 0
q3_min, q3_max = 0, 5.54
q4_min, q4_max = -4.3, 0

#define the DH parameters 
alpha_array = [pi/2, 0, 0, 0]
theta_array = [theta1, theta2-(pi/2), theta3, theta4]
d_array = [38.1,-70.60, 0, 0]  # m
a_array = [0, -190.5, -190.5, -328.93]  # m



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

#substituting the dh values into the trasformation martix
T1 = T1_gen.subs(alpha, alpha_array[0]).subs(
    theta, theta_array[0]).subs(d, d_array[0]).subs(a, a_array[0])
T2 = T2_gen.subs(alpha, alpha_array[1]).subs(
    theta, theta_array[1]).subs(d, d_array[1]).subs(a, a_array[1])
T3 = T3_gen.subs(alpha, alpha_array[2]).subs(
    theta, theta_array[2]).subs(d, d_array[2]).subs(a, a_array[2])
T4 = T4_gen.subs(alpha, alpha_array[3]).subs(
    theta, theta_array[3]).subs(d, d_array[3]).subs(a, a_array[3])


H0_1 = T1
H0_2 = T1*T2
H0_3 = T1*T2*T3
H0_4 = T1*T2*T3*T4

H0_4_initial = H0_4.subs({theta1: 0.0, theta2: 0.0, theta3: 0.0, theta4: 0.0})
P = Matrix([[H0_4[3]], [H0_4[7]], [H0_4[11]]])


Par1 = diff(P, theta1)
Par2 = diff(P, theta2)
Par3 = diff(P, theta3)
Par4 = diff(P, theta4)

Z0_1 = Matrix([[H0_1[2]], [H0_1[6]], [H0_1[10]]])
Z0_2 = Matrix([[H0_2[2]], [H0_2[6]], [H0_2[10]]])
Z0_3 = Matrix([[H0_3[2]], [H0_3[6]], [H0_3[10]]])
Z0_4 = Matrix([[H0_4[2]], [H0_4[6]], [H0_4[10]]])



# Components
J1 = Matrix([[Par1], [Z0_1]])
J2 = Matrix([[Par2], [Z0_2]])
J3 = Matrix([[Par3], [Z0_3]])
J4 = Matrix([[Par4], [Z0_4]])


#Jacobian
J = Matrix([[J1, J2, J3, J4]])
pprint(J)


r = 0.02  # m

# initial joint angles.
q1 = 0.0
q2 = 0.0
q3 = 0.0
q4 = 0.0
# print("Initial Jacobian:\n")
J_initial = J.subs({theta1: q1, theta2: q2, theta3: q3, theta4: q4})

wheel_velocities = Float64MultiArray()
wheel_velocities.data = [0.0, 0.0, 0.0, 0.0]  # Initialize all wheel velocities to 0

wheel_velocities = [0.0, 0.0, 0.0, 0.0]

time = np.linspace(0, 200, num=2500)
dt = 200 / 2500

xA, yA, zA = 0.0, 0.0, 0.0  # Starting point (A)
xB, yB, zB = 0.0, 0.0, 1.0 # Ending point (B)

#total distance and velocity
total_distance = np.sqrt((xB - xA)*2 + (yB - yA)*2 + (zB - zA)*2)
velocity = total_distance / 20 

x_pos = []
y_pos = []
z_pos = []

for q1 in np.linspace(q1_min, q1_max, num=40):
        for q2 in np.linspace(q2_min, q2_max, num=50):
            for q3 in np.linspace(q3_min, q3_max, num=1):
                for q4 in np.linspace(q4_min, q4_max, num=1):
                    H0_4_current = H0_4.subs({theta1: q1, theta2: q2, theta3: q3, theta4: q4})
                    x_pos.append(float(H0_4_current[0, 3]))
                    y_pos.append(float(H0_4_current[1, 3]))
                    z_pos.append(float(H0_4_current[2, 3]))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x_pos, y_pos, z_pos, color='r')  
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')
plt.title('Versa-Bot Manipulator 3-D Workspace in mm')
plt.show()