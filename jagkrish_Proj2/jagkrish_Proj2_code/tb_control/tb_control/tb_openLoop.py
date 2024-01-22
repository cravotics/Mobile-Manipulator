#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import select
import tty
import termios

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        # Publishers for wheel velocities and joint states
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.arm_joint_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_control(self):
        self.msg = """
        MOBILE BASE CONTROLS
        ---------------------------
        Moving around:
            w
        a    s    d

        k : EMERGENCY STOP

        MANIPULATOR CONTROLS:
        ---------------------------
            z-  INCREASE THE JOINT ANGLE +
            x - DECREASE THE JOINT ANGLE -
            n - TO CONTROL NEXT JOINT
            m - TO CONTROL OLD JOINT


        Esc to quit
        """

        self.get_logger().info(self.msg)

        wheel_velocities = Float64MultiArray()
        joint_commands = Float64MultiArray()
        joint_commands.data = [0.0] * 5  # Adjust the number based on your robot's arm joints
        current_joint = 0

        while True:
            key = self.getKey()
            if key:
                if key == '\x1b':  # Escape key
                    break
                elif key == 'k':  # Stop
                    wheel_velocities.data = [0.0, 0.0, 0.0, 0.0]
                elif key == 'w':  # Forward
                    wheel_velocities.data = [3.0, -3.0, 3.0, -3.0]
                elif key == 's':  # Reverse
                    wheel_velocities.data = [-3.0, 3.0, -3.0, 3.0]
                elif key == 'd':  # Right
                    wheel_velocities.data = [7.0, 6.0, 7.0, 6.0]
                elif key == 'a':  # Left
                    wheel_velocities.data = [-6.0, -7.0, -6.0, -7.0]
                elif key == 'z':  # Increase joint angle
                        joint_commands.data[current_joint] += 0.1
                elif key == 'x':  # Decrease joint angle
                        joint_commands.data[current_joint] -= 0.1
                elif key == 'm':  # Previous joint
                    current_joint = max(0, current_joint - 1)
                elif key == 'n':  # Next joint
                    current_joint = min(len(joint_commands.data) - 1, current_joint + 1)

                self.wheel_velocities_pub.publish(wheel_velocities)
                self.arm_joint_pub.publish(joint_commands)
                print(joint_commands)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
