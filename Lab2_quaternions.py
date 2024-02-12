import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
import numpy as np


class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

    def quaternion_multiply(self, q0, q1):
        # This function multiplies two quaternions q0 and q1
        # The multiplication is done in the order q1*q0

        # Split the quaternions into their scalar and vector parts
        w0, x0, y0, z0 = q0[0], q0[1], q0[2], q0[3]
        w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]

        # Calculate the scalar part of the resulting quaternion
        q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
        # Calculate the vector part of the resulting quaternion
        q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
        q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
        q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

        # Combine the scalar and vector parts into the final quaternion
        final_quaternion = np.array([q0q1_w, q0q1_x, q0q1_y, q0q1_z])
        return final_quaternion

    def quaternion_inverse(self, q0):
        # This function calculates the inverse of a quaternion q0

        # Calculate the length of the quaternion
        q_len = q0[0]**2 + q0[1]**2 + q0[2]**2 + q0[3]**2

        # Calculate the inverse of the quaternion
        q_inv = [0, 0, 0, 0]
        q_inv[0] = q0[0]/q_len
        q_inv[1] = -q0[1]/q_len
        q_inv[2] = -q0[2]/q_len
        q_inv[3] = -q0[3]/q_len

        return q_inv

    def calculate_relative_orientation(self, prev_pose, current_pose):
        # This function calculates the relative orientation between two poses
        # The poses are represented as quaternions

        # Calculate the inverse of the previous pose
        q1_inv = self.quaternion_inverse(
            [prev_pose.x, prev_pose.y, prev_pose.z, prev_pose.w])
        # Get the current pose
        q2 = [current_pose.x, current_pose.y, current_pose.z, current_pose.w]

        # Multiply the current pose with the inverse of the previous pose
        qr = self.quaternion_multiply(q2, q1_inv)
        return qr


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    # Define a list of quaternions for testing
    quaternions = [
        (Quaternion(x=1.0, y=2.0, z=3.0, w=4.0),
         Quaternion(x=5.0, y=6.0, z=7.0, w=8.0)),
        (Quaternion(x=3.0, y=4.0, z=-5.0, w=6.0),
         Quaternion(x=7.0, y=-8.0, z=9.0, w=-10.0)),
        (Quaternion(x=4.0, y=-5.0, z=6.0, w=-7.0),
         Quaternion(x=8.0, y=9.0, z=-10.0, w=11.0)),
        (Quaternion(x=7.0, y=8.0, z=-9.0, w=10.0),
         Quaternion(x=11.0, y=-12.0, z=13.0, w=-14.0)),
        (Quaternion(x=8.0, y=-9.0, z=10.0, w=-11.0),
         Quaternion(x=12.0, y=13.0, z=-14.0, w=15.0)),
        (Quaternion(x=10.0, y=-11.0, z=12.0, w=-13.0),
         Quaternion(x=14.0, y=15.0, z=-16.0, w=17.0))
    ]

    # For each pair of quaternions, calculate the relative rotation
    for q1, q2 in quaternions:
        relative_rotation = node.calculate_relative_orientation(q1, q2)
        print(
            f'Relative rotation for {q1.x}+{q1.y}i+{q1.z}j+{q1.w}k to {q2.x}+{q2.y}i+{q2.z}j+{q2.w}k is: {relative_rotation[0]:.3f} + {relative_rotation[1]:.3f}i + {relative_rotation[2]:.3f}j + {relative_rotation[3]:.3f}k')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
