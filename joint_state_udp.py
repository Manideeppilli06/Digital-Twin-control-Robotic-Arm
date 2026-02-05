#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class JointStatePrinter(Node):

    def __init__(self):
        super().__init__('joint_state_printer')

        self.joint_order = [
            'base_joint',
            'shoulder_joint',
            'elbow_joint',
            'wrist_joint',
            'left_finger_joint'
        ]

        self.gripper_max_rad = 0.020

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info('JointState printer node started')


    def joint_state_callback(self, msg):
        joint_map = dict(zip(msg.name, msg.position))
        output_angles = []

        for joint in self.joint_order:

            if joint not in joint_map:
                self.get_logger().warn(f'{joint} not found in /joint_states')
                return

            ros_val = joint_map[joint]

            # =============== GRIPPER ==================
            if joint == 'left_finger_joint':
                normalized = ros_val / self.gripper_max_rad
                normalized = max(0.0, min(1.0, normalized))
                servo_deg = 90.0 * (1.0 - normalized)

            # =============== SHOULDER =================
            elif joint == 'shoulder_joint':
                ros_deg = math.degrees(ros_val)   # -60 → +40
                servo_deg = ros_deg + 90.0        # maps to 30 → 130
                servo_deg = max(30.0, min(130.0, servo_deg))

            # =============== ELBOW ====================
            elif joint == 'elbow_joint':
                ros_deg = math.degrees(ros_val)
                servo_deg = max(0.0, min(90.0, ros_deg))

            # =============== WRIST ====================
            elif joint == 'wrist_joint':
                ros_deg = math.degrees(ros_val)
                servo_deg = max(0.0, min(120.0, ros_deg))

            # =============== BASE =====================
            else:
                ros_deg = math.degrees(ros_val)
                servo_deg = ros_deg + 90.0
                servo_deg = max(0.0, min(180.0, servo_deg))

            output_angles.append((joint, ros_val, servo_deg))

        self.get_logger().info('--- Joint Angles ---')
        for joint, ros, servo in output_angles:
            self.get_logger().info(
                f'{joint:15s} | ros = {ros:+.3f} | servo = {servo:6.1f} deg'
            )


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
