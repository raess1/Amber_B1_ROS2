#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import GripperCommand

class Gripper(Node):
    def __init__(self):
        super().__init__('robot_arm_gripper')
        self._action_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        self._action_client.wait_for_server()

    def open(self, wait=True):
        self._send_to_action_server(0.026, wait)

    def close(self, wait=True):
        self._send_to_action_server(0.00, wait)

    def set_pos(self, pos, wait=True):
        self._send_to_action_server(pos, wait)

    def _send_to_action_server(self, pos, wait):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = pos

        if wait:
            return self._action_client.send_goal(goal_msg)
        else:
            return self._action_client.send_goal_async(goal_msg)
