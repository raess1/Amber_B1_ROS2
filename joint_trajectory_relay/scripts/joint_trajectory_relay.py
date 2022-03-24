#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.qos import QoSProfile


class JointTrajectoryRelay(Node):
	def __init__(self):
		super().__init__('joint_trajectory_relay')
		self._trajectory_publisher = self.create_publisher(JointTrajectory, '/position_trajectory_controller/joint_trajectory', 10)

		state_subscription = self.create_subscription(
			JointTrajectoryControllerState,
			'/amber_position_trajectory_controller/state',
			self._state_callback,
			10
		)
		state_subscription

	def _state_callback(self, msg):
		trajectory_msg = JointTrajectory()
		trajectory_points = JointTrajectoryPoint()

		desired_positions = msg.actual.positions
		joint_names = msg.joint_names
		
		for pos, joint_name in zip(desired_positions, joint_names):
			trajectory_points.positions.append(pos)
			trajectory_msg.joint_names.append(joint_name)
		 
		trajectory_points.time_from_start.nanosec = 600000000
		trajectory_msg.points.append(trajectory_points)

		self._trajectory_publisher.publish(trajectory_msg)

def main(args=None):
	rclpy.init(args=args)
	relay = JointTrajectoryRelay()
	rclpy.spin(relay)
	relay.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
