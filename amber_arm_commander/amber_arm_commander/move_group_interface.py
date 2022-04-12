# Copyright 2011-2014, Michael Ferguson
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import time
import copy
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import *
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive


## @brief Pure python interface to move_group action
class MoveGroupInterface(Node):

    ## @brief Constructor for this utility
    ## @param group Name of the MoveIt! group to command
    ## @param frame Name of the fixed frame in which planning happens
    ## @param move_group Name of the action server
    ## @param listener A TF listener instance (optional, will create a new one if None)
    ## @param plan_only Should we only plan, but not execute?
    def __init__(self, group, base_frame, planning_frame, plan_only=False, move_group="move_action"):
        super().__init__('moveit2_commander')
        self._group = group
        self._fixed_frame = base_frame
        self._planning_frame = planning_frame
    
        self._action_client = ActionClient(self, MoveGroup, move_group)
        self._action_client.wait_for_server()
  
        self.plan_only = plan_only
        self.planner_id = None
        self.planning_time = 15.0

    def get_move_action(self):
        return self._action_client

    ## @brief Move the arm to set of joint position goals
    def move_to_joint_position(self,
                            joints,
                            positions,
                            tolerance=0.01,
                            wait=True,
                            **kwargs):
        # Check arguments
        supported_args = ("max_velocity_scaling_factor",
                          "planner_id",
                          "planning_scene_diff",
                          "planning_time",
                          "plan_only",
                          "start_state")
        for arg in kwargs.keys():
            if not arg in supported_args:
                rospy.loginfo("move_to_joint_position: unsupported argument: %s",
                              arg)

        # Create goal
        g = MoveGroup.Goal()

        # 1. fill in workspace_parameters

        # 2. fill in start_state
        try:
            g.request.start_state = kwargs["start_state"]
        except KeyError:
            g.request.start_state.is_diff = True

        # 3. fill in goal_constraints
        c1 = Constraints()
        for i in range(len(joints)):
            c1.joint_constraints.append(JointConstraint())
            c1.joint_constraints[i].joint_name = joints[i]
            c1.joint_constraints[i].position = positions[i]
            c1.joint_constraints[i].tolerance_above = tolerance
            c1.joint_constraints[i].tolerance_below = tolerance
            c1.joint_constraints[i].weight = 1.0
        g.request.goal_constraints.append(c1)

        # 4. fill in path constraints

        # 5. fill in trajectory constraints

        # 6. fill in planner id
        try:
            g.request.planner_id = kwargs["planner_id"]
        except KeyError:
            if self.planner_id:
                g.request.planner_id = self.planner_id

        # 7. fill in group name
        g.request.group_name = self._group

        # 8. fill in number of planning attempts
        try:
            g.request.num_planning_attempts = kwargs["num_attempts"]
        except KeyError:
            g.request.num_planning_attempts = 1

        # 9. fill in allowed planning time
        try:
            g.request.allowed_planning_time = kwargs["planning_time"]
        except KeyError:
            g.request.allowed_planning_time = self.planning_time

        # Fill in velocity scaling factor
        try:
            g.request.max_velocity_scaling_factor = kwargs["max_velocity_scaling_factor"]
        except KeyError:
            pass  # do not fill in at all

        # 10. fill in planning options diff
        try:
            g.planning_options.planning_scene_diff = kwargs["planning_scene_diff"]
        except KeyError:
            g.planning_options.planning_scene_diff.is_diff = True
            g.planning_options.planning_scene_diff.robot_state.is_diff = True

        # 11. fill in planning options plan only
        try:
            g.planning_options.plan_only = kwargs["plan_only"]
        except KeyError:
            g.planning_options.plan_only = self.plan_only

        # 12. fill in other planning options
        g.planning_options.look_around = False
        g.planning_options.replan = False

        # 13. send goal
        if wait:
            ret = self._action_client.send_goal(g)
            time.sleep(1)
            return ret
        else:
            ret = self._action_client.send_goal_async(g)
            time.sleep(1)
            return ret


    ## @brief Move the arm, based on a goal pose_stamped for the end effector.
    def move_to_pose(self,
                   pose_stamped,
                   gripper_frame=None,
                   tolerance=0.01,
                   wait=True,
                   **kwargs):
        if gripper_frame is None:
            gripper_frame = self._planning_frame

        pose_stamped = self._complete_msg(pose_stamped)

        # Check arguments
        supported_args = ("max_velocity_scaling_factor",
                          "planner_id",
                          "planning_time",
                          "plan_only",
                          "start_state")
        for arg in kwargs.keys():
            if not arg in supported_args:
                rospy.loginfo("move_to_pose: unsupported argument: %s",
                              arg)

        # Create goal
        g = MoveGroup.Goal()
        # pose_transformed = self.transform_pose(pose_stamped, self._fixed_frame)

        # 1. fill in request workspace_parameters

        # 2. fill in request start_state
        try:
            g.request.start_state = kwargs["start_state"]
        except KeyError:
            g.request.start_state.is_diff = True

        # 3. fill in request goal_constraints
        c1 = Constraints()

        c1.position_constraints.append(PositionConstraint())
        c1.position_constraints[0].header.frame_id = self._fixed_frame
        c1.position_constraints[0].link_name = gripper_frame
        b = BoundingVolume()
        s = SolidPrimitive()
        s.dimensions = [tolerance * tolerance]
        s.type = s.SPHERE
        b.primitives.append(s)
        b.primitive_poses.append(pose_stamped.pose)
        c1.position_constraints[0].constraint_region = b
        c1.position_constraints[0].weight = 1.0

        c1.orientation_constraints.append(OrientationConstraint())
        c1.orientation_constraints[0].header.frame_id = self._fixed_frame
        c1.orientation_constraints[0].orientation = pose_stamped.pose.orientation
        c1.orientation_constraints[0].link_name = gripper_frame
        c1.orientation_constraints[0].absolute_x_axis_tolerance = tolerance
        c1.orientation_constraints[0].absolute_y_axis_tolerance = tolerance
        c1.orientation_constraints[0].absolute_z_axis_tolerance = tolerance
        c1.orientation_constraints[0].weight = 1.0

        g.request.goal_constraints.append(c1)

        # 4. fill in request path constraints

        # 5. fill in request trajectory constraints

        # 6. fill in request planner id
        try:
            g.request.planner_id = kwargs["planner_id"]
        except KeyError:
            if self.planner_id:
                g.request.planner_id = self.planner_id

        # 7. fill in request group name
        g.request.group_name = self._group

        # 8. fill in request number of planning attempts
        try:
            g.request.num_planning_attempts = kwargs["num_attempts"]
        except KeyError:
            g.request.num_planning_attempts = 1

        # 9. fill in request allowed planning time
        try:
            g.request.allowed_planning_time = kwargs["planning_time"]
        except KeyError:
            g.request.allowed_planning_time = self.planning_time

        # Fill in velocity scaling factor
        try:
            g.request.max_velocity_scaling_factor = kwargs["max_velocity_scaling_factor"]
        except KeyError:
            pass  # do not fill in at all

        # 10. fill in planning options diff
        g.planning_options.planning_scene_diff.is_diff = True
        g.planning_options.planning_scene_diff.robot_state.is_diff = True

        # 11. fill in planning options plan only
        try:
            g.planning_options.plan_only = kwargs["plan_only"]
        except KeyError:
            g.planning_options.plan_only = self.plan_only

        # 12. fill in other planning options
        g.planning_options.look_around = False
        g.planning_options.replan = False

        # 13. send goal
        if wait:
            ret = self._action_client.send_goal(g)
            time.sleep(1)
            return ret
        else:
            ret = self._action_client.send_goal_async(g)
            time.sleep(1)
            return ret

    ## @brief Sets the planner_id used for all future planning requests.
    ## @param planner_id The string for the planner id, set to None to clear
    def set_planner_id(self, planner_id):
        self.planner_id = str(planner_id)

    ## @brief Set default planning time to be used for future planning request.
    def set_planning_time(self, time):
        self.planning_time = time

    def _complete_msg(self, pose_stamped):
        pose_stamped.header.frame_id = self._fixed_frame
        pose_stamped.header.stamp =  self.get_clock().now().to_msg()

        return pose_stamped
