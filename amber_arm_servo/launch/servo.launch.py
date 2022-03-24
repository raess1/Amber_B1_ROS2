import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Get URDF and SRDF
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("amber_b1_description"),
            "urdf",
            "amber_b1.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        "amber_arm_moveit_config", "config/amber_arm.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # Get parameters for the Pose Tracking node
    pose_tracking_yaml = load_yaml("moveit_servo", "config/pose_tracking_settings.yaml")
    pose_tracking_params = {"moveit_servo": pose_tracking_yaml}

    # Get parameters for the Servo node
    servo_yaml = load_yaml("amber_arm_servo", "config/servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    kinematics_yaml = load_yaml("amber_arm_moveit_config", "config/kinematics.yaml")
    joint_limits_yaml = {
        "robot_description_planning": load_yaml(
            "amber_arm_moveit_config", "config/joint_limits.yaml"
        )
    }

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    joy_node = Node(
        package="joy", executable="joy_node", name="joy_node", output="screen"
    )

    teleop_node = Node(
        package="amber_arm_servo",
        executable="teleop",
        name="teleop",
        output="screen",
    )

    return LaunchDescription([servo_node, joy_node, teleop_node])
