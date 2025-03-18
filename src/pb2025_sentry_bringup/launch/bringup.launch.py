# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("pb2025_sentry_bringup")

    serial_bringup_dir = get_package_share_directory("standard_robot_pp_ros2")
    vision_bringup_dir = get_package_share_directory("pb2025_vision_bringup")
    navigation_bringup_dir = get_package_share_directory("pb2025_nav_bringup")
    bt_bringup_dir = get_package_share_directory("pb2025_sentry_behavior")

    # Create the launch configuration variables
    ## Serial
    robot_name = LaunchConfiguration("robot_name")
    ## Vision
    detector = LaunchConfiguration("detector")
    use_hik_camera = LaunchConfiguration("use_hik_camera")
    ## Navigation
    slam = LaunchConfiguration("slam")
    world = LaunchConfiguration("world")
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    ## Common
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    record_rosbag = LaunchConfiguration("record_rosbag")
    use_rviz = LaunchConfiguration("use_rviz")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name",
        default_value="pb2025_sentry_robot",
        description="The file name of the robot xmacro to be used",
    )

    declare_detector_cmd = DeclareLaunchArgument(
        "detector",
        default_value="openvino",
        description="Type of detector to use (option: 'opencv', 'openvino')",
    )

    declare_use_hik_camera_cmd = DeclareLaunchArgument(
        "use_hik_camera",
        default_value="True",
        description="Whether to bringup hik camera node",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="Whether run a SLAM. If True, it will disable small_gicp and send static tf (map->odom)",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="",
        description="Select world. Map and PCD file share the same name as this parameter",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=[
            TextSubstitution(text=os.path.join(bringup_dir, "map", "")),
            world,
            TextSubstitution(text=".yaml"),
        ],
        description="Full path to map file to load",
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file",
        default_value=[
            TextSubstitution(text=os.path.join(bringup_dir, "pcd", "")),
            world,
            TextSubstitution(text=".pcd"),
        ],
        description="Full path to prior pcd file to load",
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "node_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "sentry_default_view.rviz"),
        description="Full path to the RViz config file to use",
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="False",
        description="Whether to start the robot state publisher",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="False", description="Whether to start RViz"
    )

    declare_record_rosbag_cmd = DeclareLaunchArgument(
        "record_rosbag", default_value="False", description="Whether to record rosbag"
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="True",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    # Specify the actions
    start_serial_driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                serial_bringup_dir, "launch", "standard_robot_pp_ros2.launch.py"
            )
        ),
        condition=IfCondition(PythonExpression(["not ", use_robot_state_pub])),
        launch_arguments={
            "robot_name": robot_name,
            "namespace": namespace,
            "params_file": params_file,
            "use_rviz": "False",
            "use_respawn": use_respawn,
            "log_level": log_level,
        }.items(),
    )

    start_vision_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vision_bringup_dir, "launch", "rm_vision_reality_launch.py")
        ),
        launch_arguments={
            "detector": detector,
            "use_hik_camera": use_hik_camera,
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "use_robot_state_pub": use_robot_state_pub,
            "use_composition": use_composition,
            "use_rviz": "False",
            "use_respawn": use_respawn,
            "log_level": log_level,
        }.items(),
    )

    start_navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_bringup_dir, "launch", "rm_navigation_reality_launch.py"
            )
        ),
        launch_arguments={
            "slam": slam,
            "map": map_yaml_file,
            "prior_pcd_file": prior_pcd_file,
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "use_robot_state_pub": use_robot_state_pub,
            "use_rviz": "False",
            "use_composition": use_composition,
            "use_respawn": use_respawn,
        }.items(),
    )

    start_behavior_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bt_bringup_dir, "launch", "pb2025_sentry_behavior_launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "log_level": log_level,
        }.items(),
    )

    start_rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "rviz_launch.py")
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    record_rosbag_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "record_rosbag_launch.py")
        ),
        condition=IfCondition(record_rosbag),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_detector_cmd)
    ld.add_action(declare_use_hik_camera_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_prior_pcd_file_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_record_rosbag_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_serial_driver_cmd)
    ld.add_action(start_vision_launch_cmd)
    # ld.add_action(start_navigation_launch_cmd)
    # ld.add_action(start_behavior_launch_cmd)
    ld.add_action(record_rosbag_cmd)

    return ld
