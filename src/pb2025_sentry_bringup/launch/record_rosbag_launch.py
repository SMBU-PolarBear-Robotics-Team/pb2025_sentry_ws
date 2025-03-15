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


from datetime import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    record_rosbag_cmd = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            f"rosbags/sentry_{current_time}",
            "/serial/gimbal_joint_state",
            "/livox/imu",
            "/livox/lidar",
            "/front_industrial_camera/image",
            "/front_industrial_camera/camera_info",
            "--compression-mode",
            "file",
            "--compression-format",
            "zstd",
            "-d",
            "30",
        ],
        output="screen",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add other nodes and processes we need
    ld.add_action(record_rosbag_cmd)

    return ld
