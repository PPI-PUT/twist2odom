# Copyright 2023 Amadeusz Szymko
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_prefix = FindPackageShare(LaunchConfiguration('param_file_pkg'))
    config = PathJoinSubstitution([pkg_prefix, LaunchConfiguration('param_file')])

    twist_to_odom_node = Node(
        name='twist_to_odom_node',
        namespace='',
        package='twist_to_odom',
        executable='twist_to_odom_node_exe',
        parameters=[
                config
        ],
        remappings=[
            ("twist_with_covariance", "/localization/twist_estimator/twist_with_covariance"),
            ("odometry", "/localization/odometry")
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs'],
        emulate_tty=True
    )

    return [
        twist_to_odom_node
    ]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'param_file_pkg',
            default_value='twist_to_odom',
            description="Package name which contains param file."
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'param_file',
            default_value='param/defaults.param.yaml',
            description="Param file (relative path)."
        )
    )

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
