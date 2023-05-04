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
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    twist2odom_node = Node(
        name='twist2odom',
        package='twist2odom',
        executable='twist2odom_node_exe',
        parameters=[{
            'publish_tf': LaunchConfiguration('publish_tf')
        }],
        remappings=[
            ('input/twist_with_covariance', LaunchConfiguration('in_twist')),
            ('output/odometry', LaunchConfiguration('out_odom'))
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs'],
        emulate_tty=True
    )

    return [twist2odom_node]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('publish_tf', 'false')
    add_launch_arg('in_twist', 'in_twist')
    add_launch_arg('out_odom', 'out_odom')

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
