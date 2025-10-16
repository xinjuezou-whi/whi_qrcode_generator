# Copyright 2025 WheelHub Intelligent
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
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Input parameters declaration
    type = LaunchConfiguration('type')
    contents = LaunchConfiguration('contents').perform(context)
    output_path = LaunchConfiguration('output_path')
    show = LaunchConfiguration('show')

    # Node definition
    start_whi_qrcode_generator_node = Node(
        package='whi_qrcode_generator',
        executable='whi_qrcode_generator_node',
        name='whi_qrcode_generator',
        output='screen',
        parameters=[
            {'type': type},
            {'contents': contents},
            {'output_path': output_path},
            {'show_generated': show},
            {'image_size': 500},
            # ArUco
            {'marker_size': 4},
            # QR
            {'code_size': 200},
            {'correction_level': 'middle'} # low, middle, quality, high
        ]
    )

    launch_nodes = [
        start_whi_qrcode_generator_node,
    ]

    return launch_nodes

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'type', default_value='qr',
            description='QR code type'
        ),
        DeclareLaunchArgument(
            'contents', default_value='hello world',
            description='Contents QR carried'
        ),
        DeclareLaunchArgument(
            'output_path', default_value=[EnvironmentVariable('HOME'), '/Desktop/'],
            description='The output path of generated QR code'
        ),
        DeclareLaunchArgument(
            'show', default_value='true',
            description='Whether to show the generated'
        ),
        OpaqueFunction(function=launch_setup)
    ])