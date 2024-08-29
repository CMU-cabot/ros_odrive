#!/usr/bin/env python3
# ******************************************************************************
#  Copyright (c) 2024  Miraikan - The National Museum of Emerging Science and Innovation
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.
# ******************************************************************************

from launch_ros.actions import Node
from launch import LaunchDescription
from launch import LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('cabot_odrive_can')
    odrive_model = LaunchConfiguration('odrive_model')
    odrive_firmware_version = LaunchConfiguration('odrive_firmware_version')
    flat_endpoints_json_path = PathJoinSubstitution([
        pkg_dir,
        PythonExpression(['"json"']),
        odrive_firmware_version,
        PythonExpression(['"flat_endpoints_',odrive_model,'.json"'])])

    return LaunchDescription([
        DeclareLaunchArgument(
            'odrive_model',
            default_value=EnvironmentVariable('ODRIVE_MODEL'),
            description='odrive model'
            ),
        DeclareLaunchArgument(
            'odrive_firmware_version',
            default_value=EnvironmentVariable('ODRIVE_FIRMWARE_VERSION'),
            description='odrive firmware version'
            ),
        Node(
            package='cabot_odrive_can',
            executable='odrive_can_gain_setter',
            name='odrive_can_gain_setter_left',
            parameters=[{'node_id': 0,
                         'json_file_path': flat_endpoints_json_path}],
            output='both'
            ),
        Node(
            package='cabot_odrive_can',
            executable='odrive_can_gain_setter',
            name='odrive_can_gain_setter_right',
            parameters=[{'node_id': 1,
                         'json_file_path': flat_endpoints_json_path}],
            output='both'
            )
        ])

if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
