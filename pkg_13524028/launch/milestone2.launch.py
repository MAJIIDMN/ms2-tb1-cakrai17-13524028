#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the path to the config file
    config_file = os.path.join(
        get_package_share_directory('pkg_13524028'),
        'config',
        'config.yaml'
    )
    
    return LaunchDescription([
        # Launch the twist_command_randomizer node
        Node(
            package='pkg_13524028',
            executable='twist_command_randomizer.py',
            name='twist_command_randomizer',
            parameters=[config_file],
            output='screen',
        ),
        # Launch the cmd_coordinator node
        Node(
            package='pkg_13524028',
            executable='cmd_coordinator.py',
            name='cmd_coordinator',
            parameters=[config_file],
            output='screen',
        ),
        
        # Launch the movement_reader node
        Node(
            package='pkg_13524028',
            executable='movement_reader.py',
            name='movement_reader',
            parameters=[config_file],
            output='screen',
        ),
    ])