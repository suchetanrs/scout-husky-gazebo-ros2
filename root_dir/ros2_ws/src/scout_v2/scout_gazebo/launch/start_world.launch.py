#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_box_car_gazebo = get_package_share_directory('scout_gazebo')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        # launch_arguments={'gui': 'false', 'gui_required':'false'}.items()
        launch_arguments={'verbose':'true'}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_box_car_gazebo, 'worlds', 'marsyard2020.world'), ''],
          description='SDF world file'),
        gazebo
        # marsyard2020
        # clearpath_playpen
    ])
