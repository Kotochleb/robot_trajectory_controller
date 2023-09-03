#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rosbot_2r_demo = get_package_share_directory("rosbot_2r_demo")

    nav2_params = PathJoinSubstitution([rosbot_2r_demo, "config", "nav2_params.yaml"])
    map = PathJoinSubstitution([rosbot_2r_demo, "maps", "map.yaml"])

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("nav2_bringup"),
                    "launch",
                    "bringup_launch.py",
                ]
            )
        ),
        launch_arguments={
            "params_file": nav2_params,
            "map": map,
            "use_sim_time": "True",
            "slam": "False",
            "use_respawn": "True"
        }.items(),
    )

    return LaunchDescription(
        [
            # Sets use_sim_time for all nodes started below (doesn't work for nodes started from ignition gazebo)
            SetParameter(name="use_sim_time", value=True),
            nav2_bringup,
        ]
    )
