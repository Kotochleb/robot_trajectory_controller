#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    SetEnvironmentVariable
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rosbot_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("rosbot_gazebo"),
                    "launch",
                    "simulation.launch.py",
                ]
            )
        ),
    )

    rosbot_2r_demo = get_package_share_directory("rosbot_2r_demo")
    rviz2_config = os.path.join(os.path.realpath(rosbot_2r_demo), "rviz", "config.rviz")

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="both",
        arguments=['-d' + rviz2_config],
    )

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
            SetEnvironmentVariable(name="__NV_PRIME_RENDER_OFFLOAD", value="1"),
            SetEnvironmentVariable(name="__GLX_VENDOR_LIBRARY_NAME", value="nvidia"),
            rviz2,
            rosbot_gazebo,
            # nav2_bringup,
            # Sets use_sim_time for all nodes started below (doesn't work for nodes started from ignition gazebo)
            # SetParameter(name="use_sim_time", value=True),
        ]
    )
