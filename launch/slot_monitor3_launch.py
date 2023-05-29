from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


prenamespace = "monitor3"
index = 3

def generate_launch_description():
    inference = Node(
        package = 'cluster',
        namespace = prenamespace,
        executable = 'cluster_exec',
        name = 'cluster',
        output = 'screen',
        parameters = [

            {"node_index": index},
            {"number_of_nodes": 4},
            {"interval": 10},
            {"model_name": "/home/avees/engine/yolov7.engine"},
            {"use_can": True},
            {"can_id": index + 101},
            {"time_interval": 500},
        ]
    )
    
    return LaunchDescription([
        inference
    ])
