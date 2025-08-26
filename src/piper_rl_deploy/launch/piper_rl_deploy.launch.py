#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 获取包路径
    pkg_piper_rl_deploy = get_package_share_directory('piper_rl_deploy')
    
    # 声明启动参数
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('piper_rl_deploy'),
            'models',
            'piper_policy.pt'
        ]),
        description='Path to the trained model file'
    )
    
    model_type_arg = DeclareLaunchArgument(
        'model_type',
        default_value='pytorch',
        description='Model type: pytorch or onnx'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('piper_rl_deploy'),
            'config',
            'piper_rl_config.yaml'
        ]),
        description='Path to the configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    
    # Piper RL 控制节点
    piper_rl_node = Node(
        package='piper_rl_deploy',
        executable='piper_rl_node',
        name='piper_rl_controller',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'model_path': LaunchConfiguration('model_path'),
                'model_type': LaunchConfiguration('model_type'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        remappings=[
            # 根据实际piper节点的话题名称进行映射
            # 注意：这些映射可能需要根据实际机器人系统调整
        ]
    )
    
    return LaunchDescription([
        model_path_arg,
        model_type_arg,
        config_file_arg,
        use_sim_time_arg,
        piper_rl_node,
    ])
