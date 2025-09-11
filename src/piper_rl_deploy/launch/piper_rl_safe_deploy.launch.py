from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    package_dir = get_package_share_directory('piper_rl_deploy')
    
    # 声明启动参数
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(package_dir, 'models', 'policy.pt'),
        description='Path to the RL model file'
    )
    
    model_type_arg = DeclareLaunchArgument(
        'model_type',
        default_value='pytorch',
        description='Model type: pytorch or onnx'
    )
    
    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='200.0',
        description='Control loop frequency in Hz'
    )
    
    inference_frequency_arg = DeclareLaunchArgument(
        'inference_frequency',
        default_value='50.0',
        description='Model inference frequency in Hz'
    )
    
    # Piper RL Safe Controller 节点
    piper_rl_safe_node = Node(
        package='piper_rl_deploy',
        executable='piper_rl_safe_node',
        name='piper_rl_safe_controller',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'model_type': LaunchConfiguration('model_type'),
            'control_frequency': LaunchConfiguration('control_frequency'),
            'inference_frequency': LaunchConfiguration('inference_frequency'),
            'obs_dim': 21,  # 6(关节角度) + 6(关节速度) + 3(末端位置) + 3(手绢位置) + 3(手绢速度)
            'action_dim': 6,  # 6个关节
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
            'default_kp': [80.0, 80.0, 80.0, 80.0, 80.0, 80.0],
            'default_kd': [2.0, 2.0, 1.0, 1.0, 1.0, 2.0],
            'action_scale': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            'joint_pos_offset': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        }],
        remappings=[
            # 重映射话题以确保正确的数据流
            ('/joint_states_single', '/joint_states_single'),
            ('/handkerchief_piperroot', '/handkerchief_piperroot'),
            ('/handkerchief_velocity', '/handkerchief_velocity'),
            ('/end_effector_piperroot', '/end_effector_piperroot'),
            ('/joint_states', '/joint_states'),
            ('/rl_actions', '/rl_safe_actions'),
            ('/piper_safe_status', '/piper_safe_status')
        ]
    )
    
    return LaunchDescription([
        model_path_arg,
        model_type_arg,
        control_frequency_arg,
        inference_frequency_arg,
        piper_rl_safe_node
    ])
