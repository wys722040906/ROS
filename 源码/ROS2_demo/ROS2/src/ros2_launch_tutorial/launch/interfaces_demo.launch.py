#!/usr/bin/env python3

"""
启动文件，用于同时启动所有接口演示节点
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    """生成启动描述"""
    
    # 获取参数值
    use_topic = LaunchConfiguration('use_topic')
    use_service = LaunchConfiguration('use_service')
    use_action = LaunchConfiguration('use_action')
    robot_id = LaunchConfiguration('robot_id')
    robot_name = LaunchConfiguration('robot_name')
    
    # 声明参数
    declare_use_topic_cmd = DeclareLaunchArgument(
        'use_topic',
        default_value='True',
        description='是否启动话题示例节点'
    )
    
    declare_use_service_cmd = DeclareLaunchArgument(
        'use_service',
        default_value='True',
        description='是否启动服务示例节点'
    )
    
    declare_use_action_cmd = DeclareLaunchArgument(
        'use_action',
        default_value='True',
        description='是否启动动作示例节点'
    )
    
    declare_robot_id_cmd = DeclareLaunchArgument(
        'robot_id',
        default_value='1',
        description='机器人ID参数'
    )
    
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='tutorial_robot',
        description='机器人名称参数'
    )
    
    # 话题示例节点
    topic_node = Node(
        package='ros2_client_tutorial',
        executable='robot_status_publisher',
        name='robot_status_publisher',
        parameters=[{
            'robot_id': robot_id,
            'robot_name': robot_name,
            'publish_freq': 0.5  # 降低频率以减少输出
        }],
        output='screen',
        condition=IfCondition(use_topic)
    )
    
    # 服务示例节点
    service_node = Node(
        package='ros2_client_tutorial',
        executable='robot_move_server',
        name='robot_move_server',
        output='screen',
        condition=IfCondition(use_service)
    )
    
    # 动作示例节点
    action_node = Node(
        package='ros2_client_tutorial',
        executable='robot_navigator_server',
        name='robot_navigator_server',
        output='screen',
        condition=IfCondition(use_action)
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加参数
    ld.add_action(declare_use_topic_cmd)
    ld.add_action(declare_use_service_cmd)
    ld.add_action(declare_use_action_cmd)
    ld.add_action(declare_robot_id_cmd)
    ld.add_action(declare_robot_name_cmd)
    
    # 添加节点
    ld.add_action(topic_node)
    ld.add_action(service_node)
    ld.add_action(action_node)
    
    return ld 