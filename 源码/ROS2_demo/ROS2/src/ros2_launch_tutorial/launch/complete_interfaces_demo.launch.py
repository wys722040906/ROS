#!/usr/bin/env python3

"""
启动文件，用于同时启动接口演示的所有服务器和客户端节点
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
    robot_id = LaunchConfiguration('robot_id')
    robot_name = LaunchConfiguration('robot_name')
    launch_move_client = LaunchConfiguration('launch_move_client')
    launch_navigator_client = LaunchConfiguration('launch_navigator_client')
    
    # 声明参数
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
    
    declare_launch_move_client_cmd = DeclareLaunchArgument(
        'launch_move_client',
        default_value='True',
        description='是否启动移动服务客户端'
    )
    
    declare_launch_navigator_client_cmd = DeclareLaunchArgument(
        'launch_navigator_client',
        default_value='True',
        description='是否启动导航动作客户端'
    )
    
    # 状态发布节点
    topic_node = Node(
        package='ros2_client_tutorial',
        executable='robot_status_publisher',
        name='robot_status_publisher',
        parameters=[{
            'robot_id': robot_id,
            'robot_name': robot_name,
            'publish_freq': 0.5  # 降低频率以减少输出
        }],
        output='screen'
    )
    
    # 移动服务服务器节点
    move_server_node = Node(
        package='ros2_client_tutorial',
        executable='robot_move_server',
        name='robot_move_server',
        output='screen'
    )
    
    # 移动服务客户端节点
    move_client_node = Node(
        package='ros2_client_tutorial',
        executable='robot_move_client',
        name='robot_move_client',
        parameters=[{
            'robot_id': robot_id,
            'target_x': 3.5,
            'target_y': 2.5,
            'target_z': 0.0,
            'speed_factor': 0.7,
            'avoid_obstacles': True
        }],
        output='screen',
        condition=IfCondition(launch_move_client)
    )
    
    # 导航动作服务器节点
    navigator_server_node = Node(
        package='ros2_client_tutorial',
        executable='robot_navigator_server',
        name='robot_navigator_server',
        output='screen'
    )
    
    # 导航动作客户端节点
    navigator_client_node = Node(
        package='ros2_client_tutorial',
        executable='robot_navigator_client',
        name='robot_navigator_client',
        parameters=[{
            'robot_id': robot_id,
            'goal_x': 5.0,
            'goal_y': 3.0,
            'goal_z': 0.0,
            'max_speed': 1.0,
            'max_rotational_speed': 0.5,
            'use_path_planning': True
        }],
        output='screen',
        condition=IfCondition(launch_navigator_client)
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加参数
    ld.add_action(declare_robot_id_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_launch_move_client_cmd)
    ld.add_action(declare_launch_navigator_client_cmd)
    
    # 添加节点
    ld.add_action(topic_node)
    ld.add_action(move_server_node)
    ld.add_action(move_client_node)
    ld.add_action(navigator_server_node)
    ld.add_action(navigator_client_node)
    
    return ld 