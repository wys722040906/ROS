#!/usr/bin/env python3

"""
启动文件，用于同时启动视觉处理节点
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
    use_mono = LaunchConfiguration('use_mono')
    use_stereo = LaunchConfiguration('use_stereo')
    camera_device_id = LaunchConfiguration('camera_device_id')
    left_camera_id = LaunchConfiguration('left_camera_id')
    right_camera_id = LaunchConfiguration('right_camera_id')
    show_image = LaunchConfiguration('show_image')
    
    # 声明参数
    declare_use_mono_cmd = DeclareLaunchArgument(
        'use_mono',
        default_value='True',
        description='是否启动单目相机节点'
    )
    
    declare_use_stereo_cmd = DeclareLaunchArgument(
        'use_stereo',
        default_value='False',
        description='是否启动双目相机节点'
    )
    
    declare_camera_device_id_cmd = DeclareLaunchArgument(
        'camera_device_id',
        default_value='0',
        description='单目相机设备ID'
    )
    
    declare_left_camera_id_cmd = DeclareLaunchArgument(
        'left_camera_id',
        default_value='0',
        description='左相机设备ID'
    )
    
    declare_right_camera_id_cmd = DeclareLaunchArgument(
        'right_camera_id',
        default_value='1',
        description='右相机设备ID'
    )
    
    declare_show_image_cmd = DeclareLaunchArgument(
        'show_image',
        default_value='True',
        description='是否显示图像窗口'
    )
    
    # 单目相机节点
    mono_camera_node = Node(
        package='ros2_vision_tutorial',
        executable='mono_camera_node',
        name='mono_camera_node',
        parameters=[{
            'camera_device_id': camera_device_id,
            'use_device': True,
            'framerate': 15.0,
            'show_image': show_image
        }],
        output='screen',
        condition=IfCondition(use_mono)
    )
    
    # 双目相机节点
    stereo_camera_node = Node(
        package='ros2_vision_tutorial',
        executable='stereo_camera_node',
        name='stereo_camera_node',
        parameters=[{
            'left_camera_id': left_camera_id,
            'right_camera_id': right_camera_id,
            'use_device': True,
            'framerate': 15.0,
            'show_image': show_image
        }],
        output='screen',
        condition=IfCondition(use_stereo)
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加参数
    ld.add_action(declare_use_mono_cmd)
    ld.add_action(declare_use_stereo_cmd)
    ld.add_action(declare_camera_device_id_cmd)
    ld.add_action(declare_left_camera_id_cmd)
    ld.add_action(declare_right_camera_id_cmd)
    ld.add_action(declare_show_image_cmd)
    
    # 添加节点
    ld.add_action(mono_camera_node)
    ld.add_action(stereo_camera_node)
    
    return ld 