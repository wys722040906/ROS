#!/usr/bin/env python3

"""
启动文件，用于同时启动所有演示节点
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    """生成启动描述"""
    
    # 获取本启动文件所在的目录
    launch_dir = LaunchConfiguration('launch_dir', default=ThisLaunchFileDir())
    
    # 包含其他启动文件的路径
    interfaces_launch_file = os.path.join(
        launch_dir, 'interfaces_demo.launch.py')
        
    vision_launch_file = os.path.join(
        launch_dir, 'vision_demo.launch.py')
    
    # 获取参数值
    include_interfaces = LaunchConfiguration('include_interfaces')
    include_vision = LaunchConfiguration('include_vision')
    use_mono = LaunchConfiguration('use_mono')
    use_stereo = LaunchConfiguration('use_stereo')
    show_image = LaunchConfiguration('show_image')
    
    # 声明参数
    declare_include_interfaces_cmd = DeclareLaunchArgument(
        'include_interfaces',
        default_value='True',
        description='是否包含接口演示节点'
    )
    
    declare_include_vision_cmd = DeclareLaunchArgument(
        'include_vision',
        default_value='True',
        description='是否包含视觉处理节点'
    )
    
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
    
    declare_show_image_cmd = DeclareLaunchArgument(
        'show_image',
        default_value='True',
        description='是否显示图像窗口'
    )
    
    # 包含接口演示的启动文件
    interfaces_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(interfaces_launch_file),
        condition=IfCondition(include_interfaces)
    )
    
    # 包含视觉处理的启动文件
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vision_launch_file),
        launch_arguments={
            'use_mono': use_mono,
            'use_stereo': use_stereo,
            'show_image': show_image
        }.items(),
        condition=IfCondition(include_vision)
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加参数
    ld.add_action(declare_include_interfaces_cmd)
    ld.add_action(declare_include_vision_cmd)
    ld.add_action(declare_use_mono_cmd)
    ld.add_action(declare_use_stereo_cmd)
    ld.add_action(declare_show_image_cmd)
    
    # 添加启动文件
    ld.add_action(interfaces_launch)
    ld.add_action(vision_launch)
    
    return ld 