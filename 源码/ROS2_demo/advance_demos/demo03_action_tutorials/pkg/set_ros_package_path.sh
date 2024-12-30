#!/bin/bash
# 设置多个 ROS_PACKAGE_PATH 环境变量
export ROS_PACKAGE_PATH=/home/wys/Documents/ROS2/demos/action_tutorials/pkg/install
export ROS_PACKAGE_PATH=/home/wys/Documents/ROS2/another_workspace/install:$ROS_PACKAGE_PATH

# 输出当前 ROS_PACKAGE_PATH 设置情况，帮助调试
echo "ROS_PACKAGE_PATH is set to: $ROS_PACKAGE_PATH"

# 启动 ROS 2 节点
ros2 run package1 node1 &

# 启动另一个程序（非 ROS 节点）
./other_program &

# 使用 wait 命令等待所有后台进程结束
wait
