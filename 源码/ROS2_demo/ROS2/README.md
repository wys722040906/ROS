# ROS2 新手教程

本项目旨在帮助具有ROS1经验的开发者快速过渡到ROS2，通过一系列教程演示ROS2的核心概念和功能。每个教程都包含可运行的示例代码，帮助你理解ROS2的通信机制和视觉处理功能。

## 目录

1. [ROS1 vs ROS2的主要区别](#ros1-vs-ros2的主要区别)
2. [工作空间结构](#工作空间结构)
3. [创建功能包](#创建功能包)
4. [编译功能包](#编译功能包)
5. [Topic通信](#topic通信)
6. [Service通信](#service通信)
7. [Action通信](#action通信)
8. [参数服务器](#参数服务器)
9. [视觉处理](#视觉处理)
10. [自定义接口](#自定义接口)
11. [启动文件](#启动文件)
12. [构建和运行](#构建和运行)

## ROS1 vs ROS2的主要区别

### 中间件变更
- ROS1使用自定义的TCPROS/UDPROS通信协议
- ROS2基于DDS（Data Distribution Service）实现通信，提供更好的QoS（服务质量）控制

### 节点模型
- ROS1使用单线程模型，依赖于ROS Master进行节点发现和连接
- ROS2使用多线程模型，支持实时系统，不需要中央ROS Master

### 构建系统
- ROS1使用catkin构建系统
- ROS2使用ament构建系统和colcon构建工具

### 接口定义
- ROS1使用.msg/.srv/.action文件定义消息
- ROS2使用IDL（Interface Definition Language）定义接口，但仍兼容.msg/.srv/.action文件格式

### 参数管理
- ROS1使用Parameter Server集中管理参数
- ROS2使用分布式参数系统，每个节点管理自己的参数

### 语言支持
- ROS2对Python 3和C++14/17提供了更好的支持
- ROS2提供了更现代化的API设计

## 工作空间结构

本教程工作空间包含以下包：

```
ros2_workspace/
├── src/
│   ├── ros2_topic_tutorial/        # Topic通信示例
│   ├── ros2_service_tutorial/      # Service通信示例
│   ├── ros2_action_tutorial/       # Action通信示例
│   ├── ros2_param_tutorial/        # 参数服务器示例
│   ├── ros2_vision_tutorial/       # 视觉处理示例
│   ├── ros2_interfaces_tutorial/   # 自定义接口示例
│   ├── ros2_client_tutorial/       # 接口客户端示例
│   └── ros2_launch_tutorial/       # 启动文件示例
```

## 创建功能包

### ROS2功能包类型

ROS2支持多种类型的功能包，主要有：

1. **C++包（ament_cmake）**：使用CMake构建的C++包
2. **Python包（ament_python）**：纯Python实现的包
3. **混合包（ament_cmake_python）**：包含C++和Python代码的包

### 创建C++功能包

```bash
# 语法：ros2 pkg create <package_name> --build-type ament_cmake --dependencies <dependencies>
# 示例：创建一个依赖rclcpp和std_msgs的C++包
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp std_msgs
```

C++包的基本结构：

```
my_cpp_pkg/
├── CMakeLists.txt           # CMake构建文件
├── include/                 # 头文件目录
│   └── my_cpp_pkg/          # 包含命名空间的头文件
├── package.xml              # 包元数据和依赖
├── src/                     # 源代码目录
│   └── my_node.cpp          # 示例节点源文件
└── test/                    # 测试代码目录
```

### 创建Python功能包

```bash
# 语法：ros2 pkg create <package_name> --build-type ament_python --dependencies <dependencies>
# 示例：创建一个依赖rclpy和std_msgs的Python包
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy std_msgs
```

Python包的基本结构：

```
my_py_pkg/
├── my_py_pkg/               # Python模块目录
│   ├── __init__.py          # 空文件，标记为Python包
│   └── my_node.py           # 示例节点源文件
├── package.xml              # 包元数据和依赖
├── resource/                # 资源文件
│   └── my_py_pkg            # 资源标识
├── setup.cfg                # 安装配置
├── setup.py                 # Python安装脚本
└── test/                    # 测试代码目录
```

### 创建接口包（自定义消息、服务和动作）

```bash
# 创建用于接口定义的包
ros2 pkg create my_interfaces --build-type ament_cmake --dependencies builtin_interfaces
```

接口文件目录结构：

```
my_interfaces/
├── CMakeLists.txt
├── msg/                     # 消息定义目录
│   └── MyMessage.msg        # 自定义消息定义
├── srv/                     # 服务定义目录
│   └── MyService.srv        # 自定义服务定义
├── action/                  # 动作定义目录
│   └── MyAction.action      # 自定义动作定义
└── package.xml              # 需要添加相关依赖
```

在`package.xml`中需要添加：

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

在`CMakeLists.txt`中需要添加：

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyMessage.msg"
  "srv/MyService.srv"
  "action/MyAction.action"
  DEPENDENCIES builtin_interfaces
)
```

## 编译功能包

### 使用colcon构建工作空间

ROS2使用colcon作为构建工具，它提供了许多灵活的选项：

```bash
# 构建整个工作空间
colcon build

# 构建特定的功能包
colcon build --packages-select my_package_name

# 构建多个指定的包
colcon build --packages-select pkg1 pkg2 pkg3

# 构建一个包及其依赖
colcon build --packages-up-to my_package_name

# 使用符号链接安装Python文件（便于开发调试）
colcon build --symlink-install

# 并行构建（加速编译过程）
colcon build --parallel-workers 8

# 构建并输出详细信息
colcon build --event-handlers console_direct+

# 仅处理修改过的功能包
colcon build --continue-on-error --packages-select-by-dep my_package_name
```

### 构建过程详解

1. **配置阶段**：colcon解析package.xml，确定包之间的依赖关系
2. **构建阶段**：
   - C++包：调用CMake进行配置，然后调用make进行编译
   - Python包：运行setup.py进行安装
3. **安装阶段**：将编译的产物安装到工作空间的install目录下

构建后的工作空间结构：

```
ros2_workspace/
├── build/         # 构建过程中生成的中间文件
├── install/       # 安装后的文件，包括可执行文件、库和配置
├── log/           # 构建过程的日志
└── src/           # 源代码
```

### 设置环境变量

构建完成后，需要设置环境变量以便能够找到新构建的功能包：

```bash
# 设置当前工作空间的环境
source install/setup.bash  # Bash
source install/setup.zsh   # Zsh
source install/setup.sh    # POSIX shell

# 为了方便，可以将上述命令添加到~/.bashrc中
echo "source ~/ros2_workspace/install/setup.bash" >> ~/.bashrc
```

### 常见构建问题解决

1. **依赖缺失**：使用rosdep解决依赖问题
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. **构建失败**：检查构建日志
   ```bash
   cat log/latest_build/my_package_name/stderr.log
   ```

3. **清理工作空间**：有时需要完全重新构建
   ```bash
   rm -rf build/ install/ log/
   ```

4. **特定编译器问题**：可以指定C++标准
   ```bash
   colcon build --cmake-args -DCMAKE_CXX_STANDARD=17
   ```

## Topic通信

Topic是ROS2中最基本的通信机制，采用发布/订阅模式：

- 发布者（Publisher）发送数据到指定的话题
- 订阅者（Subscriber）从话题接收数据
- 多个发布者和订阅者可以使用同一个话题
- 通信是异步的，发布者不知道有哪些订阅者

示例：`ros2_topic_tutorial`

```bash
# 运行发布者
ros2 run ros2_topic_tutorial publisher_example

# 运行订阅者
ros2 run ros2_topic_tutorial subscriber_example

# 查看话题列表
ros2 topic list

# 查看话题信息
ros2 topic info /topic

# 查看话题消息
ros2 topic echo /topic
```

## Service通信

Service是ROS2中的请求/响应通信机制：

- 服务器（Server）提供服务
- 客户端（Client）发送请求并等待响应
- 通信是同步的，客户端等待服务器的响应
- 一个服务名称通常只有一个服务器

示例：`ros2_service_tutorial`

```bash
# 运行服务器
ros2 run ros2_service_tutorial service_server

# 运行客户端
ros2 run ros2_service_tutorial service_client

# 查看服务列表
ros2 service list

# 调用服务
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"
```

## Action通信

Action是ROS2中适用于长时间运行任务的通信机制：

- 提供比Service更复杂的交互
- 支持取消操作
- 提供反馈和结果
- 适合机器人导航、抓取等长时间运行的任务

示例：`ros2_action_tutorial`

```bash
# 运行Action服务器
ros2 run ros2_action_tutorial action_server

# 运行Action客户端
ros2 run ros2_action_tutorial action_client

# 查看Action列表
ros2 action list

# 查看Action信息
ros2 action info /navigate_to_goal
```

## 参数服务器

参数服务器用于存储和管理节点的配置参数：

- 在ROS2中，每个节点管理自己的参数
- 参数可以通过命令行、启动文件或编程方式设置
- 参数支持多种数据类型

示例：`ros2_param_tutorial`

```bash
# 运行参数节点
ros2 run ros2_param_tutorial parameter_node

# 另一个终端运行参数客户端
ros2 run ros2_param_tutorial parameter_client

# 查看参数列表
ros2 param list

# 获取参数值
ros2 param get /parameter_demo_node string_param

# 设置参数值
ros2 param set /parameter_demo_node double_param 5.0

# 保存参数到文件
ros2 param dump /parameter_demo_node > params.yaml

# 从文件加载参数
ros2 param load /parameter_demo_node params.yaml
```

## 视觉处理

视觉处理示例展示了如何在ROS2中处理相机图像：

- 单目相机节点：读取相机图像并进行基本处理
- 双目相机节点：处理立体图像，计算深度和点云

示例：`ros2_vision_tutorial`

```bash
# 运行单目相机节点（需要连接相机或提供视频文件）
ros2 run ros2_vision_tutorial mono_camera_node

# 运行双目相机节点（需要连接双目相机或提供左右图像文件）
ros2 run ros2_vision_tutorial stereo_camera_node

# 参数设置示例
ros2 run ros2_vision_tutorial mono_camera_node --ros-args -p camera_device_id:=0 -p show_image:=true

# 使用视频文件
ros2 run ros2_vision_tutorial mono_camera_node --ros-args -p use_device:=false -p video_file:=/path/to/video.mp4
```

## 自定义接口

自定义接口允许你定义适合特定应用的消息、服务和动作：

### 接口示例

本教程包含以下自定义接口示例：

- **RobotStatus.msg**: 一个消息接口，包含机器人的状态信息
- **MoveRobot.srv**: 一个服务接口，用于控制机器人移动
- **NavigateToGoal.action**: 一个动作接口，用于实现复杂的导航任务

### 客户端示例

`ros2_client_tutorial`包演示了如何使用这些自定义接口：

```bash
# 运行机器人状态发布者
ros2 run ros2_client_tutorial robot_status_publisher

# 运行机器人移动服务器
ros2 run ros2_client_tutorial robot_move_server

# 运行机器人导航动作服务器
ros2 run ros2_client_tutorial robot_navigator_server

# 查看状态消息
ros2 topic echo /robot_status

# 调用移动服务
ros2 service call /move_robot ros2_interfaces_tutorial/srv/MoveRobot "{robot_id: 1, target_x: 2.0, target_y: 3.0, target_z: 0.0, speed_factor: 0.8, avoid_obstacles: true}"

# 发送导航动作目标
ros2 action send_goal /navigate_to_goal ros2_interfaces_tutorial/action/NavigateToGoal "{robot_id: 1, goal_x: 5.0, goal_y: 3.0, goal_z: 0.0, max_speed: 1.0, max_rotational_speed: 0.5, use_path_planning: true}" --feedback
```

## 启动文件

ROS2启动文件使用Python语言编写，提供了强大的配置选项和灵活性：

### 启动文件示例

本教程包含以下启动文件示例：

- **接口演示启动文件**: 同时启动所有接口通信节点
- **视觉演示启动文件**: 同时启动视觉处理节点
- **全部演示启动文件**: 启动所有示例节点

```bash
# 启动接口演示
ros2 launch ros2_launch_tutorial interfaces_demo.launch.py

# 启动视觉演示（单目相机）
ros2 launch ros2_launch_tutorial vision_demo.launch.py

# 只启动双目相机
ros2 launch ros2_launch_tutorial vision_demo.launch.py use_mono:=false use_stereo:=true

# 启动所有演示
ros2 launch ros2_launch_tutorial all_demos.launch.py

# 只启动接口演示，不启动视觉演示
ros2 launch ros2_launch_tutorial all_demos.launch.py include_vision:=false

# 自定义启动参数
ros2 launch ros2_launch_tutorial all_demos.launch.py robot_id:=2 robot_name:=my_robot show_image:=false
```

### 参数化启动

启动文件支持通过命令行传递参数，让你可以灵活配置节点行为：

```bash
# 自定义机器人ID和发布频率
ros2 launch ros2_launch_tutorial interfaces_demo.launch.py robot_id:=5 publish_freq:=2.0
```

## 构建和运行

### 环境设置

确保已安装ROS2（推荐使用Humble或更新版本）：

```bash
# 设置ROS2环境
source /opt/ros/humble/setup.bash
```

### 构建工作空间

```bash
# 克隆仓库（如适用）
git clone https://github.com/your_username/ros2_tutorial.git
cd ros2_tutorial

# 解决依赖
rosdep install --from-paths src --ignore-src -r -y

# 构建工作空间
colcon build --symlink-install

# 设置环境
source install/setup.bash
```

### 运行教程

参见各个教程部分的具体命令。

### 常用工具命令

```bash
# 列出所有节点
ros2 node list

# 获取节点信息
ros2 node info /node_name

# 列出所有话题
ros2 topic list

# 列出所有服务
ros2 service list

# 列出所有Action
ros2 action list

# 列出所有参数
ros2 param list
```

## 参考资料

- [ROS2官方文档](https://docs.ros.org/en/humble/index.html)
- [ROS2 Wiki](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS2设计文档](https://design.ros2.org/)

## 贡献者

- 作者：[Your Name]

## 许可证

本项目基于[LICENSE]许可证开源。 