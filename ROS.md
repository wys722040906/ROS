# ROS

- 赵须佐:http://www.autolabor.com.cn/book/ROSTutorials/

## 工作空间创建

```
catkin_init_workspace //ros工作空间调用catkin的构建脚本
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 //编译工作空间下功能包
echo “source ~/catkin_ws/devel/setup.bash” >> ~/.bashrc  //加环境变量
```

## 文件目录

```
my_package/
│
├── CMakeLists.txt        # CMake构建脚本
├── package.xml           # 包元数据描述文件
│
├── include/              # 头文件目录
│   └── my_package/       # 包含C++头文件的子目录
│       └── my_class.hpp  # 一个C++头文件
│
├── src/                  # 源代码目录
│   ├── my_node.cpp       # ROS节点程序源代码
│   └── ...
│
├── msg/                  # 消息定义目录
│   └── MyMessage.msg     # ROS消息定义文件
│
├── srv/                  # 服务定义目录
│   └── MyService.srv     # ROS服务定义文件
│
├── launch/               # 启动文件目录
│   └── my_launch.launch  # ROS启动文件
│
├── config/               # 配置文件目录
│   └── my_config.yaml    # ROS参数配置文件
│
├── scripts/              # 脚本目录
│   └── my_script.py      # Python脚本
│
└── worlds/               # 仿真环境目录
    └── my_world.world    # Gazebo仿真环境文件
```

## 常用命令

### rosnode

```
rosnode ping    测试到节点的连接状态
rosnode list    列出活动节点
rosnode info    打印节点信息
rosnode machine    列出指定设备上节点
rosnode kill    杀死某个节点
rosnode cleanup    清除不可连接的节点（ ctrl + c 关闭，该节点并没被彻底清除）
```

### rostopic

```
rostopic bw     显示主题使用的带宽
rostopic delay  显示带有 header 的主题延迟
rostopic echo   打印消息到屏幕
rostopic find   根据类型查找主题
rostopic hz     显示主题的发布频率
rostopic info   显示主题相关信息
rostopic list   显示所有活动状态下的主题   -v : 获取话题详情(比如列出：发布者和订阅者个数...)
rostopic pub    将数据发布到主题  /主题名称 消息类型 消息内容  /chatter std_msgs gagaxixi
(    
rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist  10hz
 "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 2.0)
rostopic type   打印主题类型
```

### rosmsgs

```
rosmsg show    显示消息描述
rosmsg info    显示消息信息
rosmsg list    列出所有消息
rosmsg md5    显示 md5 加密后的消息
rosmsg package    显示某个功能包下的所有消息
rosmsg packages    列出包含消息的功能包
```

### rosservice

```
rosservice args 打印服务参数  /spawn
rosservice call    使用提供的参数调用服务
rosservice find    按照服务类型查找服务
rosservice info    打印有关服务的信息
rosservice list    列出所有活动的服务
rosservice type    打印服务类型
rosservice uri    打印服务的 ROSRPC uri
```

### rossrv

```
rossrv show    显示服务消息详情
rossrv info    显示服务消息相关信息
rossrv list    列出所有服务信息
rossrv md5    显示 md5 加密后的服务消息
rossrv package    显示某个包下所有服务消息
rossrv packages    显示包含服务消息的所有包
```

### rosparam

```
rosparam set    设置参数
rosparam get    获取参数
rosparam load    从外部文件加载参数  rosparam load xxx.yaml
rosparam dump    将参数写出到外部文件   rosparam dump yyy.yaml
rosparam delete    删除参数 
rosparam list    列出所有参数
<rosparam command="load" file="$(find demo03_test_parameter)/cfg/color.yaml" />
```

### rqt

```
rqt_graph  
```









# ROS2

## 安装

- ###### 换源

```
sudo gedit /etc/apt/sources.list
```

```
deb http://mirrors.aliyun.com/ubuntu/ jammy main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ jammy-security main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ jammy-updates main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ jammy-proposed main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ jammy-backports main restricted universe multiverse——
deb-src http://mirrors.aliyun.com/ubuntu/ jammy main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ jammy-security main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ jammy-updates main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ jammy-proposed main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ jammy-backports main restricted universe multiverse
```

或：

software&&update

```
local
```

- ###### 设定环境

```
locale 
 
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
 
locale  
```

- ###### 软件源

```
sudo apt update && sudo apt install curl gnupg lsb-release
```

### ！！！DNS被污染

```
https://www.ipaddress.com/ 
输入域名 raw.githubusercontent.com
查询 ip 地址
185.199.108.133|185.199.109.133|185.199.110.133|185.199.111.133
----
(2)修改/etc/hosts文件：sudo gedit /etc/hosts
(3)添加域名与IP映射：185.199.109.133 raw.githubusercontent.com
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

- ###### 添加源

```
sudo apt install software-properties-common
sudo add-apt-repository universe
 
sudo apt update && sudo apt install curl -y sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
 
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

- ###### 安装

```
sudo apt update
 
sudo apt upgrade
 
sudo apt install ros-humble-desktop
```

- ###### 配置环境

```
source /opt/ros/humble/setup.bash
echo " source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

- ###### 试试乌龟

```
ros2 run turtlesim turtlesim_node
```

- ###### 动起来

```
source /opt/ros/humble/setup.sh
ros2 run turtlesim turtle_teleop_key
```

- test

```
ros2
```

## 工作空间创建

```
sudo apt-get install python3-colcon-common-extensions //安装colcon扩展
mkdir -p ~/ros2_ws/src
cd src
ros2 pkg create --build-type ament_cmake my_package //分功能包创建
cd ~/ros2_ws && colcon build | colcon build --packages-select xx   选择编译
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
ros2 pkg list //验证
```

## 运行

```
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
```



## 常用命令

### ros2 pkg

```
ros2 pkg create       Create a new ROS2 package
ros2 pkg find [package_name]  
ros2 pkg list
ros2 pkg depends [package_name]
ros2 pkg dependents [package_name]
ros2 pkg prefix       Output the prefix path of a package输出前缀
ros2 pkg xml          Output the XML of the package manifest or a specific tag
sudo apt install ros-<distro>-<package_name> 
```

### ros2 service

```
ros2 service list
```

### ros2  topic

```
ros2 topic list
rso2 topic info [topic_name]
ros2 topic type [topic_name] 
v4l2-ctl -d /dev/video0 --all
```

### ros2 node

```
ros2 node list
rso2 node info [node name]
ros2 run <package_name> <executable_name> 
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle 重映射节点名称
ros2 run example_parameters_rclcpp parameters_basic --ros-args -p rcl_log_level:=10 运行节点设置参数

```

## 相机标定

- [**安装网址**](https://www.ncnynl.com/archives/202110/4707.html)

- --size 内角点数
- --approximate 0.1 单目标定

```
v4l2-ctl --list-devices | v4l2-ctl -d /dev/video0 --all
&& ros2 topic hz /image_raw

ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video3" -p image_size:=[1280,720]  

ros2 run camera_calibration cameracalibrator --size 7x9  --square 0.025 --approximate 0.1 --ros-args --remap /image:=/image_raw --ros-args --remap camera:=/custom_camera
```

