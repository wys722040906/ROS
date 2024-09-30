# Robot

- ###### Gazebo仿真

```
git clone https://github.com/6-robot/wpr_simulation
cd ws
cd ./src/wpr_simulation/scripts/insatll_for_noetic.sh
catkin_make

roslaunch wpr_simulation wpb_stage_simple.launch#测试
roslaunch wpr_simulation wpb_rviz.launch #Rviz检测
rosrun wpr_simulation demo_vel_ctr #速度控制
rostopic echo /scan --noarr#激光雷达话题 信息
rosrun wpr_simulation demo_lidar_dat#接受节点
```

- ###### Rviz实时检测

```
roslaunch rviz rviz
---robot.mode失败---
Robot Description参数必须是嵌入的xml，而不是光盘上文件的路径。您可以使用启动文件加载XML： 

rosrun rviz rviz -d //=home/wsy/robot_ws/rviz/slam.rviz

#state error
:map->baselink

```

- ###### IMU惯性测量单元---协方差矩阵-欧拉角-四元数-万向锁

  ###### topic

  - imu/data_raw(sensor_msgs/Imu)#矢量a+陀螺仪融合后的w
  - imu/data(sensor_msgs/Imu)#raw+ ..融合后四元数姿态估计
  - imu/mag(sensor_msgs/MagneticField)#磁强计->磁强数据

```
  
```

- ###### SLAM实时建图与定位--旋转矩阵

```
sudo apt install ros-noetic-hector-mapping
roslaunch wpr_simulation wpb_simple.launch
rosrun hector_mapping hector_mapping
rosrun rviz rviz	#!!!
roslaunch wpr_simulation wpb_rviz.launch 
rosrun rqt_robot_steering rqt_robot_steering

roslaunch wpr_simulation wpb_corridor_hector.launch 
roslaunch wpr_simulation wpb_corridor_gmapping.launch
```

- ###### TF定位系统

```
rosrun rqt_tf_tree rqt_tf_tree
```

- ###### 电机里程计--软件算法

  轮子->位移:s=v x n

- ###### Gmaping算法:

--通过里程计计算机器人底盘的位移,再通过激光雷达扫描障碍物产生点云修正机器人位移

--目的地:只使用雷达点云和障碍物配准

- ###### Gmaping建图

```
roslaunch wpr_simulation wpb_stage_robocup.launch #仿真
rostopic echo /scan --noarry
rosrun gmapping slam_gmapping
rosrun rviz rviz
rosrun wpr_simulation keyboard_vel_ctrl #键盘控制
```

- ###### Gmaping存图

```
rosrun map_server map_saver -f  map
rosrun map_server map_server map.yaml 
```

- ###### Gmaping参数

  ![2D6D549DB44BE3E55F80AAF02F7EDDE5](Robot.assets/2D6D549DB44BE3E55F80AAF02F7EDDE5.jpg)

  ![5EB68765DC104222FF9E99DB7CF15523](Robot.assets/5EB68765DC104222FF9E99DB7CF15523.jpg)

  ![B145D259BB9202A55742A9A959B1348B](Robot.assets/B145D259BB9202A55742A9A959B1348B.jpg)

- ###### Navigation导航

![A7DD222CC51B986C98028BC9F1FC384A](Robot.assets/A7DD222CC51B986C98028BC9F1FC384A-17094277657802.jpg)

```

```

