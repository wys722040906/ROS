@[TOC](通过ROS控制仿真机械臂\真实机械臂（2）)

# 写在前面

在上一篇博客[通过ROS控制仿真机械臂\真实机械臂--原理及实现（1）](https://blog.csdn.net/xDongle/article/details/139135367?spm=1001.2014.3001.5501)中，我们已经成功实现**Movelt! + Gazebo仿真**并且将Ros的路径数据导出；在这篇文章中，我们将实现两个目标：

 1. 实现数据传输，将数据传输至单片机
 2. 向Ros反馈机器人的运行状态

# 数据传递
在实际开发中，使用了一块STM32F1单片机作为底层控制器；因此，用**串口实现PC（树莓派）与单片机的通信**；
## 准备工作
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/e1416309158a4ae999d53c5433423e58.png#pic_center)
### 1. 安装CH341驱动；

 这部分网上的教程很多，所以不再赘述；
### 2. 确定串口号；
在驱动安装成功和接线（USB转TTL模块）完成后，使用**CuteCom**确认串口号；
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/e48f982bd0ce436fb0188a29971c1c8f.jpeg#pic_center =300x300)

```
CuteCom是Ubuntu自带的串口调试助手，可以在"Ubuntu软件"中直接下载。
```
如下图，连接成功后选项中出现`/ttyCH341USBx`，同时可以通过**Settings下拉菜单**设定波特率等参数；
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/ed2847a1d47b4f61b6ea1d660394ccd6.png#pic_center)
**需要注意的是：**

 **1.  先利用简单例程确保PC与STM通讯建立成功，实现数据收发；**
 **2.  如果使用Vm虚拟机，建议Vm版本更新到16.0以上，会导致无法建立通讯；**

## 串口配置 
这部分除了要配置串口外，还要实现串口通信；

### 串口配置：串口号、超时时间等等

```cpp
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;
serial::Serial _ser_demo;

void ser_config( void )
{
    serial::Timeout _ser_to = serial::Timeout::simpleTimeout(1000); 
	serial::parity_t _ser_pt = serial::parity_t::parity_none; 
	serial::bytesize_t _ser_bt =  serial::bytesize_t::eightbits; 
	serial::flowcontrol_t _ser_ft = serial::flowcontrol_t::flowcontrol_none; 
	serial::stopbits_t _ser_st = serial::stopbits_t::stopbits_one;

	_ser_demo.setPort("/dev/ttyCH341USB0");
    _ser_demo.setBaudrate(115200);
    _ser_demo.setTimeout(_ser_to);
    _ser_demo.setParity(_ser_pt); 
    _ser_demo.setBytesize(_ser_bt);
    _ser_demo.setFlowcontrol(_ser_ft);
    _ser_demo.setStopbits(_ser_st);
}
```
这段代码中为了方便编程，重新命名了`control_msgs::FollowJointTrajectoryAction`类别，并且创建了一个名为`_ser_demo`的`serial::Serial`对象；在`ser_config（）`函数中则对串口进行了相应的配置。
### 启动串口

```cpp
    try
    {
        _ser_demo.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
    }
    if(_ser_demo.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyCH341USB0 is opened.");
    }
```
这段代码是打开串口设备，并捕获可能出现的异常。

 - 如果成功打开串口设备，则输出`"/dev/ttyCH341USB0 is opened."`表示串口已经打开； 
 - 如果出现异常，则输出`"Unable to open port."`表示无法打开串口。
## 数据发送测试
## 硬件接线
两个USB转TTL模块，一端连接虚拟机一端连接主机；
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/7e90db470113412e9ead89b43631abb2.png#pic_center)

##  代码
```cpp
#include <ros/ros.h>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <serial/serial.h>

#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <sstream>
#include <stdlib.h>
#include <sys/types.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <signal.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;
serial::Serial _ser_demo;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "real_robot_server");
	//Node_1
	ros::NodeHandle node_p;
	    
    ser_config();
    
    try
    {
        _ser_demo.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
    }
    if(_ser_demo.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyCH341USB0 is opened.");
    }
    
    uint8_t char_array[5] = "12345";
    ros::Rate loop_rate(10);
        
    while(ros::ok())
    {
   		_ser_demo.write(char_array, 5);
   		loop_rate.sleep();
    }
	
}

void ser_config( void )
{
    serial::Timeout _ser_to = serial::Timeout::simpleTimeout(1000); 
	serial::parity_t _ser_pt = serial::parity_t::parity_none; 
	serial::bytesize_t _ser_bt =  serial::bytesize_t::eightbits; 
	serial::flowcontrol_t _ser_ft = serial::flowcontrol_t::flowcontrol_none; 
	serial::stopbits_t _ser_st = serial::stopbits_t::stopbits_one;

	_ser_demo.setPort("/dev/ttyCH341USB0");
    _ser_demo.setBaudrate(115200);
    _ser_demo.setTimeout(_ser_to);
    _ser_demo.setParity(_ser_pt); 
    _ser_demo.setBytesize(_ser_bt);
    _ser_demo.setFlowcontrol(_ser_ft);
    _ser_demo.setStopbits(_ser_st);
}
```
# 机器人运行状态反馈
完成数据传递后，可以进行机器人状态进行反馈。通过编码器实现闭环控制，计算的角度数据通过STM32上传到PC中；
## 确认话题名称
在Gazebo中，观察话题名称与数据格式；
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/e1d442f10ff343b89401212ff1a8bb2c.png)
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/13385973dd1546fe99a9eaa41a572ea5.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/6046bd8847314179bc92cb39e29fc3dd.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/38bad5939b764465a493bf7ff730af8f.png)

##  代码

```cpp
#include <ros/ros.h>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <serial/serial.h>

#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <sstream>
#include <stdlib.h>
#include <sys/types.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <signal.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;
std::vector <std::vector<double>> _Pos_all;

void ser_config(void);
int _sub_flag( void );
void combine_msg( unsigned int len_arr );
void msg_dti( double pub_intput , uint8_t *char_array );
void ultimate_msg(unsigned int len_arr , std::vector <std::vector<double>> _Pos);

void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as);

serial::Serial _ser_demo;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "real_robot_server");
	
	//Node_1
	ros::NodeHandle node_f;
	Server server(node_f, "br_robot/follow_joint_trajectory", boost::bind(&execute, _1, &server), false);
	//Node_2
	ros::NodeHandle node_p;    
    
    ser_config();
    try
    {
        _ser_demo.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
    }
    if(_ser_demo.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyCH341USB0 is opened.");
    }

	server.start();
	ROS_INFO("server start!");
	ros::spin(); 
}

void ser_config( void )
{
    serial::Timeout _ser_to = serial::Timeout::simpleTimeout(1000); 
	serial::parity_t _ser_pt = serial::parity_t::parity_none; 
	serial::bytesize_t _ser_bt =  serial::bytesize_t::eightbits; 
	serial::flowcontrol_t _ser_ft = serial::flowcontrol_t::flowcontrol_none; 
	serial::stopbits_t _ser_st = serial::stopbits_t::stopbits_one;

	_ser_demo.setPort("/dev/ttyCH341USB0");
    _ser_demo.setBaudrate(115200);
    _ser_demo.setTimeout(_ser_to);
    _ser_demo.setParity(_ser_pt); 
    _ser_demo.setBytesize(_ser_bt);
    _ser_demo.setFlowcontrol(_ser_ft);
    _ser_demo.setStopbits(_ser_st);
}
int _sub_flag( void )
{
	while(! _ser_demo.available());
	_ser_demo.flushInput();
	return 1;
}
void combine_msg( unsigned int len_arr )
{
    ROS_INFO("_ser_pub_head start!");
	uint8_t len_array = len_arr + '0';
	_ser_demo.write(&len_array, 1);
	while (!_sub_flag());
	ROS_INFO("_ser_pub_head_1 start!");
	_ser_demo.write(&len_array, 1);
	while (!_sub_flag());
}
void msg_dti( double pub_intput , uint8_t *char_array )
{
    *char_array = (pub_intput >0)?2:1;
    pub_intput=abs(pub_intput);

    int length = 4;
    while(length--)
    {
		char_array++;
		*char_array = pub_intput;
		pub_intput = (pub_intput - *char_array)*10;
    }
}
void ultimate_msg(unsigned int len_arr , std::vector <std::vector<double>> _Pos)
{
    //head'
    uint8_t len_array = len_arr + '0';
	_ser_demo.write(&len_array, 1);
	//while (!_sub_flag());
	_ser_demo.write(&len_array, 1);
	//while (!_sub_flag());
    ROS_INFO("_pub_head Finish!");
    //msg'
    for (size_t i = 0; i < _Pos.size(); i++)
	{
		for (size_t j = 0; j < 6; j++)
		{
			//1byte symbol 5byte date
			uint8_t char_array[5];
			msg_dti(_Pos[i][j] , char_array);
			_ser_demo.write(char_array, 5);
		}
	}
    ROS_INFO("_pub_data Finish!");
    //tail'
    uint8_t data_tail = 'a';
	_ser_demo.write(&data_tail, 1);
	//while (!_sub_flag());
}
void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as)
{
    as->setSucceeded();
    ros::NodeHandle node_s;
    ros::Publisher joint_state_pub = node_s.advertise<sensor_msgs::JointState>("/br_robot/joint_states",1);

	sensor_msgs::JointState joint_state_data;
    joint_state_data.name.resize(6);
    joint_state_data.position.resize(6);
    ros::Rate loop_rate(10);

    std::vector<double> row(6);

    for (int j = 0; j <goal->trajectory.points.size();j++)
    {
        for (int i = 0; i < 6; i++)
        {
            row[i] = goal->trajectory.points[j].positions[i];
        }
        _Pos_all.push_back(row);
    }

    ultimate_msg(_Pos_all.size() , _Pos_all);

    for (int j = 0; j <goal->trajectory.points.size();j++)
    {
        ROS_INFO("pub start!");

        joint_state_data.header.stamp = ros::Time::now();

        joint_state_data.name[0] = "Joint_1";
        joint_state_data.name[1] = "Joint_2";
        joint_state_data.name[2] = "Joint_3";
        joint_state_data.name[3] = "Joint_4";
        joint_state_data.name[4] = "Joint_5";
        joint_state_data.name[5] = "Joint_6";

        joint_state_data.position[0] =   goal->trajectory.points[j].positions[0];
        joint_state_data.position[1] =   goal->trajectory.points[j].positions[1];
        joint_state_data.position[2] =   goal->trajectory.points[j].positions[2];
        joint_state_data.position[3] =   goal->trajectory.points[j].positions[3];
        joint_state_data.position[4] =   goal->trajectory.points[j].positions[4];
        joint_state_data.position[5] =   goal->trajectory.points[j].positions[5];

        joint_state_pub.publish(joint_state_data);

        loop_rate.sleep();
    }
    
    //

}




```

