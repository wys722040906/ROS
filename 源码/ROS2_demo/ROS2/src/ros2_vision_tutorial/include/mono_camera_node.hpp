/**
 * @file mono_camera_node.hpp
 * @brief ROS2 单目相机节点
 * @author ROS2 Tutorial
 */

#ifndef MONO_CAMERA_NODE_HPP
#define MONO_CAMERA_NODE_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"

#include <opencv2/opencv.hpp>

/**
 * @class MonoCameraNode
 * @brief 演示在ROS2中处理单目相机图像
 * 
 * 这个类展示了如何创建一个节点，可以从相机设备或图像文件捕获图像，
 * 进行处理，并发布处理后的图像。
 */
class MonoCameraNode : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     * @param options 节点选项
     */
    explicit MonoCameraNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief 析构函数
     */
    ~MonoCameraNode();

private:
    /**
     * @brief 初始化相机
     * @return 成功返回true
     */
    bool initializeCamera();

    /**
     * @brief 处理图像并发布结果
     */
    void processAndPublishImage();

    /**
     * @brief 定时回调函数
     */
    void timerCallback();

    /**
     * @brief 应用边缘检测处理
     * @param input 输入图像
     * @return 处理后的图像
     */
    cv::Mat applyEdgeDetection(const cv::Mat& input);

    /**
     * @brief 应用高斯模糊处理
     * @param input 输入图像
     * @return 处理后的图像
     */
    cv::Mat applyGaussianBlur(const cv::Mat& input);

    // 成员变量
    std::shared_ptr<cv::VideoCapture> camera_;   // 相机捕获器
    int camera_device_id_;                       // 相机设备ID
    std::string video_file_;                     // 视频文件路径
    bool use_device_;                            // 是否使用相机设备
    
    // 发布者
    image_transport::Publisher raw_image_pub_;    // 原始图像发布者
    image_transport::Publisher edge_image_pub_;   // 边缘检测图像发布者
    image_transport::Publisher blur_image_pub_;   // 模糊处理图像发布者
    
    // 相机参数发布者
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 参数
    double framerate_;        // 帧率
    int image_width_;         // 图像宽度
    int image_height_;        // 图像高度
    bool show_image_;         // 是否显示图像
    
    // 相机信息
    sensor_msgs::msg::CameraInfo camera_info_msg_;
    
    // 处理参数
    double canny_threshold1_;
    double canny_threshold2_;
    int gaussian_kernel_size_;
};

#endif // MONO_CAMERA_NODE_HPP 