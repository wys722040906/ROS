/**
 * @file stereo_camera_node.hpp
 * @brief ROS2 双目相机节点
 * @author ROS2 Tutorial
 */

#ifndef STEREO_CAMERA_NODE_HPP
#define STEREO_CAMERA_NODE_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

/**
 * @class StereoCameraNode
 * @brief 演示在ROS2中处理双目相机图像
 * 
 * 这个类展示了如何创建一个节点，可以从两个相机设备或左右图像文件捕获图像，
 * 进行双目处理，计算视差图和3D点云，并发布结果。
 */
class StereoCameraNode : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     * @param options 节点选项
     */
    explicit StereoCameraNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief 析构函数
     */
    ~StereoCameraNode();

private:
    /**
     * @brief 初始化相机
     * @return 成功返回true
     */
    bool initializeCameras();

    /**
     * @brief 加载相机标定参数
     * @return 成功返回true
     */
    bool loadCalibrationParameters();

    /**
     * @brief 处理图像并发布结果
     */
    void processAndPublishImages();

    /**
     * @brief 计算视差图
     * @param left_image 左图像
     * @param right_image 右图像
     * @return 视差图
     */
    cv::Mat computeDisparityMap(const cv::Mat& left_image, const cv::Mat& right_image);

    /**
     * @brief 生成点云数据
     * @param disparity_map 视差图
     * @param left_image 左图像
     * @return 点云数据
     */
    sensor_msgs::msg::PointCloud2 generatePointCloud(const cv::Mat& disparity_map, const cv::Mat& left_image);

    /**
     * @brief 定时回调函数
     */
    void timerCallback();

    // 成员变量
    std::shared_ptr<cv::VideoCapture> left_camera_;    // 左相机捕获器
    std::shared_ptr<cv::VideoCapture> right_camera_;   // 右相机捕获器
    int left_camera_id_;                               // 左相机设备ID
    int right_camera_id_;                              // 右相机设备ID
    std::string left_image_file_;                      // 左图像文件路径
    std::string right_image_file_;                     // 右图像文件路径
    bool use_device_;                                  // 是否使用相机设备
    
    // 发布者
    image_transport::Publisher left_image_pub_;        // 左图像发布者
    image_transport::Publisher right_image_pub_;       // 右图像发布者
    image_transport::Publisher disparity_map_pub_;     // 视差图发布者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;  // 点云发布者
    
    // 相机参数发布者
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_camera_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_camera_info_pub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 参数
    double framerate_;        // 帧率
    int image_width_;         // 图像宽度
    int image_height_;        // 图像高度
    bool show_image_;         // 是否显示图像
    std::string calibration_file_; // 标定文件路径
    
    // 立体匹配参数
    int stereo_algorithm_;    // 立体匹配算法选择
    int min_disparity_;       // 最小视差
    int num_disparities_;     // 视差数量
    int block_size_;          // 块大小
    int p1_;                  // P1参数
    int p2_;                  // P2参数
    int disp12_max_diff_;     // 左右一致性检查最大差异
    int pre_filter_cap_;      // 预过滤器截断值
    int uniqueness_ratio_;    // 唯一性比率
    int speckle_window_size_; // 斑点窗口大小
    int speckle_range_;       // 斑点范围
    
    // 相机标定参数
    cv::Mat left_camera_matrix_;     // 左相机内参矩阵
    cv::Mat right_camera_matrix_;    // 右相机内参矩阵
    cv::Mat left_distortion_;        // 左相机畸变系数
    cv::Mat right_distortion_;       // 右相机畸变系数
    cv::Mat rotation_;               // 旋转矩阵
    cv::Mat translation_;            // 平移矩阵
    cv::Mat essential_matrix_;       // 本质矩阵
    cv::Mat fundamental_matrix_;     // 基础矩阵
    
    // 立体校正变换
    cv::Mat left_map1_, left_map2_;    // 左相机校正映射
    cv::Mat right_map1_, right_map2_;  // 右相机校正映射
    cv::Mat Q_;                        // 视差到深度映射矩阵
    
    // 立体匹配器
    cv::Ptr<cv::StereoMatcher> stereo_matcher_;
    
    // 相机信息
    sensor_msgs::msg::CameraInfo left_camera_info_msg_;
    sensor_msgs::msg::CameraInfo right_camera_info_msg_;
};

#endif // STEREO_CAMERA_NODE_HPP 