/**
 * @file mono_camera_node.cpp
 * @brief ROS2 单目相机节点实现
 * @author ROS2 Tutorial
 */

#include "mono_camera_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

/**
 * @brief MonoCameraNode 构造函数
 * @param options 节点选项
 */
MonoCameraNode::MonoCameraNode(const rclcpp::NodeOptions& options)
: Node("mono_camera_node", options) {
    // 声明并获取参数
    this->declare_parameter("camera_device_id", 0);
    this->declare_parameter("video_file", "");
    this->declare_parameter("use_device", true);
    this->declare_parameter("framerate", 30.0);
    this->declare_parameter("image_width", 640);
    this->declare_parameter("image_height", 480);
    this->declare_parameter("show_image", true);
    this->declare_parameter("canny_threshold1", 50.0);
    this->declare_parameter("canny_threshold2", 150.0);
    this->declare_parameter("gaussian_kernel_size", 5);
    
    camera_device_id_ = this->get_parameter("camera_device_id").as_int();
    video_file_ = this->get_parameter("video_file").as_string();
    use_device_ = this->get_parameter("use_device").as_bool();
    framerate_ = this->get_parameter("framerate").as_double();
    image_width_ = this->get_parameter("image_width").as_int();
    image_height_ = this->get_parameter("image_height").as_int();
    show_image_ = this->get_parameter("show_image").as_bool();
    canny_threshold1_ = this->get_parameter("canny_threshold1").as_double();
    canny_threshold2_ = this->get_parameter("canny_threshold2").as_double();
    gaussian_kernel_size_ = this->get_parameter("gaussian_kernel_size").as_int();
    
    if (gaussian_kernel_size_ % 2 == 0) {
        gaussian_kernel_size_ += 1; // 确保是奇数
    }
    
    // 创建图像发布者
    raw_image_pub_ = image_transport::create_publisher(this, "image_raw");
    edge_image_pub_ = image_transport::create_publisher(this, "image_edge");
    blur_image_pub_ = image_transport::create_publisher(this, "image_blur");
    
    // 创建相机参数发布者
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
    
    // 创建并填充相机信息消息
    camera_info_msg_.header.frame_id = "camera_frame";
    camera_info_msg_.height = image_height_;
    camera_info_msg_.width = image_width_;
    camera_info_msg_.distortion_model = "plumb_bob";
    
    // 填充通用相机内参矩阵，实际应用中应使用实际标定参数
    camera_info_msg_.k = {
        image_width_, 0, image_width_ / 2.0,
        0, image_width_, image_height_ / 2.0,
        0, 0, 1
    };
    
    camera_info_msg_.d = {0.0, 0.0, 0.0, 0.0, 0.0}; // 无畸变
    
    // 初始化相机
    if (!initializeCamera()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera");
        return;
    }
    
    // 创建处理定时器
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / framerate_)),
        std::bind(&MonoCameraNode::timerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "Mono camera node initialized");
}

/**
 * @brief MonoCameraNode 析构函数
 */
MonoCameraNode::~MonoCameraNode() {
    if (camera_) {
        camera_->release();
    }
    
    if (show_image_) {
        cv::destroyAllWindows();
    }
    
    RCLCPP_INFO(this->get_logger(), "Mono camera node shutdown");
}

/**
 * @brief 初始化相机
 * @return 成功返回true
 */
bool MonoCameraNode::initializeCamera() {
    camera_ = std::make_shared<cv::VideoCapture>();
    
    if (use_device_) {
        // 打开相机设备
        if (!camera_->open(camera_device_id_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera device %d", camera_device_id_);
            return false;
        }
        
        // 设置相机分辨率
        camera_->set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
        camera_->set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
        
        RCLCPP_INFO(this->get_logger(), "Camera device %d opened at %dx%d", 
                    camera_device_id_, image_width_, image_height_);
    } else {
        // 打开视频文件
        if (video_file_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Video file path is empty");
            return false;
        }
        
        if (!camera_->open(video_file_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file: %s", video_file_.c_str());
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Video file opened: %s", video_file_.c_str());
    }
    
    return true;
}

/**
 * @brief 定时回调函数，触发图像处理
 */
void MonoCameraNode::timerCallback() {
    processAndPublishImage();
}

/**
 * @brief 处理图像并发布结果
 */
void MonoCameraNode::processAndPublishImage() {
    // 检查相机状态
    if (!camera_ || !camera_->isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Camera is not opened");
        return;
    }
    
    // 捕获图像
    cv::Mat frame;
    if (!camera_->read(frame)) {
        RCLCPP_WARN(this->get_logger(), "Failed to read frame");
        
        // 如果是视频文件，可能已经到达结尾，可以选择重新开始
        if (!use_device_) {
            RCLCPP_INFO(this->get_logger(), "Restarting video file");
            camera_->set(cv::CAP_PROP_POS_FRAMES, 0);
        }
        
        return;
    }
    
    // 检查图像尺寸，如有必要进行调整
    if (frame.cols != image_width_ || frame.rows != image_height_) {
        cv::resize(frame, frame, cv::Size(image_width_, image_height_));
    }
    
    // 应用图像处理
    cv::Mat edge_image = applyEdgeDetection(frame);
    cv::Mat blur_image = applyGaussianBlur(frame);
    
    // 显示图像窗口
    if (show_image_) {
        cv::imshow("Raw Image", frame);
        cv::imshow("Edge Detection", edge_image);
        cv::imshow("Gaussian Blur", blur_image);
        cv::waitKey(1);
    }
    
    // 创建时间戳
    rclcpp::Time now = this->now();
    std_msgs::msg::Header header;
    header.stamp = now;
    header.frame_id = "camera_frame";
    
    // 发布原始图像
    sensor_msgs::msg::Image::SharedPtr raw_msg =
        cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
    raw_image_pub_.publish(*raw_msg);
    
    // 发布边缘检测图像
    sensor_msgs::msg::Image::SharedPtr edge_msg =
        cv_bridge::CvImage(header, "mono8", edge_image).toImageMsg();
    edge_image_pub_.publish(*edge_msg);
    
    // 发布高斯模糊图像
    sensor_msgs::msg::Image::SharedPtr blur_msg =
        cv_bridge::CvImage(header, "bgr8", blur_image).toImageMsg();
    blur_image_pub_.publish(*blur_msg);
    
    // 发布相机参数
    camera_info_msg_.header = header;
    camera_info_pub_->publish(camera_info_msg_);
}

/**
 * @brief 应用边缘检测处理
 * @param input 输入图像
 * @return 处理后的图像
 */
cv::Mat MonoCameraNode::applyEdgeDetection(const cv::Mat& input) {
    cv::Mat gray, edges;
    
    // 转换为灰度图
    cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
    
    // 应用Canny边缘检测
    cv::Canny(gray, edges, canny_threshold1_, canny_threshold2_);
    
    return edges;
}

/**
 * @brief 应用高斯模糊处理
 * @param input 输入图像
 * @return 处理后的图像
 */
cv::Mat MonoCameraNode::applyGaussianBlur(const cv::Mat& input) {
    cv::Mat blurred;
    
    // 应用高斯模糊
    cv::GaussianBlur(input, blurred, 
                     cv::Size(gaussian_kernel_size_, gaussian_kernel_size_), 
                     0);
    
    return blurred;
}

/**
 * @brief 主函数
 * @param argc 参数数量
 * @param argv 参数数组
 * @return 退出代码
 */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MonoCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 