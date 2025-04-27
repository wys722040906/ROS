/**
 * @file stereo_camera_node.cpp
 * @brief ROS2 双目相机节点实现
 * @author ROS2 Tutorial
 */

#include "stereo_camera_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

using namespace std::chrono_literals;

/**
 * @brief StereoCameraNode 构造函数
 * @param options 节点选项
 */
StereoCameraNode::StereoCameraNode(const rclcpp::NodeOptions& options)
: Node("stereo_camera_node", options) {
    // 声明并获取参数
    this->declare_parameter("left_camera_id", 0);
    this->declare_parameter("right_camera_id", 1);
    this->declare_parameter("left_image_file", "");
    this->declare_parameter("right_image_file", "");
    this->declare_parameter("use_device", true);
    this->declare_parameter("framerate", 30.0);
    this->declare_parameter("image_width", 640);
    this->declare_parameter("image_height", 480);
    this->declare_parameter("show_image", true);
    this->declare_parameter("calibration_file", "");
    
    // 立体匹配参数
    this->declare_parameter("stereo_algorithm", 0);  // 0: SGBM, 1: BM
    this->declare_parameter("min_disparity", 0);
    this->declare_parameter("num_disparities", 64);
    this->declare_parameter("block_size", 5);
    this->declare_parameter("p1", 200);
    this->declare_parameter("p2", 400);
    this->declare_parameter("disp12_max_diff", 1);
    this->declare_parameter("pre_filter_cap", 31);
    this->declare_parameter("uniqueness_ratio", 15);
    this->declare_parameter("speckle_window_size", 100);
    this->declare_parameter("speckle_range", 32);
    
    // 获取参数值
    left_camera_id_ = this->get_parameter("left_camera_id").as_int();
    right_camera_id_ = this->get_parameter("right_camera_id").as_int();
    left_image_file_ = this->get_parameter("left_image_file").as_string();
    right_image_file_ = this->get_parameter("right_image_file").as_string();
    use_device_ = this->get_parameter("use_device").as_bool();
    framerate_ = this->get_parameter("framerate").as_double();
    image_width_ = this->get_parameter("image_width").as_int();
    image_height_ = this->get_parameter("image_height").as_int();
    show_image_ = this->get_parameter("show_image").as_bool();
    calibration_file_ = this->get_parameter("calibration_file").as_string();
    
    stereo_algorithm_ = this->get_parameter("stereo_algorithm").as_int();
    min_disparity_ = this->get_parameter("min_disparity").as_int();
    num_disparities_ = this->get_parameter("num_disparities").as_int();
    block_size_ = this->get_parameter("block_size").as_int();
    p1_ = this->get_parameter("p1").as_int();
    p2_ = this->get_parameter("p2").as_int();
    disp12_max_diff_ = this->get_parameter("disp12_max_diff").as_int();
    pre_filter_cap_ = this->get_parameter("pre_filter_cap").as_int();
    uniqueness_ratio_ = this->get_parameter("uniqueness_ratio").as_int();
    speckle_window_size_ = this->get_parameter("speckle_window_size").as_int();
    speckle_range_ = this->get_parameter("speckle_range").as_int();
    
    // 确保参数有效
    if (block_size_ % 2 == 0) {
        block_size_ += 1; // 确保是奇数
    }
    
    if (num_disparities_ % 16 != 0) {
        num_disparities_ = (num_disparities_ / 16 + 1) * 16; // 确保是16的倍数
    }
    
    // 创建图像发布者
    left_image_pub_ = image_transport::create_publisher(this, "left/image_raw");
    right_image_pub_ = image_transport::create_publisher(this, "right/image_raw");
    disparity_map_pub_ = image_transport::create_publisher(this, "disparity");
    
    // 创建点云发布者
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points", 10);
    
    // 创建相机参数发布者
    left_camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 10);
    right_camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 10);
    
    // 创建并填充相机信息消息
    left_camera_info_msg_.header.frame_id = "left_camera_frame";
    left_camera_info_msg_.height = image_height_;
    left_camera_info_msg_.width = image_width_;
    left_camera_info_msg_.distortion_model = "plumb_bob";
    
    right_camera_info_msg_.header.frame_id = "right_camera_frame";
    right_camera_info_msg_.height = image_height_;
    right_camera_info_msg_.width = image_width_;
    right_camera_info_msg_.distortion_model = "plumb_bob";
    
    // 加载标定参数
    if (!calibration_file_.empty()) {
        if (!loadCalibrationParameters()) {
            RCLCPP_WARN(this->get_logger(), "Failed to load calibration parameters, using default values");
            
            // 使用通用默认值
            // 实际应用中应进行标定并使用实际值
            left_camera_matrix_ = (cv::Mat_<double>(3, 3) << 
                image_width_, 0, image_width_ / 2.0,
                0, image_width_, image_height_ / 2.0,
                0, 0, 1);
            
            right_camera_matrix_ = left_camera_matrix_;
            
            left_distortion_ = cv::Mat::zeros(1, 5, CV_64F);
            right_distortion_ = cv::Mat::zeros(1, 5, CV_64F);
            
            rotation_ = cv::Mat::eye(3, 3, CV_64F);
            translation_ = (cv::Mat_<double>(3, 1) << -0.1, 0, 0); // 相机基线10厘米
            
            // 初始化立体校正
            cv::stereoRectify(
                left_camera_matrix_, left_distortion_,
                right_camera_matrix_, right_distortion_,
                cv::Size(image_width_, image_height_),
                rotation_, translation_,
                left_camera_matrix_, right_camera_matrix_,
                left_map1_, left_map2_,
                right_map1_, right_map2_,
                Q_,
                cv::CALIB_ZERO_DISPARITY, 0);
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "No calibration file provided, using default values");
        
        // 使用通用默认值
        left_camera_matrix_ = (cv::Mat_<double>(3, 3) << 
            image_width_, 0, image_width_ / 2.0,
            0, image_width_, image_height_ / 2.0,
            0, 0, 1);
        
        right_camera_matrix_ = left_camera_matrix_;
        
        left_distortion_ = cv::Mat::zeros(1, 5, CV_64F);
        right_distortion_ = cv::Mat::zeros(1, 5, CV_64F);
        
        rotation_ = cv::Mat::eye(3, 3, CV_64F);
        translation_ = (cv::Mat_<double>(3, 1) << -0.1, 0, 0); // 相机基线10厘米
        
        // 初始化立体校正
        cv::stereoRectify(
            left_camera_matrix_, left_distortion_,
            right_camera_matrix_, right_distortion_,
            cv::Size(image_width_, image_height_),
            rotation_, translation_,
            left_camera_matrix_, right_camera_matrix_,
            left_map1_, left_map2_,
            right_map1_, right_map2_,
            Q_,
            cv::CALIB_ZERO_DISPARITY, 0);
    }
    
    // 创建立体匹配器
    if (stereo_algorithm_ == 0) {
        // 半全局块匹配
        auto sgbm = cv::StereoSGBM::create(
            min_disparity_, num_disparities_, block_size_,
            p1_, p2_, disp12_max_diff_,
            pre_filter_cap_, uniqueness_ratio_,
            speckle_window_size_, speckle_range_);
        stereo_matcher_ = sgbm;
    } else {
        // 块匹配
        auto bm = cv::StereoBM::create(num_disparities_, block_size_);
        bm->setPreFilterCap(pre_filter_cap_);
        bm->setUniquenessRatio(uniqueness_ratio_);
        bm->setDisp12MaxDiff(disp12_max_diff_);
        bm->setSpeckleWindowSize(speckle_window_size_);
        bm->setSpeckleRange(speckle_range_);
        stereo_matcher_ = bm;
    }
    
    // 填充相机信息
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            left_camera_info_msg_.k[i * 3 + j] = left_camera_matrix_.at<double>(i, j);
            right_camera_info_msg_.k[i * 3 + j] = right_camera_matrix_.at<double>(i, j);
        }
    }
    
    // 填充畸变系数
    left_camera_info_msg_.d.resize(5);
    right_camera_info_msg_.d.resize(5);
    for (int i = 0; i < 5; i++) {
        left_camera_info_msg_.d[i] = left_distortion_.at<double>(0, i);
        right_camera_info_msg_.d[i] = right_distortion_.at<double>(0, i);
    }
    
    // 初始化相机
    if (!initializeCameras()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize cameras");
        return;
    }
    
    // 创建处理定时器
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / framerate_)),
        std::bind(&StereoCameraNode::timerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "Stereo camera node initialized");
}

/**
 * @brief StereoCameraNode 析构函数
 */
StereoCameraNode::~StereoCameraNode() {
    if (left_camera_) {
        left_camera_->release();
    }
    
    if (right_camera_) {
        right_camera_->release();
    }
    
    if (show_image_) {
        cv::destroyAllWindows();
    }
    
    RCLCPP_INFO(this->get_logger(), "Stereo camera node shutdown");
}

/**
 * @brief 初始化相机
 * @return 成功返回true
 */
bool StereoCameraNode::initializeCameras() {
    left_camera_ = std::make_shared<cv::VideoCapture>();
    right_camera_ = std::make_shared<cv::VideoCapture>();
    
    if (use_device_) {
        // 打开相机设备
        if (!left_camera_->open(left_camera_id_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open left camera device %d", left_camera_id_);
            return false;
        }
        
        if (!right_camera_->open(right_camera_id_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open right camera device %d", right_camera_id_);
            return false;
        }
        
        // 设置相机分辨率
        left_camera_->set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
        left_camera_->set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
        
        right_camera_->set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
        right_camera_->set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
        
        RCLCPP_INFO(this->get_logger(), "Camera devices opened at %dx%d", 
                    image_width_, image_height_);
    } else {
        // 打开图像文件
        if (left_image_file_.empty() || right_image_file_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Image file paths are empty");
            return false;
        }
        
        if (!left_camera_->open(left_image_file_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open left image file: %s", left_image_file_.c_str());
            return false;
        }
        
        if (!right_camera_->open(right_image_file_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open right image file: %s", right_image_file_.c_str());
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Image files opened");
    }
    
    return true;
}

/**
 * @brief 加载相机标定参数
 * @return 成功返回true
 */
bool StereoCameraNode::loadCalibrationParameters() {
    try {
        // 从文件加载标定参数
        cv::FileStorage fs(calibration_file_, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open calibration file: %s", calibration_file_.c_str());
            return false;
        }
        
        fs["left_camera_matrix"] >> left_camera_matrix_;
        fs["right_camera_matrix"] >> right_camera_matrix_;
        fs["left_distortion"] >> left_distortion_;
        fs["right_distortion"] >> right_distortion_;
        fs["rotation"] >> rotation_;
        fs["translation"] >> translation_;
        fs["essential_matrix"] >> essential_matrix_;
        fs["fundamental_matrix"] >> fundamental_matrix_;
        
        // 初始化立体校正
        cv::stereoRectify(
            left_camera_matrix_, left_distortion_,
            right_camera_matrix_, right_distortion_,
            cv::Size(image_width_, image_height_),
            rotation_, translation_,
            left_camera_matrix_, right_camera_matrix_,
            left_map1_, left_map2_,
            right_map1_, right_map2_,
            Q_,
            cv::CALIB_ZERO_DISPARITY, 0);
        
        fs.release();
        return true;
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV error while loading calibration: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error while loading calibration: %s", e.what());
        return false;
    }
}

/**
 * @brief 定时回调函数，触发图像处理
 */
void StereoCameraNode::timerCallback() {
    processAndPublishImages();
}

/**
 * @brief 处理图像并发布结果
 */
void StereoCameraNode::processAndPublishImages() {
    // 检查相机状态
    if (!left_camera_ || !right_camera_ || 
        !left_camera_->isOpened() || !right_camera_->isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Cameras are not opened");
        return;
    }
    
    // 捕获图像
    cv::Mat left_frame, right_frame;
    
    if (!left_camera_->read(left_frame)) {
        RCLCPP_WARN(this->get_logger(), "Failed to read left frame");
        return;
    }
    
    if (!right_camera_->read(right_frame)) {
        RCLCPP_WARN(this->get_logger(), "Failed to read right frame");
        return;
    }
    
    // 检查图像尺寸，如有必要进行调整
    if (left_frame.cols != image_width_ || left_frame.rows != image_height_) {
        cv::resize(left_frame, left_frame, cv::Size(image_width_, image_height_));
    }
    
    if (right_frame.cols != image_width_ || right_frame.rows != image_height_) {
        cv::resize(right_frame, right_frame, cv::Size(image_width_, image_height_));
    }
    
    // 应用校正变换
    cv::Mat left_rectified, right_rectified;
    
    if (!left_map1_.empty() && !left_map2_.empty() && 
        !right_map1_.empty() && !right_map2_.empty()) {
        cv::remap(left_frame, left_rectified, left_map1_, left_map2_, cv::INTER_LINEAR);
        cv::remap(right_frame, right_rectified, right_map1_, right_map2_, cv::INTER_LINEAR);
    } else {
        left_rectified = left_frame;
        right_rectified = right_frame;
    }
    
    // 计算视差图
    cv::Mat disparity_map = computeDisparityMap(left_rectified, right_rectified);
    
    // 生成点云
    sensor_msgs::msg::PointCloud2 pointcloud_msg = generatePointCloud(disparity_map, left_rectified);
    
    // 显示图像窗口
    if (show_image_) {
        cv::imshow("Left Image", left_rectified);
        cv::imshow("Right Image", right_rectified);
        
        // 将视差图归一化以便显示
        cv::Mat disparity_normalized;
        cv::normalize(disparity_map, disparity_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::imshow("Disparity Map", disparity_normalized);
        
        cv::waitKey(1);
    }
    
    // 创建时间戳
    rclcpp::Time now = this->now();
    std_msgs::msg::Header header;
    header.stamp = now;
    
    // 发布原始图像
    std_msgs::msg::Header left_header = header;
    left_header.frame_id = "left_camera_frame";
    
    std_msgs::msg::Header right_header = header;
    right_header.frame_id = "right_camera_frame";
    
    sensor_msgs::msg::Image::SharedPtr left_msg =
        cv_bridge::CvImage(left_header, "bgr8", left_rectified).toImageMsg();
    left_image_pub_.publish(*left_msg);
    
    sensor_msgs::msg::Image::SharedPtr right_msg =
        cv_bridge::CvImage(right_header, "bgr8", right_rectified).toImageMsg();
    right_image_pub_.publish(*right_msg);
    
    // 发布视差图
    sensor_msgs::msg::Image::SharedPtr disparity_msg =
        cv_bridge::CvImage(left_header, "mono16", disparity_map).toImageMsg();
    disparity_map_pub_.publish(*disparity_msg);
    
    // 发布点云
    pointcloud_msg.header = left_header;
    pointcloud_pub_->publish(pointcloud_msg);
    
    // 发布相机参数
    left_camera_info_msg_.header = left_header;
    right_camera_info_msg_.header = right_header;
    
    left_camera_info_pub_->publish(left_camera_info_msg_);
    right_camera_info_pub_->publish(right_camera_info_msg_);
}

/**
 * @brief 计算视差图
 * @param left_image 左图像
 * @param right_image 右图像
 * @return 视差图
 */
cv::Mat StereoCameraNode::computeDisparityMap(const cv::Mat& left_image, const cv::Mat& right_image) {
    cv::Mat left_gray, right_gray, disparity;
    
    // 转换为灰度图
    if (left_image.channels() == 3) {
        cv::cvtColor(left_image, left_gray, cv::COLOR_BGR2GRAY);
    } else {
        left_gray = left_image;
    }
    
    if (right_image.channels() == 3) {
        cv::cvtColor(right_image, right_gray, cv::COLOR_BGR2GRAY);
    } else {
        right_gray = right_image;
    }
    
    // 计算视差图
    stereo_matcher_->compute(left_gray, right_gray, disparity);
    
    // 立体匹配器输出的视差值已乘以16，需除以16获得实际视差值
    disparity.convertTo(disparity, CV_32F, 1.0/16.0);
    
    return disparity;
}

/**
 * @brief 生成点云数据
 * @param disparity_map 视差图
 * @param left_image 左图像
 * @return 点云数据
 */
sensor_msgs::msg::PointCloud2 StereoCameraNode::generatePointCloud(
    const cv::Mat& disparity_map, const cv::Mat& left_image) {
    
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    
    // 如果没有Q矩阵或视差图为空，返回空点云
    if (Q_.empty() || disparity_map.empty()) {
        return pointcloud_msg;
    }
    
    // 简单实现，实际应用可以使用PCL库提供更完整功能
    // 将视差图转换为3D点云
    cv::Mat points3D;
    cv::reprojectImageTo3D(disparity_map, points3D, Q_);
    
    // 创建点云消息
    pointcloud_msg.height = points3D.rows;
    pointcloud_msg.width = points3D.cols;
    pointcloud_msg.is_dense = false;
    pointcloud_msg.is_bigendian = false;
    
    // 设置点云字段
    sensor_msgs::PointCloud2Modifier modifier(pointcloud_msg);
    modifier.setPointCloud2Fields(4,
                                 "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                 "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                 "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                 "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
    
    // 调整点云大小
    modifier.resize(points3D.rows * points3D.cols);
    
    // 创建点云数据迭代器
    sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_rgb(pointcloud_msg, "rgb");
    
    // 填充点云数据
    for (int y = 0; y < points3D.rows; ++y) {
        for (int x = 0; x < points3D.cols; ++x) {
            cv::Vec3f point = points3D.at<cv::Vec3f>(y, x);
            
            // 检查点的有效性
            if (std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2])) {
                *iter_x = point[0];
                *iter_y = point[1];
                *iter_z = point[2];
                
                // 将RGB颜色编码为单个浮点数
                if (left_image.channels() == 3) {
                    cv::Vec3b color = left_image.at<cv::Vec3b>(y, x);
                    uint32_t rgb_value = 
                        (static_cast<uint32_t>(color[0]) << 16) |
                        (static_cast<uint32_t>(color[1]) << 8) |
                        static_cast<uint32_t>(color[2]);
                    *iter_rgb = *reinterpret_cast<float*>(&rgb_value);
                } else {
                    *iter_rgb = 0;
                }
            } else {
                *iter_x = 0;
                *iter_y = 0;
                *iter_z = 0;
                *iter_rgb = 0;
            }
            
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_rgb;
        }
    }
    
    return pointcloud_msg;
}

/**
 * @brief 主函数
 * @param argc 参数数量
 * @param argv 参数数组
 * @return 退出代码
 */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 