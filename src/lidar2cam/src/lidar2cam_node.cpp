#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/opencv.hpp>

class LidarToCameraProjectionNode : public rclcpp::Node {
public:
    LidarToCameraProjectionNode() : Node("lidar2cam") {
        auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        // 初始化订阅者和发布者
        image_pub_ = image_transport::create_publisher(this, "/projected_image");

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", best_effort_qos,
            std::bind(&LidarToCameraProjectionNode::cameraInfoCallback, this, std::placeholders::_1));

        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar_points", best_effort_qos,
            std::bind(&LidarToCameraProjectionNode::pointCloudCallback, this, std::placeholders::_1));

        // 订阅相机图像
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera_image", best_effort_qos,
            std::bind(&LidarToCameraProjectionNode::imageCallback, this, std::placeholders::_1));

        // 初始化TF2监听器
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 获取TF变换
        this->declare_parameter<std::string>("lidar_frame", "ego_vehicle/LIDAR_TOP");
        this->declare_parameter<std::string>("camera_frame", "ego_vehicle/CAM_FRONT");

        // 等待TF变换可用
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                try {
                    auto transform = tf_buffer_->lookupTransform(
                        this->get_parameter("camera_frame").as_string(),
                        this->get_parameter("lidar_frame").as_string(),
                        rclcpp::Time(0));

                    transform_lidar_to_cam_ = Eigen::Matrix4f::Identity();
                    Eigen::Translation3f translation(transform.transform.translation.x,
                                                       transform.transform.translation.y,
                                                       transform.transform.translation.z);
                    Eigen::Quaternionf rotation(transform.transform.rotation.w,
                                                transform.transform.rotation.x,
                                                transform.transform.rotation.y,
                                                transform.transform.rotation.z);
                    transform_lidar_to_cam_.block<3, 3>(0, 0) = rotation.toRotationMatrix();
                    transform_lidar_to_cam_.block<3, 1>(0, 3) = translation.vector();

                    // 打印平移和旋转信息
                    RCLCPP_INFO(this->get_logger(), "Translation: x = %f, y = %f, z = %f",
                                translation.x(), translation.y(), translation.z());
                    RCLCPP_INFO(this->get_logger(), "Rotation (Quaternion): w = %f, x = %f, y = %f, z = %f",
                                rotation.w(), rotation.x(), rotation.y(), rotation.z());
                                
                } catch (tf2::TransformException &ex) {
                    RCLCPP_WARN(this->get_logger(), "TF exception: %s", ex.what());
                }
            });
    }

private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        // 读取相机内参
        camera_matrix_ = cv::Mat(3, 3, CV_64F, (void*)msg->k.data()).clone();
        dist_coeffs_ = cv::Mat(1, 5, CV_64F, (void*)msg->d.data()).clone();
        image_width_ = msg->width;
        image_height_ = msg->height;

        RCLCPP_INFO(this->get_logger(), "Camera info received");
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;
            processImage(cv_image);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Could not convert image: %s", e.what());
        }
        RCLCPP_INFO(this->get_logger(), "Image received");
        
    }

    void processImage(cv::Mat& image) {
        // 确保图像和点云数据都有效
        if (image.empty() || current_cloud_.points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Image or point cloud is empty.");
            return;
        }

        // 使用从相机信息中获取的图像尺寸创建图像的副本
        cv::Mat projected_image = image.clone();

        for (const auto& point : current_cloud_.points) {
            // 将LIDAR点转换到相机坐标系
            Eigen::Vector4f point_lidar(point.x, point.y, point.z, 1.0);
            Eigen::Vector4f point_cam = transform_lidar_to_cam_ * point_lidar;

            if (point_cam.z() > 0) {  // 确保点在相机前方
                // 将3D点投射到2D图像平面
                cv::Mat point_3d = (cv::Mat_<double>(3, 1) << point_cam.x(), point_cam.y(), point_cam.z());
                cv::Mat point_2d;
                cv::projectPoints(point_3d, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), camera_matrix_, dist_coeffs_, point_2d);

                // 获取2D坐标
                int x = static_cast<int>(point_2d.at<double>(0));
                int y = static_cast<int>(point_2d.at<double>(1));

                // 在图像上绘制点
                if (x >= 0 && x < projected_image.cols && y >= 0 && y < projected_image.rows) {
                    cv::circle(projected_image, cv::Point(x, y), 2, cv::Scalar(0, 255, 0), -1);  // 使用绿色圆点表示
                }
            }
        }

        // 将处理后的图像发布到话题
        auto out_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", projected_image).toImageMsg();
        image_pub_.publish(out_msg);

        RCLCPP_INFO(this->get_logger(), "Published projected image");
    }


    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);
        current_cloud_ = cloud;
        RCLCPP_INFO(this->get_logger(), "Received point cloud");
    }

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    image_transport::Publisher image_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    Eigen::Matrix4f transform_lidar_to_cam_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    int image_width_ = 1280;  // Default values, will be updated with camera info
    int image_height_ = 720;
    rclcpp::TimerBase::SharedPtr timer_;
    pcl::PointCloud<pcl::PointXYZ> current_cloud_;  // Store current point cloud
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarToCameraProjectionNode>());
    rclcpp::shutdown();
    return 0;
}
