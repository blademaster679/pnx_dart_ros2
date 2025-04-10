// Copyright 2022 Chen Jun
// Licensed under the MIT License.

// ROS
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_ros/create_timer_ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "detector.hpp"
#include "detector_node.hpp"

namespace rm_auto_aim_dart
{
    LightDetectorNode::LightDetectorNode(const rclcpp::NodeOptions &options) : Node("light_detector", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting LightDetectorNode!");

        // detector
        detector_ = initDectector();

        // lights publisher
        light_pub_ = this->create_publisher<auto_aim_interfaces::msg::Light>(
            "lights", rclcpp::SensorDataQoS());

        // Visualization marker publisher
        // See http://wiki.ros.org/rviz/DisplayTypes/Marker
        light_marker_.ns = "light";
        light_marker_.action = visualization_msgs::msg::Marker::ADD;
        light_marker_.type = visualization_msgs::msg::Marker::CYLINDER;
        light_marker_.scale.x = 2 * light_radius_;
        light_marker_.scale.y = 2 * light_radius_;
        light_marker_.scale.z = 0.01;
        light_marker_.color.a = 1;
        light_marker_.color.r = 0.0f;
        light_marker_.color.g = 1.0f;
        light_marker_.color.b = 0.0f;
        light_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

        marker_pub_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "light_markers", 10);

        // Debug Publishers
        debug_ =
            this->declare_parameter<bool>("debug", false);
        if (debug_)
        {
            createDebugPublishers();
        }

        // Debug param change monitor
        debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        debug_cb_handle_ =
            debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter &p)
                                                     {
      debug_ = p.as_bool();
      debug_ ? createDebugPublishers() : destroyDebugPublishers(); });
        // Camera info
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info)
            {
                cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
                cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
                pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
                cam_info_sub_.reset(); // 取消订阅
            });
        // imageCallback when camera info is ready
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&LightDetectorNode::imageCallback, this, std::placeholders::_1));

        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(), this->get_node_timers_interface());
        tf2_buffer_->setCreateTimerInterface(timer_interface);
        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    }

    void LightDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
    {
        try
        {
            rclcpp::Time target_time = img_msg->header.stamp;
            auto odom_to_gimbal = tf2_buffer_->lookupTransform(
                "odom", img_msg->header.frame_id, target_time,
                rclcpp::Duration::from_seconds(0.01));
            auto msg_q = odom_to_gimbal.transform.rotation;
            tf2::Quaternion tf_q;
            tf2::fromMsg(msg_q, tf_q);
            tf2::Matrix3x3 tf2_matrix = tf2::Matrix3x3(tf_q);
            imu_to_camera << tf2_matrix.getRow(0)[0], tf2_matrix.getRow(0)[1],
                tf2_matrix.getRow(0)[2], tf2_matrix.getRow(1)[0],
                tf2_matrix.getRow(1)[1], tf2_matrix.getRow(1)[2],
                tf2_matrix.getRow(2)[0], tf2_matrix.getRow(2)[1],
                tf2_matrix.getRow(2)[2];
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "Something Wrong when lookUpTransform");
            return;
        }

        if (debug_)
            light_msg_.image = *img_msg;
        cv::Mat img;
        auto lights = detectLights(img_msg, img);

        if (pnp_solver_ != nullptr)
        {
            light_msg_.header = light_marker_.header = img_msg->header;
            light_msg_.lights.clear();
            marker_array_.markers.clear();
            light_marker_.id = 0;

            auto_aim_interfaces::msg::Light light_msg;
            for (auto &light : lights)
            {
                std::vector<cv::Mat> rvecs, tvecs;
                bool success = pnp_solver_->solvePnP(light, rvecs, tvecs);
                if (success)
                {
                    cv::Mat rotation_matrix;
                    cv::Rodrigues(rvecs[0], rotation_matrix);

                    // rotation matrix to quaternion
                    tf2::Matrix3x3 tf2_rotation_matrix(
                        rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
                        rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
                        rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                        rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
                        rotation_matrix.at<double>(2, 2));
                    tf2::Quaternion tf2_q;
                    tf2_rotation_matrix.getRotation(tf2_q);
                    // Convert Eigen::Matrix3d to tf2::Matrix3x3
                    tf2::Matrix3x3 tf2_matrix(
                        imu_to_camera(0, 0), imu_to_camera(0, 1), imu_to_camera(0, 2),
                        imu_to_camera(1, 0), imu_to_camera(1, 1), imu_to_camera(1, 2),
                        imu_to_camera(2, 0), imu_to_camera(2, 1), imu_to_camera(2, 2));
                    tf2::Quaternion R_gimbal_camera_;
                    tf2_matrix.getRotation(R_gimbal_camera_);
                    tf2::Matrix3x3(R_gimbal_camera_ * tf2_q).getRPY(armor.roll, armor.pitch, armor.yaw);
                    armor_msg.pose.orientation = tf2::toMsg(tf2_q);

                    // Fill pose
                    armor_msg.pose.position.x = tvec.at<double>(0);
                    armor_msg.pose.position.y = tvec.at<double>(1);
                    armor_msg.pose.position.z = tvec.at<double>(2);

                    // Fill the distance to image center
                    armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor.center);

                    // Fill the markers
                    armor_marker_.id++;
                    armor_marker_.pose = armor_msg.pose;
                    armors_msg_.armors.emplace_back(armor_msg);
                    marker_array_.markers.emplace_back(armor_marker_);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "PnP failed!");
                }
            }
        }
    }
}