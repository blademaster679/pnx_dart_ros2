#ifndef LIGHT_DETECTOR__DETECTOR_NODE_HPP_
#define LIGHT_DETECTOR__DETECTOR_NODE_HPP_
// ROS
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

//STD
#include <memory>
#include <string>
#include <vector>

#include "detector.hpp"
#include "pnp_solver.hpp"
#include "auto_aim_interfaces/msg/light.hpp"

namespace rm_auto_aim_dart
{

}







#endif