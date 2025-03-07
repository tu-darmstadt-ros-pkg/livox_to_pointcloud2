#pragma once

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#define ROS2
#include <livox_to_pointcloud2/livox_converter.hpp>

namespace livox_to_pointcloud2 {

class LivoxToPointCloud2 : public rclcpp::Node {
public:
  LivoxToPointCloud2(const rclcpp::NodeOptions& options);
  ~LivoxToPointCloud2();

private:
  LivoxConverter converter;

  std::vector<rclcpp::SubscriptionBase::SharedPtr> livox_subs;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> livox2_subs;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> points_pubs;
};

}  // namespace livox_to_pointcloud2