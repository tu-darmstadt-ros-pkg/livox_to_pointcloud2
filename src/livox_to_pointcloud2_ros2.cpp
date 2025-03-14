#include <livox_to_pointcloud2/livox_to_pointcloud2_ros2.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#ifdef LIVOX_ROS2_DRIVER
#include <livox_interfaces/msg/custom_msg.hpp>
#endif

#ifdef LIVOX_ROS_DRIVER2
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

namespace livox_to_pointcloud2 {

LivoxToPointCloud2::LivoxToPointCloud2(const rclcpp::NodeOptions& options) : rclcpp::Node("livox_to_pointcloud2", options) {

  // Declare parameters for input and output topics
  this->declare_parameter<std::vector<std::string>>("input_topics", {"/livox/lidar"});
  this->declare_parameter<std::vector<std::string>>("output_topics", {"/livox/points"});

  auto input_topics = this->get_parameter("input_topics").as_string_array();
  auto output_topics = this->get_parameter("output_topics").as_string_array();

  // Ensure both topic lists have the same size
  if (input_topics.size() != output_topics.size()) {
    RCLCPP_ERROR(this->get_logger(), "Number of input topics and output topics must be equal!");
    return;
  }

  // Get existing topics and their types
  auto topic_map = this->get_topic_names_and_types();

  for (size_t i = 0; i < input_topics.size(); ++i) {
    std::string livox_topic = input_topics[i];
    std::string pointcloud_topic = output_topics[i];

    if (livox_topic == pointcloud_topic) {
      RCLCPP_ERROR(this->get_logger(), "Input topic must not be equal to output topic! %s", livox_topic.c_str());
      return;
    }

    // Check if the topic already exists with a different type
    if (topic_map.find(livox_topic) != topic_map.end()) {
      auto& topic_types = topic_map[livox_topic];
      for (const auto& type : topic_types) {
        if (type != "livox_ros_driver2/msg/CustomMsg") { // TODO this only works for ROS2
          RCLCPP_ERROR(this->get_logger(), "Topic %s already exists with a different type: %s", 
                       livox_topic.c_str(), type.c_str());
          return;
        }        
      }
    }

    // Create a publisher for each topic
    auto pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, rclcpp::QoS(1));
    points_pubs.push_back(pub);

#ifdef LIVOX_ROS2_DRIVER
    // livox_ros2_driver
    auto sub = this->create_subscription<livox_interfaces::msg::CustomMsg>(
      livox_topic,
      rclcpp::SensorDataQoS(),
      [this, pub](const livox_interfaces::msg::CustomMsg::ConstSharedPtr livox_msg) {
        const auto points_msg = converter.convert(*livox_msg);
        pub->publish(*points_msg);
      });
    livox_subs.push_back(sub);
#endif

#ifdef LIVOX_ROS_DRIVER2
    // livox_ros_driver2
    auto sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      livox_topic,
      rclcpp::SensorDataQoS(),
      [this, pub](const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr livox_msg) {
        const auto points_msg = converter.convert(*livox_msg);
        pub->publish(*points_msg);
      });
    livox_subs.push_back(sub);
#endif

    RCLCPP_INFO(this->get_logger(), "Subscribed to %s and publishing to %s", livox_topic.c_str(), pointcloud_topic.c_str());
  }
}

LivoxToPointCloud2::~LivoxToPointCloud2() {}

}  // namespace livox_to_pointcloud2

RCLCPP_COMPONENTS_REGISTER_NODE(livox_to_pointcloud2::LivoxToPointCloud2);
