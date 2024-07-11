#include "awviz/rerun_logger_node.hpp"

#include "awviz/rerun_ros_interface.hpp"
#include "awviz/topic_option.hpp"
#include "rclcpp/subscription.hpp"

#include <chrono>

namespace awviz
{
RerunLoggerNode::RerunLoggerNode(const rclcpp::NodeOptions & node_options)
: Node("rerun_logger_node", node_options)
{
  using std::chrono_literals::operator""ms;

  stream_.spawn().exit_on_failure();

  topic_options_ = TopicOption::fromRosParam(this);

  parallel_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  callback_timer_ = create_wall_timer(
    100ms, [&]() -> void { createSubscriptions(); }, parallel_callback_group_);
}

void RerunLoggerNode::createSubscriptions()
{
  for (const auto & option : topic_options_) {
    if (topic_to_subscription_.find(option.topic()) != topic_to_subscription_.end()) {
      continue;
    }

    if (option.type() == MsgType::PointCloud) {
      topic_to_subscription_[option.topic()] = createPointCloudSubscription(option);
    } else if (option.type() == MsgType::Image) {
      topic_to_subscription_[option.topic()] = createImageSubscription(option);
    } else {
      RCLCPP_WARN_STREAM(get_logger(), "Unknown msg type of topic: " << option.topic());
    }
  }
}

std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>>
RerunLoggerNode::createPointCloudSubscription(const TopicOption & option)
{
  return create_subscription<sensor_msgs::msg::PointCloud2>(option.topic(), 1000, [&] {});
}

std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>>
RerunLoggerNode::createImageSubscription(const TopicOption & option)
{
  return create_subscription<sensor_msgs::msg::Image>(option.topic(), 1000, [&] {});
}
}  // namespace awviz

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(awviz::RerunLoggerNode);
