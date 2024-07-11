#ifndef AWVIZ_RERUN_LOGGER_NODE_HPP_
#define AWVIZ_RERUN_LOGGER_NODE_HPP_

#include "awviz/topic_option.hpp"
#include "rclcpp/node_options.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace awviz
{
class RerunLoggerNode : public rclcpp::Node
{
public:
  explicit RerunLoggerNode(const rclcpp::NodeOptions & node_options);

private:
  /**
   * @brief Craete subscribers.
   */
  void createSubscriptions();

  /**
   * @brief Create a subscriber for PointCloud2 msg.
   */
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> createPointCloudSubscription(
    const TopicOption & option);

  /**
   * @brief Craete subscriber for Image msg.
   * @param option Topic option.
   */
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> createImageSubscription(
    const TopicOption & option);

private:
  rclcpp::CallbackGroup::SharedPtr parallel_callback_group_;
  std::map<std::string, std::shared_ptr<rclcpp::SubscriptionBase>> topic_to_subscription_;
  std::map<std::string, std::string> frame_id_to_entity_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr callback_timer_;
  rclcpp::TimerBase::SharedPtr update_tf_timer_;

  std::vector<TopicOption> topic_options_;
  const rerun::RecordingStream stream_{"rerun_logger_node"};
};
}  // namespace awviz

#endif  // AWVIZ_RERUN_LOGGER_NODE_HPP_
