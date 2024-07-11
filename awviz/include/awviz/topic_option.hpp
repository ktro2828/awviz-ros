#ifndef AWVIZ__TOPIC_OPTION_HPP_
#define AWVIZ__TOPIC_OPTION_HPP_

#include <rclcpp/rclcpp.hpp>

#include <stdexcept>
#include <string>
#include <vector>

namespace awviz
{

/**
 * @brief Represent ROS msg types.
 */
enum MsgType { PointCloud, Image, Unknown };

/**
 * @brief Convert string name of ROS msg into MsgType.
 * @param name Name of msg in string.
 * @return MsgType enum member.
 */
MsgType nameToMsgType(const std::string & name)
{
  if (name == "PointCloud") {
    return MsgType::PointCloud;
  } else if (name == "Image") {
    return MsgType::Image;
  } else {
    return MsgType::Unknown;
  }
}

/**
 * @brief A class representing topic options.
 */
class TopicOption
{
public:
  /**
   * @brief Construct a instance.
   * @param topic Name of topic.
   * @param type MsgType member representing ROS msg type.
   */
  TopicOption(const std::string & topic, const MsgType & type) : topic_(topic), type_(type) {}

  /**
   * @brief Construct vector of options from ROS parameters.
   * @param node ROS node.
   * @return std::vector<TopicOption> Topic options.
   */
  static std::vector<TopicOption> fromRosParam(rclcpp::Node * node)
  {
    std::vector<std::string> topic_names =
      node->declare_parameter<std::vector<std::string>>("topic_names");

    std::vector<TopicOption> options;
    for (const auto & topic : topic_names) {
      std::string type_name = node->declare_parameter<std::string>("topic_info." + topic + "type");
      auto type = nameToMsgType(type_name);

      options.emplace_back(topic, type);
    }

    return options;
  }

  /**
   * @brief Return the topic name.
   * @return std::string Name of topic.
   */
  const std::string & topic() const noexcept { return topic_; }

  /**
   * @brief Return the type of ROS msg.
   * @return MsgType Type of ROS msg.
   */
  const MsgType & type() const noexcept { return type_; }

private:
  std::string topic_;
  MsgType type_;
};
}  // namespace awviz
#endif  // AWVIZ__TOPIC_OPTION_HPP_
