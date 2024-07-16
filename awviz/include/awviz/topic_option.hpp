// Copyright 2024 Kotaro Uetake.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AWVIZ__TOPIC_OPTION_HPP_
#define AWVIZ__TOPIC_OPTION_HPP_

#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace awviz
{

/**
 * @brief Represent ROS msg types.
 */
enum MsgType { Unknown, PointCloud, CameraInfo, Image, CompressedImage, DetectedObjects, TrackedObjects };

/**
 * @brief Convert string name of ROS msg into MsgType.
 * @param name Name of msg in string.
 * @return MsgType enum member.
 */
MsgType nameToMsgType(const std::string & name)
{
  if (name == "PointCloud") {
    return MsgType::PointCloud;
  } else if (name == "CameraInfo") {
    return MsgType::CameraInfo;
  } else if (name == "Image") {
    return MsgType::Image;
  } else if (name == "CompressedImage") {
    return MsgType::CompressedImage;
  } else if (name == "DetectedObjects") {
    return MsgType::DetectedObjects;
  } else if (name == "TrackedObjects") {
    return MsgType::TrackedObjects;
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
  TopicOption(const std::string & topic, const MsgType & type)
  : topic_(topic), type_(type), entity_("/topics" + topic)
  {
  }

  /**
   * @brief Construct a instance with rerun recodring entity.
   * @param topic Name of topic.
   * @param type MsgType member representing ROS msg type.
   * @param entity Entity name of recodring.
   */
  TopicOption(const std::string & topic, const MsgType & type, const std::string & entity)
  : topic_(topic), type_(type), entity_(entity)
  {
  }

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
      std::string type_name =
        node->declare_parameter<std::string>("topic_options." + topic + ".type");
      auto type = nameToMsgType(type_name);

      try {
        std::string entity =
          node->declare_parameter<std::string>("topic_options." + topic + ".entity");
        options.emplace_back(topic, type, entity);
      } catch (std::exception & e) {
        options.emplace_back(topic, type);
      }
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

  /**
   * @brief Return the recording entity.
   * @return std::string Entity of recording.
   */
  const std::string & entity() const noexcept { return entity_; }

private:
  std::string topic_;
  MsgType type_;
  std::string entity_;
};
}  // namespace awviz
#endif  // AWVIZ__TOPIC_OPTION_HPP_
