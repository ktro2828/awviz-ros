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

#ifndef AWVIZ_COMMON__PROPERTY_HPP_
#define AWVIZ_COMMON__PROPERTY_HPP_

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

namespace awviz_common
{
/**
 * @brief Struct representing a property of ROS msg display instance.
 */
class RosTopicProperty
{
public:
  RosTopicProperty() = default;

  /**
   * @brief Construct instance.
   * @param type ROS msg type.
   * @param topic Name of topic.
   * @param entity_roots Root entity paths to recording.
   */
  RosTopicProperty(
    const std::string & type, const std::string & topic,
    const std::shared_ptr<std::unordered_map<std::string, std::string>> entity_roots)
  : type_(type), topic_(topic), entity_roots_(entity_roots)
  {
  }

  /**
   * @brief Set ROS message type name.
   * @param type Name of ROS message type.
   */
  void set_type(const std::string & type) { type_ = type; }

  /**
   * @brief Set ROS topic name.
   * @param topic Name of topic.
   */
  void set_topic(const std::string & topic) { topic_ = topic; }

  /**
   * @brief Set entity path of record.
   * @param entity Entity path of record.
   */
  void set_entity_roots(
    const std::shared_ptr<std::unordered_map<std::string, std::string>> entity_roots)
  {
    entity_roots_ = entity_roots;
  }

  /**
   * @brief Get ROS message type.
   * @return ROS message type name.
   */
  const std::string & type() const noexcept { return type_; }

  /**
   * @brief Get ROS topic name.
   * @return Name of topic.
   */
  const std::string & topic() const noexcept { return topic_; }

  /**
   * @brief Return the entity path using topic name.
   * @return Topic name used as entity path.
   */
  const std::string & entity() const noexcept { return topic_; }

  /**
   * @brief Return the entity path using the corresponding root and its frame ID.
   *
   * @param frame_id Frame ID.
   * @return Return "/<Root>/<FrameID>/<Topic>" if the corresponding root exists, otherwise return
   * `std::nullopt`.
   */
  std::optional<std::string> entity(const std::string & frame_id) const noexcept
  {
    if (entity_roots_ && entity_roots_->count(frame_id) > 0) {
      return entity_roots_->at(frame_id) + topic_;
    } else {
      return std::nullopt;
    }
  }

  bool is_initialized() const { return !type_.empty() && !topic_.empty() && entity_roots_; }

private:
  std::string type_;   //!< Type of ROS message.
  std::string topic_;  //!< Name of topic.
  std::shared_ptr<std::unordered_map<std::string, std::string>>
    entity_roots_;  //!< Entity root paths of recording.
};
}  // namespace awviz_common

#endif  // AWVIZ_COMMON__PROPERTY_HPP_
