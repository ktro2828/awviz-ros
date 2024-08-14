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

#include <string>

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
   * @param entity Entity path to log.
   */
  RosTopicProperty(const std::string & type, const std::string & topic, const std::string & entity)
  : type_(type), topic_(topic), entity_(entity)
  {
  }

  /**
   * @brief Set ROS message type name.
   * @param type Name of ROS message type.
   */
  void setType(const std::string & type) { type_ = type; }

  /**
   * @brief Set ROS topic name.
   * @param topic Name of topic.
   */
  void setTopic(const std::string & topic) { topic_ = topic; }

  /**
   * @brief Set entity path of record.
   * @param entity Entity path of record.
   */
  void setEntity(const std::string & entity) { entity_ = entity; }

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
   * @brief Get entity path of record.
   * @return Entity path of record.
   */
  const std::string & entity() const noexcept { return entity_; }

private:
  std::string type_;    //!< Type of ROS message.
  std::string topic_;   //!< Name of topic.
  std::string entity_;  //!< Entity path of recording.
};
}  // namespace awviz_common

#endif  // AWVIZ_COMMON__PROPERTY_HPP_
