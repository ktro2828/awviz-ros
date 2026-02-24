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

#ifndef AWVIZ_COMMON__TOPIC_SCANNER_HPP_
#define AWVIZ_COMMON__TOPIC_SCANNER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace awviz_common
{
/**
 * @brief Simple topic scanning interface for ROS nodes.
 *
 * This class provides an abstraction around topic discovery so it can be
 * separated from visualization logic.
 */
class ITopicScanner
{
public:
  using TopicMap = std::map<std::string, std::vector<std::string>>;

  /**
   * @brief Construct a new topic scanner interface.
   */
  ITopicScanner() = default;

  virtual ~ITopicScanner() = default;

  /**
   * @brief Scan current topics and return their names and types.
   * @return Map of topic name to a list of types.
   */
  virtual TopicMap scan() const = 0;
};

/**
 * @brief Concrete topic scanner using rclcpp node discovery.
 */
class TopicScanner final : public ITopicScanner
{
public:
  explicit TopicScanner(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}

  TopicMap scan() const override { return node_->get_topic_names_and_types(); }

private:
  rclcpp::Node::SharedPtr node_;  //!< Node used for discovery.
};
}  // namespace awviz_common

#endif  // AWVIZ_COMMON__TOPIC_SCANNER_HPP_
