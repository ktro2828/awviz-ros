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

#ifndef AWVIZ_COMMON__DISPLAY_HPP_
#define AWVIZ_COMMON__DISPLAY_HPP_

#include "awviz_common/property.hpp"

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>
#include <rosidl_runtime_cpp/traits.hpp>

#include <memory>
#include <string>

namespace awviz_common
{
/**
 * @brief Intermediate class for display items.
 */
class Display
{
public:
  Display() = default;

  virtual ~Display() = default;

  /**
   * @brief Initialize attributes.
   * @param node ROS node instance.
   * @param stream Recording stream.
   */
  virtual void initialize(
    rclcpp::Node::SharedPtr node, std::shared_ptr<rerun::RecordingStream> stream) = 0;

  /**
   * @brief Set status of attributes.
   * @param topic Name of topic.
   * @param entity Entity path of the record.
   */
  virtual void setStatus(const std::string & topic, const std::string & entity) = 0;

  /**
   * @brief Start to display.
   */
  virtual void start() = 0;

  /**
   * @brief End to display
   */
  virtual void end() = 0;

  /**
   * @brief Return true if the initialization is completed.
   * @return bool Return the value of the private member named `is_initialized_`.
   */
  bool isInitialized() const { return is_initialized_; }

protected:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rerun::RecordingStream> stream_;
  bool is_initialized_;
};

/**
 * @brief Inherited plugin class to display ROS topics.
 */
template <typename MsgType>
class RosTopicDisplay : public Display
{
public:
  /**
   * @brief Construct an instance.
   */
  RosTopicDisplay()
  {
    const auto msg_type = rosidl_generator_traits::name<MsgType>();
    property_.setType(msg_type);
  }

  /**
   * @brief Destruct an instance.
   */
  ~RosTopicDisplay() override { unsubscribe(); }

  /**
   * @brief Initialize the instance specifying the root ROS node and recording stream.
   * @param node Root ROS node.
   * @param stream Recording stream.
   */
  void initialize(
    rclcpp::Node::SharedPtr node, std::shared_ptr<rerun::RecordingStream> stream) override
  {
    node_ = node;
    stream_ = stream;
    is_initialized_ = true;
  };

  /**
   * @brief Set status of attributes.
   * @param topic Name of topic.
   * @param entity Entity path of the record.
   */
  void setStatus(const std::string & topic, const std::string & entity) override
  {
    property_.setTopic(topic);
    property_.setEntity(entity);
  }

  /**
   * @brief Start to display.
   */
  void start() override { subscribe(); }

  /**
   * @brief End to display.
   */
  void end() override { unsubscribe(); }

protected:
  static constexpr const char * TIMELINE_NAME = "timestamp";  //!< Entity name of timeline record.

  /**
   * @brief Start to subscribing the specified topic.
   */
  virtual void subscribe()
  {
    if (!isInitialized()) {
      return;
    }

    // TODO(ktro2828): QoS setting
    subscription_ = node_->create_subscription<MsgType>(
      property_.topic(), rclcpp::SensorDataQoS{},
      [this](const typename MsgType::ConstSharedPtr msg) { logToStream(msg); });
  };

  /**
   * @brief End to subscribing the topic.
   */
  virtual void unsubscribe() { subscription_.reset(); }

  /**
   * @brief Log subscribed ROS message to recording stream.
   * @param msg ROS message.
   */
  virtual void logToStream(typename MsgType::ConstSharedPtr msg) = 0;

protected:
  typename rclcpp::Subscription<MsgType>::SharedPtr subscription_;  //!< Subscription of the topic.
  RosTopicProperty property_;                                       //!< Topic property.
};
}  // namespace awviz_common
#endif  // AWVIZ_COMMON__DISPLAY_HPP_
