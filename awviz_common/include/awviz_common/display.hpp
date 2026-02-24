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
#include <unordered_map>
#include <utility>

namespace awviz_common
{
/**
 * @brief Lifecycle state of a Display instance.
 */
enum class DisplayState : uint8_t {
  kCreated = 0,
  kInitialized = 1,
  kConfigured = 2,
  kRunning = 3,
};

/**
 * @brief Intermediate class for display items.
 *
 * Lifecycle expectations:
 * - Created: constructor runs (do not access ROS/Rerun resources here).
 * - Initialized: initialize(node, stream) is called.
 * - Configured: set_property(topic, entity_roots) is called.
 * - Running: start() is called and subscriptions should be active.
 * - Stopped: end() is called and subscriptions should be released.
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
   * @brief Set attributes of property.
   * @param topic Name of topic.
   * @param entity Entity path of the record.
   */
  virtual void set_property(
    const std::string & topic,
    const std::shared_ptr<std::unordered_map<std::string, std::string>> entity_roots) = 0;

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
  virtual bool is_initialized() const { return state_ != DisplayState::kCreated; }

protected:
  void set_state(DisplayState state) { state_ = state; }
  DisplayState state() const { return state_; }

  rclcpp::Node::SharedPtr node_;                    //!< Node shared pointer.
  std::shared_ptr<rerun::RecordingStream> stream_;  //!< RecordingStream shared pointer.
  DisplayState state_{DisplayState::kCreated};      //!< Lifecycle state of the display.
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
    property_.set_type(msg_type);
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
    node_ = std::move(node);
    stream_ = std::move(stream);
    set_state(DisplayState::kInitialized);
  };

  /**
   * @brief Set status of attributes.
   * @param topic Name of topic.
   * @param entity Entity path of the record.
   */
  void set_property(
    const std::string & topic,
    const std::shared_ptr<std::unordered_map<std::string, std::string>> entity_roots) override
  {
    property_.set_topic(topic);
    property_.set_entity_roots(entity_roots);
    set_state(DisplayState::kConfigured);
  }

  /**
   * @brief Start to display.
   */
  void start() override
  {
    subscribe();
    set_state(DisplayState::kRunning);
  }

  /**
   * @brief End to display.
   */
  void end() override
  {
    unsubscribe();
    set_state(DisplayState::kConfigured);
  }

  bool is_initialized() const override
  {
    return (state() == DisplayState::kConfigured || state() == DisplayState::kRunning) &&
           property_.is_initialized();
  }

protected:
  static constexpr const char * TIMELINE_NAME = "timestamp";  //!< Entity name of timeline record.

  /**
   * @brief Start to subscribing the specified topic.
   * @todo Currently, `rclcpp::SensorDataQoS` is used for QoS profile setting.
   */
  virtual void subscribe()
  {
    if (!is_initialized()) {
      return;
    }

    // TODO(ktro2828): QoS setting
    subscription_ = node_->create_subscription<MsgType>(
      property_.topic(), rclcpp::SensorDataQoS{},
      [this](const typename MsgType::ConstSharedPtr msg) { log_message(msg); });
  };

  /**
   * @brief End to subscribing the topic.
   */
  virtual void unsubscribe() { subscription_.reset(); }

  /**
   * @brief Log subscribed ROS message to recording stream.
   * @param msg Constant shared pointer of ROS message.
   * @note Currently, if the corresponding entity path doesn't exist this just logs warning as text.
   */
  virtual void log_message(typename MsgType::ConstSharedPtr msg) = 0;

protected:
  typename rclcpp::Subscription<MsgType>::SharedPtr subscription_;  //!< Subscription of the topic.
  RosTopicProperty property_;                                       //!< Topic property.
};
}  // namespace awviz_common
#endif  // AWVIZ_COMMON__DISPLAY_HPP_
