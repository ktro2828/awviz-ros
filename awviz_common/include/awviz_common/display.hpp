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

#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>

#include <memory>
#include <string>

namespace awviz_common
{
/**
 * @brief Interdimediate class for display items.
 */
class Display
{
public:
  Display() {}

  virtual ~Display() {}

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
 * @brief Struct representing a property of ROS msg display instance.
 */
class RosTopicProperty
{
public:
  RosTopicProperty() {}

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
  std::string type_;
  std::string topic_;
  std::string entity_;
};

/**
 * @brief
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
    auto msg_type = rosidl_generator_traits::name<MsgType>();
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

private:
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

private:
  typename rclcpp::Subscription<MsgType>::SharedPtr subscription_;
  RosTopicProperty property_;
};
}  // namespace awviz_common
#endif  // AWVIZ_COMMON__DISPLAY_HPP_
