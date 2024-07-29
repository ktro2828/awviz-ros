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

#ifndef AWVIZ_COMMON__VIEWER_HPP_
#define AWVIZ_COMMON__VIEWER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>

#include <memory>

namespace awviz_common
{
/**
 * @brief Application class used in a client code.
 */
class ViewerApp
{
public:
  /**
   * @brief Construct an application instance with the root ROS node.
   * @param node Root ROS node.
   */
  explicit ViewerApp(rclcpp::Node::SharedPtr node);

  /**
   * @brief Destruct an application instance.
   */
  ~ViewerApp();

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rerun::RecordingStream> stream_;
};
}  // namespace awviz_common

#endif  // AWVIZ_COMMON__VIEWER_HPP_
