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

#ifndef AWVIZ_PLUGIN__AUTOWARE_PERCEPTION__PREDICTED_OBJECTS_DISPLAY_HPP_
#define AWVIZ_PLUGIN__AUTOWARE_PERCEPTION__PREDICTED_OBJECTS_DISPLAY_HPP_

#include <awviz_common/display.hpp>
#include <rerun.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

namespace awviz_plugin
{
/**
 * @brief Display plugin of `autoware_perception_msgs::msg::PredictedObjects`.
 */
class PredictedObjectsDisplay
: public awviz_common::RosTopicDisplay<autoware_perception_msgs::msg::PredictedObjects>
{
public:
  /**
   * @brief Construct a new object.
   */
  PredictedObjectsDisplay();

protected:
  /**
   * @todo Add support of colorizing each predicted path mode.
   */
  void log_message(autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg) override;
};
}  // namespace awviz_plugin

#endif  // AWVIZ_PLUGIN__AUTOWARE_PERCEPTION__PREDICTED_OBJECTS_DISPLAY_HPP_
