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

#ifndef AWVIZ_PLUGIN__AUTOWARE_PERCEPTION__DETECTED_OBJECTS_DISPLAY_HPP_
#define AWVIZ_PLUGIN__AUTOWARE_PERCEPTION__DETECTED_OBJECTS_DISPLAY_HPP_

#include <awviz_common/display.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>

namespace awviz_plugin
{
class DetectedObjectsDisplay
: public awviz_common::RosTopicDisplay<autoware_perception_msgs::msg::DetectedObjects>
{
public:
  /**
   * @brief Construct a new object.
   */
  DetectedObjectsDisplay();

protected:
  void logToStream(autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr msg) override;
};
}  // namespace awviz_plugin

#endif  // AWVIZ_PLUGIN__AUTOWARE_PERCEPTION__DETECTED_OBJECTS_DISPLAY_HPP_
