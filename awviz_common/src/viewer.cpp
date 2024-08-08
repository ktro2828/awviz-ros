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

#include "awviz_common/viewer.hpp"

#include "awviz_common/visualization_manager.hpp"
#include "rclcpp/utilities.hpp"
#include "rerun/recording_stream.hpp"

#include <memory>

namespace awviz_common
{
ViewerApp::ViewerApp()
{
  node_ = std::make_shared<rclcpp::Node>("awviz");
  stream_ = std::make_shared<rerun::RecordingStream>("awviz");
  stream_->spawn().exit_on_failure();
  manager_ = std::make_unique<VisualizationManager>(node_, stream_);
}

ViewerApp::~ViewerApp()
{
  rclcpp::shutdown();
}
}  // namespace awviz_common
