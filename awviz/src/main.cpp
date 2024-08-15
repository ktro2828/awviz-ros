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

#include <awviz_common/viewer.hpp>
#include <rerun/third_party/cxxopts.hpp>

#include <string>

int main(int argc, char ** argv)
{
  cxxopts::Options options("awviz", "A 3D ROS viewer for Autoware powered by Rerun.");
  // clang-format off
  options.add_options()
    ("h,help", "Print usage.")
  ;
  // clang-format on

  auto args = options.parse(argc, argv);

  if (args.count("help")) {
    std::cout << options.help() << std::endl;
    exit(0);
  }

  rclcpp::init(argc, argv);
  awviz_common::ViewerApp app;
  app.run();
}
