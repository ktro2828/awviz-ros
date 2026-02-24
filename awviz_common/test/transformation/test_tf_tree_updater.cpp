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

#include "test_utility.hpp"

#include <awviz_common/transformation/tf_tree.hpp>
#include <awviz_common/transformation/tf_tree_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>

#include <memory>
#include <mutex>

TEST(TfTreeUpdaterTest, UpdatesTreeFromBuffer)
{
  auto node = std::make_shared<rclcpp::Node>("tf_tree_updater_test");
  auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tree_mtx = std::make_shared<std::mutex>();
  awviz_common::TfTree tree;

  const auto now = node->get_clock()->now();
  buffer->setTransform(make_transform(awviz_common::TF_ROOT, "base_link", now), "test_authority");
  buffer->setTransform(make_transform("base_link", "camera", now), "test_authority", true);

  awviz_common::TfTreeUpdater updater(node, buffer, tree_mtx);
  updater.update(tree);

  EXPECT_TRUE(tree.contains("base_link"));
  EXPECT_TRUE(tree.contains("camera"));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
