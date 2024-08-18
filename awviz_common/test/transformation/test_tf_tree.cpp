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

#include <awviz_common/transformation/tf_tree.hpp>

#include <gtest/gtest.h>

#include <optional>

/**
 * Test suite for the `TfTree` class.
 *
 * This test case includes constructing a new `TfTree` object.
 */
TEST(TfTreeTest, Construct)
{
  awviz_common::TfTree tree;

  const auto & frames = tree.get_frames();
  ASSERT_EQ(frames.size(), 1);
  EXPECT_EQ(frames.at(awviz_common::TF_ROOT), awviz_common::TfFrame(awviz_common::TF_ROOT));
}

/**
 * Test suite for the `TfTree` class.
 *
 * This test case includes emplacing a another `TfFrame` object.
 */
TEST(TfTreeTest, Emplace)
{
  awviz_common::TfTree tree;

  const awviz_common::TfFrame frame("base_link", awviz_common::TF_ROOT);
  tree.emplace(frame);

  const auto & frames = tree.get_frames();
  ASSERT_EQ(frames.size(), 2);
  EXPECT_EQ(frames.at(frame.id()), frame);
}

/**
 * Test suite for the `TfTree` class.
 *
 * This test case includes retrieving all frames in the tree.
 */
TEST(TfTreeTest, GetFrames)
{
  awviz_common::TfTree tree;

  // first emplacement
  const awviz_common::TfFrame frame1("base_link", awviz_common::TF_ROOT);
  tree.emplace(frame1);

  const auto & frames1 = tree.get_frames();
  ASSERT_EQ(frames1.size(), 2);
  EXPECT_EQ(frames1.at(frame1.id()), frame1);

  // second emplacement
  const awviz_common::TfFrame frame2("sensor_kit_base_link", "base_link");
  tree.emplace(frame2);
  const auto & frames2 = tree.get_frames();
  ASSERT_EQ(frames2.size(), 3);
  EXPECT_EQ(frames2.at(frame2.id()), frame2);
}

/**
 * Test suite for the `TfTree` class.
 *
 * This test case includes retrieving the specified frame from the tree.
 * If the corresponding frame doesn't exist returns `std::nullopt`.
 */
TEST(TfTreeTest, GetFrame)
{
  awviz_common::TfTree tree;

  const awviz_common::TfFrame frame("base_link", awviz_common::TF_ROOT);
  tree.emplace(frame);

  // Get existent frame
  EXPECT_EQ(tree.get_frame(frame.id()), frame);

  // Get nonexistent frame
  EXPECT_EQ(tree.get_frame("sensor_kit_base_link"), std::nullopt);
}

/**
 * Test suite for the `TfTree` class.
 *
 * This test case includes retrieving the parent of the specified frame from the tree.
 * If the corresponding parent frame doesn't exist returns `std::nullopt`.
 */
TEST(TfTreeTest, GetParent)
{
  awviz_common::TfTree tree;

  const awviz_common::TfFrame frame1("base_link", awviz_common::TF_ROOT);
  const awviz_common::TfFrame frame2("sensor_kit_base_link", "base_link");
  tree.emplace(frame1);
  tree.emplace(frame2);

  // Get existent parent
  EXPECT_EQ(tree.get_parent(frame1.id()), awviz_common::TfFrame(awviz_common::TF_ROOT));
  EXPECT_EQ(tree.get_parent(frame2.id()), frame1);

  // Get nonexistent parent
  EXPECT_EQ(tree.get_parent("camera_optical_link"), std::nullopt);
}

/**
 * Test suite for `TfTree` class.
 *
 * This test case includes checking if the specified frame or ID is contained in the tree.
 *
 * @note When checking the frame, returns true if an item whose key and value both are the same as
 * the input frame is contained in the tree.
 * @note When checking the ID, returns true if an item whose key is the same as the ID of the input
 * frame is contained in the tree.
 */
TEST(TfTreeTest, ContainsFrame)
{
  awviz_common::TfTree tree;
  const awviz_common::TfFrame frame("base_link", awviz_common::TF_ROOT);
  tree.emplace(frame);

  // Check whether the frame is contained
  EXPECT_TRUE(tree.contains(frame));
  // key is the same, but value is different
  EXPECT_FALSE(tree.contains(awviz_common::TfFrame("base_link")));
  // key and value both are different
  EXPECT_FALSE(tree.contains(awviz_common::TfFrame("camera_optical_link")));

  // Check whether the ID is contained
  EXPECT_TRUE(tree.contains(frame.id()));
  EXPECT_FALSE(tree.contains("camera_optical_link"));
}

/**
 * Test suite for `TfTree` class.
 *
 * This test case includes retrieving a entity path of the specified frame.
 *
 * @note The entity path will be in the format
 * `"/<Parent0>/<Parent1>/.../<FrameID>"`, where `"<Parent0>"` represents the deepest parent
 * frame.
 */
TEST(TfTreeTest, EntityPath)
{
  awviz_common::TfTree tree;

  // Case 1: base_link -> map
  const awviz_common::TfFrame frame1("base_link", awviz_common::TF_ROOT);
  tree.emplace(frame1);
  EXPECT_EQ(tree.entity_path(frame1), "/map/base_link");

  // Case 2: sensor_kit_base_ink -> base_link -> map
  const awviz_common::TfFrame frame2("sensor_kit_base_link", "base_link");
  tree.emplace(frame2);
  EXPECT_EQ(tree.entity_path(frame2), "/map/base_link/sensor_kit_base_link");

  // Case 3: camera_optical_link -> sensor_kit_base_link -> base_link -> map
  const awviz_common::TfFrame frame3("camera_optical_link", "sensor_kit_base_link");
  EXPECT_EQ(tree.entity_path(frame3), "/map/base_link/sensor_kit_base_link/camera_optical_link");

  // Case 4: camera_optical_link -> map
  const awviz_common::TfFrame frame4("camera_optical_link", awviz_common::TF_ROOT);
  EXPECT_EQ(tree.entity_path(frame4), "/map/camera_optical_link");

  // Case 5: camera_optical_link
  const awviz_common::TfFrame frame5("camera_optical_link");
  EXPECT_EQ(tree.entity_path(frame5), "/camera_optical_link");
}

/**
 * Test suite for `TfTree` class.
 *
 * This test case includes checking if the input frame can link to the specified frame ID.
 */
TEST(TfTreeTest, CanLinkedTo)
{
  awviz_common::TfTree tree;
  const awviz_common::TfFrame frame("base_link", awviz_common::TF_ROOT);
  tree.emplace(frame);

  // Test linkable frames
  EXPECT_TRUE(tree.can_link_to(frame, awviz_common::TF_ROOT));
  const awviz_common::TfFrame frame1("sensor_kit_base_link", "base_link");
  EXPECT_TRUE(tree.can_link_to(frame1, awviz_common::TF_ROOT));

  // Test disconnected frames
  const awviz_common::TfFrame frame2("camera_optical_link", "sensor_kit_base_link");
  EXPECT_FALSE(tree.can_link_to(frame2, awviz_common::TF_ROOT));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
