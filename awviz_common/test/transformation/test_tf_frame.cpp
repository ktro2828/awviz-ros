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

/**
 * Test suite for the `TfFrame` class.
 *
 * This test case includes constructing a new object specifying both `id` and `parent` using
 * `awviz_common::TF_ROOT` as `parent`.
 */
TEST(TfFrameTest, ConstructWithRoot)
{
  constexpr const char * id = "base_link";

  // specify both `id` and `parent`
  awviz_common::TfFrame frame(id, awviz_common::TF_ROOT);

  // `awviz_common::TfFrame::id()`
  EXPECT_EQ(frame.id(), id);

  // `awviz_common::TfFrame::parent()`
  EXPECT_EQ(frame.parent(), awviz_common::TF_ROOT);

  // `awviz_common::TfFrame::is_root()`
  EXPECT_FALSE(frame.is_root());

  // `awviz_common::TfFrame::is_static()`
  EXPECT_FALSE(frame.is_static());
}

/**
 * Test suite for the `TfFrame` class.
 *
 * This test case includes constructing a new object specifying only `id`. Then, `parent` will set
 * empty string `""`.
 */
TEST(TfFrameTest, ConstructOnlyId)
{
  constexpr const char * id = "base_link";

  // specify only `id`
  awviz_common::TfFrame frame(id);

  // `awviz_common::TfFrame::id()`
  EXPECT_EQ(frame.id(), id);

  // `awviz_common::TfFrame::parent()`
  EXPECT_EQ(frame.parent(), "");

  // `awviz_common::TfFrame::is_root()`
  EXPECT_TRUE(frame.is_root());

  // `awviz_common::TfFrame::is_static()`
  EXPECT_TRUE(frame.is_static());
}

/**
 * Test suite for the `TfFrame` class.
 *
 * This test case includes constructing a new object specifying both `id` and `parent` without using
 * static frame name.
 */
TEST(TfFrameTest, ConstructWithOtherBoth)
{
  constexpr const char * id = "sensor_kit_base_link";
  constexpr const char * parent = "base_link";

  // specify both `id` and `parent`
  awviz_common::TfFrame frame(id, parent);

  // `awviz_common::TfFrame::id()`
  EXPECT_EQ(frame.id(), id);

  // `awviz_common::TfFrame::parent()`
  EXPECT_EQ(frame.parent(), parent);

  // `awviz_common::TfFrame::is_root()`
  EXPECT_FALSE(frame.is_root());

  // `awviz_common::TfFrame::is_static()`
  EXPECT_TRUE(frame.is_static());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
