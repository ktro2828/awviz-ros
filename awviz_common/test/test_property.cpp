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

#include <awviz_common/property.hpp>

#include <gtest/gtest.h>

/**
 * Test suite for `RosTopicProperty` class.
 *
 * This test case includes testing `::entity()` function expecting to return topic name.
 */
TEST(PropertyTest, EntityTopic)
{
  const std::string type{"DummyType"};
  const std::string topic{"/dummy/topic"};
  const std::unordered_map<std::string, std::string> entity_roots{
    {"foo", "/foo"}, {"bar", "/foo/bar"}};

  awviz_common::RosTopicProperty property(
    type, topic, std::make_shared<std::unordered_map<std::string, std::string>>(entity_roots));

  EXPECT_EQ(property.entity(), topic);
}

/**
 * Test suite for `RosTopicProperty` class.
 *
 * This test case includes testing `::entity(frame_id)` function expecting to return the
 * corresponding entity root + topic name.
 */
TEST(PropertyTest, EntityWithRootAndTopic)
{
  const std::string type{"DummyType"};
  const std::string topic{"/dummy/topic"};
  const std::unordered_map<std::string, std::string> entity_roots{
    {"foo", "/foo"}, {"bar", "/foo/bar"}};

  awviz_common::RosTopicProperty property(
    type, topic, std::make_shared<std::unordered_map<std::string, std::string>>(entity_roots));

  EXPECT_EQ(property.entity("foo"), "/foo" + topic);
  EXPECT_EQ(property.entity("bar"), "/foo/bar" + topic);
}

/**
 * Test suite for `RosTopicProperty` class.
 *
 * This test case includes testing `::entity_without_topic(frame_id)` function expecting to return
 * the corresponding entity root.
 */
TEST(PropertyTest, EntityWithoutTopic)
{
  const std::string type{"DummyType"};
  const std::string topic{"/dummy/topic"};
  const std::unordered_map<std::string, std::string> entity_roots{
    {"foo", "/foo"}, {"bar", "/foo/bar"}};

  awviz_common::RosTopicProperty property(
    type, topic, std::make_shared<std::unordered_map<std::string, std::string>>(entity_roots));

  EXPECT_EQ(property.entity_without_topic("foo"), "/foo");
  EXPECT_EQ(property.entity_without_topic("bar"), "/foo/bar");
}
