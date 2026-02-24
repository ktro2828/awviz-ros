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

#include <awviz_common/transformation/entity_path_resolver.hpp>
#include <awviz_common/transformation/tf_tree.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <mutex>

TEST(EntityPathResolverTest, ResolvesEntityPath)
{
  auto tree = std::make_shared<awviz_common::TfTree>();
  auto tree_mtx = std::make_shared<std::mutex>();
  auto entities_mtx = std::make_shared<std::mutex>();

  const awviz_common::TfFrame base_link("base_link", awviz_common::TF_ROOT);
  tree->emplace(base_link);

  awviz_common::EntityPathResolver resolver(tree, tree_mtx, entities_mtx);
  resolver.update_entity(base_link);

  const auto entities = resolver.entities();
  ASSERT_TRUE(entities);
  ASSERT_GT(entities->count("base_link"), 0);
  EXPECT_EQ(entities->at("base_link"), "/map/base_link");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
