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

#include "awviz_common/transformation/entity_path_resolver.hpp"

#include <mutex>
#include <utility>

namespace awviz_common
{
EntityPathResolver::EntityPathResolver(
  std::shared_ptr<TfTree> tf_tree, std::shared_ptr<std::mutex> tf_tree_mtx,
  std::shared_ptr<std::mutex> entities_mtx)
: tf_tree_(std::move(tf_tree)),
  entities_(std::make_shared<std::unordered_map<std::string, std::string>>()),
  tf_tree_mtx_(std::move(tf_tree_mtx)),
  entities_mtx_(std::move(entities_mtx))
{
}

std::shared_ptr<std::unordered_map<std::string, std::string>> EntityPathResolver::entities() const
{
  std::lock_guard<std::mutex> lock(*entities_mtx_);
  return entities_;
}

void EntityPathResolver::update_entity(const TfFrame & frame)
{
  std::scoped_lock lock(*tf_tree_mtx_, *entities_mtx_);
  if (!tf_tree_ || !tf_tree_->can_link_to(frame, TF_ROOT)) {
    return;
  }
  entities_->insert_or_assign(frame.id(), tf_tree_->entity_path(frame));
}
}  // namespace awviz_common
