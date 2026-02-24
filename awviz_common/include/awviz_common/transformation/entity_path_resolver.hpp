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

#ifndef AWVIZ_COMMON__TRANSFORMATION__ENTITY_PATH_RESOLVER_HPP_
#define AWVIZ_COMMON__TRANSFORMATION__ENTITY_PATH_RESOLVER_HPP_

#include "awviz_common/transformation/tf_tree.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace awviz_common
{
/**
 * @brief Default resolver that uses TfTree to build entity paths.
 */
class EntityPathResolver final
{
public:
  EntityPathResolver(
    std::shared_ptr<TfTree> tf_tree, std::shared_ptr<std::mutex> tf_tree_mtx,
    std::shared_ptr<std::mutex> entities_mtx);

  void update_entity(const TfFrame & frame);

  std::shared_ptr<std::unordered_map<std::string, std::string>> entities() const;

private:
  std::shared_ptr<TfTree> tf_tree_;
  std::shared_ptr<std::unordered_map<std::string, std::string>> entities_;
  std::shared_ptr<std::mutex> tf_tree_mtx_;
  std::shared_ptr<std::mutex> entities_mtx_;
};
}  // namespace awviz_common

#endif  // AWVIZ_COMMON__TRANSFORMATION__ENTITY_PATH_RESOLVER_HPP_
