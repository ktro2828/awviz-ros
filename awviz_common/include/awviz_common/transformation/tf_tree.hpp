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

#ifndef AWVIZ_COMMON__TF_TREE_HPP_
#define AWVIZ_COMMON__TF_TREE_HPP_

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace awviz_common
{
/**
 * @brief A class to represent a TF frame information.
 */
class TfFrame
{
public:
  /**
   * @brief Construct a new object.
   *
   * @param id Frame ID.
   * @param parent Parent frame ID.
   */
  TfFrame(const std::string & id, const std::string & parent) : id_(id), parent_(parent) {}

  /**
   * @brief Construct a new object with empty string for parent.
   *
   * @param id Frame ID.
   */
  explicit TfFrame(const std::string & id) : id_(id), parent_("") {}

  /**
   * @brief Return own frame ID.
   *
   * @return Own frame ID.
   */
  const std::string & id() const { return id_; }

  /**
   * @brief Return the parent frame ID.
   *
   * @return Parent frame ID.
   */
  const std::string & parent() const { return parent_; }

  /**
   * @brief Indicate whether the frame is root by checking if `parent_` is empty.
   *
   * @return Return true, if the `parent_` is empty.
   */
  bool is_root() const { return parent_.empty(); }

  /**
   * @brief Return whether the tf frame is static or not.
   * @note Currently, this returns true if the parent id is not `"map"`.
   *
   * @return Return true if the parent id is not `"map"`.
   */
  bool is_static() const { return parent_ != "map"; }

private:
  std::string id_;      //!< Frame ID.
  std::string parent_;  //!< Parent frame ID.
};

class TfTree
{
public:
  /**
   * @brief Add a new tf frame to the tree.
   *
   * @param frame A new tf frame. If it has been already registered, skip adding.
   */
  void emplace(const TfFrame & frame)
  {
    if (!contains(frame.id())) {
      frames_.emplace(frame.id(), frame);
    }

    if (!frame.is_root()) {
      emplace(frame.parent());
    }
  }

  /**
   * @brief Add a new tf frame to the tree with the empty string parent.
   *
   * @param id Frame ID. If it has been already registered, skip adding.
   */
  void emplace(const std::string & id)
  {
    if (!contains(id)) {
      frames_.emplace(id, id);
    }
  }

  /**
   * @brief Return map of all frames.
   *
   * @return Shared pointer of all frames.
   */
  const std::unordered_map<std::string, TfFrame> & get_frames() const { return frames_; }

  /**
   * @brief Get the `TfFrame` object.
   *
   * @param id Frame ID.
   * @return Corresponding `TfFrame` object. If there is no target `TfFrame`, returns `nullptr`.
   */
  std::optional<TfFrame> get_frame(const std::string & id) const
  {
    return contains(id) ? std::make_optional(frames_.at(id)) : std::nullopt;
  }

  /**
   * @brief Get the parent `TfFrame` object.
   *
   * @param id Frame ID.
   * @return Parent `TfFrame` object. If there is no parent, returns `nullptr`.
   */
  std::optional<TfFrame> get_parent(const std::string & id) const
  {
    auto frame = get_frame(id);
    return frame ? get_frame(frame->parent()) : std::nullopt;
  }

  /**
   * @brief Whether to the specified frame is contained in the tree.
   *
   * @param id Frame ID.
   * @return Returns true, if the frame is contained.
   */
  bool contains(const std::string & id) const { return frames_.count(id) > 0; }

  /**
   * @brief Whether to the parent of specified frame is contained in the tree.
   *
   * @param id Frame ID.
   * @return Returns true, if the parent frame is contained.
   */
  bool is_root(const std::string & id) const { return contains(id) && frames_.at(id).is_root(); }

private:
  std::unordered_map<std::string, TfFrame> frames_;  //!< Map to store frames.
};
}  // namespace awviz_common

#endif  // AWVIZ_COMMON__TF_TREE_HPP_
