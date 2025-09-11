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
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace awviz_common
{

constexpr const char * TF_ROOT = "map";  //!< Root transformation frame.

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
   * @note Currently, this returns true if the parent id is not `TF_ROOT` or is empty.
   *
   * @return Return true if the parent id is not `TF_ROOT` or is empty.
   */
  bool is_static() const { return std::strcmp(parent_.c_str(), TF_ROOT) != 0 || parent_.empty(); }

  /**
   * @brief Compare with an another object.
   *
   * @param other Another object.
   * @return Return true if both id and parent are the same.
   */
  bool operator==(const TfFrame & other) const
  {
    return id_ == other.id() && parent_ == other.parent();
  }

private:
  std::string id_;      //!< Frame ID.
  std::string parent_;  //!< Parent frame ID.
};

class TfTree
{
public:
  /**
   * @brief Construct a object setting `TF_ROOT` as root.
   */
  TfTree() : frames_() { frames_.emplace(TF_ROOT, TF_ROOT); }

  /**
   * @brief Add a new tf frame to the tree.
   *
   * @param frame A new tf frame. If it has been already registered, skip adding.
   */
  void emplace(const TfFrame & frame) { frames_.emplace(frame.id(), frame); }

  /**
   * @brief Add a new tf frame to the tree.
   *
   * @param id Frame ID. If it has been already registered, skip adding.
   * @param parent Parent frame ID.
   */
  void emplace(const std::string & id, const std::string & parent)
  {
    frames_.emplace(id, TfFrame(id, parent));
  }

  /**
   * @brief Add a new tf frame to the tree.
   *
   * @param id Frame ID. If it has been already registered, skip adding.
   * @param parent Parent frame ID.
   */
  void emplace(std::string && id, std::string && parent)
  {
    auto key = id;
    frames_.emplace(std::move(key), TfFrame(std::move(id), std::move(parent)));
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
   * @brief Whether to the specified frame ID is contained as a node of tree.
   *
   * @param id Frame ID.
   * @return Returns true, if the frame is contained.
   */
  bool contains(const std::string & id) const { return frames_.count(id) > 0; }

  /**
   * @brief Whether to the specified frame is contained in the tree checking both key and value.
   *
   * @param frame `TfFrame` object.
   * @return true Return true if the tree contains the ID of `frame` as a key, and then if its value
   * is equivalent to `frame`.
   */
  bool contains(const TfFrame & frame) const
  {
    return contains(frame.id()) && frame == frames_.at(frame.id());
  }

  /**
   * @brief Return entity path of the specified frame. The entity path will be in the format
   * `"/<Parent0>/<Parent1>/.../<FrameID>"`, where `"<Parent0>"` represents the deepest parent
   * frame.
   *
   * @param frame `TfFrame` Object.
   * @return Entity path of the frame.
   */
  std::string entity_path(const TfFrame & frame) const
  {
    auto current = std::make_optional<TfFrame>(frame);
    std::string entity = "/" + current->id();
    while (current && !current->is_root()) {
      current = get_frame(current->parent());
      if (current) {
        entity = "/" + current->id() + entity;
      } else {
        break;
      }
    }
    return entity;
  }

  /**
   * @brief Check whether the input frame is linked to the input `id`.
   *
   * @param frame `TfFrame` object.
   * @param id Frame ID.
   * @return Return true if the input frame can link to `id`.
   */
  bool can_link_to(const TfFrame & frame, const std::string & id) const
  {
    auto current = std::make_optional<TfFrame>(frame);
    while (current && !current->is_root()) {
      current = get_frame(current->parent());
    }
    return current->id() == id;
  }

private:
  std::unordered_map<std::string, TfFrame> frames_;  //!< Map to store frames.
};
}  // namespace awviz_common

#endif  // AWVIZ_COMMON__TF_TREE_HPP_
