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

#ifndef AWVIZ_COMMON__DISPLAY_FACTORY_HPP_
#define AWVIZ_COMMON__DISPLAY_FACTORY_HPP_

#include "awviz_common/display.hpp"
#include "awviz_common/factory/plugin_factory.hpp"

#include <tinyxml2.h>

#include <map>
#include <optional>
#include <set>
#include <string>

namespace awviz_common
{

/**
 * @brief Factory class for Display.
 */
class DisplayFactory : public PluginFactory<Display>
{
public:
  DisplayFactory();

  ~DisplayFactory() = default;

  /**
   * @brief Get the set of declared message types.
   *
   * @param lookup_name Lookup name of the class.
   * @return Set of declared message types.
   */
  const std::set<std::string> & get_message_types(const std::string & lookup_name);

  /**
   * @brief Get the Class Id of the corresponding message type.
   *
   * @param msg_type ROS message type.
   * @return Return a string of lookup name if the specified type has been declared, otherwise
   * returns `nullptr`.
   */
  std::optional<std::string> get_class_lookup_name(const std::string & msg_type) const;

public:
  static constexpr const char * LIBRARY_TAG = "library";            //!< XML tag of library.
  static constexpr const char * CLASS_TAG = "class";                //!< XML tag of class.
  static constexpr const char * MESSAGE_TYPE_TAG = "message_type";  //!< XML tag of message type.
  static constexpr const char * TYPE_ATTRIBUTE = "type";            //!< XML attribute of type.
  static constexpr const char * NAME_ATTRIBUTE = "name";            //!< XML attribute of name.

private:
  /**
   * @brief Check whether xml element has a root node.
   * @param element Parsed XML element.
   * @return Return true if the element has the root node.
   */
  bool has_root_node(const tinyxml2::XMLElement * element) const;

  /**
   * @brief Check whether xml element has a library root.
   * @param element Parsed XML element.
   * @return Return true if the element has the library root.
   */
  bool has_library_root(const tinyxml2::XMLElement * element) const;

  /**
   * @brief Cache all classes associated with the library.
   * @param library Parsed XML element for library.
   */
  void cache_classes(const tinyxml2::XMLElement * library);

  /**
   * @brief Parse ROS message types from XML elements.
   * @param element Root XML element.
   * @return Set of message types.
   */
  std::set<std::string> parse_message_types(const tinyxml2::XMLElement * element) const;

  /**
   * @brief Lookup derived class name.
   * @param element Parsed XML element.
   * @return Class name if the element has a `type` key, otherwise returns empty string.
   */
  std::string lookup_derived_class(const tinyxml2::XMLElement * element) const;

  /**
   * @brief Lookup the class id.
   * @param element Parsed XML element.
   * @param derived Name of derived class.
   * @return Class id if the element has `name` key, otherwise returns derived name.
   */
  std::string lookup_class_id(
    const tinyxml2::XMLElement * element, const std::string & derived) const;

private:
  std::map<std::string, std::set<std::string>>
    msg_type_buffer_;  //!< Buffer storing declared classes, which key is class id and values are
                       //!< message types.
};
}  // namespace awviz_common
#endif  // AWVIZ_COMMON__DISPLAY_FACTORY_HPP_
