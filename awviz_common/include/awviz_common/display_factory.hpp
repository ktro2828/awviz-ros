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

  virtual const std::set<std::string> & getMessageTypes(const std::string & class_id);

public:
  static constexpr const char * LIBRARY_KEY = "library";
  static constexpr const char * CLASS_LIBRARIES_KEY = "class_libraries";
  static constexpr const char * CLASS_KEY = "class";
  static constexpr const char * MSG_TYPE_KEY = "message_type";
  static constexpr const char * TYPE_KEY = "type";
  static constexpr const char * NAME_KEY = "name";

private:
  // Buffer storing ROS message types.
  std::map<std::string, std::set<std::string>> msg_type_buffer_;

private:
  /**
   * @brief Check whether xml element has a root node.
   * @param element Parsed XML element.
   * @return Return true if the element has the root node.
   */
  bool hasRootNode(const tinyxml2::XMLElement * element) const;

  /**
   * @brief Check whether xml element has a library root.
   * @param element Parsed XML element.
   * @return Return true if the element has the library root.
   */
  bool hasLibraryRoot(const tinyxml2::XMLElement * element) const;

  /**
   * @brief Cache all classes associated with the library.
   * @param library Parsed XML element for library.
   */
  void cacheAllClassElements(const tinyxml2::XMLElement * library);

  /**
   * @brief Parse ROS message types from XML elements.
   * @param element Root XML element.
   */
  std::set<std::string> parseMsgTypes(const tinyxml2::XMLElement * element) const;

  /**
   * @brief Lookup derived class name.
   * @param element Parsed XML element.
   * @param Return the class name if the element has a `type` key, otherwise returns empty string.
   */
  std::string lookupDerivedClass(const tinyxml2::XMLElement * element) const;

  /**
   * @brief Lookup the class id.
   * @param element Parsed XML element.
   * @param derived Name of derived class.
   * @return Return the class id if the element has `name` key, otherwise returns derived name.
   */
  std::string lookupClassId(
    const tinyxml2::XMLElement * element, const std::string & derived) const;
};
}  // namespace awviz_common
#endif  // AWVIZ_COMMON__DISPLAY_FACTORY_HPP_
