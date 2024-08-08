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

#include "awviz_common/display_factory.hpp"

#include "awviz_common/display.hpp"
#include "awviz_common/factory/plugin_factory.hpp"

#include <tinyxml2.h>

#include <set>
#include <string>

namespace awviz_common
{
DisplayFactory::DisplayFactory() : PluginFactory<Display>("awviz_common", "awviz_common::Display")
{
}

const std::set<std::string> & DisplayFactory::getMessageTypes(const std::string & class_id)
{
  if (msg_type_buffer_.find(class_id) != msg_type_buffer_.cend()) {
    return msg_type_buffer_[class_id];
  }

  // Initialize cache as empty.
  msg_type_buffer_[class_id] = std::set<std::string>();

  auto xml_path = getPluginManifestPath(class_id);

  if (!xml_path.empty()) {
    tinyxml2::XMLDocument document;
    document.LoadFile(xml_path.c_str());
    auto * config = document.RootElement();
    if (!hasRootNode(config) || !hasLibraryRoot(config)) {
      return msg_type_buffer_[class_id];
    }

    if (config->Value() == CLASS_LIBRARIES_KEY) {
      config = config->FirstChildElement(LIBRARY_KEY);
    }

    auto * library = config;
    while (library) {
      cacheAllClassElements(library);
      library = library->NextSiblingElement(LIBRARY_KEY);
    }
  }

  return msg_type_buffer_[class_id];
}

bool DisplayFactory::hasRootNode(const tinyxml2::XMLElement * element) const
{
  return element != nullptr;
}

bool DisplayFactory::hasLibraryRoot(const tinyxml2::XMLElement * element) const
{
  return element->Value() == LIBRARY_KEY || element->Value() == CLASS_LIBRARIES_KEY;
}

void DisplayFactory::cacheAllClassElements(const tinyxml2::XMLElement * library)
{
  auto element = library->FirstChildElement();
  while (element) {
    const auto derived = lookupDerivedClass(element);
    const auto current = lookupClassId(element, derived);

    msg_type_buffer_[current] = parseMsgTypes(element);

    // search child element
    element = element->NextSiblingElement(CLASS_KEY);
  }
}

std::set<std::string> DisplayFactory::parseMsgTypes(const tinyxml2::XMLElement * element) const
{
  std::set<std::string> output;

  auto msg_type = element->FirstChildElement(MSG_TYPE_KEY);

  while (msg_type) {
    if (msg_type->GetText()) {
      const auto msg_type_name = msg_type->GetText();
      output.insert(msg_type_name);
    }

    // search child element
    msg_type = msg_type->NextSiblingElement(MSG_TYPE_KEY);
  }

  return output;
}

std::string DisplayFactory::lookupDerivedClass(const tinyxml2::XMLElement * element) const
{
  return element->Attribute(TYPE_KEY) ? element->Attribute(TYPE_KEY) : "";
}

std::string DisplayFactory::lookupClassId(
  const tinyxml2::XMLElement * element, const std::string & derived) const
{
  return element->Attribute(NAME_KEY) ? element->Attribute(NAME_KEY) : derived;
}

}  // namespace awviz_common
