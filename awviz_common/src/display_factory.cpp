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

namespace awviz_common
{
DisplayFactory::DisplayFactory() : PluginFactory<Display>("awviz_common", "awviz_common::Display")
{
  for (const auto & lookup_name : get_declared_classes()) {
    auto xml_path = get_plugin_manifest_path(lookup_name);
    // Initialize cache as empty.
    msg_type_buffer_[lookup_name] = std::set<std::string>();
    if (!xml_path.empty()) {
      tinyxml2::XMLDocument document;
      document.LoadFile(xml_path.c_str());
      auto * config = document.RootElement();
      if (!has_root_node(config) || !has_library_root(config)) {
        continue;
      }

      auto * library = config;
      while (library) {
        cache_classes(library);
        library = library->NextSiblingElement(LIBRARY_TAG);
      }
    }
  }
}

const std::set<std::string> & DisplayFactory::get_message_types(const std::string & lookup_name)
{
  if (msg_type_buffer_.find(lookup_name) != msg_type_buffer_.cend()) {
    return msg_type_buffer_[lookup_name];
  }
  // Initialize cache as empty.
  msg_type_buffer_[lookup_name] = std::set<std::string>();
  return msg_type_buffer_[lookup_name];
}

std::optional<std::string> DisplayFactory::get_class_lookup_name(const std::string & msg_type) const
{
  for (const auto & [class_id, msg_types] : msg_type_buffer_) {
    if (msg_types.find(msg_type) != msg_types.cend()) {
      return class_id;
    }
  }
  return {};
}

bool DisplayFactory::has_root_node(const tinyxml2::XMLElement * element) const
{
  return element != nullptr;
}

bool DisplayFactory::has_library_root(const tinyxml2::XMLElement * element) const
{
  return strcmp(element->Value(), LIBRARY_TAG) == 0;
}

void DisplayFactory::cache_classes(const tinyxml2::XMLElement * library)
{
  auto element = library->FirstChildElement();
  while (element) {
    const auto derived = lookup_derived_class(element);
    const auto current = lookup_class_id(element, derived);

    msg_type_buffer_[current] = parse_message_types(element);

    // search child element
    element = element->NextSiblingElement(CLASS_TAG);
  }
}

std::set<std::string> DisplayFactory::parse_message_types(
  const tinyxml2::XMLElement * element) const
{
  std::set<std::string> output;

  auto msg_type = element->FirstChildElement(MESSAGE_TYPE_TAG);

  while (msg_type) {
    if (msg_type->GetText()) {
      const auto msg_type_name = msg_type->GetText();
      output.insert(msg_type_name);
    }

    // search child element
    msg_type = msg_type->NextSiblingElement(MESSAGE_TYPE_TAG);
  }

  return output;
}

std::string DisplayFactory::lookup_derived_class(const tinyxml2::XMLElement * element) const
{
  return element->Attribute(TYPE_ATTRIBUTE) ? element->Attribute(TYPE_ATTRIBUTE) : "";
}

std::string DisplayFactory::lookup_class_id(
  const tinyxml2::XMLElement * element, const std::string & derived) const
{
  return element->Attribute(NAME_ATTRIBUTE) ? element->Attribute(NAME_ATTRIBUTE) : derived;
}

}  // namespace awviz_common
