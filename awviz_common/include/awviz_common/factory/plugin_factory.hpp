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

#ifndef AWVIZ_COMMON__PLUGIN_FACTORY_HPP_
#define AWVIZ_COMMON__PLUGIN_FACTORY_HPP_

#include <pluginlib/class_loader.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace awviz_common
{
/**
 * @brief Struct to bundle the information of plugin.
 */
struct PluginInfo
{
  std::string id;           //!< Class id.
  std::string name;         //!< Name of class.
  std::string type;         //!< Type of class.
  std::string package;      //!< Package name.
  std::string description;  //!< Description of class.
};

/**
 * @brief Abstract base class representing a plugin load-able class factory.
 */
template <typename T>
class PluginFactory
{
public:
  /**
   * @brief Construct factory loading `pluginlib::ClassLoader<T>`.
   * @param package Name of package.
   * @param base_class_type A string with the fully qualified base class type for the plugin, i.e.
   * `NAMESPACE::PLUGIN`.
   */
  PluginFactory(const std::string & package, const std::string & base_class_type)
  {
    class_loader_ = std::make_shared<pluginlib::ClassLoader<T>>(package, base_class_type);
  }

  /**
   * @brief Return plugin manifest path.
   * @param lookup_name Lookup name of the class.
   * @return The path of the associated plugin manifest.
   */
  std::string getPluginManifestPath(const std::string & lookup_name) const
  {
    return class_loader_->getPluginManifestPath(lookup_name);
  }

  /**
   * @brief Return plugin information.
   * @param lookup_name Lookup name of the class.
   * @return PluginInfo instance.
   */
  PluginInfo getPluginInfo(const std::string & lookup_name) const
  {
    auto name = class_loader_->getName(lookup_name);
    auto type = class_loader_->getClassType(lookup_name);
    auto package = class_loader_->getClassPackage(lookup_name);
    auto description = class_loader_->getClassDescription(lookup_name);
    return {lookup_name, name, type, package, description};
  }

  /**
   * @brief Create a instance of the plugin.
   * @param lookup_name Lookup name of the class.
   * @return Instance of plugin. Returns `nullptr` if exception occurred.
   */
  virtual std::shared_ptr<T> createInstance(const std::string & lookup_name) const
  {
    try {
      return class_loader_->createSharedInstance(lookup_name);
    } catch (pluginlib::PluginlibException & ex) {
      std::cerr << "[PluginlibException]: " << ex.what() << std::endl;
      return nullptr;
    }
  }

  /**
   * @brief Return a list of all available classes for this ClassLoader's base class type.
   *
   * @return std::vector<std::string>
   */
  std::vector<std::string> getDeclaredClasses() const
  {
    return class_loader_->getDeclaredClasses();
  }

private:
  std::shared_ptr<pluginlib::ClassLoader<T>> class_loader_;  //!< ClassLoader object.
};
}  // namespace awviz_common
#endif  // AWVIZ_COMMON__PLUGIN_FACTORY_HPP_
