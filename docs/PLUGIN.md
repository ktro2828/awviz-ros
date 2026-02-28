# Plugin Support & Development

Display plugins are responsible for subscribing to ROS messages and logging them in the recording stream.

## Display Lifecycle (Plugin Developers)

A display instance follows a simple lifecycle. Implementations should assume the following order:

1. **Created**: Constructor runs. Do not access ROS or Rerun resources here.
2. **Initialized**: `initialize(node, stream)` is called. You may store the node/stream.
3. **Configured**: `set_property(topic, entity_roots)` is called. Topic/entity settings are ready.
4. **Running**: `start()` is called. Subscriptions should be created here.
5. **Stopped (back to Configured)**: `end()` is called. Subscriptions should be released, but the display remains configured and may be started again.

### Key Rules

- `start()` is only called after `initialize()` and `set_property()` have completed.
- `log_message()` is invoked only while running.
- `end()` should be safe to call even if no subscription exists.

### Recommended Pattern

- Keep heavy initialization out of the constructor.
- Validate entity paths inside `log_message()` and log a warning if missing.

## ROS Message Support

`awviz_plugin` exports built-in display plugins.
Even if there are ROS messages that are not supported by built-in plugins, we can define custom plugins.

### ROS Built-in Messages

| ROS Package   | Message Type      | Plugin Type                            |
| :------------ | :---------------- | :------------------------------------- |
| `sensor_msgs` | `CameraInfo`      | `awviz_plugin::CameraInfoDisplay`      |
|               | `CompressedImage` | `awviz_plugin::CompressedImageDisplay` |
|               | `Image`           | `awviz_plugin::ImageDisplay`           |
|               | `PointCloud2`     | `awviz_plugin::PointCloud2Display`     |
|               | `NavSatFix`       | `awviz_plugin::NavSatFixDisplay`       |

### Autoware Messages

| ROS Package                | Message Type       | Plugin Type                             |
| :------------------------- | :----------------- | :-------------------------------------- |
| `autoware_map_msgs`        | `LaneletMapBin`    | `awviz_plugin::LaneletDisplay`          |
| `autoware_perception_msgs` | `DetectedObjects`  | `awviz_plugin::DetectedObjectsDisplay`  |
|                            | `TrackedObjects`   | `awviz_plugin::TrackedObjectsDisplay`   |
|                            | `PredictedObjects` | `awviz_plugin::PredictedObjectsDisplay` |

## Custom Plugin Definition

`awviz` allows us to define custom display plugins powered by `pluginlib`.  
`pluginlib` is a C++ library for loading and unloading plugins from within a ROS package.
Plugins are dynamically loadable classes that are loaded from a runtime library.

### Example

As an example, let's create a plugin to display a custom ROS message `my_custom_msgs::msg::Foo` as below:

```msg
builtin_interfaces/Time stamp
float32 data
```

#### 1. Package Configuration

Then create your ROS packages named `my_custom_plugin`.  
The package structure is as follows:

```bash
my_custom_plugin
├── CMakeLists.txt
├── include
│   └── my_custom_plugin
│       └── foo_display.hpp
├── package.xml
├── plugin_description.xml
└── src
    └── foo_display.cpp
```

#### 2. Plugin Implementation

Edit `include/my_custom_plugin/foo_display.hpp` and declare the inheritance of `awviz_common::RosTopicDisplay`:

```cpp
#ifndef MY_CUSTOM_PLUGIN__FOO_DISPLAY_HPP_
#define MY_CUSTOM_PLUGIN__FOO_DISPLAY_HPP_

#include <awviz_common/display.hpp>
#include <my_custom_msgs/msg/foo.hpp>

namespace my_custom_plugin
{
    class FooDisplay : public awviz_common::RosTopicDisplay<my_custom_msgs::msg::Foo>
    {
    public:
        // Construct object.
        FooDisplay();

    protected:
        // Callback to log subscribed message to the recording stream.
        void log_message(my_custom_msgs::msg::Foo::ConstSharedPtr msg) override;
    };
}  // namespace my_custom_plugin

#endif  // MY_CUSTOM_PLUGIN__FOO_DISPLAY_HPP_
```

Edit `src/foo_display.cpp` and write implementations:

```cpp
#include "my_custom_plugin/foo_display.hpp"

namespace my_custom_plugin
{
FooDisplay::FooDisplay() : awviz_common::RosTopicDisplay<my_custom_msgs::msg::Foo>()
{
}

void FooDisplay::log_message(my_custom_msgs::msg::Foo::ConstSharedPtr msg)
{
    log_timestamp(rclcpp::Time(msg->stamp.sec, msg->stamp.nanosec));
    const auto entity_path = resolve_entity_path(msg->header.frame_id);
    stream_->log(property_.entity(), rerun::Scalar(msg->data));
}
}  // namespace my_custom_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(my_custom_plugin::FooDisplay, awviz_common::Display);
```

#### 3. Plugin Declaration XML

Edit `plugin_description.xml` with the following code:

```xml
<library path="my_custom_plugin">
    <class name="my_custom_plugin/FooDisplay" type="my_custom_display::FooDisplay"
        base_class_type="awviz_common::Display">
        <description>Some description of the plugin.</description>
        <message_type>my_custom_msgs/msg/FooDisplay</message_type>
    </class>
</library>
```

XML tag and attribute represent followings:

- `library`: The Name of the library.
- `class`: A plugin declaration that we want to export.
  - `name`: There used to be a name attribute, but it is no longer required.
  - `type`: The fully qualified type of the plugin. Now, that is `my_custom_plugin::FooDisplay`.
  - `base_class_type`: The fully qualified base class type for the plugin. Use `awviz_common::Display`.
  - `description`: A description of the plugin and what it does.
  - `message_type`: A ROS message type that the plugin displays. Now, that is `my_custom_msgs/msg/Foo`.

#### 4. CMake Plugin Declaration

Edit `CMakeLists.txt` to export the plugin and its description:

```cmake
cmake_minimum_required(VERSION 3.14)
project(my_custom_plugin)

# -------- find dependencies --------
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# -------- link targets --------
add_library(${PROJECT_NAME} SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/foo.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} awviz_common::awviz_common rerun_sdk)
ament_target_dependencies(${PROJECT_NAME} rclcpp my_custom_msgs)

# -------- export plugin description --------
pluginlib_export_plugin_description_file(awviz_common plugin_description.xml)

# -------- export targets --------
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME}Export)

ament_auto_package()
```
