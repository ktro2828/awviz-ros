# awviz-cpp

`awviz` offers a ROS viewer of [Autoware](https://github.com/autowarefoundation/autoware) powered by [rerun](https://github.com/rerun-io/rerun).

## Build

```shell
git clone git@github.com:ktro2828/awviz-cpp
cd awviz-cpp
mkdir src && vcs import src < depends.repos
rospde update && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colon build --symlink-install
source install/setup.bash
```

## Run

```shell
ros2 launch awviz awviz.launch.xml
```

## ROS msg support

### `common_interfaces`

#### `sensor_msgs`

|       Type        | Support |
| :---------------: | :-----: |
|   `PointCloud2`   |   ✅    |
|      `Image`      |   ✅    |
| `CompressedImage` |   ✅    |
|   `CameraInfo`    |         |

### `autoware_msgs`

#### `autoware_perception_msgs`

|        Type        | Support |
| :----------------: | :-----: |
| `DetectedObjects`  |   ✅    |
|  `TrackedObjects`  |         |
| `PredictedObjects` |         |
