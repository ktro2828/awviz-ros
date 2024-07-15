# awviz-cpp

`awviz` offers a ROS viewer of [Autoware](https://github.com/autowarefoundation/autoware) powered by [Rerun](https://github.com/rerun-io/rerun).

## Build

```shell
git clone git@github.com:ktro2828/awviz-cpp && cd awviz-cpp

# import dependencies
mkdir src && vcs import src < depends.repos
rosdep update && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# build awviz
colon build --symlink-install
source install/setup.bash
```

## Run

```shell
ros2 launch awviz awviz.launch.xml
```

## Configuration

Sample topic configuration is following:

```yaml
/**:
  ros__parameters:
    topic_names:
      - xxx

    topic_options:
      xxx:
        type: PointCloud
        entity: /topics/xxx/pointcloud
```

For details, please refer to [awviz/config/awviz.param.yaml](./awviz/config/awviz.param.yaml).

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
