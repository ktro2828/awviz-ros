# awviz-cpp

`awviz` offers a ROS viewer of [Autoware](https://github.com/autowarefoundation/autoware) powered by [rerun](https://github.com/rerun-io/rerun).

## ROS msg Support

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
| `DetectedObjects`  |         |
|  `TrackedObjects`  |         |
| `PredictedObjects` |         |
