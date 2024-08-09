# AWViz

AWViz offers a 3D ROS viewer for [Autoware](https://autoware.org) powered by [Rerun](https://rerun.io).

## Build & Run

### Build

```shell
# download repository
git clone git@github.com:ktro2828/awviz-ros && cd awviz-ros

# build awviz
colon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Run

```shell
# run awviz
ros2 run awviz awviz
```
