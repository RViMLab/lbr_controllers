# LBR Controllers

## Build this Repository
Clone all dependencies
```shell
mkdir -p lbr_controllers_ws/src && \
wget https://raw.githubusercontent.com/RViMLab/lbr_controllers/humble/lbr_controllers/repos.yml -P lbr_controllers_ws/src && \
vcs import lbr_controllers_ws/src < lbr_controllers_ws/src/repos.yml
```
Build the controllers
```shell
source /opt/ros/humble/setup.bash && \
cd lbr_controllers_ws && \
colcon build
```

## Example Usage

```shell
source install/setup.bash
ros2 launch lbr_bringup lbr_bringup.launch.py model:=med7 sim:=false controller_package:=lbr_velocity_controllers controller_file:=config/sample_config.yml controller:=lbr_velocity_controller
```

## Write a new controller
https://control.ros.org/master/doc/ros2_controllers/doc/writing_new_controller.html
