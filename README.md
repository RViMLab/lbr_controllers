# RViM ROS2 Controllers Experimental

## Build this Repository
Clone all dependencies
```shell
mkdir -p rvim_ros2_controllers_experimental_ws/src && \
wget https://raw.githubusercontent.com/RViMLab/rvim_ros2_controllers_experimental/foxy/rvim_ros2_controllers_experimental/repos.yml -P rvim_ros2_controllers_experimental_ws/src && \
vcs import rvim_ros2_controllers_experimental_ws/src < rvim_ros2_controllers_experimental_ws/src/repos.yml
```
Build the controllers
```shell
source /opt/ros/foxy/setup.bash && \
cd rvim_ros2_controllers_experimental_ws && \
colcon build --allow-overriding qpoases_vendor
```

## Example Usage

```shell
source install/setup.bash --extend
ros2 launch lbr_bringup lbr_bringup.launch.py model:=med7 controller_configurations_package:=rvim_position_controllers sim:=false controller:=hand_guide_position_controller controller_configurations:=config/sample_config.yml
```

## Write a new controller
https://control.ros.org/foxy/doc/ros2_controllers/doc/writing_new_controller.html
