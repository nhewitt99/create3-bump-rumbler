# create3-bump-rumbler
Simple ros2 package to give controller vibration feedback when Create3 experiences a bump.

Currently tested on Humble with Ubuntu 22.04 and a Steam Controller using [sc-controller](https://github.com/kozec/sc-controller) to emulate XBox input. Windows / Xinput support will not be added.

This simple package contains a node that listens to the `hazard_detection` topic published by the Create3's ROS2 API and causes controller vibration if a bump is detected.

## Requirements
- irobot-create-msgs
- evdev (tested with 1.6.1)
- A working ROS2 Humble install and associated goodies
- A working controller configured to expose events at `/dev/input`

## Installation
Clone this repository to your workspace `src` folder. Build with colcon as usual.

## Running
Bringup with:
```
ros2 run rumble rumble_node
```

## Steam Controller Advice
Mostly recording this here so I don't forget haha. To use the Steam controller natively on Ubuntu, I use sc-controller. This can then interface with the basic teleop package by setting launch parameters appropriately:
```
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox' joy_dev:='dev/input/js1'
```
