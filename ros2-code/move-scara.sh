#!/bin/sh

ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [$1, $2]}"