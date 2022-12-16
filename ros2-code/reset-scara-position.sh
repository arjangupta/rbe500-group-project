#!/bin/sh

ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ["forward_position_controller"], deactivate_controllers: [forward_effort_controller, forward_velocity_controller]}"
./position-move-scara.sh 0.261799 2.61799 1.0