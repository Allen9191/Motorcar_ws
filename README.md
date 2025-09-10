# AMR Motor Control

## Description
AMR Lower Control 

Input: joy, Keyboard joy

Controller: ZLAC8015d

output: RS485 signal

## Relative prolem solution
At aarch64 snap version:
>[Chromium, other browsers not working after flashing or updating](https://forums.developer.nvidia.com/t/chromium-other-browsers-not-working-after-flashing-or-updating-heres-why-and-quick-fix/338891)

sudo apt-get install chromium-browser
Install joystick test
>[測試搖桿工具](https://shengyu7697.github.io/ubuntu-joystick-tool/)
Joysitck Driver:
>[Jetpack 6 Joystick Detection Issue on Nvidia Jetson Orin Nano Dev Board](https://nvidia-jetson.piveral.com/jetson-orin-nano/jetpack-6-joystick-detection-issue-on-nvidia-jetson-orin-nano-dev-board/)



>github:
  [jetpack6-joysitck manual install](https://github.com/woawo1213/jetpack6-joy)

## Bring up step

### I/O port test
joystick connect port:
>GUI test:
>>jstest-gtk

>>lsusb
    
### Test alone
cd Motorcar_ws/

source install/setup.bash

ros2 launch keyboard_joy keyboard_joy.launch.py

ros2 run joy joy_node --ros-args -p dev:="/dev/input/js0" -p deadzone:=0.05 -p autorepeat_rate:=0.0


ros2 run zlac8015d_serial Modectl

ros2 run zlac8015d_serial Motorctl

### Launch
cd Motorcar_ws/

source install/setup.bash

Motor control:

>ros2 launch zlac8015d_serial serial_test.launch.py


>ros2 launch zlac8015d_serial joyCH_motor.launch.py

>ros2 launch zlac8015d_serial joyCH_motor.launch.py dev:=/dev/input/js0 port_f:=/dev/ttyUSB2 port_r:=/dev/ttyUSB3

TF Displayer:

>ros2 launch amr_description display.launch.py

>rviz2
