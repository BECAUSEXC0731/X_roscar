#!/bin/bash

echo "***************"

echo "remap the device serial port(ttyACM0) to lidar"
echo "start copy 99-lidar.rules to  /etc/udev/rules.d/"
sudo cp ./99-lidar.rules  /etc/udev/rules.d

echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "finish"
echo "***************"
