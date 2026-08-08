#!/bin/bash

echo "***************"
echo "delete the remap device serial port to 99-lidar"
sudo rm   /etc/udev/rules.d/99-lidar.rules
echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "finish"
echo "***************"
