#!/bin/bash

echo "***************"
echo "delete the remap device serial port to ftdi"
sudo rm   /etc/udev/rules.d/ftdi.rules
echo "delete the remap device serial port to rplidar"
sudo rm   /etc/udev/rules.d/rplidar.rules
echo "delete the kinect2 rules"
sudo rm   /etc/udev/rules.d/90-kinect2.rules
echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "finish"
echo "***************"
