#!/bin/bash

echo "***************"
echo "remap the device serial port(ttyUSBX) to ftdi"
echo "start copy ftdi.rules to  /etc/udev/rules.d/"
sudo cp `rospack find wpb_home_bringup`/scripts/ftdi.rules  /etc/udev/rules.d
sudo cp ftdi.rules  /etc/udev/rules.d

echo "remap the device serial port(ttyUSBX) to rplidar"
echo "start copy rplidar.rules to  /etc/udev/rules.d/"
sudo cp `rospack find wpb_home_bringup`/scripts/rplidar.rules  /etc/udev/rules.d
sudo cp rplidar.rules  /etc/udev/rules.d

echo "set kinect2 rules"
echo "start copy 90-kinect2.rules to  /etc/udev/rules.d/"
sudo cp `rospack find wpb_home_bringup`/scripts/90-kinect2.rules  /etc/udev/rules.d
sudo cp 90-kinect2.rules  /etc/udev/rules.d

echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "finish"
echo "***************"
