#!/bin/bash

echo "***************"
echo "remap the device serial port(ttyUSBX) to ftdi"
echo "start copy ftdi.rules to  /etc/udev/rules.d/"
sudo cp `rospack find wpb_home_bringup`/scripts/ftdi.rules  /etc/udev/rules.d
echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "finish"
echo "***************"
