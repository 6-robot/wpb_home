#!/bin/bash

echo "***************"
echo "delete the remap device serial port to ftdi"
sudo rm   /etc/udev/rules.d/ftdi.rules
echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "finish"
echo "***************"
