#!/bin/bash

# source the catkin workspace
if [ -d "../../devel" ]; then
  source ../../devel/setup.bash
else
  source ../../../devel/setup.bash
fi

# ensure realtimeness
# (you might have to set permissions for your user in /etc/security/limits.conf)
ulimit -S -c unlimited
ulimit -r 99

# check if a roscore is running
python configuration/check_roscore.py

if [ $? -ne 0 ]
then
  echo "[Error] No roscore active."
  exit 1
fi
 


echo "Starting OROCOS Deployer"
rttlua -i scripts/deploy/my_deploy.lua

 
