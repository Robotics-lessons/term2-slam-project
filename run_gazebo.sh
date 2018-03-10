#!/bin/bash

sudo killall rosmaster
sudo killall gzserver
sudo killall gzclient
echo $1
roslaunch slam_project $1
