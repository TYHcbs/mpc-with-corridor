#!/usr/bin/env bash

roslaunch trajectory_generator demo.launch

rosbag record -a -O my_bag_4.bag
