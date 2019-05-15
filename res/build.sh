#!/usr/bin/env bash

. /opt/ros/kinetic/setup.bash

catkin_make && catkin_make install && catkin_make tests

exit $?
