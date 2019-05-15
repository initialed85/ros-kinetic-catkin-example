#!/usr/bin/env bash

catkin_make && catkin_make install && catkin_make tests

exit $?