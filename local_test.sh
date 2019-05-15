#!/usr/bin/env bash

source install/setup.bash

catkin_make run_tests && catkin_test_results --all

exit $?
