#!/usr/bin/env bash

. install/setup.bash

catkin_make run_tests && catkin_test_results

RETURN_CODE=$?

cp -frv build/test_results/*/*.xml /srv/test_results

exit ${RETURN_CODE}
