#!/usr/bin/env bash

TAG=ros-kinetic-catkin-example

docker ps | grep ${TAG}

if [ $? -ne 0 ]; then
    ./build.sh
fi

docker exec -t ${TAG} ./test.sh
RETURN_CODE=$?

rm -fr test_results

docker cp ${TAG}:/srv/test_results test_results

docker rm -f ${TAG}

exit ${RETURN_CODE}
