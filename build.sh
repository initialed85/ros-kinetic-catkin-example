#!/usr/bin/env bash

TAG=ros-kinetic-catkin-example

if [ "$1" == "force" ]; then
    docker build -t ${TAG} --no-cache .
else
    docker build -t ${TAG} .
fi

docker run -d --name ${TAG} ${TAG}

docker exec -t ${TAG} ./build.sh
