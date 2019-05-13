#!/usr/bin/env bash

TAG=ros-kinetic-catkin-example

docker build -t ${TAG} .

docker run -d --name ${TAG} ${TAG}

docker exec -t ${TAG} ./build.sh
