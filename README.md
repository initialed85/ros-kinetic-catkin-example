# ros-kinetic-catkin-example
This repo is a `ros-kinetic` workspace with a package called `example` for the purposes of demonstrating `rostest`.

## What's in it?

The package is very contrived, it's comprised of the following:

* Messages
    * `ExampleMessage`
* Services
    * `ExampleService`
* Nodes
    * Relevant
        * `rtps_topic_publisher.cpp` (C++)
            * Publishes an RTPS `ExampleMessage` message to RTPS `ExamplePubSubTopic` topic with some fixed and random properties
        * `rtps_to_ros_gateway.cpp` (C++)
            * Receives RTPS `ExampleMessage` messages from RTPS `ExamplePubSubTopic` topic and publishes a ROS `ExampleMessage` message to ROS `topic` topic
        * `topic_subscriber.py` (Python)
            * Receives and logs ROS `ExampleMessage` messages from ROS `topic` topic
        * `service_hoster.py` (Python)
            * Receives ROS `ExampleMessage` messages from ROS `topic` topic and catalogues them by `first_name` and `last_name`, exposing that catalogue via ROS `ExampleService` service
        * `service_caller.py` (Python)
            * Calls the ROS `ExampleService` service with the `first_name` and `last_name` provided on the command line
    * Deprecated (context only)
        * `topic_publisher.py` (Python)
            * Publishes a ROS `ExampleMessage` message to ROS `topic` topic with some fixed and random properties
* Tests
    * `ros_kinetic_catkin_example_test.cpp` (C++)
        * Instantiates `rtps_topic_publisher.cpp`, `rtps_to_ros_gateway.cpp` and `service_hoster.py` and validates their behaviour (using C++ `gtest`)
    * `ros_kinetic_catkin_example_test.py` (Python)
        * Instantiates `rtps_topic_publisher.cpp`, `rtps_to_ros_gateway.cpp` and `service_hoster.py` and validates their behaviour (using Python `unittest`)

## How do I use it?

Before doing anything, ensure you have `ros-kinetic` installed and you're in the root of the repo, then run the following:

        source /opt/ros/kinetic/setup.bash
        catkin_make
        source devel/setup.bash

It should be as simple as the following (from the root of the repo):

        # to run the C++ tests
        rostest example ros_kinetic_catkin_example_test_cpp.launch

        # to run the Python tests
        rostest example ros_kinetic_catkin_example_test_python.launch

If you want to use `catkin` to run the tests: 

        catkin_make
        catkin_make run_tests
