# ros-kinetic-catkin-example
This repo is a `ros-kinetic` workspace with a package called `example` for the purposes of demonstrating `rostest`.

## What's in it?

The package is very contrived, it's comprised of the following:

* Messages
    * `ExampleMessage`
* Services
    * `ExampleService`
* Nodes
    * `topic_publisher.py`
        * Publishes an `ExampleMessage` to `topic` with some fixed and random properties (Python version)
    * `topic_publisher.cpp
        * Publishes an `ExampleMessage` to `topic` with some fixed and random properties (C++ version)
    * `topic_subscriber.py`
        * Receives and logs `ExampleMessage` messages from `topic`
    * `service_hoster.py`
        * Receives `ExampleMessage` messages from `topic` and catalogues them by `first_name` and `last_name`, exposing that catalogue via `ExampleService`
    * `service_caller.py`
        * Calls the `ExampleService` service with the `first_name` and `last_name` provided on the command line
* Tests
    * `ros_kinetic_catkin_example_test.py`
        * Instantiates `topic_publisher.cpp` and `service_hoster.py` and validates their behaviour (Python version)
    * `ros_kinetic_catkin_example_test.py`
        * Instantiates `topic_publisher.cpp` and `service_hoster.py` and validates their behaviour (C++ version)

## How do I use it?

Before doing anything, ensure you have `ros-kinetic` installed and you're in the root of the repo, then run the following:

        source /opt/ros/kinetic/setup.bash
        catkin_make
        source devel/setup.bash

It should be as simple as the following (from the root of the repo):

        # to run the Python tests
        rostest example ros_kinetic_catkin_example_test_python.launch

        # to run the C++ tests
        rostest example ros_kinetic_catkin_example_test_cpp.launch

If you want to use `catkin` to run the tests: 

        catkin_make
        catkin_make run_tests
