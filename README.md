# ros-kinetic-catkin-example
This repo is a `ros-kinetic` workspace with a package called `example` for the purposes of demonstrating `rostest` for both Python testing (using `rosunit` and `unittest`) and C++ (using `gtest`).

Additionally, it will demonstrate the integration of ROS with eProsima's Fast-RTPS (DDS).

The concept is as follows:

    RTPS topic -> ROS topic -> ROS service
    
And so sequentially, it looks a bit like this:

- Node A publishes to an RTPS topic
- Node B subscribes to that RTPS topic and publishes to a ROS topic
- Node C subscribes to that ROS topic and hosts the data as a service
- Node D calls the service

## What's in it?

The package is very contrived, it's comprised of the following:

* RTPS Messages
    * `ExampleMessage`
* ROS Messages
    * `ExampleMessage`
* ROS Services
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

Prerequisites:

- For local building
    - ROS Kinetic
    - A shell with `/opt/ros/kinetic/setup.bash` sourced
- For building in Docker
    - Docker

To build the package locally:

    ./local_build.sh
    
To test the package locally:

    ./local_test.sh
    
To build the package in Docker:

    ./build.sh
    
To test the package in Docker:

    ./test.sh
    
## How else do I use it? 

NOTE: The following stuff assumes you're operating locally and you've built the package already.

To manually build everything:

    catkin_make && catkin_make_install && catkin_make tests

To individually spin up all the pieces (run each command in a separate terminal):

    roscore
    source devel/setup.bash; rosrun example example_rtps_topic_publisher
    source devel/setup.bash; rosrun example example_rtps_to_ros_gateway
    source devel/setup.bash; rosrun example topic_subscriber.py
    source devel/setup.bash; rosrun example service_hoster.py
    
    # and to call the service
    source devel/setup.bash; rosrun example service_caller.py Edward Beech

To run the Python tests:

    source devel/setup.bash; rostest example ros_kinetic_catkin_example_test_python.launch 
    
Underneath, `rostest` seems to call `rosunit` which in turn calls `unittest`.

To run the C++ tests:

    source devel/setup.bash; rostest example ros_kinetic_catkin_example_test_cpp.launch
    
Underneath, `rostest` seems to call `rosunit` which in turn calls `gtest`.

To run all the tests:

    source devel/setup.bash; catkin_make run_tests && catkin_test_results --all
    
The `catkin_test_results` step is required to truthfully represent the test results, as `catkin_make run_tests` invokes `rostest` which swallows up test failures for some reason.

## What should I be aware of? 

- Running `rostest` generates both `rostest` and `rosunit` jUnit XML test results in `~/.ros/test_results/example`
    - Only `rosunit` results seems to record failures when they happen, `rostest` records a single pass for all tests runs (regardless of outcome)
    - You can differentiate between results of the two kinds by jUnit XML file prefix (`rostest` vs `rosunit`)
- In the `rostest` jUnit XML, as mentioned above, there is a single `<testcase>` entry per `<test>` tag in the `.launch` file, bearing the name specified in `type=` for the `<test>` tag
- In the `rosunit` jUnit XML, all the test methods appear named as they are in your test code

The above is demonstrated (for `rosunit`) through the naming of the methods in the test files, e.g.:

    // C++ and gtest
    TEST(SomeTestSuite, testRTPSTopicPublisherIdeT2859)
    
    # Python and unittest
    def test_topic_publisher_ide_t2857(self):

And also demonstrated (for `rostest`) through the naming of the `<test>` elements in the `.launch` files, e.g:
    
    <test pkg="example" test-name="ros_kinetic_catkin_example_cpp_test_ide_t2855" type="example-test"/>

And finally represented in the jUnit XML test results; e.g.:

    # for rostest
    
    # for rosunit