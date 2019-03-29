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
        * Publishes an `ExampleMessage` to `topic` with some fixed and random properties
    * `topic_subscriber.py`
        * Receives and logs `ExampleMessage` messages from `topic`
    * `service_hoster.py`
        * Receives `ExampleMessage` messages from `topic` and catalogues them by `first_name` and `last_name`, exposing that catalogue via `ExampleService`
    * `service_caller.py`
        * Calls the `ExampleService` service with the `first_name` and `last_name` provided on the command line
* Tests
    * `everything_test`
        * Instantiates `topic_publisher.py` and `service_hoster.py` and validates their behaviour

## How do I use it?

Before doing anything, ensure you have `ros-kinetic` installed and you're in the root of the repo, then run the following:

        source /opt/ros/kinetic/setup.bash
        catkin_make
        source devel/setup.bash

It should be as simple as the following (from the root of the repo):

        rostest example everything_test.launch

If you want to use `catkin` to run the tests, it seems to have to run `catkin_make` twice: 

        catkin_make
        catkin_make
        catkin_make run_tests
        
But that complains that the tests don't return results at the moment- I'll figure it out some other time.

## What are the takeaways?

In `package.xml`:

    <build_depend>rostest</build_depend>

In `CMakeLists.txt` (though I think this is related to `catkin_make run_tests`):

    if (CATKIN_ENABLE_TESTING)
        add_rostest(test/everything_test.launch)
    endif ()

In `test/everything_test.launch`:

    <launch>
    
        <node name="topic_publisher" pkg="example" type="topic_publisher.py"/>
        <node name="service_hoster" pkg="example" type="service_hoster.py"/>
    
        <test test-name="everything_test" pkg="example" type="everything_test.py"/>
    
    </launch>

In `test/everything_test.py` device your tests.