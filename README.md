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

In theory, you should also be able to run:

        catkin_make run_tests
        
But that complains that the tests don't return results at the moment- I'll figure it out some other time.

## What are the takeaways?

In `package.xml`:

    <build_depend>rostest</build_depend>

In `CMakeLists.txt`:

    