cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_STANDARD 11)

project(example)

add_definitions(-std=c++11)

find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        rospy
        rostest
        std_msgs
        message_generation
)

# required by (part of) Fast-RTPS
find_package(fastcdr REQUIRED)

# Fast-RTPS itself
find_package(fastrtps REQUIRED)

# standard ROS- add our messages
add_message_files(
        FILES
        ExampleMessage.msg
)

# standard ROS- add our services
add_service_files(
        FILES
        ExampleService.srv
)

# standard ROS- generate our messages
generate_messages(
        DEPENDENCIES
        std_msgs
)

# standard ROS- add our include directory (incl. our Fast-RTPS message library headers)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        rospy
        std_msgs
        message_runtime

        DEPENDS

        INCLUDE_DIRS
        include

        LIBRARIES
        example
        Example # our Fast-RTPS message library
)

# build our Fast-RTPS message library
add_library(
        Example
        src/ExamplePubSubTypes.cxx
        src/Example.cxx
)

# link it to the Fast-RTPS dependencies
target_link_libraries(
        Example
        fastcdr
        fastrtps
        ${catkin_LIBRARIES}
)

# build the C++ ROS node that publishes via Fast-RTPS
add_executable(
        ${PROJECT_NAME}_rtps_topic_publisher
        nodes/rtps_topic_publisher.cpp
)

# link it to our Fast-RTPS message library
target_link_libraries(
        ${PROJECT_NAME}_rtps_topic_publisher
        Example
        ${catkin_LIBRARIES}
)

# not 100% certain this needed
add_dependencies(
        ${PROJECT_NAME}_rtps_topic_publisher
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

# build the C++ ROS node that subscribes via Fast-RTPS and publishes to a ROS topic
add_executable(
        ${PROJECT_NAME}_rtps_to_ros_gateway
        nodes/rtps_to_ros_gateway.cpp
)

# link it to our Fast-RTPS message library
target_link_libraries(
        ${PROJECT_NAME}_rtps_to_ros_gateway
        Example
        ${catkin_LIBRARIES}
)

# again, not 100% certain this is needed
add_dependencies(
        ${PROJECT_NAME}_rtps_to_ros_gateway
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

# install all our Python ROS nodes and Python tests
install(
        PROGRAMS
        nodes/service_caller.py
        nodes/service_hoster.py
        nodes/topic_publisher.py
        nodes/topic_subscriber.py
        test/ros_kinetic_catkin_example_test.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install our built C++ ROS nodes
install(
        TARGETS
        ${PROJECT_NAME}_rtps_topic_publisher Example
        ${PROJECT_NAME}_rtps_to_ros_gateway Example
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install our .launch files
install(
        FILES
        launch/example.launch
        test/ros_kinetic_catkin_example_test_python.launch
        test/ros_kinetic_catkin_example_test_cpp.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# only while under test
if (CATKIN_ENABLE_TESTING)
    # tell rostest to test some C++ code using gtest
    add_rostest_gtest(
            ${PROJECT_NAME}-test # the name of the test executable to be generated
            test/ros_kinetic_catkin_example_test_cpp.launch # .launch file that describes the setup and execution
            test/ros_kinetic_caktin_example_test.cpp # test file that describes the actual tests
    )

    # link it to our Fast-RTPS message library
    target_link_libraries(
            ${PROJECT_NAME}-test
            Example
            ${catkin_LIBRARIES}
    )

    # tell rostest to test some Python code (point it to a .launch file that describes the setup and execution)
    add_rostest(test/ros_kinetic_catkin_example_test_python.launch)
endif ()
