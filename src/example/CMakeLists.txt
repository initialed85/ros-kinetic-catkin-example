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

find_package(fastcdr REQUIRED)

find_package(fastrtps REQUIRED)

add_message_files(
        FILES
        ExampleMessage.msg
)

add_service_files(
        FILES
        ExampleService.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
        CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
        DEPENDS
        INCLUDE_DIRS include
        LIBRARIES example HelloWorld
)

add_library(
        HelloWorld
        src/HelloWorldPublisher.cxx
        src/HelloWorldPubSubTypes.cxx
        src/HelloWorld.cxx
)

target_link_libraries(
        HelloWorld
        fastcdr
        fastrtps
        ${catkin_LIBRARIES}
)

add_executable(
        ${PROJECT_NAME}_topic_publisher
        nodes/topic_publisher.cpp
)

target_link_libraries(
        ${PROJECT_NAME}_topic_publisher
        HelloWorld
        ${catkin_LIBRARIES}
)

add_dependencies(
        ${PROJECT_NAME}_topic_publisher
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

install(
        PROGRAMS
        nodes/service_caller.py
        nodes/service_hoster.py
        nodes/topic_publisher.py
        nodes/topic_subscriber.py
        test/ros_kinetic_catkin_example_test.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
        TARGETS ${PROJECT_NAME}_topic_publisher HelloWorld
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
        FILES
        launch/example_service_host_and_topic_publisher.launch
        launch/example_service_caller.launch
        launch/example_topic_subscriber.launch
        test/ros_kinetic_catkin_example_test_python.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
    add_rostest_gtest(
            ${PROJECT_NAME}-test
            test/ros_kinetic_catkin_example_test_cpp.launch
            test/ros_kinetic_caktin_example_test.cpp
    )

    target_link_libraries(
            ${PROJECT_NAME}-test
            HelloWorld
            ${catkin_LIBRARIES}
    )

    add_rostest(test/ros_kinetic_catkin_example_test_python.launch)
endif ()
