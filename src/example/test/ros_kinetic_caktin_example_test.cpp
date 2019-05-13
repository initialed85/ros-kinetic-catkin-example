#include <gtest/gtest.h>
#include "ros/ros.h"
#include "example/ExampleMessage.h"
#include "example/ExampleService.h"

std::shared_ptr<ros::NodeHandle> nh;

const std::string TOPIC_NAME = "topic";
const std::string SERVICE_NAME = "service";

const std::string firstName = "Edward";
const std::string lastName = "Beech";
const int minAge = 0;
const int permittedScores[3] = {420, 1337, 8008135};


TEST(SomeTestSuite, testTopicPublisher) {
    example::ExampleMessage msg = *ros::topic::waitForMessage<example::ExampleMessage>(TOPIC_NAME, ros::Duration(3.0));

    ASSERT_EQ(firstName, msg.first_name);
    ASSERT_EQ(lastName, msg.last_name);
    ASSERT_LT(minAge, msg.age);

    bool found = false;
    for (int i = 0; i < sizeof(permittedScores); i++) {
        if (permittedScores[i] == msg.score) {
            found = true;
        }
    }
    ASSERT_TRUE(found);
}

TEST(SomeTestSuite, testServiceHost) {
    ros::topic::waitForMessage<example::ExampleMessage>(TOPIC_NAME, ros::Duration(3.0));

    bool serviceAvailable = ros::service::waitForService(SERVICE_NAME, ros::Duration(3.0));

    ASSERT_TRUE(serviceAvailable);

    ros::NodeHandle n = *nh;

    ros::ServiceClient client = n.serviceClient<example::ExampleService>(SERVICE_NAME);

    example::ExampleService srv;

    srv.request.first_name = firstName;
    srv.request.last_name = lastName;

    ASSERT_TRUE(client.call(srv));

    ASSERT_EQ(srv.response.first_name, firstName);
    ASSERT_EQ(srv.response.last_name, lastName);
    ASSERT_LT(minAge, srv.response.age);

    bool found = false;
    for (int i = 0; i < sizeof(permittedScores); i++) {
        if (permittedScores[i] == srv.response.score) {
            found = true;
        }
    }
    ASSERT_TRUE(found);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "everything_test");

    nh.reset(new ros::NodeHandle);

    return RUN_ALL_TESTS();
}
