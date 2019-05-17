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


// note the TM4J test case number (IdeTnnnn); if you copy and paste this code, be sure to update or remove this
TEST(SomeTestSuite, testRTPSTopicPublisherIdeT2859) {

    // arrange
    boost::shared_ptr<const example::ExampleMessage> sharedMsg = ros::topic::waitForMessage<example::ExampleMessage>(
            TOPIC_NAME,
            ros::Duration(3.0)
    );

    ASSERT_NE(nullptr, sharedMsg) << "message not on topic after 3 seconds"; // pre-check for arrange

    // act
    example::ExampleMessage msg = *sharedMsg;

    // assert
    bool found = false;
    for (int i = 0; i < sizeof(permittedScores); i++) {
        if (permittedScores[i] == msg.score) {
            found = true;
        }
    }

    ASSERT_EQ(firstName, msg.first_name) << "msg.first_name was not " << firstName;
    ASSERT_EQ(lastName, msg.last_name) << "msg.last_name was not " << lastName;
    ASSERT_LT(minAge, msg.age) << "msg.age was not greater than" << minAge;
    ASSERT_TRUE(found) << "response.score was not one of 420, 1337 or 8008135";
}

// note the TM4J test case number (IdeTnnnn); if you copy and paste this code, be sure to update or remove this
TEST(SomeTestSuite, testServiceHostIdeT2860) {

    // arrange
    boost::shared_ptr<const example::ExampleMessage> sharedMsg = ros::topic::waitForMessage<example::ExampleMessage>(
            TOPIC_NAME,
            ros::Duration(3.0)
    );

    ASSERT_NE(nullptr, sharedMsg) << "message not on topic after 3 seconds"; // pre-check for arrange

    bool serviceAvailable = ros::service::waitForService(SERVICE_NAME, ros::Duration(3.0));

    ros::NodeHandle n = *nh;

    ros::ServiceClient client = n.serviceClient<example::ExampleService>(SERVICE_NAME);

    example::ExampleService srv;

    srv.request.first_name = firstName;
    srv.request.last_name = lastName;

    // act
    bool callPassed = client.call(srv);

    // assert
    bool found = false;
    for (int i = 0; i < sizeof(permittedScores); i++) {
        if (permittedScores[i] == srv.response.score) {
            found = true;
        }
    }

    ASSERT_TRUE(serviceAvailable) << "service not available after 3 seconds";
    ASSERT_TRUE(callPassed) << "service call failed";
    ASSERT_EQ(srv.response.first_name, firstName) << "response.first_name was not " << firstName;
    ASSERT_EQ(srv.response.last_name, lastName) << "response.last_name was not " << lastName;
    ASSERT_LT(minAge, srv.response.age) << "response.age was not greater than" << minAge;
    ASSERT_TRUE(found) << "response.score was not one of 420, 1337 or 8008135";
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "everything_test");

    nh.reset(new ros::NodeHandle);

    return RUN_ALL_TESTS();
}
