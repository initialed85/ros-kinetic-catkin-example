#include <gtest/gtest.h>
#include "ros/ros.h"

TEST(SomeTestSuite, someTestCase
) {
    ASSERT_TRUE(true);
}

TEST(SomeTestSuite, otherTestCase
) {
    ASSERT_TRUE(false);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "everything_test");

    ros::NodeHandle n;

    return RUN_ALL_TESTS();
}