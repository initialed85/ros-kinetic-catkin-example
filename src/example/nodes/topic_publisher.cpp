#include "ros/ros.h"
#include "example/ExampleMessage.h"

#include "HelloWorld/HelloWorldPublisher.h"


using namespace eprosima::fastrtps;
using namespace rtps;


const std::string NODE_NAME = "topic_publisher";
const std::string TOPIC_NAME = "topic";

unsigned int rand_between(int min, int max) {
    return rand() % (max - min);
}

int main(int argc, char **argv) {
    HelloWorldPublisher helloWorldPublisher;

    helloWorldPublisher.init();

    ros::init(argc, argv, NODE_NAME);

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<example::ExampleMessage>(TOPIC_NAME, 1000);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        example::ExampleMessage msg;

        int scores[3] = {420, 1337, 8008135};

        msg.first_name = "Edward";
        msg.last_name = "Beech";
        msg.age = rand_between(1, 100);
        msg.score = scores[rand_between(0, 2)];

        ROS_INFO(
                "first_name=%s, last_name=%s, age=%i, score=%i",
                msg.first_name.c_str(),
                msg.last_name.c_str(),
                msg.age,
                msg.score
        );

        pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}