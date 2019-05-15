#include "ros/ros.h"
#include "example/ExampleMessage.h"

#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/publisher/PublisherListener.h>

#include <fastrtps/Domain.h>

#include "Example.h"
#include "ExamplePubSubTypes.h"


const std::string NODE_NAME = "rtps_topic_publisher";

class PubListener : public eprosima::fastrtps::PublisherListener {
public:
    PubListener() = default;

    ~PubListener() override = default;
};

unsigned int rand_between(int min, int max) {
    return rand() % (max - min);
}

int main(int argc, char **argv) {
    //
    // Fast-RTPS init
    //

    eprosima::fastrtps::ParticipantAttributes PParam;
    PParam.rtps.builtin.domainId = 0;
    PParam.rtps.builtin.leaseDuration = eprosima::fastrtps::c_TimeInfinite;
    PParam.rtps.setName("Participant_publisher");
    eprosima::fastrtps::Participant *mp_participant = eprosima::fastrtps::Domain::createParticipant(PParam);
    if (mp_participant == nullptr) {
        return 1;
    }

    ExampleMessagePubSubType myType;

    eprosima::fastrtps::Domain::registerType(mp_participant, static_cast<eprosima::fastrtps::TopicDataType *>(&myType));

    eprosima::fastrtps::PublisherAttributes Wparam;
    Wparam.topic.topicKind = eprosima::fastrtps::NO_KEY;
    Wparam.topic.topicDataType = myType.getName();
    Wparam.topic.topicName = "ExamplePubSubTopic";

    PubListener m_listener;

    eprosima::fastrtps::Publisher *rtpsPublisher = eprosima::fastrtps::Domain::createPublisher(
            mp_participant,
            Wparam,
            static_cast<eprosima::fastrtps::PublisherListener *>(&m_listener)
    );

    //
    // ROS init
    //

    ros::init(argc, argv, NODE_NAME);

    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        int scores[3] = {420, 1337, 8008135};

        //
        // publish Fast-RTPS message
        //

        ExampleMessage msg;

        msg.first_name("Edward");
        msg.last_name("Beech");
        msg.age(rand_between(1, 100));
        msg.score(scores[rand_between(0, 2)]);

        ROS_INFO(
                "RTPS first_name=%s, last_name=%s, age=%i, score=%i",
                msg.first_name().c_str(),
                msg.last_name().c_str(),
                msg.age(),
                msg.score()
        );

        rtpsPublisher->write(&msg);

        //
        // sleep
        //

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}