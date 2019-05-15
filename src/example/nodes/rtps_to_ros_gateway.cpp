#include "ros/ros.h"
#include "example/ExampleMessage.h"

#include <fastrtps/subscriber/SubscriberListener.h>
#include <fastrtps/subscriber/SampleInfo.h>
#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/attributes/SubscriberAttributes.h>

#include <fastrtps/Domain.h>

#include "Example.h"
#include "ExamplePubSubTypes.h"


const std::string NODE_NAME = "rtps_to_dds_gateway";
const std::string TOPIC_NAME = "topic";

class SubListener : public eprosima::fastrtps::SubscriberListener {
public:
    SubListener() = default;

    ~SubListener() override = default;

    void onNewDataMessage(eprosima::fastrtps::Subscriber *sub) {
        ExampleMessage st;

        if (!sub->takeNextData(&st, &m_info)) {
            return;
        }

        if (m_info.sampleKind != eprosima::fastrtps::ALIVE) {
            return;
        }

        this->msgAvailale = true;
        this->lastMsg = st;
    };

    eprosima::fastrtps::SampleInfo_t m_info;
    bool msgAvailale = false;
    ExampleMessage lastMsg;
};

int main(int argc, char **argv) {
    //
    // Fast-RTPS init
    //

    eprosima::fastrtps::ParticipantAttributes PParam;
    PParam.rtps.builtin.domainId = 0;
    PParam.rtps.builtin.leaseDuration = eprosima::fastrtps::c_TimeInfinite;
    PParam.rtps.setName("Participant_subscriber");
    eprosima::fastrtps::Participant *mp_participant = eprosima::fastrtps::Domain::createParticipant(PParam);
    if (mp_participant == nullptr) {
        return 1;
    }

    ExampleMessagePubSubType myType;

    eprosima::fastrtps::Domain::registerType(mp_participant, static_cast<eprosima::fastrtps::TopicDataType *>(&myType));

    eprosima::fastrtps::SubscriberAttributes Rparam;
    Rparam.topic.topicKind = eprosima::fastrtps::NO_KEY;
    Rparam.topic.topicDataType = myType.getName();
    Rparam.topic.topicName = "ExamplePubSubTopic";

    SubListener m_listener;

    eprosima::fastrtps::Subscriber *mp_subscriber = eprosima::fastrtps::Domain::createSubscriber(
            mp_participant,
            Rparam,
            static_cast<eprosima::fastrtps::SubscriberListener *>(&m_listener)
    );

    if (mp_subscriber == nullptr) {
        return 1;
    }

    //
    // ROS init
    //

    ros::init(argc, argv, NODE_NAME);

    ros::NodeHandle nh;

    ros::Publisher rosPublisher = nh.advertise<example::ExampleMessage>(TOPIC_NAME, 1000);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        //
        // publish ROS message
        //

        if (m_listener.msgAvailale) {
            example::ExampleMessage rosMsg;

            rosMsg.first_name = m_listener.lastMsg.first_name();
            rosMsg.last_name = m_listener.lastMsg.last_name();
            rosMsg.age = m_listener.lastMsg.age();
            rosMsg.score = m_listener.lastMsg.score();

            ROS_INFO(
                    "ROS first_name=%s, last_name=%s, age=%i, score=%i",
                    rosMsg.first_name.c_str(),
                    rosMsg.last_name.c_str(),
                    rosMsg.age,
                    rosMsg.score
            );

            rosPublisher.publish(rosMsg);
        }

        //
        // sleep
        //

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}