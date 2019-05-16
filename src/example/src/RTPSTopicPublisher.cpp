#include <chrono>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/publisher/PublisherListener.h>

#include <fastrtps/Domain.h>

#include "Example.h"
#include "ExamplePubSubTypes.h"

const double ONE_SECOND_IN_MICROSECONDS = 1000000;
const double TARGET_SLEEP = ONE_SECOND_IN_MICROSECONDS / 10;

volatile sig_atomic_t shutdown = 0;

void handler(int signum) {
    shutdown = signum;
}

class PubListener : public eprosima::fastrtps::PublisherListener {
};

unsigned int rand_between(int min, int max) {
    return (unsigned int) (rand() % (max - min));
}

double now() {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
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
    // main loop
    //

    signal(SIGINT, handler);

    double before, after;

    int scores[3] = {420, 1337, 8008135};

    while (shutdown == 0) {
        before = now();

        //
        // publish Fast-RTPS message
        //

        ExampleMessage msg;

        msg.first_name("Edward");
        msg.last_name("Beech");
        msg.age((uint8_t) rand_between(1, 100));
        msg.score((uint32_t) scores[rand_between(0, 2)]);

        printf(
                "[ INFO] [%lf] RTPS transmitting first_name=%s, last_name=%s, age=%i, score=%i\n",
                now(),
                msg.first_name().c_str(),
                msg.last_name().c_str(),
                msg.age(),
                msg.score()
        );

        rtpsPublisher->write(&msg);

        //
        // sleep
        //

        after = now();

        usleep((__useconds_t) (TARGET_SLEEP - ((after - before) * ONE_SECOND_IN_MICROSECONDS)));
    }

    return 0;
}