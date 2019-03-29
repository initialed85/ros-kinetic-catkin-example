#!/usr/bin/env python

"""This script hosts the ExampleService that returns the last ExampleMessage seen on a topic for a given
first_name and last_name"""

from threading import RLock

import rospy
from example.msg import ExampleMessage
from example.srv import *

NODE_NAME = 'service_hoster'
SERVICE_NAME = 'service'
TOPIC_NAME = 'topic'

lock = RLock()
example_message_by_name = {}


def handle_request(request):
    with lock:
        request = example_message_by_name.get(
            '{} {}'.format(request.first_name, request.last_name)
        )

    rospy.loginfo('request is {}'.format(request))

    response = None if request is None else ExampleServiceResponse(
        request.first_name,
        request.last_name,
        request.age,
        request.score
    )

    rospy.loginfo('response is {}'.format(response))

    return response


def topic_callback(data):
    rospy.loginfo('data is {}'.format(data))

    with lock:
        example_message_by_name.update({
            '{} {}'.format(data.first_name, data.last_name): data
        })


def run():
    rospy.init_node(NODE_NAME)

    _ = rospy.Service(SERVICE_NAME, ExampleService, handle_request)

    _ = rospy.Subscriber(TOPIC_NAME, ExampleMessage, topic_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
