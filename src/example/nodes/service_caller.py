#!/usr/bin/env python

"""This script asks the ExampleService for the last ExampleMessage belonging to a given first_name and last_name"""

import traceback

import rospy
from example.srv import *

NODE_NAME = 'service_caller'
SERVICE_NAME = 'service'


def run(first_name, last_name):
    rospy.init_node(NODE_NAME)

    rospy.wait_for_service(SERVICE_NAME)

    service = rospy.ServiceProxy(SERVICE_NAME, ExampleService)

    try:
        data = service(first_name, last_name)
        rospy.loginfo(data)
    except rospy.ServiceException:
        rospy.logerr(traceback.format_exc())


if __name__ == '__main__':
    import sys

    if len(sys.argv) < 3:
        print 'usage: {} [first_name] [last_name]'.format(
            sys.argv[0]
        )

    try:
        run(sys.argv[1], sys.argv[2])
    except rospy.ROSInterruptException:
        pass
