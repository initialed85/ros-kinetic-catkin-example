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

        sys.exit(1)

    _first_name = sys.argv[1]
    _last_name = sys.argv[2]

    try:
        run(_first_name, _last_name)
    except rospy.ROSInterruptException:
        pass
