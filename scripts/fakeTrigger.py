#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger

def trigger_move():
    rospy.wait_for_service('go_to_person')
    try:
        serv = rospy.ServiceProxy('go_to_person', Trigger)
        print serv()
    except rospy.ServiceException, e:

        print "Service call failed: %s"%e

if __name__ == '__main__':
    trigger_move()
