#!/usr/bin/env python

import rospy
from go_to_person.srv import *

def foo_server():
    rospy.init_node('move_foo_server')
    rospy.Service('move_foo', MoveFoo, move_foo_handler)
    rospy.spin()

def move_foo_handler(closest):
    print 'Moving to' + str((closest.x, closest.y))
    return MoveFooResponse(0)   #Success

if __name__ == '__main__':
    foo_server()