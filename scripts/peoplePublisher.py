#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from go_to_person.msg import People
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
import random



def generatePeople(n):
    people = []
    #listener = tf.TransformListener()
    #print listener.allFramesAsString()
    t = quaternion_from_euler(0, 0, 0)
    for i in range(0,1):

        head = Header(n,rospy.Time(0), '/map') #/kinect_rgb_optical_frame
        x = 3
        y = -2
        pose = PoseStamped(head, Pose(Point(x,y,0),Quaternion(t[0],t[1],t[2],t[3])))
        #point = listener.transformPoint('/map', point)
        people.append(pose)

    pub = rospy.Publisher('/foo/People', People, queue_size=1)
    pub.publish(People(people))


if __name__ == '__main__':
    rospy.init_node('sensor_node')
    rate = rospy.Rate(1)
    n = 0
    while(True):
        generatePeople(n)
        rate.sleep()
        n += 1
    rospy.spin()
