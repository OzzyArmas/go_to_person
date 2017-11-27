#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from go_to_person.msg import People
from std_msgs.msg import Header
import random



def generatePeople(n):
    people = []
    #listener = tf.TransformListener()
    #print listener.allFramesAsString()
    for i in range(0,1):

        head = Header(n,rospy.Time.now(), '/map') #/kinect_rgb_optical_frame
        x = 1
        y = 1
        pose = PoseStamped(head, Pose(Point(x,y,0),Quaternion(0,0,0,1)))
        #point = listener.transformPoint('/map', point)
        people.append(pose)

    pub = rospy.Publisher('/foo/People', People, queue_size=1)
    pub.publish(People(people))


if __name__ == '__main__':
    rospy.init_node('sensor_node')
    rate = rospy.Rate(2)
    n = 0
    while(True):
        generatePeople(n)
        rate.sleep()
        n += 1
    rospy.spin()
