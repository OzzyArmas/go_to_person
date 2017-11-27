#!/usr/bin/env python


import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('transformations')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    try:
        while True:

            br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), '/base_link', '/world')

            br.sendTransform((5,5,0), (0, 0, 0, 1), rospy.Time.now(), '/sensor', '/base_link')

            rate.sleep()
    except KeyboardInterrupt:
        print 'stop'