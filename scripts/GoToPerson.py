#!/usr/bin/env python
# Something something module tells the
# robot which person to go based on trigger
import math

import actionlib
import rospy
import tf
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion
from go_to_person.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from rospy.exceptions import ROSException
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class GoToPerson():

    def __init__(self, zero_base_link = True):

        self.node = rospy.init_node('go_to_person_server')

        self.person_subscriber = rospy.Subscriber('/foo/People', People, self._handlePerson, queue_size=1)

        rospy.Service('go_to_person', Trigger, self._handle_server)

        self.tf_listener = tf.TransformListener()

        self._id = 0

    def _handlePerson(self, points):
        self.people = points

    def _handle_server(self, data):

        closest_distance = float('inf')
        final_pose = None
        #global_pose = None
        for person in self.people.people:
            stampedPose = self.tf_listener.transformPose('/base_link', person)
            dist = (stampedPose.pose.position.x ** 2 + stampedPose.pose.position.y**2)**(1/2)
            if dist < closest_distance:
                closest_distance = dist
                final_pose = stampedPose
                #for debugging purposes

        #global_pose_person = self.tf_listener.transformPose('/map', final_pose)
        final_pose = self.plan_approach(final_pose)
        #global_pose_target = self.tf_listener.transformPose('/map', final_pose)

        #print 'person: ' + str(global_pose_person)
        #print 'target: ' + str(global_pose_target)

        goal = MoveBaseGoal(final_pose)
        client = actionlib.SimpleActionClient('move_base_navi', MoveBaseAction)
        client.wait_for_server()
        #print 'Waited Server?'
        client.send_goal(goal)
        client.wait_for_result()
        return TriggerResponse(True, '\nGoal Found at:\n ' + str(goal))




    def plan_approach(self, person_pose):
        #person_pose is relative to /base_link
        base_transform = self.tf_listener.lookupTransform('map','/base_link', rospy.Time(0))
        print base_transform
        header = Header(None, rospy.Time(0), '/map')
        base_link_pose = Pose(Point(base_transform[0][0],
                                    base_transform[0][1],
                                    base_transform[0][2]),
                              Quaternion(base_transform[1][0],
                                         base_transform[1][1],
                                         base_transform[1][2],
                                         base_transform[1][3]))
        base_link_pose = PoseStamped(header, base_link_pose)

        if person_pose is None:
            return base_link_pose

        #Now I want to define an acceptable goal here.
        try:
            robot_lenght = 0.665
            robot_width = 0.275
            data = []
            final_pose = base_link_pose
            angle_shift_right = math.pi
            angle_shift_left = math.pi
            while len(data) == 0:

                p_orientation = person_pose.pose.orientation
                theta_right = euler_from_quaternion((p_orientation.x, p_orientation.y, p_orientation.z, p_orientation.w))[
                            2] + angle_shift_right
                target_x_right = -(0.65 + robot_lenght)  * math.cos(theta_right) + person_pose.pose.position.x
                target_y_right = (0.65 + robot_width) * math.sin(theta_right) + person_pose.pose.position.y
                theta_right = quaternion_from_euler(0, 0, theta_right)
                header = person_pose.header
                pose = Pose(Point(target_x_right, target_y_right, 0), Quaternion(theta_right[0],
                                                                                 theta_right[1],
                                                                                 theta_right[2],
                                                                                 theta_right[3]))
                final_pose = PoseStamped(header, pose)
                final_pose = self.tf_listener.transformPose('/map', final_pose)
                client = rospy.wait_for_service('move_base/make_plan', timeout=5)
                get_plan_service = rospy.ServiceProxy('move_base/make_plan', GetPlan)
                resp = get_plan_service(base_link_pose,final_pose, 0.1)
                angle_shift_right -= math.pi/8 #pi/2
                #print self.tf_listener.transformPose('/map', final_pose)
                #print resp
                #If cannot plan by approaching the person from the right, approach them from the left
                if (len(resp.plan.poses) == 0):
                    theta_left = euler_from_quaternion((p_orientation.x, p_orientation.y, p_orientation.z, p_orientation.w))[
                                2] - angle_shift_left
                    target_x = -(0.7 + robot_lenght) * math.cos(theta_left) + person_pose.pose.position.x
                    target_y = (0.7 + robot_width) * math.sin(theta_left) + person_pose.pose.position.y
                    theta_left = quaternion_from_euler(0, 0, theta_left)

                    header = person_pose.header
                    pose = Pose(Point(target_x, target_y, 0), Quaternion(theta_left[0],
                                                                         theta_left[1],
                                                                         theta_left[2],
                                                                         theta_left[3]))
                    final_pose = PoseStamped(header, pose)
                    final_pose = self.tf_listener.transformPose('/map', final_pose)
                    client = rospy.wait_for_service('move_base/make_plan', timeout=5)
                    get_plan_service = rospy.ServiceProxy('move_base/make_plan', GetPlan)
                    resp = get_plan_service(base_link_pose, final_pose, 0.1)

                    angle_shift_left += math.pi / 8

                data = resp.plan.poses
                #print data

            return final_pose
        except ROSException as rosExc:
            print rosExc.message

        return base_link_pose

'''
orient = person_pose.pose.orientation

'''

if __name__ == '__main__':
    goToPersonNode = GoToPerson()
    rospy.spin()
