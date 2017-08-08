#!/usr/bin/python
import roslib;	roslib.load_manifest('ipa_benchmark')
import rospy
from geometry_msgs.msg import PoseStamped
from ipa_benchmark.srv import *

if __name__ == '__main__':

    rospy.init_node('test_plan')

    rospy.wait_for_service("move_group/pathPlanning")
    plan_client = rospy.ServiceProxy("move_group/pathPlanning", pathPlanning)

    try:
        start_pose = geometry_msgs.msg.PoseStamped()
        start_pose.header.frame_id = "base_link"
        start_pose.header.stamp = rospy.Time(0)
        start_pose.pose.position.x = 0.3531
        start_pose.pose.position.y = -0.3447
        start_pose.pose.position.z = 1.02691
        start_pose.pose.orientation.x = 0.9999
        start_pose.pose.orientation.y = 0.0018
        start_pose.pose.orientation.z = 0.00155
        start_pose.pose.orientation.w = 0.00037

        goal_pose = geometry_msgs.msg.PoseStamped()
        goal_pose.header.frame_id = "base_link"
        goal_pose.header.stamp = rospy.Time(0)
        goal_pose.pose.position.x = 0.1531
        goal_pose.pose.position.y = -0.3447
        goal_pose.pose.position.z = 1.02691
        goal_pose.pose.orientation.x = 0.9999
        goal_pose.pose.orientation.y = 0.0018
        goal_pose.pose.orientation.z = 0.00155
        goal_pose.pose.orientation.w = 0.00037

        srv_req = pathPlanningRequest()
        srv_req.start_pose = start_pose
        srv_req.goal_pose = goal_pose
        call_plan_service = plan_client(srv_req)
        rospy.loginfo("plan service call successful")
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request attach_object" + str(exc))

