#!/usr/bin/env python

"""
Converts obstacle_detecotr/Obastacles into a geometry_msgs/PoseArray.
"""

import rospy
from obstacle_detector.msg import Obstacles, CircleObstacle
from geometry_msgs.msg import PoseArray, Pose

def newMessageReceived(obstacles):
    poseArray = PoseArray()    
    poseArray.header = obstacles.header

    for obstacle in obstacles.circles:
        pose = Pose()
        pose.position.x = obstacle.center.x
        pose.position.y = obstacle.center.y
        pose.position.z = obstacle.center.z
        poseArray.poses.append(pose)

    pub.publish(poseArray)


# Initialize node
rospy.init_node("obstacles_to_pose_array")

# Create publisher and subscriber
inputTopic = rospy.resolve_name("/obstacle_detector/raw_obstacles")
outputTopic = rospy.resolve_name("/obstacle_detector/pose_array")

sub = rospy.Subscriber(inputTopic, Obstacles, newMessageReceived, queue_size=5)
pub = rospy.Publisher(outputTopic, PoseArray, queue_size=5)

rospy.loginfo("Re-publishing obstacle_detecotr/Obastacles from %s as geometry_msgs/PoseArray at %s" % (inputTopic, outputTopic) )
rospy.spin()
