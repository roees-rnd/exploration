#!/usr/bin/env python3
import net_db

import rosparam
import rospy
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs import msg as geom_msg
import numpy as np


ndb = net_db.net_db()
# ndb.add_node((0, 0.1), False)
# ndb.add_node((2, 0.1), True)

pos, is_door = ndb.get_all_nodes()


def updatePos(msg):
    global ndb
    x = msg.pose.position.x
    y = msg.pose.position.y
    ndb.add_node((x, y), False)


def publishGraph(pubNodes, pub_ns='net', stamp=None, frame_id='world', size=0.03, numLmks=0):
    global ndb
    marker = Marker(type=Marker.LINE_LIST, ns=pub_ns, action=Marker.ADD)
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp if stamp is not None else rospy.Time.now()
    marker.scale.x = size
    marker.scale.y = size
    marker.color.b = 0.3
    marker.color.a = 1.0
    marker.color.g = 0.7
    marker.pose.position = geom_msg.Point(0, 0, 0)
    marker.pose.orientation = geom_msg.Quaternion(0, 0, 0, 1)

    edg = ndb.get_all_edges()
    marker.points = []
    for e in edg:
        marker.points.extend([geom_msg.Point(e[0][0], e[0][1], 0),
                              geom_msg.Point(e[1][0], e[1][1], 0)])

    marker.lifetime = rospy.Duration(1)
    pubNodes.publish(marker)    


# def addFrontier(msg):
subPose = rospy.Subscriber('/slam_out_pose', PoseStamped, updatePos)
# /localization/out/pose
# subFrontiers = rospy.Subscriber('/pathpl PoseArray, addFrontier)
# 

pubNodes = rospy.Publisher('/pubNodes', Marker, queue_size=10)

rospy.init_node("buildGraph", anonymous=True)

rate = rospy.Rate(1)
while not rospy.is_shutdown():
    publishGraph(pubNodes, frame_id='map')
    rate.sleep()

# rospy.spin()

if __name__ == "__main__":
    print("buildGraph.py run as main")
