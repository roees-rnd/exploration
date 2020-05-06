#!/usr/bin/env python3
import net_db

import rosparam
import rospy
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import OccupancyGrid
from geometry_msgs import msg as geom_msg
import numpy as np
import copy


# ndb.add_node((0, 0.1), False)
# ndb.add_node((2, 0.1), True)
# pos, is_door = ndb.get_all_nodes()

class buildGraph:
    def __init__(self):
        rospy.Subscriber('/slam_out_pose', PoseStamped, self.updatePos)
        # rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.publishPath)
        self.ndb = net_db.net_db()
        self.pubNodes = rospy.Publisher('/pubNodes', Marker, queue_size=10)
        self.pubPathMarker = rospy.Publisher('/pubPath', Marker, queue_size=10)
        

    def updatePos(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.ndb.add_node((x, y))
        self.publishGraph(frame_id='map')


    def publishGraph(self, pub_ns='net', stamp=None, frame_id='world', size=0.03, numLmks=0):
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

        edg = self.ndb.get_all_edges()
        marker.points = []
        for e in edg:
            marker.points.extend([geom_msg.Point(e[0][0], e[0][1], 0),
                                geom_msg.Point(e[1][0], e[1][1], 0)])

        marker.lifetime = rospy.Duration(1)
        self.pubNodes.publish(marker)

    # def publishPath(self, msg, pub_ns='net', stamp=None, frame_id='map', size=0.1):
    #     marker = Marker(type=Marker.LINE_LIST, ns=pub_ns, action=Marker.ADD)
    #     marker.header.frame_id = frame_id
    #     marker.header.stamp = stamp if stamp is not None else rospy.Time.now()
    #     marker.scale.x = size
    #     marker.scale.y = size
    #     marker.color.b = 0.7
    #     marker.color.a = 1.0
    #     marker.color.g = 0.3
    #     marker.pose.position = geom_msg.Point(0, 0, 0)
    #     marker.pose.orientation = geom_msg.Quaternion(0, 0, 0, 1)

    #     xy = (msg.pose.position.x, msg.pose.position.y)
    #     nodes, _ = self.ndb.nodes_are_eq(xy, thresh=0.5)
    #     if len(nodes)<1:
    #         return

    #     node_list = self.path_to_target(nodes[0])
    #     marker.points = []
    #     if node_list is None:
    #         return

    #     for i in range(1,len(node_list)):
    #         marker.points.extend([geom_msg.Point(node_list[i-1][0], node_list[i-1][1], 0),
    #                             geom_msg.Point(node_list[i][0], node_list[i][1], 0)])

    #     # marker.lifetime = rospy.Duration(5)
    #     self.pubPath.publish(marker)

    

    def getPoseArrayToTarget_as_poseArray(self, xy_trg=(0, 0), stamp=None, frame_id='map', vis=True):
        route_as_pose_array = PoseArray()
        route_as_pose_array.header.frame_id = frame_id
        route_as_pose_array.header.stamp = rospy.Time.now()

        nodes, _ = self.ndb.nodes_are_eq(xy_trg, thresh=0.5)
        
        if len(nodes)<1:
            return route_as_pose_array

        # TODO: select closest node:
        node_list = self.path_to_target(nodes[0])

        if node_list is None:
            return route_as_pose_array

        p=Pose()
        for n in node_list:
            p.position.x = n[0]
            p.position.y = n[1]
            p.position.z = 0.7
            route_as_pose_array.poses.append(copy.deepcopy(p))

        if vis:
            self.publishPoseArray_as_markerLineList(route_as_pose_array)

        return route_as_pose_array
    
    def publishPoseArray_as_markerLineList(self, poseArray, pub_ns='net', size=0.1):
        marker = Marker(type=Marker.LINE_LIST, ns=pub_ns, action=Marker.ADD)
        marker.header.frame_id = poseArray.header.frame_id
        marker.header.stamp = poseArray.header.stamp
        marker.scale.x = size
        marker.scale.y = size
        marker.color.b = 0.7
        marker.color.a = 1.0
        marker.color.g = 0.3
        marker.pose.position = geom_msg.Point(0, 0, 0)
        marker.pose.orientation = geom_msg.Quaternion(0, 0, 0, 1)

        for i in range(1,len(poseArray.poses)):
            marker.points.extend([geom_msg.Point(poseArray.poses[i-1].position.x, poseArray.poses[i-1].position.y, 0),
                                geom_msg.Point(poseArray.poses[i].position.x, poseArray.poses[i].position.y, 0)])

        # marker.lifetime = rospy.Duration(5)
        self.pubPathMarker.publish(marker)


    def path_to_target(self, xy):
        nodes, _ = self.ndb.nodes_are_eq(xy, thresh=0.5)
        if len(nodes)>0 and (self.ndb.last_node is not None):
            trg = tuple(nodes[0])
            src = self.ndb.last_node
            node_list, weights = self.ndb.get_path(src, trg)
            return node_list
        else:
            node_list = None



if __name__ == "__main__":
    import buildGraph
    rospy.init_node("buildGraph", anonymous=True)
    bg = buildGraph.buildGraph()
    rospy.spin()
    print("buildGraph.py run as main")
