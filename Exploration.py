
import copy
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from MapInfo import MapInfo
from geometry_msgs.msg import PoseArray, PoseStamped, WrenchStamped, Pose
import time
import numpy as np
# import cv2
from Frontiers import FrontierClass
import buildGraph

# import sys
# sys.path.append("/home/roee/catkin_ws/src/exploration/env/lib/python3.6/site-packages")

class ExplorationCalss:
    def __init__(self):
        self.DEBUG_FLAG = False

        # publishers
        self.PubFrontier = rospy.Publisher(
            '/path_planner/frontiers', PoseArray, queue_size=1)  # list of frontiers to publish
        # PubVer.publish(version)
        # rospy.set_param('/version/path_planner', version)

        # subdcribers
        # rospy.Subscriber('/localization/out/pose', PoseStamped, self.save_self_pos) # saves self.pos
        # rospy.Subscriber('/mavros/local_position/pose',
        # 					PoseStamped, self.save_self_pos)  # saves self.pos
        rospy.Subscriber("/map", OccupancyGrid, callback=self.do_step)

        self.frontiers = FrontierClass()
        self.current_frontiers = []
        self.route_to_point = []
        #self.graph = Graph()
        #self.chose_mission = ChoseExplorationPointClass()
        self.buildGraph = buildGraph.buildGraph()

        self.map = None

    def saveMap(self, msg):
        if self.map is None:
            self.map = MapInfo(msg.info.width, msg.info.height, msg.info.resolution,
                               msg.info.origin.position.x, msg.info.origin.position.y, msg.info.origin.position.z)
        self.map.set_map(msg.data)
        self.buildGraph.ndb.map_info = self.map

    def do_step(self, mapData):
        self.saveMap(mapData)
        self.current_frontiers = self.frontiers.do_step(self.map)
        #self.publish_frontiers(self.current_frontiers)
        nodes_to_add = self.frontiers.new_frontiers
        for n in current_frontiers:  # nodes_to_add:
            self.buildGraph.ndb.add_node_map((n[0], n[1]))
        # nodes_to_remove = self.frontiers.irrelevant_frontiers
        #self.graph.update(self.map, nodes_to_add, nodes_to_remove)
        #currentTarget = self.chose_mission.do_step(self.graph, currentPos, self.current_frontiers)
        #path = self.graph.route(currentPos, currentTarget)

    def route_to_point(self, goal):
        self.route_to_point = goal
        path = self.graph.route(goal)
        return path

    def route_to_best_frontier(self):
        currentTarget = self.chose_mission.do_step(self.graph, current_frontiers)
        self.route_to_point = currentTarget
        path = self.graph.route(currentTarget)
        return path


    def publish_frontiers(self, frontiers_list):
		msgOut = PoseArray()
		msgOut.header.frame_id = 'map'
		msgOut.header.stamp = rospy.Time.now()
		pose = Pose()
		for i in range(len(frontiers_list)):
			pose.position.x = frontiers_list[i][0]
			pose.position.y = frontiers_list[i][1]
			pose.position.z = 0.7
			pose.orientation.w = 1
			msgOut.poses.append(copy.deepcopy(pose))
		self.PubFrontier.publish(msgOut)

    def publish_frontiers(self, frontiers_list):
        msgOut = PoseArray()
        msgOut.header.frame_id = 'map'
        msgOut.header.stamp = rospy.Time.now()
        pose = Pose()
        for i in range(len(frontiers_list)):
            pose.position.x = frontiers_list[i][0]
            pose.position.y = frontiers_list[i][1]
            pose.position.z = 0.7
            pose.orientation.w = 1
            msgOut.poses.append(copy.deepcopy(pose))
        self.PubFrontier.publish(msgOut)


if __name__ == "__main__":
    rospy.init_node('ExplorationCalssNode')
    exploration = ExplorationCalss()
    rospy.spin()
