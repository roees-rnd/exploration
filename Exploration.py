
import copy
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from MapInfo import MapInfo
from geometry_msgs.msg import PoseArray, PoseStamped, WrenchStamped, Pose
import time
import numpy as np
import cv2
from Frontiers import FrontierClass


class ExplorationCalss:
    def __init__(self):
        self.DEBUG_FLAG = False

        # publishers
        self.PubFrontier = rospy.Publisher('/path_planner/frontiers', PoseArray, queue_size=1)  # list of frontiers to publish
        # PubVer.publish(version)
        # rospy.set_param('/version/path_planner', version)


        # subdcribers
        # rospy.Subscriber('/localization/out/pose', PoseStamped, self.save_self_pos) # saves self.pos
        # rospy.Subscriber('/mavros/local_position/pose',
        # 					PoseStamped, self.save_self_pos)  # saves self.pos
        rospy.Subscriber("/map", OccupancyGrid, callback=self.do_step)

        self.frontiers = FrontierClass()
        #self.graph = Graph()
        #self.chose_mission = ChoseExplorationPointClass()


    def do_step(self, mapData):
        current_frontiers = self.frontiers.do_step(mapData)
        self.publish_frontiers(current_frontiers)
        nodes_to_add = self.frontiers.new_frontiers
        nodes_to_remove = self.frontiers.irrelevant_frontiers
        #self.graph.update(nodes_to_add, nodes_to_remove)
        #currentTarget = self.chose_mission.do_step(self.graph, currentPos, current_frontiers)
        #path = self.graph.route(currentPos, currentTarget)

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