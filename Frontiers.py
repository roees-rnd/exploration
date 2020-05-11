##!/usr/bin/env python


#--------Include modules---------------
import copy
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from MapInfo import MapInfo
from geometry_msgs.msg import PoseArray, PoseStamped, WrenchStamped, Pose
import time
import numpy as np
import sys
sys.path.append("/usr/lib/python3/dist-packages")  # this is for raspi cv2 (already installed outside venv)
import cv2
import matplotlib.pyplot as plt
from sklearn.cluster import MeanShift, AffinityPropagation



class FrontierClass:
	def __init__(self, DEBUG_FLAG=False):
		self.DEBUG_FLAG = DEBUG_FLAG
		self.list_of_frontiers = []
		self.new_frontiers = []
		self.irrelevant_frontiers = []
		self.irrelevant_frontiers_full_run = []
		self.map_of_contours = []
		self.mapInfo = None

		# publishers
		self.PubFrontier = rospy.Publisher('/path_planner/frontiers', PoseArray, queue_size=1)  # list of frontiers to publish
		# Subscribers
		#rospy.Subscriber("/map", OccupancyGrid, callback=self.do_step)

	def saveMap(self, msg):
		if self.mapInfo is None:
			self.map = MapInfo(msg.info.width, msg.info.height, msg.info.resolution,
								msg.info.origin.position.x, msg.info.origin.position.y, msg.info.origin.position.z)
		self.map.set_map(msg.data)

	def do_step(self, mapData):
		# self.saveMap(mapData)
		# mapData = self.map
		# relevant_frontiers = self.remove_irrelevant_frontiers()
		# current_frontiers = self.getfrontier(mapData)

		relevant_frontiers = self.remove_irrelevant_frontiers()
		current_frontiers = self.getfrontier(mapData)

		new_shifted_frontiers = self.add_frontier_context(current_frontiers)
		Frontiers_list = self.add_new_frontiers_to_FL(new_shifted_frontiers)
		self.list_of_frontiers = Frontiers_list
		#self.publish_frontiers(relevant_frontiers)
		return relevant_frontiers


	def cluster_points(self, all_pts):
		# front = copy.deepcopy(all_pts)
		if len(all_pts)>1:
			# ms = MeanShift(bandwidth=0.3, bin_seeding=True)   
			# ms.fit(front)
			# centroids= ms.cluster_centers_	 #centroids array is the centers of each cluster

			clustering = AffinityPropagation().fit(all_pts)
			centroids= clustering.cluster_centers_


		else:
			#if only one froniter exists- no need to do fit
			centroids = all_pts
		return centroids


	def getfrontier(self, mapInfo):
		start_time = time.time()
		self.mapInfo = mapInfo
		data = self.mapInfo.map
		# self.mapData_w=mapData.info.width
		# self.mapData_h=mapData.info.height

		# create image from OCG
		start_time_np_where = time.time()
		img = np.zeros((mapInfo.map_sizeY, mapInfo.map_sizeX, 1), np.uint8)
		# x = np.where(data==100) 
		# img_np_where[x[0], x[1]]=0
		x = np.where(data==0) 
		img[x[0], x[1]]=255
		x = np.where(data==-1) 
		img[x[0], x[1]]=205
		o=cv2.inRange(img,0,1)
		end_time_np_where = time.time()
		t_np_where = (end_time_np_where -start_time_np_where) * 1000  # msec
		t_image =t_np_where

		# use canny algo to find edges in img
		start_time_canny = time.time()
		edges_np_where = cv2.Canny(img,0,255)
		end_time_canny = time.time()
		t_canny = (end_time_canny - start_time_canny) * 1000  # msec
		edges= edges_np_where
		

		if self.DEBUG_FLAG:
			# print edges
			fig = plt.figure()
			ax1 = fig.add_subplot(111)
			ax1.imshow(edges)
			plt.show()

		# find contours in img
		if cv2.__version__.split('.')[0]=="3":
			start_time_contours = time.time()
			_, contours, _ = cv2.findContours(o,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
			cv2.drawContours(o, contours, -1, (255,255,255), 5)
			o=cv2.bitwise_not(o) 
			res = cv2.bitwise_and(o,edges)
			#------------------------------

			frontier=copy.deepcopy(res)
			_, contours, _ = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
			cv2.drawContours(frontier, contours, -1, (255,255,255), 2)

			im2, contours, _ = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
			#im2 = cv2.drawContours(frontier, contours, -1, (255, 0, 0))
			self.map_of_contours = im2
			end_time_contours = time.time()
			t_contours = (end_time_contours - start_time_contours) * 1000  # msec
		else:
			start_time_contours = time.time()
			contours, _ = cv2.findContours(o,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
			cv2.drawContours(o, contours, -1, (255,255,255), 5)
			o=cv2.bitwise_not(o) 
			res = cv2.bitwise_and(o,edges)
			#------------------------------

			frontier=copy.deepcopy(res)
			contours, _ = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
			cv2.drawContours(frontier, contours, -1, (255,255,255), 2)

			contours, _ = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
			im2 = cv2.drawContours(frontier, contours, -1, (255, 0, 0))
			self.map_of_contours = im2
			end_time_contours = time.time()
			t_contours = (end_time_contours - start_time_contours) * 1000  # msec

		if self.DEBUG_FLAG:
			# print contours
			cv2.imshow('image',im2)
			cv2.waitKey(0)

		all_pts=[]
		if len(contours)>0:
			upto=len(contours)-1
			i=0
			maxx=0
			maxind=0
			
			for i in range(0,len(contours)):
					cnt = contours[i]
					M = cv2.moments(cnt)
					cx = int(M['m10']/M['m00'])
					cy = int(M['m01']/M['m00'])
					xr, yr = mapInfo.ij_to_xy(cx, cy)
					pt=[np.array([xr,yr])]
					if len(all_pts)>0:
						all_pts=np.vstack([all_pts,pt])
					else:
						all_pts=pt

		# centroids_start_time = time.time()
		# centroids = self.cluster_points(all_pts)
		# centroids_end_time = time.time()
		# t_centroids = (centroids_end_time - centroids_start_time) * 1000  # msec

		end_time = time.time()
		t = (end_time - start_time) * 1000  # msec
		#print("finding frontiers took", t, "msec. the frontiers are: ", centroids)
		#print("finding frontiers took", t, "msec. the frontiers are: ")#, all_pts)
		print("finding frontiers took", t, "msec. ", str(len(all_pts)), " fruntiers found!")
		print("contours clac took: ", str(t_contours), ", creat image took: ", str(t_image), ", canny took: ", str(t_canny), " msec\n")#", clusering took: ", str(t_centroids), " msec\n")
		#self.print_frontiers(all_pts) , centroids)
		return all_pts


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


	def print_frontiers(self, point_list_1, point_list_2=[]):
		grid = copy.deepcopy(self.mapInfo.map)
		#grid = np.reshape(copy.deepcopy(self.mapData_data), (self.mapData_w ,self.mapData_h))
		grid[np.where(grid == -1)] = 150
		fig = plt.figure()
		ax1 = fig.add_subplot(122)
		ax1.imshow(grid)
		#plot points in point_list_1
		for a in range(len(point_list_1)):
			i, j = self.mapInfo.xy_to_ij(point_list_1[a][0], point_list_1[a][1])
			plt.scatter(i, j, s=25, c='white', marker='o')

		#plot points in point_list_2
		if (point_list_2!=[]):
			ax2 = fig.add_subplot(121)
			ax2.imshow(grid)
			for a in range(len(point_list_2)):
				i, j = self.mapInfo.xy_to_ij(point_list_2[a][0], point_list_2[a][1])
				plt.scatter(i, j, s=25, c='white', marker='o')

		plt.show()

	def add_frontier_context(self, current_frontiers):
		if current_frontiers==[]:
			print("no frontiers found!")
			return []

		shifted_frontiers = []
		for p in current_frontiers:
			
			#TODO: add logic
			#if p in BB_door- change p to be door
			#shifted_p = self.move_point_to_free(p)
			#shifted_frontiers.append(shifted_p)
			shifted_frontiers.append(p)
			
		# #print before and after shift
		# self.print_frontiers(self.list_of_frontiers, shifted_frontiers)
		
		return shifted_frontiers

	def move_point_to_free(self, p):
		BB_margin = 3
		pi,pj = self.mapInfo.xy_to_ij(p[0],p[1])
		if self.mapInfo.map[pj,pi] ==-1:
			s = self.mapInfo.map[(-BB_margin+pj):(pj+BB_margin),(-BB_margin+pi):(pi+BB_margin)]
			free_points_in_s = np.where(s==0)
			first_free_point_s_index_x = free_points_in_s[0][0]
			first_free_point_full_index_x = first_free_point_s_index_x+pj-BB_margin

			first_free_point_s_index_y = free_points_in_s[1][0]
			first_free_point_full_index_y = first_free_point_s_index_x+pi-BB_margin
			px, py = self.mapInfo.ij_to_xy(first_free_point_full_index_x, first_free_point_full_index_y)
			return np.array([px,py])

		return p
		# fig = plt.figure()
		# ax1 = fig.add_subplot(111)
		# self.mapInfo.map[pj,pi]=200
		# self.mapInfo.map[first_free_point_full_index_x,first_free_point_full_index_y]=300
		# ax1.imshow(self.mapInfo.map)
		# plt.show()

	
	def shift_frontiers(self, new_FL):
		frontier_context = self.add_frontier_context(list())
		return frontier_context

	def add_new_frontiers_to_FL(self, new_FL):
		old_FL = self.list_of_frontiers
		uniq_FL = self.remove_dubble_frontiers(old_FL, new_FL)
		return uniq_FL

	def is_point_new_frontier(self, list_of_points, p2):
		delta = 0.3

		diff = np.abs(np.array(list_of_points)-np.array(p2))
		if np.min(diff)<delta:
			return True
		return False

	def remove_dubble_frontiers(self, old_FL, new_FL):
		if not old_FL: #if there are no old frontiers- return the newly found frontiers
			self.new_frontiers = new_FL
			return new_FL

		self.new_frontiers = []
		uniq_FL = [] + old_FL
		for p2 in new_FL:
			if not self.is_point_new_frontier(old_FL,p2):
				uniq_FL.append(p2)
				self.new_frontiers.append(p2)
		return uniq_FL


	def is_point_still_frontier(self, p):
		BB_margin = 3
		pi, pj = self.mapInfo.xy_to_ij(p[0], p[1])
		slice_contours = self.map_of_contours[(-BB_margin+int(pj)):(BB_margin+int(pj)), (-BB_margin+int(pi)):(BB_margin+int(pi))]
		s = np.sum(slice_contours)
		if s == 0:
			if self.DEBUG_FLAG:
				# print contours
				fig = plt.figure()
				ax1 = fig.add_subplot(111)
				ax1.imshow(slice_contours)
				plt.show()
				# cv2.imshow('image',slice_contours)
				# cv2.waitKey(0)

				fig = plt.figure()
				ax1 = fig.add_subplot(111)
				map_of_contours = copy.deepcopy(self.map_of_contours)
				map_of_contours[pj, pi]=200
				ax1.imshow(map_of_contours)
				plt.show()
			return False
		return True

	def remove_irrelevant_frontiers(self):
		relevant_frontiers=[]
		irrelevant_frontiers = []
		if self.list_of_frontiers is None:
			return []
		for p in self.list_of_frontiers:
			if self.is_point_still_frontier(p):
				relevant_frontiers.append(p)
			else:
				irrelevant_frontiers.append(p)
				print("point: ", p, " is no longer frontier!")
		self.list_of_frontiers = relevant_frontiers
		self.irrelevant_frontiers = irrelevant_frontiers
		self.irrelevant_frontiers_full_run = self.irrelevant_frontiers_full_run + irrelevant_frontiers
		return relevant_frontiers




		




if __name__ == "__main__":
    rospy.init_node('FrontierClassNode')
    frontiers = FrontierClass()
    rospy.spin()
