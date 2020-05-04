#!/usr/bin/python
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped, WrenchStamped, Pose
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from nav_msgs.msg import OccupancyGrid
from MapInfo import MapInfo

import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import matplotlib.cm as cm
import Astar
import copy
import time
import cProfile
import re


class PathPlannerClass:
    def __init__(self):
        version = '1.0.1'

        self.drone_pos_world = np.zeros(3)
        self.drone_yaw = 0.0

        self.goal_pos_world = np.zeros(3)
        self.goal_pos_world[2] = 0.7
        self.goal_yaw = 0.0

        self.statusMsg = DiagnosticArray()
        self.statusMsg.status.append(DiagnosticStatus())
        self.statusMsg.status[0].name = 'path_planner'
        self.statusMsg.status[0].level = DiagnosticStatus.OK
        # make sure its ok with Alex
        self.statusMsg.status[0].message = 'NOT_OPERATIONAL'
        # self.status_msgs = ['NOT_OPERATIONAL', 'PROCESSING', 'INVALID_SETPOINT', 'FINISHED', 'EXECUTING', 'NO_VALID_ROUTE'] # make sure its ok with Alex
        self.exection_status_msg = 'IDLE'
        self.mode_msg = 'GOTO'

        self.map = None
        self.Astar_Movement = []
        self.no_path_flag = False

        self.params = {'distance_converge_last_point_m': 0.0,
                       'distance_converge_m': 0.0,
                       'heading_convergence_last_point_rad': 0.0,
                       'heading_convergence_rad': 0.0,
                       'counter_last_point': 0,
                       'failure_angle_rad': 0.0,
                       'failure_angle_counter': 0}
        # publishers
        PubVer = rospy.Publisher(
            '/version/path_planner', String, latch=True, queue_size=1)  # version
        # status: ['NOT_OPERATIONAL', 'PROCESSING', 'INVALID_SETPOINT', 'FINISHED', 'EXECUTING', 'NO_VALID_ROUTE']
        self.PubStatus = rospy.Publisher(
            '/path_planner/out/status', DiagnosticArray, queue_size=10)
        self.PubRoute = rospy.Publisher(
            '/path_planner/wp_handler/route', PoseArray, queue_size=1)  # list of minimal WP
        PubVer.publish(version)
        rospy.set_param('/version/path_planner', version)

        # get parameters from rosparam server. exit if any parameter not found
        if not self.GetParams():
            exit(0)

         # subdcribers
        # rospy.Subscriber('/localization/out/pose', PoseStamped, self.save_self_pos) # saves self.pos
        rospy.Subscriber('/mavros/local_position/pose',
                         PoseStamped, self.save_self_pos)  # saves self.pos
        rospy.Subscriber('/wp_provider/path_planner/set_point', PoseStamped,
                         self.save_goal_pos)  # saves position of goal point
        # recives from wp_handler: ['IDLE', 'PROCESSING', 'FINISHED', 'FAILURE']
        rospy.Subscriber('/wp_handler/out/status',
                         String, self.get_exection_status)
        # set mode from ['IDLE', 'GOTO', 'EXPLORATION']
        rospy.Subscriber('/FFK/path_planner/mode',
                         DiagnosticArray, self.set_mode)
        rospy.Subscriber("/map", OccupancyGrid, callback=self.get_map)

        rospy.Timer(rospy.Duration(1), self.SendStatus)

    def GetParams(self):
        # for prm in self.params.keys():
        #     if rospy.has_param('/path_planner/' + prm):
        #         self.params[prm] = rospy.get_param('/path_planner/' + prm)
        #     else:
        #         self.statusMsg.status[0].level = DiagnosticStatus.ERROR
        #         self.statusMsg.status[0].message = 'Failed to get parameters: ' + '/path_planner/' + prm
        #         self.PubStatus.publish(self.statusMsg)
        #         rospy.logerr('Failed to get parameters: ' + '/path_planner/'  + prm)
        #         return False
        return True

    def SendStatus(self, event):
        self.statusMsg.header.seq += 1
        self.statusMsg.header.stamp = rospy.Time.now()
        self.PubStatus.publish(self.statusMsg)

    def save_self_pos(self, msg):
        if self.map is None:
            delta_x = delta_y = delta_z = 0
        else:
            delta_x = self.map.map_delta_x
            delta_y = self.map.map_delta_y
            delta_z = self.map.map_delta_z
        self.drone_pos_world[0] = msg.pose.position.x+delta_x
        self.drone_pos_world[1] = msg.pose.position.y+delta_y
        self.drone_pos_world[2] = msg.pose.position.z+delta_z
        self.drone_yaw = euler_from_quaternion(np.array(
            [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]))[2]

    def save_goal_pos(self, msg):
        yaw_goal = euler_from_quaternion(np.array(
            [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]))[2]
        if self.map is None:
            delta_x = delta_y = delta_z = 0
        else:
            delta_x = self.map.map_delta_x
            delta_y = self.map.map_delta_y
            delta_z = self.map.map_delta_z
        if ((np.linalg.norm(np.array([msg.pose.position.x+delta_x, msg.pose.position.y+delta_y, msg.pose.position.z+delta_z])-self.goal_pos_world) > 0.1) or
                (np.abs(self.goal_yaw - yaw_goal) > 0.01)):

            # self.use_Gui(msg, yaw_goal, delta_x, delta_y, delta_z)

            self.goto_with_gui(msg, yaw_goal, delta_x, delta_y, delta_z)


    def goto_with_gui(self, msg, yaw_goal, delta_x, delta_y, delta_z):
        self.goal_pos_world[0] = msg.pose.position.x+delta_x
        self.goal_pos_world[1] = msg.pose.position.y+delta_y
        self.goal_pos_world[2] = msg.pose.position.z+delta_z
        self.goal_yaw = yaw_goal
        self.gotoAB(self.drone_pos_world ,self.goal_pos_world)
        #self.print_map()
        if self.no_path_flag == False:

            msgOut = PoseArray()
            msgOut.header.frame_id = 'map'
            msgOut.header.stamp = rospy.Time.now()
            pose = Pose()
            for i in range(len(self.Astar_Movement)):
                pose.position.x = self.Astar_Movement[i][0] * self.map.map_resolution - delta_x
                pose.position.y = self.Astar_Movement[i][1] * self.map.map_resolution - delta_y
                pose.position.z = self.drone_pos_world[2]
                pose.orientation.w = 1
                msgOut.poses.append(copy.deepcopy(pose))

            pose = Pose()
            pose.position.x = self.goal_pos_world[0] - delta_x
            pose.position.y = self.goal_pos_world[1] - delta_y
            pose.position.z = self.drone_pos_world[2]
            pose.orientation.w = 1
            msgOut.poses.append(copy.deepcopy(pose))

            self.PubRoute.publish(msgOut)


    def use_Gui(self, msg, yaw_goal, delta_x, delta_y, delta_z):
        self.goal_pos_world[0] = msg.pose.position.x+delta_x
        self.goal_pos_world[1] = msg.pose.position.y+delta_y
        self.goal_pos_world[2] = msg.pose.position.z+delta_z
        self.goal_yaw = yaw_goal

        # for now - msg in msg out! TODO:change this!
        msg_out = PoseArray()
        msg_out.header = msg.header
        msg_out.poses.append(msg.pose)
        self.PubRoute.publish(msg_out)


    def get_exection_status(self, msg):
        # self.exection_status_msg = msg.status[0].message #['IDLE', 'PROCESSING', 'FINISHED', 'FAILURE']
        # ['IDLE', 'PROCESSING', 'FINISHED', 'FAILURE']
        self.exection_status_msg = msg.data

    def set_mode(self, msg):
        # ['IDLE', 'GOTO', 'EXPLORATION']
        self.mode_msg = msg.status[0].message

    def get_map(self, msg):
        if self.map is None:
            self.map = MapInfo(msg.info.width, msg.info.height, msg.info.resolution,
                               msg.info.origin.position.x, msg.info.origin.position.y, msg.info.origin.position.z)
        self.map.set_map(msg.data)
        # self.print_map()
  

    def gotoAB(self, s, t):
        if self.map is None or self.map.map is None:
            self.Astar_Movement = []
            self.no_path_flag = True
            #print("gotoAB error: No map recived!")
        else:
            start = s/self.map.map_resolution
            goal = t/self.map.map_resolution

            env_limits = np.array(
                [0, self.map.map_sizeX, 0, self.map.map_sizeY])
            grid = self.map.map.T

            num_of_nodes = [25, 40, 75]#, 100, 120]

            # success_flag = False
            # for n in num_of_nodes:
            #     print("\nTry planning again- now with ", str(n), " nodes")
            #     start_time = time.time()
            #     self.Astar_Movement, self.no_path_flag = Astar.build_trj(
            #     start, env_limits, 1, grid, goal, 0, n)
            #     end_time = time.time()
            #     t = (end_time - start_time) * 1000  # msec
            #     print("planning with ", n, " nodes took ", t, " msec")
            #     if self.no_path_flag ==False:
            #         print(" ")
            #         return
            # print("Faild to find path....\n")
            n=55
            self.no_path_flag = True
            max_number_of_tries = 7
            while self.no_path_flag and max_number_of_tries:
                # print("")
                # print("")
                # print("Planning try number: ", str(7-max_number_of_tries+1))
                # start_time = time.time()
                # cProfile.run('re.compile("Astar.build_trj(start, env_limits, 1, grid, goal, 0, n) #n= the maximum number of nodes allowed")')
                # self.Astar_Movement, self.no_path_flag = Astar.build_trj(
                # start, env_limits, 1, grid, goal, 0, n) #n= the maximum number of nodes allowed
                # end_time = time.time()
                # t = (end_time - start_time) * 1000  # msec
                # print("planning with ", n, " nodes took ", t, " msec")
                # print("")
                # print("")
                # max_number_of_tries-=1



                # print("")
                # print("")
                # print("Planning try number: ", str(7-max_number_of_tries+1))
                start_time = time.time()
                import cProfile, pstats, StringIO
                pr = cProfile.Profile()
                pr.enable()
                self.Astar_Movement, self.no_path_flag = Astar.build_trj(
                start, env_limits, 1, grid, goal, 0, n) #n= the maximum number of nodes allowed
                pr.disable()
                s = StringIO.StringIO()
                sortby = 'cumtime'#'cumulative' 
                ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
                #ps.print_stats()
                #print s.getvalue()
                end_time = time.time()
                t = (end_time - start_time) * 1000  # msec
                print("planning with ", n, " nodes took ", t, " msec")
                print("")
                print("")
                max_number_of_tries-=1



    def print_map(self):
        if self.map is None or self.map.map is None:
            return
        grid = copy.deepcopy(self.map.map)
        grid[np.where(grid == -1)] = 150
        fig = plt.figure()
        ax1 = fig.add_subplot(111)
        ax1.imshow(grid)
        if self.drone_pos_world[0] != 0 and self.drone_pos_world[1] != 0:
            plt.scatter(self.drone_pos_world[0]/self.map.map_resolution,
                        self.drone_pos_world[1]/self.map.map_resolution, s=25, c='red', marker='o')
        if self.goal_pos_world[0] != 0 and self.goal_pos_world[1] != 0:
            plt.scatter(self.goal_pos_world[0]/self.map.map_resolution,
                        self.goal_pos_world[1]/self.map.map_resolution, s=25, c='blue', marker='o')
        for a in range(len(self.Astar_Movement)):
            plt.scatter(
                self.Astar_Movement[a][0], self.Astar_Movement[a][1], s=25, c='white', marker='o')
        plt.show()


import tempfile

if __name__ == "__main__":
    rospy.init_node('PathPlannerNode')
    path_planner = PathPlannerClass()
    rospy.spin()
    # (fd, filename) = tempfile.mkstemp(suffix=".prof")
    # print("Profile info: {}".format(filename))
    # cProfile.runctx("rospy.spin()", globals(), locals(), filename=filename)
    # cProfile.runctx('rospy.spin()', None, locals())
