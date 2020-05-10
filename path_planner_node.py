#!/usr/bin/python
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped, WrenchStamped
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from nav_msgs.msg import OccupancyGrid
from Exploration import ExplorationCalss

class PathPlannerClass:
    def __init__(self):
        version = '1.0.1'

        self.drone_pos = np.zeros(3)
        self.drone_yaw = 0.0

        self.goal_pos = np.zeros(3)
        self.goal_yaw = 0.0

        self.statusMsg = DiagnosticArray()
        self.statusMsg.status.append(DiagnosticStatus())
        self.statusMsg.status[0].name = 'path_planner'
        self.statusMsg.status[0].level = DiagnosticStatus.OK
        self.statusMsg.status[0].message = 'NOT_OPERATIONAL' # make sure its ok with Alex
        # self.status_msgs = ['NOT_OPERATIONAL', 'PROCESSING', 'INVALID_SETPOINT', 'FINISHED', 'EXECUTING', 'NO_VALID_ROUTE'] # make sure its ok with Alex
        self.exection_status_msg = 'IDLE'
        self.mode_msg = 'GOTO'
        self.explortion = ExplorationCalss()

        self.params = {'distance_converge_last_point_m': 0.0,
                        'distance_converge_m': 0.0,
                        'heading_convergence_last_point_rad': 0.0,
                        'heading_convergence_rad': 0.0,
                        'counter_last_point': 0,
                        'failure_angle_rad': 0.0,
                        'failure_angle_counter': 0}
        # publishers
        PubVer = rospy.Publisher('/version/path_planner', String, latch=True, queue_size=1) # version
        self.PubStatus = rospy.Publisher('/path_planner/out/status', DiagnosticArray, queue_size=10) # status: ['NOT_OPERATIONAL', 'PROCESSING', 'INVALID_SETPOINT', 'FINISHED', 'EXECUTING', 'NO_VALID_ROUTE']
        self.PubRoute = rospy.Publisher('/path_planner/wp_handler/route', PoseArray, queue_size=1) # list of minimal WP
        PubVer.publish(version)
        rospy.set_param('/version/path_planner', version)
        
        # get parameters from rosparam server. exit if any parameter not found
        if not self.GetParams():
            exit(0)

         # subdcribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.route_from_exp)
        rospy.Subscriber('/localization/out/pose', PoseStamped, self.save_self_pos) # saves self.pos
        rospy.Subscriber('/wp_provider/path_planner/set_point', PoseStamped, self.save_goal_pos) # saves position of goal point 
        rospy.Subscriber('/wp_handler/out/status', String, self.get_exection_status) #  recives from wp_handler: ['IDLE', 'PROCESSING', 'FINISHED', 'FAILURE']
        rospy.Subscriber('/FFK/path_planner/mode', DiagnosticArray, self.set_mode) # set mode from ['IDLE', 'GOTO', 'EXPLORATION']
        #rospy.Subscriber("/map", OccupancyGrid, callback=self.get_map)
        
        rospy.Timer(rospy.Duration(1),self.SendStatus)


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
        self.drone_pos[0]= msg.pose.position.x
        self.drone_pos[1]= msg.pose.position.y
        self.drone_pos[2]= msg.pose.position.z
        self.drone_yaw = euler_from_quaternion(np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]))[2]

    def save_goal_pos(self, msg):
        if self.mode_msg=="GO_TO":
            yaw_goal = euler_from_quaternion(np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]))[2]
            if ((np.linalg.norm(np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])-self.goal_pos) > 0.1) or
                (np.abs(self.goal_yaw - yaw_goal) > 0.01)):
                self.goal_pos[0]= msg.pose.position.x
                self.goal_pos[1]= msg.pose.position.y
                self.goal_pos[2]= msg.pose.position.z
                self.goal_yaw = yaw_goal
                
                
                # for now - msg in msg out! TODO:change this!
                msg_out = PoseArray()
                msg_out.header = msg.header
                msg_out.poses.append(msg.pose)
                self.PubRoute.publish(msg_out)

    def route_from_exp(self, msg):
        if self.mode_msg == "EXPLORATION":
            xy = (msg.pose.position.x, msg.pose.position.y)
            pose_array = self.exploration.buildGraph.getPoseArrayToTarget_as_poseArray(xy, vis=True)
            self.PubRoute.publish(pose_array)
        

    def get_exection_status(self, msg):
        # self.exection_status_msg = msg.status[0].message #['IDLE', 'PROCESSING', 'FINISHED', 'FAILURE']
        self.exection_status_msg = msg.data #['IDLE', 'PROCESSING', 'FINISHED', 'FAILURE']

    def set_mode(self, msg):
        self.mode_msg = msg.status[0].message #['IDLE', 'GOTO', 'EXPLORATION']



if __name__ == "__main__":
    rospy.init_node('PathPlannerNode')
    path_planner = PathPlannerClass()
    rospy.spin()