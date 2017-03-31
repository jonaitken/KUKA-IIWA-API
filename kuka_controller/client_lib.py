#!/usr/bin/env python

# KUKA API for ROS
version = 'V15032017'

# Marhc 2017 Saeid Mokaram  saeid.mokaram@gmail.com
# Sheffield Robotics    http://www.sheffieldrobotics.ac.uk/
# The university of sheffield   http://www.sheffield.ac.uk/

# This script generats a ROS node for comunicating with KUKA iiwa
# Dependencies: conf.txt, ROS server, Rospy, KUKA iiwa java SDK, KUKA iiwa robot.

#######################################################################################################################
import rospy, os
from std_msgs.msg import String


def cl_black(msge): return '\033[30m'+msge+'\033[0m'
def cl_red(msge): return '\033[31m'+msge+'\033[0m'
def cl_green(msge): return '\033[32m'+msge+'\033[0m'
def cl_orange(msge): return '\033[33m'+msge+'\033[0m'
def cl_blue(msge): return '\033[34m'+msge+'\033[0m'
def cl_purple(msge): return '\033[35m'+msge+'\033[0m'
def cl_cyan(msge): return '\033[36m'+msge+'\033[0m'
def cl_lightgrey(msge): return '\033[37m'+msge+'\033[0m'
def cl_darkgrey(msge): return '\033[90m'+msge+'\033[0m'
def cl_lightred(msge): return '\033[91m'+msge+'\033[0m'
def cl_lightgreen(msge): return '\033[92m'+msge+'\033[0m'
def cl_yellow(msge): return '\033[93m'+msge+'\033[0m'
def cl_lightblue(msge): return '\033[94m'+msge+'\033[0m'
def cl_pink(msge): return '\033[95m'+msge+'\033[0m'
def cl_lightcyan(msge): return '\033[96m'+msge+'\033[0m'

#######################################################################################################################


#######################################################################################################################
#   Class: Kuka iiwa ROS node    #####################
class kuka_iiwa_ros_client:
    #   M: __init__ ===========================
    def __init__(self): # Makes kuka_iiwa ROS node
        self.JointPosition = ([None,None,None,None,None,None,None],None)
        self.ToolPosition = ([None,None,None,None,None,None],None)
        self.ToolForce = ([None,None,None],None)
        self.ToolTorque = ([None,None,None],None)
        self.JointAcceleration = (None, None)
        self.JointVelocity = (None, None)
        self.JointJerk = (None, None)
        self.isready = False
        self.isCompliance = (False, None)
        self.isReadyToMove  = (False, None)
        self.isCollision  = (False, None)
        self.isMastered = (False, None)
        self.OperationMode = (None, None)

        os.system('clear')
        print cl_pink('\n==========================================')
        print cl_pink('<   <  < << SHEFFIELD ROBOTICS >> >  >   >')
        print cl_pink('==========================================')
        print cl_pink(' KUKA API for ROS')
        print cl_pink(' Client Version: ' + version)
        print cl_pink('==========================================\n')


        #    Make a listener for all kuka_iiwa data
        rospy.Subscriber("JointPosition", String, self.JointPosition_callback)
        rospy.Subscriber("ToolPosition", String, self.ToolPosition_callback)
        rospy.Subscriber("ToolForce", String, self.ToolForce_callback)
        rospy.Subscriber("ToolTorque", String, self.ToolTorque_callback)
        rospy.Subscriber("JointAcceleration", String, self.JointAcceleration_callback)
        rospy.Subscriber("JointVelocity", String, self.JointVelocity_callback)
        rospy.Subscriber("JointJerk", String, self.JointJerk_callback)
        rospy.Subscriber("isCompliance", String, self.isCompliance_callback)
        rospy.Subscriber("isCollision", String, self.isCollision_callback)
        rospy.Subscriber("isMastered", String, self.isMastered_callback)
        rospy.Subscriber("OperationMode", String, self.OperationMode_callback)
        rospy.Subscriber("isReadyToMove", String, self.isReadyToMove_callback)

        #   Make Publishers for kuka_iiwa commands
        self.pub_kuka_command = rospy.Publisher('kuka_command', String, queue_size=10)

        #   Make kuka_iiwa client
        rospy.init_node('kuka_iiwa_client', anonymous=False)
        self.rate = rospy.Rate(100) #    100hz update rate.

    #   ~M: __init__ ==========================

    def send_command(self, command_str):
        #rospy.loginfo(command_str)
        self.pub_kuka_command.publish(command_str)
        self.rate.sleep()

    #   M: callbacks ===========================
    #   Receiving command string and sending it to KUKA iiwa
    def JointPosition_callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "Received JointPosition " + str(data.data) )
        # e.g. [0.0, 0.17, 0.0, 1.92, 0.0, 0.35, 0.0] 1459253274.1
        self.JointPosition = ([float(x) for x in data.data.split(']')[0][1:].split(', ')], float(data.data.split(']')[1]))

    def isCompliance_callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "Received isCompliance " + str(data.data) )
        d = str(data.data).split() # e.g. off 1459253274.11
        if d[0] == 'True': stat = True
        if d[0] == 'False': stat = False
        self.isCompliance = (stat, float(d[1]))

    def isReadyToMove_callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "Received isReadyToMove " + str(data.data) )
        d = str(data.data).split() # e.g. off 1459253274.11
        if d[0] == 'True': stat = True
        if d[0] == 'False': stat = False
        self.isReadyToMove = (stat, float(d[1]))

    def isCollision_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "Received isCollision " + str(data.data) )
        d = str(data.data).split()  # e.g. off 1459253274.11
        if d[0] == 'True': stat = True
        if d[0] == 'False': stat = False
        self.isCollision = (stat, float(d[1]))

    def isMastered_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "Received isMastered " + str(data.data) )
        d = str(data.data).split()  # e.g. off 1459253274.11
        if d[0] == 'True': stat = True
        if d[0] == 'False': stat = False
        self.isMastered = (stat, float(d[1]))

    def OperationMode_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "Received isMastered " + str(data.data) )
        d = str(data.data).split()  # e.g. off 1459253274.11
        self.OperationMode = (d[0], float(d[1]))
        self.isready = True

    def ToolPosition_callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "Received ToolPosition " + str(data.data) )
        # e.g. [433.59711426170867, 0.028881929589094864, 601.4449734558293, 3.1414002368275726, 1.0471367465304213, 3.141453681799645] 1459253274.11
        self.ToolPosition = ([float(x) for x in data.data.split(']')[0][1:].split(', ')], float(data.data.split(']')[1]))

    def ToolForce_callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "Received ToolForce " + str(data.data) )
        # e.g, '[13.485958070668463, 0.3785658886199012, 5.964988607372689] 1459253274.11'
        self.ToolForce = ([float(x) for x in data.data.split(']')[0][1:].split(', ')], float(data.data.split(']')[1]))
        
    def ToolTorque_callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "Received ToolTorque " + str(data.data) )
        # e.g, '[13.485958070668463, 0.3785658886199012, 5.964988607372689] 1459253274.11'
        self.ToolTorque = ([float(x) for x in data.data.split(']')[0][1:].split(', ')], float(data.data.split(']')[1]))
    
    def JointAcceleration_callback(self, data):
        # e.g. 1.0 1459253274.11
        self.JointAcceleration = ( float(data.data.split(' ')[0]), float(data.data.split(' ')[1]) )
    
    def JointVelocity_callback(self, data):
        # e.g. 1.0 1459253274.11
        self.JointVelocity = ( float(data.data.split(' ')[0]), float(data.data.split(' ')[1]) )
    
    def JointJerk_callback(self, data):
        # e.g. 1.0 1459253274.11
        self.JointJerk = ( float(data.data.split(' ')[0]), float(data.data.split(' ')[1]) )
    
    #   ~M: callbacks ===========================

#   ~Class: Kuka iiwa ROS client    #####################
######################################################################################################################
