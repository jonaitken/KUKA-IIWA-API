#!/usr/bin/env python

# KUKA API for ROS
version = 'V15032017'

# Marhc 2016 Saeid Mokaram  saeid.mokaram@gmail.com
# Sheffield Robotics    http://www.sheffieldrobotics.ac.uk/
# The university of sheffield   http://www.sheffield.ac.uk/

# This script generats a ROS node for comunicating with KUKA iiwa
# Dependencies: conf.txt, ROS server, Rospy, KUKA iiwa java SDK, KUKA iiwa robot.

# This application is intended for floor mounted robots.
#######################################################################################################################
from client_lib import *
import time, os

# Making a connection object.
my_client = kuka_iiwa_ros_client()

while (not my_client.isready):
    pass  # Wait until iiwa is connected zzz!

# Initializing Tool 1
my_client.send_command('setTool tool1')


for i in range(1000):
    os.system('clear')
    print cl_pink('\n==========================================')
    print cl_pink('<   <  < << SHEFFIELD ROBOTICS >> >  >   >')
    print cl_pink('==========================================')
    print cl_pink(' Read KUKA data demo')
    print cl_pink(' Version: ' + version)
    print cl_pink('==========================================\n')

    print '#####################################'
    print 'OperationMode\t=', my_client.OperationMode  # True when a collision has accured.
    print 'isCollision\t=', my_client.isCollision        # True when a collision has accured.
    print 'isCompliance\t=', my_client.isCompliance      # True when robot is in Compliance mode.
    print 'isMastered\t=', my_client.isMastered
    print 'isready\t=', my_client.isready                # True when robot is connected
    print 'isReadyToMove\t=', my_client.isReadyToMove    # True when robot can move, e.g. when the safety key is pressed...

    print 'ToolPosition\t=', my_client.ToolPosition      # Reading Tool cartesian position
    print 'ToolForce\t=', my_client.ToolForce            # Reading Tool cartesian force
    print 'ToolTorque\t=', my_client.ToolTorque          # Reading Tool cartesian torque

    print 'JointAcceleration\t=', my_client.JointAcceleration    # Current joint acceleration
    print 'JointJerk\t=', my_client.JointJerk                    # Current joint jerk
    print 'JointPosition\t=', my_client.JointPosition            # Reading joints position
    print 'JointVelocity\t=', my_client.JointVelocity            # Reading joints velocity
    print '#####################################'
    time.sleep(1)