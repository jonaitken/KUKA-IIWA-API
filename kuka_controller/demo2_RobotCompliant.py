#!/usr/bin/env python

# KUKA API for ROS

# Marhc 2016 Saeid Mokaram  saeid.mokaram@gmail.com
# Sheffield Robotics    http://www.sheffieldrobotics.ac.uk/
# The university of sheffield   http://www.sheffield.ac.uk/

# This script generats a ROS node for comunicating with KUKA iiwa
# Dependencies: conf.txt, ROS server, Rospy, KUKA iiwa java SDK, KUKA iiwa robot.

#######################################################################################################################
from client_lib import *



if __name__ == '__main__':
    my_client = kuka_iiwa_ros_client()  # Making a connection object.
    while (not my_client.isready):
        pass  # Wait until iiwa is connected zzz!
    print('Started')

    # Initializing Tool 1
    my_client.send_command('setTool tool2')

    # Initializing
    my_client.send_command('setJointAcceleration 1.0')
    my_client.send_command('setJointVelocity 1.0')
    my_client.send_command('setJointJerk 1.0')
    my_client.send_command('setCartVelocity 10000')


    # Move close to a start position.
    my_client.send_command('setPosition 0 49.43 0 -48.5 0 82.08 0')


    # Move to the exact start position.
    my_client.send_command('setPositionXYZABC 700 0 300 -180 0 -180 ptp')  # ptp motions move with setJointAcceleration


    # Robot is compliant in XY plain.
    print('Robot is compliant in XY plain.')
    my_client.send_command('setCompliance 10 10 5000 300 300 300')
    my_client.send_command('sleep 10')  # Sleep 1s

    # Robot is compliant in YZ plain.
    print('Robot is compliant in YZ plain.')
    my_client.send_command('setCompliance 5000 10 10 300 300 300')
    my_client.send_command('sleep 10')  # Sleep 1s

    # Robot is compliant in rotation only.
    print('Robot is compliant in rotation only.')
    my_client.send_command('setCompliance 5000 5000 5000 10 10 10')
    my_client.send_command('sleep 10')  # Sleep 1s

    # Compliance OFF
    my_client.send_command('resetCompliance')