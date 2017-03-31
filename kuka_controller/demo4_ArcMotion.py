#!/usr/bin/env python

# KUKA API for ROS

# Marhc 2016 Saeid Mokaram  saeid.mokaram@gmail.com
# Sheffield Robotics    http://www.sheffieldrobotics.ac.uk/
# The university of sheffield   http://www.sheffield.ac.uk/

# This script generats a ROS node for comunicating with KUKA iiwa
# Dependencies: conf.txt, ROS server, Rospy, KUKA iiwa java SDK, KUKA iiwa robot.

#######################################################################################################################
from client_lib import *
import scipy.spatial.distance as Dist

def MaxT1Speed(my_client):
    my_client.send_command('setJointAcceleration 1.0')
    my_client.send_command('setJointVelocity 1.0')
    my_client.send_command('setJointJerk 1.0')
    my_client.send_command('setCartVelocity 1000')

def MaxT2Speed(my_client):
    my_client.send_command('setJointAcceleration 0.3')
    my_client.send_command('setJointVelocity 1.0')
    my_client.send_command('setJointJerk 0.1')
    my_client.send_command('setCartVelocity 1000')

if __name__ == '__main__':
    my_client = kuka_iiwa_ros_client()  # Making a connection object.
    while (not my_client.isready):
        pass  # Wait until iiwa is connected zzz!

    if my_client.OperationMode[0] == 'T1':
        print 'Hello Sheffield Robotics!'
        MaxT1Speed(my_client)
    elif my_client.OperationMode[0] == 'T2':
        MaxT2Speed(my_client)
    else:
        print 'The robot is in', my_client.OperationMode[0], 'mode.'
        print 'This demo is safe to work at T1 or T2 modes only.'
        exit()

    print('Started')

    # Initializing Tool 1
    my_client.send_command('setTool tool1')

    # Initializing (max speed)



    # Move close to a start position.
    my_client.send_command('setPosition 0 0 0 0 0 0 0')
    my_client.send_command('setPosition 0 49.43 0 -48.5 0 82.08 0')
    while Dist.euclidean(my_client.JointPosition[0], [0, 49.43, 0, -48.5, 0, 82.08, 0]) > 5: pass

    # Move to the exact start position.
    my_client.send_command('setPositionXYZABC 700 0 300 -180 0 -180 ptp')  # ptp motions move with setJointAcceleration


    #################################################
    # Motion in YZ plain.

    # Performing an arch motion from [700, 0, 400] to [700, 0, 200] passing trough [700, 100, 300].
    my_client.send_command('MoveCirc 700 100 200 -180 0 -180 700 0 100 -180 0 -180 0.1')  # MoveCirc motion move with CartVelocity

    # Performing a reverce arch motion from [700, 0, 200] to [700, 0, 400] passing trough [700, -100, 300].
    # my_client.send_command('setCartVelocity 100') # Performing same motion slower (CartVelocity 100mm/s')
    my_client.send_command('MoveCirc 700 -100 200 -180 0 -180 700 0 300 -180 0 -180 0.1')  # MoveCirc motion move with CartVelocity

    #################################################
    # Motion in XY plain.

    # Performing an arch motion from [700, 0, 400] to [500, 0, 400] passing trough [600, 100, 400].

    my_client.send_command('MoveCirc 600 100 300 -180 0 -180 500 0 300 -180 0 -180 0.1') # MoveCirc motion move with CartVelocity

    # Performing a reverce arch motion from [500, 0, 400] to [700, 0, 400] passing trough [600, -100, 400].
    # my_client.send_command('setCartVelocity 100') # Performing same motion slower (CartVelocity 100mm/s')
    my_client.send_command('MoveCirc 600 -100 300 -180 0 -180 700 0 300 -180 0 -180 0.1') # MoveCirc motion move with CartVelocity


    #################################################
    # Motion in XYZ plain.

    # Performing an arch motion from [700, 0, 400] to [500, 0, 200] passing trough [600, 100, 300].

    my_client.send_command(
        'MoveCirc 600 100 200 -180 0 -180 500 0 100 -180 0 -180 0.1')  # MoveCirc motion move with CartVelocity

    # Performing a reverce arch motion from [500, 0, 200] to [700, 0, 400] passing trough [600, -100, 300].
    # my_client.send_command('setCartVelocity 100') # Performing same motion slower (CartVelocity 100mm/s')
    my_client.send_command(
        'MoveCirc 600 -100 200 -180 0 -180 700 0 300 -180 0 -180 0.1')  # MoveCirc motion move with CartVelocity
