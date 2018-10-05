#!/usr/bin/env python

# KUKA API for ROS
version = 'V15032017++'

# Marhc 2017 Saeid Mokaram  saeid.mokaram@gmail.com
# Sheffield Robotics    http://www.sheffieldrobotics.ac.uk/
# The university of sheffield   http://www.sheffield.ac.uk/

# This script generats a ROS node for comunicating with KUKA iiwa
# Dependencies: conf.txt, ROS server, Rospy, KUKA iiwa java SDK, KUKA iiwa robot.

#######################################################################################################################
import thread, time, os
import rospy
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
#   Class: Kuka iiwa TCP communication    #####################
class iiwa_socket:
    #   M: __init__ ===========================
    def __init__(self, ip, port):
        self.BUFFER_SIZE = 1024
        self.isconnected = False
        self.JointPosition = ([None,None,None,None,None,None,None],None)
        self.ToolPosition = ([None,None,None,None,None,None],None)
        self.ToolForce = ([None,None,None],None)
        self.ToolTorque = ([None,None,None],None)
        self.isCompliance = (False, None)
        self.isCollision = (False, None)
        self.isReadyToMove = (False, None)
        self.isMastered = (False, None)
        self.OperationMode = (None, None)
        self.OprationMode = (None, None)
        self.JointAcceleration = (None,None)
        self.JointVelocity = (None,None)
        self.JointJerk = (None,None)
        self.isFinished = (False, None)
        self.hasError = (False, None)
        self.isready = False
        

        try:
            # Starting connection thread
            thread.start_new_thread( self.socket, (ip, port, ) )
        except:
            print cl_red('Error: ') + "Unable to start connection thread"
    #   ~M: __init__ ==========================

    #   M: Stop connection ====================
    def close(self):
        self.isconnected = False
    #   ~M: Stop connection ===================

    #   M: Connection socket ==================
    def socket(self, ip, port):
        import socket

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Bind the socket to the port
        server_address = (ip, port)

        os.system('clear')
        print cl_pink('\n==========================================')
        print cl_pink('<   <  < << SHEFFIELD ROBOTICS >> >  >   >')
        print cl_pink('==========================================')
        print cl_pink(' KUKA API for ROS')
        print cl_pink(' Server Version: ' + version)
        print cl_pink('==========================================\n')


        print cl_cyan('Starting up on:'), 'IP:', ip, 'Port:', port
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(server_address)
        except:
            print cl_red('Error: ') + "Connection for KUKA cannot assign requested address:", ip, port
            os._exit(-1)


        # Listen for incoming connections
        sock.listen(1)

        # Wait for a connection
        print cl_cyan('Waiting for a connection...')
        self.connection, client_address = sock.accept()
        self.connection.settimeout(0.01)
        print cl_cyan('Connection from'), client_address
        self.isconnected = True
        last_read_time = time.time()

        while self.isconnected:
            try:
                data = self.connection.recv(self.BUFFER_SIZE)
                last_read_time = time.time()    # Keep received time

                # Process the received data pacage
                for pack in data.split(">"):  # parsing data pack
                    cmd_splt = pack.split()

                    if len(pack) and cmd_splt[0]=='Joint_Pos':  # If it's JointPosition
                        tmp = [float(''.join([c for c in s if c in '0123456789.eE-'])) for s in cmd_splt[1:]]
                        if len(tmp)==7: self.JointPosition = (tmp, last_read_time)

                    if len(pack) and cmd_splt[0]=='Tool_Pos':  # If it's ToolPosition
                        tmp = [float(''.join([c for c in s if c in '0123456789.eE-'])) for s in cmd_splt[1:]]
                        if len(tmp)==6: self.ToolPosition = (tmp, last_read_time)

                    if len(pack) and cmd_splt[0]=='Tool_Force':  # If it's ToolForce
                        tmp = [float(''.join([c for c in s if c in '0123456789.eE-'])) for s in cmd_splt[1:]]
                        if len(tmp)==3: self.ToolForce = (tmp, last_read_time)
                        
                    if len(pack) and cmd_splt[0]=='Tool_Torque':  # If it's ToolTorque
                        tmp = [float(''.join([c for c in s if c in '0123456789.eE-'])) for s in cmd_splt[1:]]
                        if len(tmp)==3: self.ToolTorque = (tmp, last_read_time)

                    if len(pack) and cmd_splt[0]=='isCompliance':  # If isCompliance
                        if cmd_splt[1] == "false": self.isCompliance = (False, last_read_time)
                        elif cmd_splt[1] == "true": self.isCompliance = (True, last_read_time)
                        
                    if len(pack) and cmd_splt[0]=='isCollision':  # If isCollision
                        if cmd_splt[1] == "false": self.isCollision = (False, last_read_time)
                        elif cmd_splt[1] == "true": self.isCollision = (True, last_read_time)
                    
                    if len(pack) and cmd_splt[0]=='isReadyToMove':  # If isReadyToMove
                        if cmd_splt[1] == "false": self.isReadyToMove = (False, last_read_time)
                        elif cmd_splt[1] == "true": self.isReadyToMove = (True, last_read_time)
                        
                    if len(pack) and cmd_splt[0]=='isMastered':  # If isMastered
                        if cmd_splt[1] == "false": self.isMastered = (False, last_read_time)
                        elif cmd_splt[1] == "true": self.isMastered = (True, last_read_time)

                    if len(pack) and cmd_splt[0] == 'OperationMode':  # If OperationMode
                        self.OperationMode = (cmd_splt[1], last_read_time)
                    
                    if len(pack) and cmd_splt[0]=='JointAcceleration':  # If it's JointAcceleration
                        self.JointAcceleration = (float(cmd_splt[1]), last_read_time)
                    
                    if len(pack) and cmd_splt[0]=='JointVelocity':  # If it's JointVelocity
                        self.JointVelocity = (float(cmd_splt[1]), last_read_time)
                    
                    if len(pack) and cmd_splt[0]=='JointJerk':  # If it's JointJerk
                        self.JointJerk = (float(cmd_splt[1]), last_read_time)

                    if len(pack) and cmd_splt[0]=='isFinished':  # If isFinished
                        if cmd_splt[1] == "false": self.isFinished = (False, last_read_time)
                        elif cmd_splt[1] == "true": self.isFinished = (True, last_read_time)
                    if len(pack) and cmd_splt[0]=='hasError':  # If hasError
                        if cmd_splt[1] == "false": self.hasError = (False, last_read_time)
                        elif cmd_splt[1] == "true": self.hasError = (True, last_read_time)
                    

                    if ( all(item != None for item in self.JointPosition[0]) and
                         all(item != None for item in self.ToolPosition[0]) and
                         all(item != None for item in self.ToolForce[0]) and
                         all(item != None for item in self.ToolTorque[0]) and
                         self.isCompliance[1] != None  and
                         self.isCollision[1] != None  and
                         self.isReadyToMove[1] != None  and
                         self.isMastered[1] != None and
                         self.OperationMode[1] != None ):
                         #self.OprationMode = (None, None)
                        self.isready = True
                    else:
                        self.isready = False


            except:
                elapsed_time = time.time() - last_read_time
                if elapsed_time > 5.0:  # Didn't receive a pack in 5s
                    self.close() # Disconnect from iiwa
                    self.isconnected = False
                    print cl_lightred('No packet received from iiwa for 5s!')

        self.connection.shutdown(socket.SHUT_RDWR)
        self.connection.close()
        sock.close()
        self.isconnected = False
        print cl_lightred('Connection is closed!')
    #   ~M: Connection socket ===================

    #   M: Command send thread ==================
    # Each send command runs as a thread. May need to control the maximum running time (valid time to send a command).
    def send(self, cmd):
        thread.start_new_thread( self.__send, (cmd, ) )
    def __send(self, cmd):
        self.connection.sendall(cmd+'\r\n')
    #   ~M: Command send thread ==================

#   ~Class: Kuka iiwa TCP communication    #####################
#######################################################################################################################

#######################################################################################################################
#   Class: Kuka iiwa ROS node    #####################
class kuka_iiwa_ros_node:
    #   M: __init__ ===========================
    def __init__(self, ip, port): # Makes kuka_iiwa ROS node
        self.iiwa_soc = iiwa_socket(ip, port)

        #   Wait until iiwa is connected zzz!
        while (not self.iiwa_soc.isready): pass

        #    Make a listener for kuka_iiwa commands
        rospy.Subscriber("kuka_command", String, self.callback)

        #   Make Publishers for all kuka_iiwa data
        pub_JointPosition = rospy.Publisher('JointPosition', String, queue_size=10)
        pub_ToolPosition = rospy.Publisher('ToolPosition', String, queue_size=10)
        pub_ToolForce = rospy.Publisher('ToolForce', String, queue_size=10)
        pub_ToolTorque = rospy.Publisher('ToolTorque', String, queue_size=10)
        pub_isCompliance = rospy.Publisher('isCompliance', String, queue_size=10)
        pub_isCollision = rospy.Publisher('isCollision', String, queue_size=10)
        pub_isReadyToMove = rospy.Publisher('isReadyToMove', String, queue_size=10)
        pub_isMastered = rospy.Publisher('isMastered', String, queue_size=10)
        pub_OperationMode = rospy.Publisher('OperationMode', String, queue_size=10)
        pub_JointAcceleration = rospy.Publisher('JointAcceleration', String, queue_size=10)
        pub_JointVelocity = rospy.Publisher('JointVelocity', String, queue_size=10)
        pub_JointJerk = rospy.Publisher('JointJerk', String, queue_size=10)
        pub_isFinished = rospy.Publisher('isFinished', String, queue_size=10)
        pub_hasError = rospy.Publisher('hasError', String, queue_size=10)

        #   Make kuka_iiwa node
        rospy.init_node('kuka_iiwa', anonymous=False)
        rate = rospy.Rate(100) #    100hz update rate.
        while not rospy.is_shutdown() and self.iiwa_soc.isconnected:
            #data_str = self.iiwa_soc.data + " %s" % rospy.get_time()

            #   Update all the kuka_iiwa data
            for [pub, data] in [ [pub_JointPosition, self.iiwa_soc.JointPosition],
                                 [pub_ToolPosition, self.iiwa_soc.ToolPosition],
                                 [pub_ToolForce, self.iiwa_soc.ToolForce],
                                 [pub_ToolTorque, self.iiwa_soc.ToolTorque],
                                 [pub_isCompliance, self.iiwa_soc.isCompliance],
                                 [pub_isCollision, self.iiwa_soc.isCollision],
                                 [pub_isReadyToMove, self.iiwa_soc.isReadyToMove],
                                 [pub_isMastered, self.iiwa_soc.isMastered],
                                 [pub_OperationMode, self.iiwa_soc.OperationMode],
                                 [pub_JointAcceleration, self.iiwa_soc.JointAcceleration],
                                 [pub_JointVelocity, self.iiwa_soc.JointVelocity],
                                 [pub_JointJerk, self.iiwa_soc.JointJerk],
                                 [pub_isFinished, self.iiwa_soc.isFinished],
                                 [pub_hasError, self.iiwa_soc.hasError]]:

                data_str = str(data[0]) +' '+ str(rospy.get_time())
                ##########rospy.loginfo(data_str)
                pub.publish(data_str)


            rate.sleep()
    #   ~M: __init__ ==========================

    #   M: callback ===========================
    #   Receiving command string and sending it to KUKA iiwa
    def callback(self, data):
        ###########rospy.loginfo(rospy.get_caller_id() + "Received command " + str(data.data) )
        self.iiwa_soc.send(data.data)  # e.g 'setPosition 45 60 0 -25 0 95 0' for going to start position
    #   ~M: callback ===========================

#   ~Class: Kuka iiwa ROS node    #####################
######################################################################################################################

#   M:  Reading config file for Server IP and Port =================
def read_conf():
    f_conf = os.path.abspath(os.path.dirname(__file__)) + '/conf.txt'
    if os.path.isfile(f_conf):
        IP = ''
        Port = ''
        for line in open(f_conf, 'r'):
            l_splt = line.split()
            if len(l_splt)==4 and l_splt[0] == 'server':
                IP = l_splt[1]
                Port = int(l_splt[3])
        if len(IP.split('.'))!=4 or Port<=0:
            print cl_red('Error:'), "conf.txt doesn't include correct IP/Port! e.g. server 172.31.1.50 port 1234"
            exit()
    else:
        print cl_red('Error:'), "conf.txt doesn't exist!"
        exit()

    return [IP, Port]
#   ~M:  Reading config file for Server IP and Port ================


## MAIN ##
######################################################################################################################
if __name__ == '__main__':
    [IP, Port] = read_conf()

    try:
        node = kuka_iiwa_ros_node(IP, Port) # Make a Kuka_iiwa ROS node
    except rospy.ROSInterruptException:
        pass
######################################################################################################################
