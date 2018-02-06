#!/usr/bin/env python
# This file is used to start the socket server to receive the dmp rollout traj file
import socket
import os, sys

import roslib; roslib.load_manifest('wam_node')
import rospy

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import *

'''
######################tcp begining
HOST = 'eve'

PORT = 9999

BUFFER = 8192

sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

sock.bind((HOST,PORT))

sock.listen(5)

################ros begining
rospy.init_node('Socket_Server',anonymous=0)

print "socket is listening"

while not rospy.is_shutdown():
    cli, addr = sock.accept()
    try:
        #cli.settimeout(20)
        f = open("test.json", "wb")
        #while True:
        print cli.recv(8192)
        cli.send('Hello Joy!')
        l = cli.recv(8192)
        print "receiving file!"
        while (l):
            #print l
            f.write(l)
            l = cli.recv(1024)
        f.close()
    except socket.timeout:
        print 'time out'
    #cli.send('file received!')
    print "file received!"
cli.close()
'''

class SocketServer:
    def __init__(self):
        # create a socket object
        # ipv4 address family, TCP protocal
        rospy.init_node('Socket_Server')
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.port = 9999
        self.server.bind(("eve", self.port))
        print "socket binded to %s." %(self.port)

        # set up the file name where bag file is stored
        self.package_dir = roslib.packages.get_pkg_dir('wam_node')

        # register all the necessary rosservice server
        self.initialhandshakeserver = rospy.Service("server_handshake", Empty, self.initialhandshake)
        self.transferfileserver = rospy.Service("server_transferfile", Empty, self.transferfile)
        self.shutdownserver = rospy.Service("server_shutdown", Empty, self.shutdown)

        # start listen to the client
        self.server.listen(10)
        print "socket starts to listening !"


    def initialhandshake(self, req):
        self.receiver, address = self.server.accept() # accept the handshake message from Joy
        print self.receiver.recv(8192)
        self.receiver.send('Hello Joy! This is a handshake from Eve!') # send back a handshake message
        self.receiver.close()
        return EmptyResponse()

    def transferfile(self, req):
        self.receiver, address = self.server.accept() # accept the handshake message from Joy
        f = open(self.package_dir + "/dmp_rollout.bag", "wb")

        # receive the file and write it
        l = self.receiver.recv(8192)

        while(l):
            f.write(l)
            l = self.receiver.recv(1024)
        #f.close()
        print "File reveived from Joy !"
        #self.receiver.close()
        return EmptyResponse()



    def shutdown(self, req):
        #self.__del__()
        self.receiver.close()
        rospy.signal_shutdown("socket server is shutdown !")
        return EmptyResponse()

    #def __del__(self):
        #self.server.close()

if __name__ == "__main__":
    socket_server = SocketServer()
    rospy.spin()


