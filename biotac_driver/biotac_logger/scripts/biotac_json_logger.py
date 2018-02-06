#!/usr/bin/env python
# Copyright (c) 2012, University of Pennsylvania
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the University of Pennsylvania nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Authors: Vivian Chu (chuv@grasp.upenn.edu) 
#          Ian McMahon (imcmahon@grasp.upenn.edu)


import roslib; roslib.load_manifest('biotac_logger'), roslib.load_manifest('wam_node')
import rospy
import os, sys
from rosjson_time import rosjson_time
from threading import Thread 
from types import *
import rosbag
import rosnode

from std_msgs.msg import String
from biotac_sensors.msg import BioTacHand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint 
from wam_msgs.msg import *
from wam_msgs.srv import *

class BioTacListener:

  def __init__(self):
    self.frame_count = 1;
    rospy.init_node('biotac_json_logger')
    self.time_to_log = rospy.myargv(argv=sys.argv[1])
    self.traj = JointTrajectory()
    #Check to see if logging time was set
    try:
      input_arg = float(self.time_to_log)
      if type(input_arg) is FloatType:
        self.node_log_time = input_arg
      else:
        self.node_log_time = 'inf'
    except:
      self.node_log_time = 'inf'

    #Set up the path to the bag file
    self.pathtobag = roslib.packages.get_pkg_dir('wam_node')
    
    # FILE WRITING SETUP 
    # Find Node Parameter Name
    self.file_param = rospy.get_name() + '/filename'
    # Grab directory
    self.package_dir = roslib.packages.get_pkg_dir('biotac_logger')
    # Check for a 'data' directory
    dir_status = self.check_dir(os.path.expanduser('~/.ros/biotac_data' ))
    if dir_status:
      rospy.loginfo('The ''~/.ros/biotac_data'' directory was successfully created.')
    # Set output filename
    self.fileName =  os.path.expanduser('~/.ros/biotac_data/') + rospy.get_param(self.file_param,'default.json')
    if not self.fileName.endswith('.json'):
      self.fileName = self.fileName + '.json'
    # Create initial file - delete existing file with same name 
    self.fout = open(self.fileName,'w')
    self.fout.write("[\n")

    rospy.loginfo(rospy.get_name()+' Starting to Log to file %s:',self.fileName);

    #Begin starting to log for specified time
    if self.node_log_time is not 'inf':
      while not rospy.get_time(): pass
      self.start_time = rospy.get_time()
      rospy.loginfo("Logging start time: %f" % self.start_time)

  # Called each time there is a new message
  def biotacCallback(self,data):
    #rospy.loginfo(rospy.get_name()+' FRAME ID: %d',self.frame_count)

    # Stores the frame count into the message
    data.header.frame_id = self.frame_count

    # Uses rosjson to convert message to JSON 
    toWrite = rosjson_time.ros_message_to_json(data) + '\n' + ','
    self.fout.write(toWrite);

    #Check to see if Elapsed time has run out
    if self.node_log_time is not 'inf':
      elapsed_time = rospy.get_time() - self.start_time
      if elapsed_time >= self.node_log_time:
        finish_msg = 'Logging for %f second(s) Complete!' % self.node_log_time
        rospy.loginfo(finish_msg)
        self.__del__()
        rospy.signal_shutdown(finish_msg)
      
    # Move to next frame 
    self.frame_count += 1

  #Check if directory exits & create it
  def check_dir(self, f):
    if not os.path.exists(f):
      os.makedirs(f)
      return True
    return False

  # Setup the subscriber Node
  def listener(self):
    self.sub = rospy.Subscriber('biotac_pub', BioTacHand, self.biotacCallback,queue_size=1000)
    rospy.spin()

  def killlistener(self):
    self.__del__()
    rospy.signal_shutdown("subscriber killed !")

  # Read the trajectory message from the bag file generated by DMP
  def readtrajfrombagfile(self, filename, verbose=True):
    if verbose:
      bagname = self.pathtobag + '/' + filename
      rospy.loginfo('read trajectory messages from bag file %s', bagname)

    # Set up the bag class
    bag = rosbag.Bag(bagname)

    # According to the DMO roll-out format, there is only one trajectory message
    for topic, msg, t in bag.read_messages(topics = '/trajectory_msgs'):
      self.traj = msg 
    print len(self.traj.points)

  def callDMProllout(self, traj_bag_filename):
    rospy.wait_for_service('joint_trajectory_dmp_rollout')
    try:
      joint_traj_dmp_rollout = rospy.ServiceProxy('joint_trajectory_dmp_rollout', JointTrajectoryDMPRollout)
      res = joint_traj_dmp_rollout(traj_bag_filename)
      return res.status
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
    



  def __del__(self):
    self.fout.write("]")
    self.fout.close()


if __name__ == '__main__':

    bt_listener = BioTacListener()
    t1 = Thread(target=bt_listener.listener, )
    t1.start()
    #bt_listener.listener()
    bt_listener.callDMProllout(sys.argv[2])
    t2 = Thread(target=bt_listener.killlistener, )
    t2.start()
    t2.join()
    rosnode.kill_nodes('biotac_json_logger')
  
