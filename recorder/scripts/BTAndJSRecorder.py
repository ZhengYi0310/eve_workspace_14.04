#!/usr/bin/env python

# A class used to record biotac data and joint states using aprroximate time policy
import roslib; roslib.load_manifest('recorder')
import rospy
import os, sys
from rosjson_time import rosjson_time
from threading import Thread 
from types import *
import rosbag

from std_msgs.msg import String
from biotac_sensors.msg import BioTacHand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint 
from sensor_msgs.msg import JointState
from wam_msgs.msg import *
from wam_msgs.srv import *
import message_filters
class BTAndJSRecorder:

  def __init__(self):
    self.frame_count = 1;
    rospy.init_node('biotac_jointstates_logger')
    self.time_to_log = rospy.myargv(argv=sys.argv[1])
  
    #Check to see if logging time was set
    try:
      input_arg = float(self.time_to_log)
      if type(input_arg) is FloatType:
        self.node_log_time = input_arg
      else:
        self.node_log_time = 'inf'
    except:
      self.node_log_time = 'inf'

    
    # FILE WRITING SETUP 
    # Find Node Parameter Name
    self.file_param = rospy.get_name() + '/filename'
    # Grab directory
    self.package_dir = roslib.packages.get_pkg_dir('recorder')
    # Check for a 'data' directory
    dir_status = self.check_dir(os.path.expanduser('~/.ros/biotac_jointstates_data' ))
    if dir_status:
      rospy.loginfo('The ''~/.ros/biotac_data'' directory was successfully created.')
    # Set output filename
    self.fileName =  os.path.expanduser('~/.ros/biotac_jointstates_data/') + rospy.get_param(self.file_param,'default.json')
    if not self.fileName.endswith('.json'):
      self.fileName = self.fileName + '.json'

    '''
    # Register the sync policy
    self.atss = message_filters.TimeSynchronizer(message_filters.Subscriber("/joint_states", JointState), message_filters.Subscriber("/biotac_pub", BioTacHand))
    '''

    # Create initial file - delete existing file with same name 
    self.fout = open(self.fileName,'w')
    self.fout.write("[\n")

    rospy.loginfo(rospy.get_name()+' Starting to Log to file %s:',self.fileName);

    #Begin starting to log for specified time
    if self.node_log_time is not 'inf':
      while not rospy.get_time(): pass
      self.start_time = rospy.get_time()
      rospy.loginfo("Logging start time: %f" % self.start_time)

      # Called each time there two messages are aligned
  def biotacandjointstatesCallback(self, data):
    #rospy.loginfo(rospy.get_name()+' FRAME ID: %d',self.frame_count)
    # Stores the frame count into the messages
    data.joint_state.header.frame_id = self.frame_count
    data.biotac_hand.header.frame_id = self.frame_count

    # Uses rosjson to convert message to JSON 
    toWrite = rosjson_time.ros_message_to_json(data) + ',' +'\n'
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
    self.sub = rospy.Subscriber('joint_states_and_biotac', JointStateAndBiotac, self.biotacandjointstatesCallback,queue_size=1000)
    rospy.spin()

  def killlistener(self):
    self.__del__()
    rospy.signal_shutdown("subscriber killed !")

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

    bt_js_recorder = BTAndJSRecorder()
    #bt_listener.callDMProllout(sys.argv[2])
    t1 = Thread(target=bt_js_recorder.listener, )
    t1.start()
    #bt_listener.listener()
    bt_js_recorder.callDMProllout(sys.argv[2])
    t2 = Thread(target=bt_js_recorder.killlistener, )
    t2.start()
    t2.join()