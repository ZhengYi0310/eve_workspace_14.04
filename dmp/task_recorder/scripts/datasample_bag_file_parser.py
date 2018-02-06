#!/usr/bin/python

PKG =  "task_recorder"
import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag

import task_recorder.msg
import csv
import sys
import os.path, time

class DataSampleBagFileParser:
    def __init__(self, topicname):
        self.data_samples_topic = topicname
        self.bag_extension = ".bag"
        self.txt_extension = ".txt"
        self.trial_base = "trial"
        self.connector = "_"
        self.trial_counter_extension = "trial_counter.txt"
        self.package_name = PKG
        self.data_dir_name = "recorder_data"
        self.raw_data_dir = "raw"
        self.resampled_data_dir = "resampled"
        self.directory = roslib.packages.get_pkg_dir(self.package_name) + '/' + self.data_dir_name + '/'

    def convert(self, i):
        txt_filename_base = self.recorder_directory + self.addTrailingSlash(self.raw_data_dir) + self.data_samples_topic +'__' + self.trial_base + self.connector
        bag_filename = txt_filename_base  + str(i) + self.bag_extension

        bag = rosbag.Bag(bag_filename)
        bagContents = bag.read_messages()

        data_fd = open(txt_filename_base + str(i) + self.txt_extension, 'wb')
        names_writen = False
        data_writer = csv.writer(data_fd, quoting=csv.QUOTE_NONE)

        # read bag file
        for (topic, msg, t) in bag.read_messages():
            # read data sample topic
            if self.getTopic(topic) == self.data_samples_topic:
                # first write variable_names
                if not names_writen:
                    names_writen = True
                    names = list(msg.names)
                    data_writer.writerow(names)

                # write data
                data = list(msg.data)
                data_writer.writerow(data)

        bag.close()
        data_fd.close()
    def parse(self, description):
        description_directory = description.description + self.connector + str(description.id)
        self.recorder_directory = self.directory + description_directory
        self.recorder_directory = self.addTrailingSlash(self.recorder_directory)
        self.counter_frame_dir = self.addTrailingSlash(self.recorder_directory + self.resampled_data_dir)
        rospy.loginfo("Reading >%s<." % self.recorder_directory)

        counter_frame = self.counter_frame_dir + self.data_samples_topic + '__' + self.trial_counter_extension
        counter_fd = open(counter_frame, 'r')

        self.num_files = int(counter_fd.readline())
        rospy.loginfo("Reading >%i< files in >%s<." % (self.num_files, counter_frame))

        self.num_labels = 0
        for i in range(self.num_files):
            self.convert(i)


    def getTopic(self, topic):
        return topic[topic.rfind("/")+1:]

    def addTrailingSlash(self, directory_name):
        if directory_name[-1:] != '/':
            directory_name = directory_name + '/'
        return directory_name

def main(topicname):
    rospy.init_node('DataSampleBagFileParser')

    description = task_recorder.msg.Description()
    if rospy.has_param('~description'):
        description.description = rospy.get_param('~description')
    else:
        rospy.logwarn('Parameter >description< does not exist. Using >data< instead.')
        description.description = "data"

    if rospy.has_param('~id'):
        description.id = rospy.get_param('~id')
    else:
        rospy.logwarn('Parameter >id< does not exist. Using >0< instead.')
        description.id = 0

    data_sample_bag_file_parser = DataSampleBagFileParser(topicname)
    data_sample_bag_file_parser.parse(description)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise ValueError("must insert a topic name")
    else:
        try:
            main(sys.argv[1])
        except rospy.ROSInterruptException:
            pass
        finally:
            rospy.signal_shutdown('Done parsing file')