/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks   ...
 \file    test_task_recorder_node.cpp
 \author  Peter Pastor
 \date    Jun 7, 2011
 *********************************************************************/

// system includes

// local includes
#include <task_recorder/joint_states_recorder.h>
#include <task_recorder/pose_recorder.h>
#include <task_recorder/arm_cartesian_state_recorder.h>
#include <task_recorder/biotac_state_recorder.h>
#include <task_recorder/task_recorder_client.h>
#include <math.h>

using namespace task_recorder;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TestTaskRecorder");
  ros::NodeHandle node_handle("~");


  TaskRecorderClient<barrett_hw::joint_state> joint_states_recorder(node_handle);
  joint_states_recorder.initialize("/joint_states_rt");

  
  /*
  TaskRecorderClient<alsa_audio::AudioSample> audio_recorder(node_handle);
  audio_recorder.initialize("/AudioProcessor/audio_samples");
  */

  TaskRecorderClient<barrett_hw::arm_cartesian_state> arm_cartesian_state_recorder(node_handle);
  arm_cartesian_state_recorder.initialize("/cartesian_pose");

  
  //TaskRecorderClient<geometry_msgs::PoseStamped> pose_recorder(node_handle);
  //pose_recorder.initialize("/pose_r");

  TaskRecorderClient<biotac_sensors::BioTacHand> biotac_state_recorder(node_handle);
  biotac_state_recorder.initialize("/biotac_sensors");
  
  ros::WallDuration(1.0).sleep();

  ROS_INFO("Starting to record...");
  joint_states_recorder.startRecording();
  //pose_recorder.startRecording();
  //audio_recorder.startRecording();
  arm_cartesian_state_recorder.startRecording();
  biotac_state_recorder.startRecording();

  ros::Time start_time = biotac_state_recorder.getStartRecordingResponse().start_time;
  ros::WallDuration(12).sleep();
  //ros::Time end_time = joint_states_recorder.getStartRecordingResponse().start_time + ros::Duration (4.0);
  ros::Time end_time = start_time + ros::Duration(10);
  const int num_samples = std::floor((end_time - start_time).toSec()) * 25;
  ROS_INFO("Stopping recording...");

  joint_states_recorder.stopRecording(start_time, end_time, num_samples);
  //pose_recorder.stopRecording(start_time, end_time, num_samples);
  //audio_recorder.stopRecording(start_time, end_time, num_samples);
  arm_cartesian_state_recorder.stopRecording(start_time, end_time, num_samples);
  biotac_state_recorder.stopRecording(start_time, end_time, num_samples);
  return 0;
}
