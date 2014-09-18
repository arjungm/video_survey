/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Willow Garage, Inc. 
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Arjun Menon */

#include <ros/ros.h>

#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/regex.hpp>

#include "video_survey/experiment_utils.h"
#include <moveit_recorder/utils.h>
#include <moveit_recorder/trajectory_video_lookup.h>
#include <moveit_recorder/trajectory_retimer.h>

#include <rviz_video_recorder/RecorderRequest.h>
#include <moveit_msgs/DisplayRobotState.h>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "start_goal_display_node");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //wait for RVIZ;
  sleep(20);

  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("planning_scene_topic",boost::program_options::value<std::string>(), "Topic for publishing the planning scene for recording")
    ("camera_topic",boost::program_options::value<std::string>(), "Topic for publishing the view to take the snapshot from")
    ("display_pose_topic",boost::program_options::value<std::string>(), "Topic for visualizing state of the robot for recording")
    ("reset",boost::program_options::value<bool>(), "reset the image states")
    ("save_dir",boost::program_options::value<std::string>(), "Directory for videos");
  boost::program_options::variables_map vm;
  boost::program_options::parsed_options po = boost::program_options::parse_command_line(argc, argv, desc);
  boost::program_options::store(po, vm);
  boost::program_options::notify(vm);

  if (vm.count("help") || argc == 1) // show help if no parameters passed
  {
    std::cout << desc << std::endl;
    return 1;
  }

  try
  {
    // read the bag file to get the file names
    std::string save_dir = utils::get_option(vm, "save_dir", "");
    boost::filesystem::path save_directory(save_dir);
    TrajectoryVideoLookup video_lookup_table;
    video_lookup_table.loadFromBag( save_directory.string() );
    ROS_INFO("Opening bag at %s", save_directory.string().c_str());

    // construct recorder 
    std::string planning_scene_topic =utils:: get_option(vm, "planning_scene_topic", "planning_scene");
    std::string display_pose_topic = utils::get_option(vm, "display_pose_topic", "/visualize_state");
    std::string camera_topic = utils::get_option(vm, "camera_topic", "/rviz/camera_placement");
    std::string recorder_command_topic = "/command";
    bool reset = vm.count("reset")? vm["reset"].as<bool>() : false;

    ros::Publisher rec_pub = node_handle.advertise<rviz_video_recorder::RecorderRequest>(recorder_command_topic, 1, true);
    ros::Publisher ps_pub = node_handle.advertise<moveit_msgs::PlanningScene>(planning_scene_topic, 1, true);
    ros::Publisher state_pub = node_handle.advertise<moveit_msgs::DisplayRobotState>(display_pose_topic, 1, true);
    ros::Publisher view_pub = node_handle.advertise<view_controller_msgs::CameraPlacement>(camera_topic, 1, true);

    utils::rostopic::waitOnSubscribersToTopic(rec_pub, recorder_command_topic);
    utils::rostopic::waitOnSubscribersToTopic(ps_pub, planning_scene_topic);
    utils::rostopic::waitOnSubscribersToTopic(state_pub, display_pose_topic);
    utils::rostopic::waitOnSubscribersToTopic(view_pub, camera_topic);

    boost::shared_ptr<TrajectoryRetimer> retimer;
    // iterate over the table and upload the named videos
    TrajectoryVideoLookup::iterator trajectory_it = video_lookup_table.begin();
    for( ; trajectory_it!=video_lookup_table.end(); ++trajectory_it)
    {
      std::string traj_id = trajectory_it->first;

      if(!retimer)
        retimer = boost::make_shared<TrajectoryRetimer>( "robot_description", trajectory_it->second.mpr.group_name );
      retimer->configure(trajectory_it->second.ps, trajectory_it->second.mpr);

      // get display trajectories
      moveit_msgs::RobotState start_state = retimer->getStartState(trajectory_it->second.rt);
      moveit_msgs::RobotState goal_state = retimer->getGoalState(trajectory_it->second.rt);
      moveit_msgs::DisplayRobotState display_state;

      rviz_video_recorder::RecorderRequest req;
      req.command = rviz_video_recorder::RecorderRequest::SCREENSHOT;

      // name the video
      {
        std::string video_id = boost::str(boost::format("%simg") % "start");
        std::string video_name = boost::str(boost::format("%s-%s.%s") % traj_id % video_id % "jpg");
        std::string video_file = (save_directory/video_name).string();

        if(reset)
          video_lookup_table.putVideoFile(traj_id, video_id, "");

        boost::filesystem::path check_image(video_file);
        if(  video_file != video_lookup_table.getVideoFile( traj_id, video_id ) || !boost::filesystem::exists(check_image))
        {
          // publish
          display_state.state = start_state;

          // make snapshot
          req.filepath = video_file;

          if(!reset)
          {
            state_pub.publish(display_state);
            sleep(4);
            ros::spinOnce();
            rec_pub.publish(req);
            sleep(1);
            if(boost::filesystem::exists(check_image))
            {
              ROS_INFO("Image saved. Proceeding to next image.");
              video_lookup_table.putVideoFile(traj_id, video_id, video_file);
              video_lookup_table.saveToBag( save_directory.string() );
            }
            else
            {
              ROS_WARN("Image not saved. Halting the node.");
              ros::shutdown();
            }
          }
        }
        else
        {
          ROS_INFO("Previously saved image detected. Skipping...");
        }

        if(state_pub.getNumSubscribers()<1)
          ros::shutdown();
      }

      // goal state
      {
        std::string video_id = boost::str(boost::format("%simg") % "goal");
        std::string video_name = boost::str(boost::format("%s-%s.%s") % traj_id % video_id % "jpg");
        std::string video_file = (save_directory/video_name).string();

        if(reset)
          video_lookup_table.putVideoFile(traj_id, video_id, "");

        boost::filesystem::path check_image(video_file);
        if(  video_file != video_lookup_table.getVideoFile( traj_id, video_id ) || !boost::filesystem::exists(check_image))
        {
          // publish
          display_state.state = goal_state;
          // make snapshot
          req.filepath = video_file;

          if(!reset)
          {
            state_pub.publish(display_state);
            sleep(4);
            ros::spinOnce();
            rec_pub.publish(req);
            sleep(1);
            if(boost::filesystem::exists(check_image))
            {
              ROS_INFO("Image saved. Proceeding to next image.");
              video_lookup_table.putVideoFile(traj_id, video_id, video_file);
              video_lookup_table.saveToBag( save_directory.string() );
            }
            else
            {
              ROS_WARN("Image not saved. Halting the node.");
              ros::shutdown();
            }
          }
        }
        else
        {
          ROS_INFO("Previously saved image detected. Skipping...");
        }

        if(state_pub.getNumSubscribers()<1)
          ros::shutdown();
      } 
    }
    video_lookup_table.saveToBag( save_directory.string() );
  }
  catch(...)
  {
    std::cout << "Failure!" << std::endl;
  }

  ROS_INFO("Completed recording display videos.");
  ros::shutdown();

  return 0;
}

