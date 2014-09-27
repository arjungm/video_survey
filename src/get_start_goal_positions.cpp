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
#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/regex.hpp>

#include "video_survey/experiment_utils.h"
#include <moveit_recorder/utils.h>
#include <moveit_recorder/trajectory_video_lookup.h>
#include <moveit_recorder/trajectory_retimer.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model/joint_model.h>
#include <Eigen/Core>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_uploader_node");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("path_dir",boost::program_options::value<std::string>(), "Directory for path pairs file")
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
    std::string save_dir = utils::get_option(vm, "save_dir", "");
    boost::filesystem::path save_directory(save_dir);
    
    std::string path_dir = utils::get_option(vm, "path_dir", "");
    boost::filesystem::path path_directory(path_dir);

    TrajectoryVideoLookup video_lookup_table;
    video_lookup_table.loadFromBag( save_directory.string() );
    ROS_INFO("Opening bag at %s", save_directory.string().c_str());
    TrajectoryRetimer trajproc("robot_description");
    
    std::ofstream path_pairs_file;
    path_pairs_file.open( (path_directory/"problemset.path").string().c_str(), std::ios::out );
    
    // iterate over the table and upload the named videos
    TrajectoryVideoLookup::iterator traj_it = video_lookup_table.begin();
    for( ; traj_it!=video_lookup_table.end(); ++traj_it)
    {
      trajproc.configure(traj_it->second.ps, traj_it->second.mpr);
      bool result = trajproc.retime(traj_it->second.rt);
      ROS_INFO("Is configured? %s", result?"yes":"no");
      
      planning_scene::PlanningScenePtr ps = trajproc.getPlanningScene();
      robot_trajectory::RobotTrajectoryPtr rt = trajproc.getRobotTrajectory();
      robot_state::RobotStatePtr start = rt->getFirstWayPointPtr();
      robot_state::RobotStatePtr goal = rt->getLastWayPointPtr();

      // ===========
      // Hacks to fix the stupid base position
      moveit::core::robotStateMsgToRobotState( traj_it->second.mpr.start_state, *start);
      start->update();
      std::vector<std::string> variables = start->getRobotModel()->getRootJoint()->getVariableNames();
      for(int v=0; v<variables.size(); ++v)
        goal->setVariablePosition(variables[v], start->getVariablePosition(variables[v]));
      goal->update();
      // ===========

      Eigen::Affine3d start_pose = start->getGlobalLinkTransform("r_gripper_tool_frame");
      Eigen::Affine3d goal_pose = goal->getGlobalLinkTransform("r_gripper_tool_frame");
      Eigen::Affine3d torso_pose = goal->getGlobalLinkTransform("torso_lift_link");

      // get max arm extension distance
      std::vector<const moveit::core::JointModel*> joints=goal->getJointModelGroup("right_arm")->getActiveJointModels();
      double zero=0.0;
      for(size_t j=0; j<joints.size(); ++j)
        goal->setJointPositions( joints[j], &zero );
      goal->update();
      Eigen::Affine3d far_pose = goal->getGlobalLinkTransform("r_gripper_tool_frame");
      double distance = sqrt(pow(torso_pose.translation().x()-far_pose.translation().x(),2)+
                        pow(torso_pose.translation().y()-far_pose.translation().y(),2)+
                        pow(torso_pose.translation().z()-far_pose.translation().z(),2));

      // save to path computation input file
      path_pairs_file << boost::str(boost::format("%s %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f")
                                        % traj_it->first
                                        % start_pose.translation().x()
                                        % start_pose.translation().y()
                                        % start_pose.translation().z()
                                        % goal_pose.translation().x()
                                        % goal_pose.translation().y()
                                        % goal_pose.translation().z() 
                                        % torso_pose.translation().x()
                                        % torso_pose.translation().y()
                                        % torso_pose.translation().z()
                                        % distance) << std::endl;
      
      std::ofstream scene_stream;
      scene_stream.open( (path_directory/(traj_it->first+".scene")).string().c_str(), std::ios::out);
      trajproc.getPlanningScene()->saveGeometryToStream( scene_stream );
      scene_stream.close();
    }
    path_pairs_file.close();
  }
  catch(...)
  {
    std::cout << "Failure!" << std::endl;
  }

  ROS_INFO("Completed generating paths.");
  ros::shutdown();

  return 0;
}

