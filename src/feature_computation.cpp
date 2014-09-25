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

#include <vector>
#include <string>

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

#include "video_survey/experiment_utils.h"
#include "video_survey/features.h"

#include <moveit_recorder/trajectory_video_lookup.h>
#include <moveit_recorder/trajectory_retimer.h>
#include <moveit_recorder/utils.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "split_screen_creator");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
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
    
    // read the bag file to get the file names
    ROS_INFO("Opening bag at %s", save_directory.string().c_str());
    TrajectoryVideoLookup video_lookup_table;
    video_lookup_table.loadFromBag( save_directory.string() );

    // get traj processor
    TrajectoryRetimer trajproc("robot_description");

    // get Feature engine
    TrajectoryFeatures tfeats;
      
    // file io
    std::ofstream featfile;
    featfile.open( (save_directory/"features.csv").string().c_str(), std::ios::out );
    
    // compute away
    TrajectoryVideoLookup::iterator traj_it = video_lookup_table.begin();
    for(; traj_it!=video_lookup_table.end();++traj_it)
    {
      trajproc.configure(traj_it->second.ps, traj_it->second.mpr);
      //retime
      moveit_msgs::RobotTrajectory post_processed_rt = traj_it->second.rt;
      bool success = trajproc.retime(post_processed_rt);
      ROS_INFO("Successfully retimed trajectory \"%s\"",traj_it->first.c_str());
      
      //compute all feats
      TrajectoryFeatures tfeats;
      tfeats.setPlanningScene( trajproc.getPlanningScene() );
      tfeats.setRobotTrajectory( trajproc.getRobotTrajectory() );
      tfeats.computeAll();
      
      // write feature names
      TrajectoryFeatures::iterator feat = tfeats.begin();
      if( traj_it == video_lookup_table.begin() )
      {
        featfile << "traj_id";
        for(; feat!=tfeats.end(); ++feat)
          featfile << "," << feat->first;
        featfile << std::endl;
      }
      // write feature values
      feat = tfeats.begin();
      featfile << traj_it->first;
      for(;feat!=tfeats.end();++feat)
        featfile << "," << feat->second;
      featfile << std::endl;

      ROS_INFO("Computed features for \'%s\'", traj_it->first.c_str());
      ROS_INFO("Press enter to continue...");
    }

    featfile.close();
  }
  catch(...)
  {
    std::cout << "Failure!" << std::endl;
  }

  ROS_INFO("Finished computing features");
  ros::shutdown();

  return 0;
}
