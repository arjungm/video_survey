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
    ("script", boost::program_options::value<std::string>(), "Script to upload with")
    ("token", boost::program_options::value<std::string>(), "Token to upload to")
    ("folder_name", boost::program_options::value<std::string>(), "folder name for upload")
    ("folder_id", boost::program_options::value<std::string>(), "folder id for upload")
    ("regex", boost::program_options::value<std::string>(), "Regex of named images to upload")
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
    std::string token = utils::get_option(vm, "token", "");
    std::string upload_script = utils::get_option(vm, "script", "");
    std::string folder_name = utils::get_option(vm, "folder_name", "");
    std::string folder_id = utils::get_option(vm, "folder_id", "");
    std::string save_dir = utils::get_option(vm, "save_dir", "");
    boost::regex img_regex( utils::get_option(vm, "regex", "view." ) );

    std::string url = "https://googledrive.com/host";
    boost::filesystem::path save_directory(save_dir);

    TrajectoryVideoLookup video_lookup_table;
    video_lookup_table.loadFromBag( save_directory.string() );
    // read the bag file to get the file names
    ROS_INFO("Opening bag at %s", save_directory.string().c_str());

    // iterate over the table and upload the named videos
    TrajectoryVideoLookup::iterator trajectory_it = video_lookup_table.begin();
    for( ; trajectory_it!=video_lookup_table.end(); ++trajectory_it)
    {
      std::string traj_id = trajectory_it->first;
      // iterate over the videos and match regex for name
      TrajectoryVideoLookupEntry::iterator video_it = trajectory_it->second.begin();
      for( ; video_it!=trajectory_it->second.end(); ++video_it )
      {
        //match regex
          // if yes upload,
            // if success save URL back
            // else retry
        boost::cmatch matches;
        if(boost::regex_match( video_it->name.c_str(), matches, img_regex ))
        {
          ROS_INFO("Matched video to view \"%s\" for trajectory \"%s\"", video_it->name.c_str(), traj_id.c_str() );
          std::string title_name = boost::str(boost::format("%s%s") % video_it->name % traj_id);

          std::string upload_command = boost::str(
                                       boost::format("%s %s %s \"%s\" %s %s")
                                       % upload_script
                                       % token
                                       % video_it->file
                                       % title_name
                                       % folder_id
                                       % "image/jpeg");

          std::cout << upload_command << std::endl;

          std::string response_str = exp_utils::system::runCommand( upload_command );
          // if yes, save to bag under /url/videoname/viewname
          video_it->url = boost::str(boost::format("%s/%s/%s") % url % folder_id % title_name);
        }
      }
    }
    video_lookup_table.saveToBag( save_directory.string() );
  }
  catch(...)
  {
    std::cout << "Failure!" << std::endl;
  }

  ROS_INFO("Completed uploading videos.");
  ros::shutdown();

  return 0;
}

