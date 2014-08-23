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

#include <map>
#include <fstream>
#include <iostream>

#include <ros/ros.h>

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

#include "video_survey/experiment_utils.h"
#include <moveit_recorder/utils.h>
#include <moveit_recorder/trajectory_video_lookup.h>

using namespace std;

void write_experiment_file(const string& filename, const vector<string>& labels, const vector<vector<string> >& entries)
{
  ofstream file;
  ROS_INFO("Saving to experiment file: %s", filename.c_str());
  file.open(filename.c_str());
  //write labels
  file << labels[0];
  for(size_t i=0; i<labels.size(); ++i)
  { 
    file << ",";
    file << labels[i];
  }
  file << "\n";
  //write entries
  for(size_t i=0; i<entries.size(); ++i)
  {
    file << entries[i][0];
    for(size_t r=1; r<entries[i].size(); ++r)
    {
      file << ",";
      file << entries[i][r];
    }
    file << "\n";
  }
  file.close();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "youtube_url_to_experiment_creator");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("regexes", boost::program_options::value< vector<string> >()->multitoken(), "Regexes of named resources to compose experiment with")
    ("labels", boost::program_options::value< vector<string> >()->multitoken(), "Labels of named resources to compose experiment with")
    ("randomize_num", boost::program_options::value<int>(), "Number of sets to create with randomized order")
    ("save_dir",boost::program_options::value<string>(), "Directory for videos");
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
    string save_dir = utils::get_option(vm, "save_dir", "");

    int randomize_num = vm.count("randomize_num") ? vm["randomize_num"].as<int>() : 0;
    
    vector<string> regex_list = vm.count("regexes") ? vm["regexes"].as<vector<string> >(): vector<string>();
    vector<string> label_list = vm.count("labels") ? vm["labels"].as<vector<string> >(): vector<string>();

    boost::filesystem::path save_directory(save_dir);

    // read the bag file to get the file names
    ROS_INFO("Opening bag at %s", save_directory.string().c_str());
    TrajectoryVideoLookup video_lookup_table;
    video_lookup_table.loadFromBag( save_directory.string() );

    // for all trajectories in look up
    vector< vector<string> > assignments;
    TrajectoryVideoLookup::iterator traj_it = video_lookup_table.begin();
    for(; traj_it!=video_lookup_table.end();++traj_it)
    {
      vector<string> assignment_resources;
      bool has_all_fields = false;
      
      // for each required named resource
      vector<string>::iterator regex_it = regex_list.begin();
      for(; regex_it!=regex_list.end();++regex_it)
      {
        boost::regex resource_regex( *regex_it );
        boost::cmatch matches;

        //search for resource
        TrajectoryVideoLookupEntry::iterator video_it = traj_it->second.begin();
        for(; video_it!=traj_it->second.end();++video_it)
        {
          if(boost::regex_match( video_it->name.c_str(), matches, resource_regex))
            assignment_resources.push_back( video_it->url );
        }
      }

      if(assignment_resources.size()!=regex_list.size())
      {
        // not enough resources for this trajectory
        // notify and skip adding it
        ROS_WARN("Not enough matching named resources, skipping %s", traj_it->first.c_str());
        continue;
      }
      
      // else put to file
      assignments.push_back( assignment_resources );
    }
    if( assignments.size() == 0 )
      ROS_ERROR("No trajectories had all the named resources!");

    
    // randomization options
    if(randomize_num!=0)
    {
      for(int i=0; i<randomize_num; ++i)
      {
        boost::filesystem::path experiment_file = save_directory / boost::str(boost::format("%s%d.csv") % "experiment" % i);
        random_shuffle(assignments.begin(), assignments.end());
        write_experiment_file(experiment_file.string(), label_list, assignments);
      }
    }
    else
    {
      write_experiment_file("experiment.csv", label_list, assignments);
    }
  }
  catch(...)
  {
    std::cout << "Failure!" << std::endl;
  }

  ROS_INFO("Completed creating experiment csv.");
  ros::shutdown();

  return 0;
}

