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
#include <iostream>
#include <fstream>
#include <exception>

#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>

#include "video_survey/experiment_utils.h"
#include <moveit_recorder/utils.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

void tokenize(const std::string& str, std::vector<std::string>& token_list)
{
  boost::char_separator<char> sep(",");
  boost::tokenizer< boost::char_separator<char> > tokens(str, sep);
  for ( boost::tokenizer< boost::char_separator<char> >::iterator it = tokens.begin();
      it != tokens.end();
      ++it)
    token_list.push_back(*it);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "split_screen_creator");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("level",boost::program_options::value<std::string>(), "Which end")
    ("feature",boost::program_options::value<std::string>(), "Which feature")
    ("vis_dir",boost::program_options::value<std::string>(), "Directory for visuals")
    ("out_dir",boost::program_options::value<std::string>(), "Directory for saving the required visuals")
    ("csv",boost::program_options::value<std::string>(), "Directory for videos");
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
    std::string level = utils::get_option(vm, "level", "");
    std::string vis_dir = utils::get_option(vm, "vis_dir", "");
    std::string out_dir = utils::get_option(vm, "out_dir", "");
    std::string csv = utils::get_option(vm, "csv", "");
    std::string feature = utils::get_option(vm, "feature", "");
    
    cv::namedWindow( "Preview" , cv::WINDOW_AUTOSIZE );
    const int NEXT_KEY = 113;
    const int ACCEPT_KEY = 97;
    const int NEXT_VIEW = 122;
    const int NEXT_TRAJ = 32;

    std::vector<std::string> selected_feature;
    std::vector<std::string> selected_image;
    std::vector<int> selected_view;

    std::ifstream csv_file;
    csv_file.open(csv.c_str());
    std::string line;
    getline(csv_file,line); // gobble Rank row
    while(getline(csv_file, line))
    {
      std::vector<std::string> feature_data;
      tokenize(line, feature_data);

      if(feature_data[0]!=feature)
        continue;
      
      int inum = 1;
      int vnum = 1;
      bool image_selected = false;
      while( !image_selected )
      {
        std::string image_fname = boost::str(boost::format("%s/%s-%s-%d.jpg")
            % vis_dir
            % feature_data[inum]
            % "hand-elbow-bfs"
            % vnum );
        ROS_INFO("Showing %s", image_fname.c_str());

        cv::Mat image;
        image = cv::imread( image_fname.c_str(), CV_LOAD_IMAGE_COLOR );
        cv::imshow( "Preview", image );
        int key = cv::waitKey(-1.0);

        ROS_INFO("Feature: %s", feature_data[0].c_str());

        if(key==NEXT_KEY)
          image_selected=true;

        if(key==ACCEPT_KEY)
        {
          selected_feature.push_back(feature_data[0]);
          selected_image.push_back(feature_data[inum]);
          selected_view.push_back(vnum);
        }
        
        if(key==NEXT_VIEW)
        {
          vnum = (vnum+1)%4;
          vnum = (vnum!=0)?vnum:4;
        }

        if(key==NEXT_TRAJ)
        {
          inum = (inum+1)%feature_data.size();
          inum = (inum==0)?1:inum;
        }
      }
    }

    for(int i=0; i<selected_image.size(); ++i)
    {
      std::string copy = boost::str(boost::format("cp %s/%s-%s-%d.jpg %s/%s_%s_%d.jpg")
          % vis_dir
          % selected_image[i]
          % "hand-elbow-bfs"
          % selected_view[i]
          % out_dir
          % selected_feature[i]
          % level
          % i);
      std::cout << copy << std::endl;
      exp_utils::system::runCommand(copy);
    }

    csv_file.close();
  }
  catch(...)
  {
    // std::cout << e.what() << std::endl;
  }

  ros::shutdown();

  return 0;
}
