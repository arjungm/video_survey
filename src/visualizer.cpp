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
#include <visualization_msgs/Marker.h>

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
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

using namespace std;

void publishRobotStateAndCapture(const robot_state::RobotState& rstate, const std::string& file, ros::Publisher& state_pub, ros::Publisher& rec_pub)
{
  moveit_msgs::DisplayRobotState display_state;
  moveit_msgs::RobotState state_to_vis;
  moveit::core::robotStateToRobotStateMsg( rstate, state_to_vis );
  // display
  display_state.state = state_to_vis;
  state_pub.publish(display_state);
  sleep(4);
  ros::spinOnce();
  // record
  rviz_video_recorder::RecorderRequest req;
  req.command = rviz_video_recorder::RecorderRequest::SCREENSHOT;
  req.filepath = file;
  rec_pub.publish(req);
  sleep(1);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualization_node");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //wait for RVIZ;
  sleep(20);

  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("planning_scene_topic",boost::program_options::value<std::string>(), "Topic for publishing the planning scene for recording")
    ("display_pose_topic",boost::program_options::value<std::string>(), "Topic for visualizing state of the robot for recording")
    ("camera_topic",boost::program_options::value<std::string>(), "Topic for publishing the view to take the snapshot from")
    ("vis_dir",boost::program_options::value<std::string>(),"Directory for visualizations")
    ("path_dir",boost::program_options::value<std::string>(), "Directory for paths")
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
    ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::Marker>("path_visualizations",1, true) ;

    utils::rostopic::waitOnSubscribersToTopic(rec_pub, recorder_command_topic);
    utils::rostopic::waitOnSubscribersToTopic(ps_pub, planning_scene_topic);
    utils::rostopic::waitOnSubscribersToTopic(state_pub, display_pose_topic);
    utils::rostopic::waitOnSubscribersToTopic(view_pub, camera_topic);

    // path directory for path files
    std::string path_dir = utils::get_option(vm, "path_dir", "");
    boost::filesystem::path path_directory(path_dir);

    // visualization directory
    std::string vis_dir = utils::get_option(vm, "vis_dir", "");
    boost::filesystem::path vis_directory(vis_dir);

    boost::shared_ptr<TrajectoryRetimer> retimer;
    // iterate over the table and upload the named videos
    TrajectoryVideoLookup::iterator trajectory_it = video_lookup_table.begin();
    for( ; trajectory_it!=video_lookup_table.end(); ++trajectory_it)
    {
      std::string traj_id = trajectory_it->first;

      if(!retimer)
        retimer = boost::make_shared<TrajectoryRetimer>( "robot_description", trajectory_it->second.mpr.group_name );
      retimer->configure(trajectory_it->second.ps, trajectory_it->second.mpr);
      retimer->retime(trajectory_it->second.rt);
      retimer->zeroRootJointPositions();

      robot_trajectory::RobotTrajectoryPtr rt = retimer->getRobotTrajectory();

      // get display trajectories
      moveit_msgs::RobotState start_state, goal_state;
      moveit::core::robotStateToRobotStateMsg( rt->getWayPoint(0), start_state );
      moveit::core::robotStateToRobotStateMsg( rt->getWayPoint( rt->getWayPointCount()-1 ), goal_state );

      // visuals to take
      // trajectory-snapshots (4-5 images of the trajectory state)
      // elbow spline path + start_state
      // end effector spline path + start_state
      
      bool skip = false;
      if(skip)
      {
      int n = 5;
      for(int i=0; i<n; i++)
      {
        std::string visual_id = boost::str(boost::format("snapshot-%d") % (i+1) );
        std::string visual_fname = boost::str(boost::format("%s-%s.%s") % traj_id % visual_id % "jpg");
        std::string visual_fpath = (vis_directory / visual_fname).string();
        boost::filesystem::path check_image(visual_fpath);
        if(  visual_fpath != video_lookup_table.getVideoFile( traj_id, visual_id ) || !boost::filesystem::exists(check_image))
        {
          //get throttle index
          size_t waypoint_index;
            if(i==(n-1))
              waypoint_index = rt->getWayPointCount()-1;
            else
              waypoint_index = (i*rt->getWayPointCount())/n;
          // image and cap
          publishRobotStateAndCapture( rt->getWayPoint( waypoint_index ), visual_fpath, state_pub, rec_pub);
          // verify
          if(boost::filesystem::exists(check_image))
          {
            ROS_INFO("Image saved. Proceeding to next image.");
            video_lookup_table.putVideoFile(traj_id, visual_id, visual_fpath);
            // video_lookup_table.saveToBag( save_directory.string() );
          }
          else
          {
            ROS_ERROR("Image not saved. Halting the node.");
            ros::shutdown();
            exit(0);
          }
        }
        else
          ROS_WARN("Image already saved. Skipping.");
      }// done with snapshots
      }//skip check


      //load the path into a visualization marker message
      visualization_msgs::Marker elbow_lines, elbow_points, hand_lines, hand_points, bfs;
      elbow_lines.header.frame_id = elbow_points.header.frame_id = hand_lines.header.frame_id = hand_points.header.frame_id = bfs.header.frame_id = "/odom_combined";
      elbow_lines.header.stamp = elbow_points.header.stamp = hand_lines.header.stamp = hand_points.header.stamp = bfs.header.stamp = ros::Time::now();
      elbow_lines.ns = elbow_points.ns = hand_lines.ns = hand_points.ns = bfs.ns = "paths";
      elbow_lines.action = elbow_points.action = hand_lines.action = hand_points.action = bfs.action = visualization_msgs::Marker::ADD;
      elbow_lines.pose.orientation.w = elbow_points.pose.orientation.w = hand_lines.pose.orientation.w = hand_points.pose.orientation.w = bfs.pose.orientation.w = 1.0;
      elbow_lines.scale.x = hand_lines.scale.x = bfs.scale.x = 0.01;
      elbow_lines.color.a = hand_lines.color.a = bfs.color.a = 1.0;
      elbow_lines.type = hand_lines.type = bfs.type = visualization_msgs::Marker::LINE_STRIP;
      elbow_points.type = hand_points.type = visualization_msgs::Marker::POINTS;
      elbow_points.color.a = hand_points.color.a = 1.0;
      elbow_points.scale.x = hand_points.scale.x = 0.02;
      elbow_points.scale.y = hand_points.scale.y = 0.02;
      int unique_id = 0;
      double x,y,z;
      std::ifstream bfs_path_file;
      bfs_path_file.open( (path_directory/(traj_id+".path")).string().c_str() );
      // ===============
      // ELBOW
      // ===============
      elbow_lines.id = unique_id++;
      elbow_lines.color.r = 1.0;
      elbow_points.id = unique_id++;
      elbow_points.color.b = 1.0;
      for(int i=0; i<rt->getWayPointCount(); ++i)
      {
        geometry_msgs::Point p;
        p.x = rt->getWayPointPtr(i)->getGlobalLinkTransform("r_elbow_flex_link").translation().x();
        p.y = rt->getWayPointPtr(i)->getGlobalLinkTransform("r_elbow_flex_link").translation().y();
        p.z = rt->getWayPointPtr(i)->getGlobalLinkTransform("r_elbow_flex_link").translation().z();
        elbow_lines.points.push_back(p);
        elbow_points.points.push_back(p);
      }
      // ===============
      // END EFFECTOR
      // ===============
      hand_lines.id = unique_id++;
      hand_lines.color.r = 1.0;
      hand_points.id = unique_id++;
      hand_points.color.b = 1.0;
      for(int i=0; i<rt->getWayPointCount(); ++i)
      {
        geometry_msgs::Point p;
        p.x = rt->getWayPointPtr(i)->getGlobalLinkTransform("r_wrist_roll_link").translation().x();
        p.y = rt->getWayPointPtr(i)->getGlobalLinkTransform("r_wrist_roll_link").translation().y();
        p.z = rt->getWayPointPtr(i)->getGlobalLinkTransform("r_wrist_roll_link").translation().z();
        hand_lines.points.push_back(p);
        hand_points.points.push_back(p);
      }
      // ===============
      // BFS
      // ===============
      bfs.id = unique_id++;
      bfs.color.r = 1.0;
      while( bfs_path_file >> x >> y >> z )
      {
        geometry_msgs::Point p;
        p.x=x; p.y=y; p.z=z;
        bfs.points.push_back(p);  
      }
      
      std::string visual_id = "hand-elbow-bfs";
      std::string visual_fname = boost::str(boost::format("%s-%s.jpg") % traj_id % visual_id);
      std::string visual_fpath = (vis_directory/visual_fname).string();
      boost::filesystem::path check_image(visual_fpath);
      if(  visual_fpath != video_lookup_table.getVideoFile( traj_id, visual_id ) || !boost::filesystem::exists(check_image))
      {
        marker_pub.publish( elbow_lines );
        marker_pub.publish( elbow_points );
        marker_pub.publish( hand_lines );
        marker_pub.publish( hand_points );
        marker_pub.publish( bfs );
        publishRobotStateAndCapture( rt->getWayPoint(0), visual_fpath, state_pub, rec_pub );
        // verify
        if(boost::filesystem::exists(check_image))
        {
          ROS_INFO("Image saved. Proceeding to next image.");
          video_lookup_table.putVideoFile(traj_id, visual_id, visual_fpath);
          // video_lookup_table.saveToBag( save_directory.string() );
        }
        else
        {
          ROS_ERROR("Image not saved. Halting the node.");
          ros::shutdown();
          exit(0);
        }
      }
      else
        ROS_WARN("Image already saved. Skipping.");

      if(state_pub.getNumSubscribers()<1)
      {
        ROS_ERROR("A node might have crashed because no one is listening to the state publisher. Exiting.");
        ros::shutdown();
        exit(0);
      }
      ROS_WARN("Pausing...");
      std::cin.get();
    }
    // video_lookup_table.saveToBag( save_directory.string() );
  }
  catch(...)
  {
    std::cout << "Failure!" << std::endl;
  }

  ROS_INFO("Completed recording display of trajectory drawings.");
  ros::shutdown();

  return 0;
}

