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
#include <visualization_msgs/MarkerArray.h>

#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/regex.hpp>

#include "video_survey/experiment_utils.h"
#include <moveit_recorder/utils.h>
#include <moveit_recorder/trajectory_video_lookup.h>
#include <moveit_recorder/trajectory_retimer.h>
#include <moveit_recorder/cli_controller.h>

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
void publishView(view_controller_msgs::CameraPlacement view, ros::Publisher& view_pub)
{
  view.time_from_start = ros::Duration(0.001);
  ros::Time t_now = ros::Time::now();
  view.eye.header.stamp = t_now;
  view.focus.header.stamp = t_now;
  view.up.header.stamp = t_now;
  view_pub.publish( view );
  ros::spinOnce();
  usleep(1000);
}
void publishPlanningScene(const moveit_msgs::PlanningScene& ps_msg, ros::Publisher& ps_pub)
{
  moveit_msgs::PlanningScene empty_ps_msg;
  ps_pub.publish(empty_ps_msg);
  ros::spinOnce(); usleep(1000);
  ps_pub.publish(ps_msg);
  ros::spinOnce(); usleep(1000);
}

visualization_msgs::Marker initLineStripMarker(const std::string& ns, int id)
{
  visualization_msgs::Marker line;
  line.header.frame_id = "/odom_combined";
  line.header.stamp = ros::Time::now();
  line.action = visualization_msgs::Marker::ADD;
  line.pose.orientation.w = 1.0;
  line.scale.x = 0.01;
  line.color.a = 1.0;
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.id = id;
  line.ns = ns;
  return line;
}

visualization_msgs::Marker getTrajectoryMarker( const robot_trajectory::RobotTrajectoryPtr& rt, 
                                                const std::string& link, 
                                                const std::string& ns, 
                                                int id)
{
  visualization_msgs::Marker line = initLineStripMarker(ns,id);
  for(int k=0; k<rt->getWayPointCount(); ++k)
  {
    geometry_msgs::Point p;
    p.x = rt->getWayPointPtr(k)->getGlobalLinkTransform(link).translation().x();
    p.y = rt->getWayPointPtr(k)->getGlobalLinkTransform(link).translation().y();
    p.z = rt->getWayPointPtr(k)->getGlobalLinkTransform(link).translation().z();
    line.points.push_back(p);
  }
  return line;
}

static view_controller_msgs::CameraPlacement last_recorded_msg;

void recordViewpointCallback(const boost::shared_ptr<view_controller_msgs::CameraPlacement const>& msg)
{
    last_recorded_msg = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualization_node");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //wait for RVIZ;
  sleep(20);
  std::cin.get();

  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("planning_scene_topic",boost::program_options::value<std::string>(), "Topic for publishing the planning scene for recording")
    ("display_pose_topic",boost::program_options::value<std::string>(), "Topic for visualizing state of the robot for recording")
    ("camera_topic",boost::program_options::value<std::string>(), "Topic for publishing the view to take the snapshot from")
    ("vis_traj",boost::program_options::value<bool>(),"flag for visualization of trajectory snapshots")
    ("vis_dir",boost::program_options::value<std::string>(),"Directory for visualizations")
    ("path_dir",boost::program_options::value<std::string>(), "Directory for paths")
    ("save_dir",boost::program_options::value<std::string>(), "Directory for videos");
  boost::program_options::variables_map vm;
  boost::program_options::parsed_options po = boost::program_options::parse_command_line(argc, argv, desc);
  boost::program_options::store(po, vm);
  boost::program_options::notify(vm);

  boost::function<void(void)> alert_me = boost::bind(&exp_utils::system::runCommand, "aplay /usr/share/sounds/alsa/Front_Center.wav");

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
    bool visualize_trajectory_snapshots = vm.count("vis_traj")? vm["vis_traj"].as<bool>() : true;

    ros::Subscriber cam_sub = node_handle.subscribe("/rviz/current_camera_placement",1,recordViewpointCallback);
    ros::Publisher rec_pub = node_handle.advertise<rviz_video_recorder::RecorderRequest>(recorder_command_topic, 1, true);
    ros::Publisher ps_pub = node_handle.advertise<moveit_msgs::PlanningScene>(planning_scene_topic, 1, true);
    ros::Publisher state_pub = node_handle.advertise<moveit_msgs::DisplayRobotState>(display_pose_topic, 1, true);
    ros::Publisher view_pub = node_handle.advertise<view_controller_msgs::CameraPlacement>(camera_topic, 1, true);
    ros::Publisher marker_array_pub = node_handle.advertise<visualization_msgs::MarkerArray>("path_visuals",1, true) ;

    utils::rostopic::waitOnSubscribersToTopic(rec_pub, recorder_command_topic);
    utils::rostopic::waitOnSubscribersToTopic(ps_pub, planning_scene_topic);
    utils::rostopic::waitOnSubscribersToTopic(state_pub, display_pose_topic);
    utils::rostopic::waitOnSubscribersToTopic(view_pub, camera_topic);
    utils::rostopic::waitOnSubscribersToTopic(marker_array_pub, "path_visuals");

    // path directory for path files
    std::string path_dir = utils::get_option(vm, "path_dir", "");
    boost::filesystem::path path_directory(path_dir);

    // visualization directory
    std::string vis_dir = utils::get_option(vm, "vis_dir", "");
    boost::filesystem::path vis_directory(vis_dir);

    // view grabbing
    /*
    std::vector<view_controller_msgs::CameraPlacement> views_to_get;
    int c = recorder_utils::getch();
    ROS_INFO("Give the views");
    while(c!='n')
    {
      ros::spinOnce();
      usleep(1000);
      if(c=='a')
      {
        views_to_get.push_back(last_recorded_msg);
        ROS_INFO("Added %d views so far.", (int)views_to_get.size());
      }
      c = recorder_utils::getch();
    }
    ROS_INFO("Done with views");
    */

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

      std::map<std::string,double> var_map = retimer->getWorldJointVariables();
      Eigen::Affine3d corrected_pose = Eigen::Affine3d::Identity();
      corrected_pose = corrected_pose.translate(Eigen::Vector3d( var_map["world_joint/x"], var_map["world_joint/y"], 0.0 ));
      corrected_pose = corrected_pose.rotate(Eigen::AngleAxisd( var_map["world_joint/theta"], Eigen::Vector3d::UnitZ() ));
      
      retimer->correctRootJointPositions();

      // get display trajectories
      moveit_msgs::RobotState start_state, goal_state;
      moveit::core::robotStateToRobotStateMsg( rt->getWayPoint(0), start_state );
      moveit::core::robotStateToRobotStateMsg( rt->getWayPoint( rt->getWayPointCount()-1 ), goal_state );
      // visuals to take
      // trajectory-snapshots (4-5 images of the trajectory state)
      // elbow spline path + start_state
      // end effector spline path + start_state
      if(visualize_trajectory_snapshots)
      {
        int n = 5;
        for(int i=0; i<n; i++)
        {
          //get throttle index
          size_t waypoint_index;
          if(i==(n-1))
            waypoint_index = rt->getWayPointCount()-1;
          else
            waypoint_index = (i*rt->getWayPointCount())/n;
          
          //get traj
          int unique_id = 0;
          visualization_msgs::Marker wrist_line = getTrajectoryMarker(rt, "r_wrist_roll_link", "wrist_path", unique_id++);
          visualization_msgs::Marker tool_line = getTrajectoryMarker(rt, "r_gripper_tool_frame", "tool path", unique_id++);
          wrist_line.color.r=1.0;
          tool_line.color.g=1.0;
          //contain
          visualization_msgs::MarkerArray hand_path_visuals;
          hand_path_visuals.markers.push_back(wrist_line);
          hand_path_visuals.markers.push_back(tool_line);
          // for each view
          for(int v=0; v<trajectory_it->second.views.size();++v)
          {
            publishPlanningScene(trajectory_it->second.ps, ps_pub);
            marker_array_pub.publish(hand_path_visuals);
            ros::spinOnce(); 
            usleep(1000);
            std::string visual_id = boost::str(boost::format("snapshot-%d-%d") % (i+1) % (v+1));
            std::string visual_fname = boost::str(boost::format("%s-%s.%s") % traj_id % visual_id % "jpg");
            std::string visual_fpath = (vis_directory / visual_fname).string();
            boost::filesystem::path check_image(visual_fpath);
            if( visual_fpath != video_lookup_table.getVideoFile( traj_id, visual_id ) || !boost::filesystem::exists(check_image))
            {
              publishView(trajectory_it->second.views[v], view_pub);
              usleep(500000);
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
          }// views
        }// done with snapshots
      }
      else
      {
        //load the path into a visualization marker message
        visualization_msgs::Marker elbow_lines, hand_lines, bfs;
        int unique_id = 0;
        // ===============
        // ELBOW
        // ===============
        elbow_lines = getTrajectoryMarker(rt,"r_elbow_flex_link","elbow_path",unique_id++);
        elbow_lines.color.b = 0.8;
        elbow_lines.color.g = 0.8;
        // ===============
        // END EFFECTOR
        // ===============
        hand_lines = getTrajectoryMarker(rt,"r_gripper_tool_frame","hand_path",unique_id++);
        hand_lines.color.r = 1.0;
        // ===============
        // BFS
        // ===============
        double x,y,z;
        std::ifstream bfs_path_file;
        bfs_path_file.open( (path_directory/(traj_id+".path")).string().c_str() );
        bfs = initLineStripMarker("bfs_path",unique_id++);
        bfs.color.g = 1.0;
        while( bfs_path_file >> x >> y >> z )
        {
          // Eigen::Vector3d fixed_point = corrected_pose.inverse() * Eigen::Vector3d(x,y,z);
          geometry_msgs::Point p;
          // p.x=fixed_point(0); p.y=fixed_point(1); p.z=fixed_point(2);
          p.x=x; p.y=y; p.z=z;
          bfs.points.push_back(p);  
        }

        // for each view
        // for(int v=0; v<views_to_get.size();++v)
        for(int v=0; v<trajectory_it->second.views.size();++v)
        {
          std::string visual_id = boost::str(boost::format("%s-%d") % "hand-elbow-bfs" % (v+1) );
          std::string visual_fname = boost::str(boost::format("%s-%s.jpg") % traj_id % visual_id);
          std::string visual_fpath = (vis_directory/visual_fname).string();
          boost::filesystem::path check_image(visual_fpath);
          if( !boost::filesystem::exists(check_image))
          {
            publishPlanningScene(trajectory_it->second.ps, ps_pub);
            // views
            publishView(trajectory_it->second.views[v], view_pub);
            ROS_INFO("Visualizing %d line segments", elbow_lines.points.size()+hand_lines.points.size()+bfs.points.size());
            visualization_msgs::MarkerArray path_visuals;
            // path_visuals.markers.push_back(elbow_lines);
            path_visuals.markers.push_back(hand_lines);
            path_visuals.markers.push_back(bfs);

            marker_array_pub.publish(path_visuals);
            ros::spinOnce(); 
            usleep(1000);
            publishRobotStateAndCapture( rt->getWayPoint(0), visual_fpath, state_pub, rec_pub );
            // verify
            if(boost::filesystem::exists(check_image))
            {
              ROS_INFO("Image saved. Proceeding to next image.");
              video_lookup_table.putVideoFile(traj_id, visual_id, visual_fpath);
              video_lookup_table.saveToBag( save_directory.string() );
            }
            else
            {
              ROS_ERROR("Image not saved. Halting the node.");
              alert_me();
              ros::shutdown();
              exit(0);
            }
          }
          else
            ROS_WARN("Image already saved. Skipping.");
        }// views
      }

      if(state_pub.getNumSubscribers()<1)
      {
        ROS_ERROR("A node might have crashed because no one is listening to the state publisher. Exiting.");
        alert_me();
        ros::shutdown();
        exit(0);
      }
    }
    // video_lookup_table.saveToBag( save_directory.string() );
  }
  catch(...)
  {
    std::cout << "Failure!" << std::endl;
  }

  ROS_INFO("Completed recording display of trajectory drawings.");
  alert_me();
  ros::shutdown();

  return 0;
}

