#include <ros/ros.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <iostream>
#include <vector>
#include <string>

#include <moveit_recorder/utils.h>

using namespace std;

bool enable;
planning_scene::PlanningScenePtr scene_;

void sceneCallback(const boost::shared_ptr<moveit_msgs::PlanningScene const>& msg)
{
  if(enable)
  {
    scene_->usePlanningSceneMsg(*msg);
    ROS_INFO("Updated the scene.");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "split_screen_creator");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  enable = true;

  // publisher to scene
  string scene_topic = "planning_scene";
  ros::Publisher ps_pub = node_handle.advertise<moveit_msgs::PlanningScene>(scene_topic,1, true);
  utils::rostopic::waitOnSubscribersToTopic(ps_pub, scene_topic);

  // subscriber to scene
  ros::Subscriber ps_sub = node_handle.subscribe(scene_topic, 1, &sceneCallback);
  utils::rostopic::waitOnPublishersToTopic(ps_sub, scene_topic);

  // load robot
  planning_scene_monitor::PlanningSceneMonitor psm("robot_description");
  scene_ = psm.getPlanningScene();
  
  ros::spinOnce();
  sleep(1);

  // print joint names
  robot_state::RobotState& rs = scene_->getCurrentStateNonConst();
  vector<string> variable_names = rs.getVariableNames();
  ROS_INFO("Variable Joints: %d", (int)variable_names.size());
  map<string, double> config;
  for(vector<string>::iterator i=variable_names.begin(); i!=variable_names.end(); ++i)
  {
    config[*i] = rs.getVariablePosition(*i);
    ROS_INFO("%s = %.2f", i->c_str(), config[*i]);
  }

  // control robot
  for(int i=0; i<10; ++i)
  {
    config["world_joint/trans_x"] += 0.1;
    config["world_joint/trans_y"] += 0.1;
    config["world_joint/trans_z"] += 0.1;
    rs.setVariablePositions(config);

    moveit_msgs::PlanningScene ps_msg;
    scene_->getPlanningSceneMsg(ps_msg);
    ps_pub.publish(ps_msg);
    sleep(1);
    ros::spinOnce();
    ROS_INFO("Publishing...");
    for(vector<string>::iterator i=variable_names.begin(); i!=variable_names.end(); ++i)
    {
      config[*i] = rs.getVariablePosition(*i);
      ROS_INFO("%s = %.2f", i->c_str(), scene_->getCurrentState().getVariablePosition(*i));
    }
  }

  return 0;
}
