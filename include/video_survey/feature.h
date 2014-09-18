#ifndef TRAJ_FEAT_H
#define TRAJ_FEAT_H

#include <string>
#include <boost/shared_ptr.hpp>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

class DoubleFeature
{
  private:
    std::string name_;
  public:
    typedef boost::shared_ptr<DoubleFeature> Ptr;
    DoubleFeature(const std::string& name) : name_(name){}
    ~DoubleFeature() {}
    virtual double compute(const planning_scene::PlanningScenePtr& ps, const robot_trajectory::RobotTrajectoryPtr& rt) = 0;
    std::string getName() { return name_; }
};

#endif
