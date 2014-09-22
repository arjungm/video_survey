#ifndef TRAJ_FEAT_H
#define TRAJ_FEAT_H

#include <string>
#include <boost/shared_ptr.hpp>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

class Feature
{
  private:
    std::string name_;
  public:
    Feature() {}
    Feature(const std::string& name) : name_(name) {}
    ~Feature() {}
    std::string getName() { return name_ };
};

class DoubleFeature : public Feature
{
  private:
    std::vector<double> values_;
  public:
    typedef std::vector<double>::iterator iterator;;
    typedef std::vector<double>::const_iterator const_iterator;

    DoubleFeature(const std::string& name) : name_(name){}
    ~DoubleFeature() {}
    virtual void compute(const planning_scene::PlanningScenePtr& ps, const robot_trajectory::RobotTrajectoryPtr& rt) = 0;

    iterator begin() { return values_.begin(); }
    iterator end() { return values_.end(); }
};

#endif
