#include <string>
#include <vector>
#include <utility>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

using namespace std;

class TrajectoryFeatures
{
  private:
    vector< pair<string,double> > features_;
    planning_scene::PlanningScenePtr ps_;
    robot_trajectory::RobotTrajectoryPtr rt_;
  protected:
    double linear_distance(const Eigen::Affine3d& lhs, const Eigen::Affine3d& rhs);
  public:
    typedef vector< pair<string,double> >::iterator iterator;
    TrajectoryFeatures();
    ~TrajectoryFeatures();
    void setPlanningScene(planning_scene::PlanningScenePtr& ps);
    void setRobotTrajectory(robot_trajectory::RobotTrajectoryPtr& rt);

    void computeAll();

    void computeClearance();
    void computeSmoothness();
    void computeLength();
    void computeCartesianFeatures(const string& link_name);
    void computeJointLimitDistance();

    iterator begin() { return features_.begin(); }
    iterator end() { return features_.end(); }
};

