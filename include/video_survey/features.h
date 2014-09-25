#ifndef TRAJ_FEAT_H
#define TRAJ_FEAT_H

#include <ros/ros.h>

#include <string>
#include <vector>
#include <utility>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <ecl/containers.hpp>
#include <ecl/geometry.hpp>

using namespace std;

struct Point3
{
  double x,y,z;
  Point3(double _x, double _y, double _z) : x(_x), y(_y), z(_z){}
  double dot(const Point3& p) const { return x*p.x + y*p.y + z*p.z; }
};

Point3 operator+(const Point3& a, const Point3& b){ return Point3(a.x+b.x, a.y+b.y, a.z+b.z); }
Point3 operator-(const Point3& a, const Point3& b){ return Point3(a.x-b.x, a.y-b.y, a.z-b.z); }
Point3 operator*(double s, const Point3& p){ return Point3(s*p.x, s*p.y, s*p.z); }
    
typedef pair<string,double> Feat;

class TrajectoryFeatures
{
  private:
    vector< pair<string,double> > features_;
    planning_scene::PlanningScenePtr ps_;
    robot_trajectory::RobotTrajectoryPtr rt_;
  protected:
    double linear_distance(const Eigen::Affine3d& lhs, const Eigen::Affine3d& rhs);
    double linear_distance(double, double, double, double, double, double);
    double linear_distance(const Point3& a, const Point3& b);
    Point3 spline_interp(const ecl::CubicSpline& xc, const ecl::CubicSpline& yc, const ecl::CubicSpline& zc, double t);
    double line_interp(double ymax, double ymin, double tmax, double tmin, double t);
    Point3 line_interp2(Point3 a, Point3 b, double ta, double tb, double t);
    double min_dist_line(const Point3& p, const Point3& a, const Point3& b);
    vector<Feat> computeHausdorffLine(const string& link_name, ecl::CubicSpline& xc, ecl::CubicSpline& yc, ecl::CubicSpline& zc);
    double computeDTW(const string& link_name, ecl::CubicSpline& xc, ecl::CubicSpline& yc, ecl::CubicSpline& zc);
  public:
    typedef vector<Feat>::iterator iterator;

    TrajectoryFeatures();
    ~TrajectoryFeatures();
    void setPlanningScene(planning_scene::PlanningScenePtr ps);
    void setRobotTrajectory(robot_trajectory::RobotTrajectoryPtr rt);

    void computeAll();

    void computeClearance();
    void computeSmoothness();
    void computeLength();
    void computeCartesianFeatures(const string& link_name);
    void computeJointLimitDistance(); 

    iterator begin() { return features_.begin(); }
    iterator end() { return features_.end(); }
};
#endif
