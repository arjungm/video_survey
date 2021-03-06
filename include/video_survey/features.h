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
  string to_string() { return "("+boost::lexical_cast<string>(x)+","+boost::lexical_cast<string>(y)+","+boost::lexical_cast<string>(z)+")"; }
};

Point3 operator+(const Point3& a, const Point3& b){ return Point3(a.x+b.x, a.y+b.y, a.z+b.z); }
Point3 operator-(const Point3& a, const Point3& b){ return Point3(a.x-b.x, a.y-b.y, a.z-b.z); }
Point3 operator*(double s, const Point3& p){ return Point3(s*p.x, s*p.y, s*p.z); }
    
typedef pair<string,double> Feat;
typedef boost::function<Point3 (double)> PointInterp;

class TrajectoryFeatures
{
  private:
    vector<Feat> features_;
    planning_scene::PlanningScenePtr ps_;
    robot_trajectory::RobotTrajectoryPtr rt_;
    boost::function<Point3 (double)> comparison_spline_;
    boost::function<Point3 (double)> bfs_path_;
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
    double computeSquaredAccelerations(const string& name, const ecl::CubicSpline& xc, const ecl::CubicSpline& yc, const ecl::CubicSpline& zc);
    void computeSquaredAccelerations();
    double computeQuaternionDistance(const string& link_name, ecl::CubicSpline& xc, ecl::CubicSpline& yc, ecl::CubicSpline& zc);
    void add(const string& name, double value);
    void add(const vector<Feat>& feats);

    vector<double> getTimes();
    vector<Point3> getLinkPositions(const string& link);
    boost::function<Point3 (double)> getLineInterpolation(const vector<Point3>& trajectory, const vector<double>& times);
    boost::function<Point3 (double)> getSplineInterpolation(const vector<Point3>& trajectory, const vector<double>& times);
    vector<Feat> computeHausdorffFeatures(const PointInterp& from, const PointInterp& to, const string& from_name, const string& to_name);
    double computeDTW(const PointInterp& from_traj, const PointInterp& to_traj);
  public:
    typedef vector<Feat>::iterator iterator;

    TrajectoryFeatures();
    ~TrajectoryFeatures();
    void setPlanningScene(planning_scene::PlanningScenePtr ps);
    void setRobotTrajectory(robot_trajectory::RobotTrajectoryPtr rt);

    PointInterp processBFSpath(const vector<Point3>& path);
    void setComparisonPath( const vector<Point3>& path );
    void computeAll();

    void computeClearance();
    void computeSmoothness();
    void computeLength();
    void computeCartesianFeatures(const string& link_name, const string& link_short_name);
    void computeJointLimitDistance(); 

    iterator begin() { return features_.begin(); }
    iterator end() { return features_.end(); }
};
#endif
