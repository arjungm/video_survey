#include "video_survey/features.h"
#include <boost/format.hpp>
#include <boost/math/constants/constants.hpp>
#include <utility>

#include <Eigen/Geometry>

#include <ecl/containers.hpp>
#include <ecl/geometry.hpp>

void TrajectoryFeatures::setPlanningScene(planning_scene::PlanningScenePtr& ps)
{
  ps_ = ps;
}
void TrajectoryFeatures::setRobotTrajectory(robot_trajectory::RobotTrajectoryPtr& rt)
{
  rt_ = rt;
}

void TrajectoryFeatures::computeClearance()
{
  double clearance;
  
  collision_detection::CollisionRequest req;
  for (std::size_t k = 0 ; k < rt_->getWayPointCount() ; ++k)
  {
    collision_detection::CollisionResult res;
    ps_->checkCollisionUnpadded(req, res, rt_->getWayPoint(k));

    double d = ps_->distanceToCollisionUnpadded(rt_->getWayPoint(k));
    if (d > 0.0) // in case of collision, distance is negative
      clearance += d;
  }
  clearance /= (double)rt_->getWayPointCount();
  
  features_.push_back(make_pair("clearance", clearance));
}

void TrajectoryFeatures::computeSmoothness()
{
  double smoothness;
  
  double a = rt_->getWayPoint(0).distance(rt_->getWayPoint(1));
  for (std::size_t k = 2 ; k < rt_->getWayPointCount() ; ++k)
  {
    double b = rt_->getWayPoint(k-1).distance(rt_->getWayPoint(k));
    double cdist = rt_->getWayPoint(k-2).distance(rt_->getWayPoint(k));
    double acosValue = (a*a + b*b - cdist*cdist) / (2.0*a*b);
    if (acosValue > -1.0 && acosValue < 1.0)
    {
      double angle = (boost::math::constants::pi<double>() - acos(acosValue));
      double u = 2.0 * angle; /// (a + b);
      smoothness += u * u;
    }
    a = b;
  }
  smoothness /= (double)rt_->getWayPointCount();

  features_.push_back(make_pair("smoothness", smoothness));
}

void TrajectoryFeatures::computeLength()
{
  double length=0.0;
  for (std::size_t k = 1 ; k < rt_->getWayPointCount() ; ++k)
    length += rt_->getWayPoint(k-1).distance(rt_->getWayPoint(k));
  //normalize
  features_.push_back(make_pair("path_length", length));
}
    
double TrajectoryFeatures::linear_distance(const Eigen::Affine3d& lhs, const Eigen::Affine3d& rhs)
{
  return (lhs.translation() - rhs.translation()).norm();
}

void TrajectoryFeatures::computeCartesianFeatures(const string& link_name)
{
  double length=0.0, smoothness=0.0;
  
  for (std::size_t k = 1 ; k < rt_->getWayPointCount() ; ++k)
    length += linear_distance( rt_->getWayPoint(k-1).getGlobalLinkTransform(link_name),
                                rt_->getWayPoint(k).getGlobalLinkTransform(link_name) );


  double a = linear_distance( rt_->getWayPoint(0).getGlobalLinkTransform(link_name),
                              rt_->getWayPoint(1).getGlobalLinkTransform(link_name));
  for (std::size_t k = 2 ; k < rt_->getWayPointCount() ; ++k)
  {
    double b = linear_distance( rt_->getWayPoint(k-1).getGlobalLinkTransform(link_name),
                                rt_->getWayPoint(k).getGlobalLinkTransform(link_name));
    double cdist = linear_distance( rt_->getWayPoint(k-2).getGlobalLinkTransform(link_name),
                                    rt_->getWayPoint(k).getGlobalLinkTransform(link_name));   
    double acosValue = (a*a + b*b - cdist*cdist) / (2.0*a*b);
    if (acosValue > -1.0 && acosValue < 1.0)
    {
      double angle = (boost::math::constants::pi<double>() - acos(acosValue));
      double u = 2.0 * angle; /// (a + b);
      smoothness += u * u;
    }
    a = b;
  }
  smoothness /= (double)rt_->getWayPointCount();

  // get xyz spline of ee wrt time
  int N = rt_->getWayPointCount();
  ecl::Array<double> x_set(N), y_set(N), z_set(N), t_set(N);
  for(size_t k=0; k<rt_->getWayPointCount(); ++k)
  {
    x_set.at(k) = rt_->getWayPoint(k).getGlobalLinkTransform(link_name).translation().x();
    y_set.at(k) = rt_->getWayPoint(k).getGlobalLinkTransform(link_name).translation().y();
    z_set.at(k) = rt_->getWayPoint(k).getGlobalLinkTransform(link_name).translation().z();
    t_set.at(k) = rt_->getWaypointDurationFromStart(k);
  }
  ecl::CubicSpline xc, yc, zc;
  xc = ecl::CubicSpline::ContinuousDerivatives(t_set, x_set, 0, 0);
  yc = ecl::CubicSpline::ContinuousDerivatives(t_set, y_set, 0, 0);
  zc = ecl::CubicSpline::ContinuousDerivatives(t_set, z_set, 0, 0);

  // compute r'(t), r"(t)
  double max_curvature = 0.0;
  for(size_t k=0; k<N; ++k)
  {
    double t=rt_->getWaypointDurationFromStart(k);
    Eigen::Vector3d rd(xc.derivative(t),
        yc.derivative(t),
        zc.derivative(t));
    Eigen::Vector3d rdd(xc.dderivative(t),
        yc.dderivative(t),
        zc.dderivative(t));
    double curv = (rd.cross(rdd)).norm() / pow(rd.norm(),3);
    max_curvature = (curv > max_curvature) ? curv : max_curvature;
  }

  features_.push_back(make_pair(boost::str(boost::format("%s_%s") % link_name % "length" ), length));
  features_.push_back(make_pair(boost::str(boost::format("%s_%s") % link_name % "smoothness") , smoothness)); 
  features_.push_back(make_pair(boost::str(boost::format("%s_%s") % link_name % "max_curvature" ), max_curvature));
}

void TrajectoryFeatures::computeJointLimitDistance()
{
  // assuming right arm
  const moveit::core::JointModelGroup* jmg = rt_->getWayPoint(0).getJointModelGroup("right_arm");
  const vector<const moveit::core::JointModel*> joints = jmg->getJointModels();
  vector< vector<double> > joint_lower_bounds( joints.size() );
  vector< vector<double> > joint_upper_bounds( joints.size() );
  for(size_t j=0; j<joints.size(); ++j)
  {
    // get limits
    const moveit::core::JointModel::Bounds& bounds = joints[j]->getVariableBounds();
    vector<double> lower_bounds(bounds.size());
    vector<double> upper_bounds(bounds.size());
    for(size_t k=0; k<bounds.size();++k)
    {
      lower_bounds[k] = bounds[k].min_position_;
      upper_bounds[k] = bounds[k].max_position_;
    }
    joint_lower_bounds[j] = lower_bounds;
    joint_upper_bounds[j] = upper_bounds;
  }

  // get min distance 
  double max_min_jdist = 0.0;
  for(size_t k=0; k<rt_->getWayPointCount(); ++k)
  {
    //for each joint
    double min_jdist = std::numeric_limits<double>::max();
    for(size_t j=0; j<joints.size(); j++)
    {
      const double* jpos = rt_->getWayPoint(k).getJointPositions(joints[j]);
      double ldist = joints[j]->distance(jpos, &(joint_lower_bounds[j][0]));
      double udist = joints[j]->distance(jpos, &(joint_upper_bounds[j][0]));
      double jdist = min(ldist, udist) / (udist+ldist);
      min_jdist = min(jdist, min_jdist);
    }
    max_min_jdist = max(max_min_jdist, min_jdist);
  }
  features_.push_back(make_pair("maxmin_joint_limit_distance", max_min_jdist));
}

void TrajectoryFeatures::computeAll()
{
  features_.clear();

  computeClearance();
  computeSmoothness();
  computeLength();
  computeCartesianFeatures("r_wrist_roll_link");
  computeCartesianFeatures("r_elbow_flex_link");
  computeJointLimitDistance();
}
