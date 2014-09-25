#include "video_survey/features.h"
#include <boost/format.hpp>
#include <boost/math/constants/constants.hpp>
#include <utility>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <Eigen/Geometry>

TrajectoryFeatures::TrajectoryFeatures() {}
TrajectoryFeatures::~TrajectoryFeatures() {}

void TrajectoryFeatures::setPlanningScene(planning_scene::PlanningScenePtr ps)
{
  ps_ = ps;
}
void TrajectoryFeatures::setRobotTrajectory(robot_trajectory::RobotTrajectoryPtr rt)
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
  
  ROS_INFO("\t Compute length");
  // get linear path length in workspace
  for (std::size_t k = 1 ; k < rt_->getWayPointCount() ; ++k)
    length += linear_distance( rt_->getWayPoint(k-1).getGlobalLinkTransform(link_name),
                                rt_->getWayPoint(k).getGlobalLinkTransform(link_name) );

  ROS_INFO("\t Compute smoothness");
  // get smoothness value for trajectory in workspace
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

  ROS_INFO("\t Compute splines");
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

  ROS_INFO("\t Compute curvature");
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

  ROS_INFO("\t Compute Hausdorff features");
  vector< pair<string,double> > hausdorff_feats = computeHausdorffLine(link_name, xc, yc, zc);
  ROS_INFO("\t Compute DTW");
  double dtw_cost = computeDTW(link_name, xc, yc, zc);

  features_.push_back(make_pair(boost::str(boost::format("%s_%s") % link_name % "length" ), length));
  features_.push_back(make_pair(boost::str(boost::format("%s_%s") % link_name % "smoothness") , smoothness)); 
  features_.push_back(make_pair(boost::str(boost::format("%s_%s") % link_name % "max_curvature" ), max_curvature));
  for(vector<Feat>::iterator i=hausdorff_feats.begin(); i!=hausdorff_feats.end(); ++i)
    features_.push_back(*i);
  features_.push_back(make_pair(boost::str(boost::format("%s_%s") % link_name % "DTW"), dtw_cost));
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

double TrajectoryFeatures::line_interp(double ymax, double ymin, double tmax, double tmin, double t)
{
  return ymin+(t*(ymax-ymin)/(tmax-tmin));
}

Point3 TrajectoryFeatures::line_interp2(Point3 a, Point3 b, double ta, double tb, double t)
{
  return Point3(line_interp(a.x,b.x,ta,tb,t),
                line_interp(a.y,b.y,ta,tb,t),
                line_interp(a.z,b.z,ta,tb,t));
}

double TrajectoryFeatures::linear_distance(double ax, double ay, double az, double bx, double by, double bz)
{
  return sqrt(pow(ax-bx,2)+pow(ay-by,2)+pow(az-bz,2));
}
double TrajectoryFeatures::linear_distance(const Point3& a, const Point3& b)
{
  return linear_distance(a.x, a.y, a.z, b.x, b.y, b.z);
}
double TrajectoryFeatures::min_dist_line(const Point3& p, const Point3& a, const Point3& b)
{
  double L2 = pow(linear_distance(a,b),2);
  if(L2==0.0){ return linear_distance(p,a); }
  double t = (p-a).dot(b-a)/L2;
  if( t<0.0 ) { return linear_distance(p,a); }
  if( t>1.0 ) { return linear_distance(p,b); }
  return linear_distance(p,a+t*(b-a));
}

vector< pair<string,double> > TrajectoryFeatures::computeHausdorffLine(const string& link_name, ecl::CubicSpline& xc, ecl::CubicSpline& yc, ecl::CubicSpline& zc)
{
  // get interpolated straight line trajectory in work-space
  boost::function<double (double)> x_int = boost::bind(&TrajectoryFeatures::line_interp, this, xc( xc.domain().back() ), xc(0.0), xc.domain().back(), 0.0, _1);
  boost::function<double (double)> y_int = boost::bind(&TrajectoryFeatures::line_interp, this, yc( yc.domain().back() ), yc(0.0), yc.domain().back(), 0.0, _1);
  boost::function<double (double)> z_int = boost::bind(&TrajectoryFeatures::line_interp, this, zc( zc.domain().back() ), zc(0.0), zc.domain().back(), 0.0, _1);
  
  // for each timestamp
  // traj -> interp (A -> B)
  double hAB = numeric_limits<double>::min();
  double hBA = numeric_limits<double>::min();
  double imhAB;

  for(int k1=0; k1<rt_->getWayPointCount(); ++k1)
  {
    double t1 = rt_->getWaypointDurationFromStart(k1);
    // find min over interp
    double min_dAB = numeric_limits<double>::max();
    double min_dBA = numeric_limits<double>::max();
    double min_dAB_seg = numeric_limits<double>::max();
    for(int k2=0; k2< rt_->getWayPointCount(); ++k2)
    {
      double t2 = rt_->getWaypointDurationFromStart(k2);
      double t0 = rt_->getWaypointDurationFromStart(k2);
      double te = rt_->getWaypointDurationFromStart(k2+1);

      double dAB = linear_distance(xc(t1),yc(t1),zc(t1), x_int(t2), y_int(t2), z_int(t2));
      double dBA = linear_distance(x_int(t1),y_int(t1),z_int(t1), xc(t2), yc(t2), zc(t2));
      min_dAB = min(min_dAB,dAB);
      min_dBA = min(min_dBA,dBA);

      if(k2+1 < rt_->getWayPointCount())
      {
        double dAB_seg = min_dist_line( Point3(xc(t1),yc(t1),zc(t1)), 
                                  Point3(x_int(t0), y_int(t0), z_int(t0)),
                                  Point3(x_int(te), y_int(te), z_int(te)));
        min_dAB_seg = min(min_dAB_seg, dAB_seg);
      }
    }
    hAB = max(min_dAB, hAB);
    hBA = max(min_dBA, hBA);

    imhAB+=min_dAB_seg;
  }
  imhAB /= rt_->getWayPointCount();
  double H = max(hAB, hBA);

  vector< pair<string, double> > hausdorff_feats(4);
  hausdorff_feats[0] = make_pair(link_name+"_h_traj_line", hAB);
  hausdorff_feats[1] = make_pair(link_name+"_h_line_traj", hBA);
  hausdorff_feats[2] = make_pair(link_name+"_H_traj_line", H);
  hausdorff_feats[3] = make_pair(link_name+"_imh_traj_line", imhAB);
  return hausdorff_feats;
}

Point3 TrajectoryFeatures::spline_interp(const ecl::CubicSpline& xc, const ecl::CubicSpline& yc, const ecl::CubicSpline& zc, double t)
{
  return Point3(xc(t), yc(t), zc(t));
}

double TrajectoryFeatures::computeDTW(const string& link_name, ecl::CubicSpline& xc, ecl::CubicSpline& yc, ecl::CubicSpline& zc)
{
  double t_end = xc.domain().back();
  Point3 start(xc(0.0), yc(0.0), zc(0.0));
  Point3 end(xc(t_end), yc(t_end), zc(t_end));
  boost::function<Point3 (double)> p_int = boost::bind(&TrajectoryFeatures::line_interp2, this, start, end, 0.0, t_end, _1); 
  boost::function<Point3 (double)> s_int = boost::bind(&TrajectoryFeatures::spline_interp, this, xc, yc, zc, _1);
  boost::function<double (size_t)> get_time = boost::bind(&robot_trajectory::RobotTrajectory::getWaypointDurationFromStart, rt_, _1);

  double* dtw = new double[rt_->getWayPointCount()*rt_->getWayPointCount()];
  struct Ind
  {
    size_t stride_;
    Ind(size_t stride) : stride_(stride) {}
    size_t operator()(size_t i, size_t j){ return i+j*stride_; }
  }ind(rt_->getWayPointCount());
  
  dtw[ind(0,0)] = 0;
  // i -> spline
  // j -> line
  size_t N = rt_->getWayPointCount();
  for(size_t i=1; i<N; ++i)
    dtw[ind(i,0)] = dtw[ind(i-1,0)] + linear_distance(s_int(get_time(i)),p_int(get_time(0)));
  for(size_t j=1; j<N; ++j)
    dtw[ind(0,j)] = dtw[ind(0,j-1)] + linear_distance(s_int(get_time(0)),p_int(get_time(j)));
  for(size_t i=1; i<N; ++i)
  {
    for(size_t j=1; j<N; ++j)
    {
      dtw[ind(i,j)] = linear_distance(s_int(get_time(i)),p_int(get_time(j)))
                                     + min( dtw[ind(i-1,j-1)],
                                            min( dtw[ind(i,j-1)] , dtw[ind(i-1,j)] ) );
    }
  }
  
  double cost = dtw[ind(N-1,N-1)];
  delete dtw;
  return cost;
}

void TrajectoryFeatures::computeAll()
{
  features_.clear();
  
  ROS_INFO("Compute clearance");
  computeClearance();
  ROS_INFO("Compute smoothness");
  computeSmoothness();
  ROS_INFO("Compute length");
  computeLength();
  ROS_INFO("Compute cartesian 1");
  computeCartesianFeatures("r_wrist_roll_link");
  ROS_INFO("Compute cartesian 2");
  computeCartesianFeatures("r_elbow_flex_link");
  ROS_INFO("Compute joint limit distance");
  computeJointLimitDistance();
}
