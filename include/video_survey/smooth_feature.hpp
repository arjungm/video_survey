#ifndef CLEAR_FEAT
#define CLEAR_FEAT

#include "video_survey/feature.h"

class SmoothnessFeature : public DoubleFeature
{
  public:
    SmoothnessFeature() : DoubleFeature("clearance") {}
    ~SmoothnessFeature() {}
    double compute(const planning_scene::PlanningScenePtr& ps, const robot_trajectory::RobotTrajectoryPtr& rt)
    {
      double a = rt.getWayPoint(0).distance(rt.getWayPoint(1));
      for (std::size_t k = 2 ; k < rt.getWayPointCount() ; ++k)
      {
        // view the path as a sequence of segments, and look at the triangles it forms:
        //          s1
        //          /\          s4
        //      a  /  \ b       |
        //        /    \        |
        //       /......\_______|
        //     s0    c   s2     s3
        //
        // use Pythagoras generalized theorem to find the cos of the angle between segments a and b
        double b = rt.getWayPoint(k-1).distance(rt.getWayPoint(k));
        double cdist = rt.getWayPoint(k-2).distance(rt.getWayPoint(k));
        double acosValue = (a*a + b*b - cdist*cdist) / (2.0*a*b);
        if (acosValue > -1.0 && acosValue < 1.0)
        {
          // the smoothness is actually the outside angle of the one we compute
          double angle = (boost::math::constants::pi<double>() - acos(acosValue));

          // and we normalize by the length of the segments
          double u = 2.0 * angle; /// (a + b);
          smoothness += u * u;
        }
        a = b;
      }
      smoothness /= (double)p.getWayPointCount();

      return smoothness;
    }
};

#endif

