#ifndef CLEAR_FEAT
#define CLEAR_FEAT

#include "video_survey/feature.h"

class ClearanceFeature : public DoubleFeature
{
  public:
    ClearanceFeature() : DoubleFeature("clearance") {}
    ~ClearanceFeature() {}
    double compute(const planning_scene::PlanningScenePtr& ps, const robot_trajectory::RobotTrajectoryPtr& rt)
    {
      double clearance;
      collision_detection::CollisionRequest req;
      for (std::size_t k = 0 ; k < rt->getWayPointCount() ; ++k)
      {
        collision_detection::CollisionResult res;
        ps->checkCollisionUnpadded(req, res, rt->getWayPoint(k));

        double d = ps->distanceToCollisionUnpadded(rt->getWayPoint(k));
        if (d > 0.0) // in case of collision, distance is negative
          clearance += d;
      }
      clearance /= (double)rt->getWayPointCount();
      return clearance;
    }
};

#endif
