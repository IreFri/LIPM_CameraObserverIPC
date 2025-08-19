#pragma once
#include <ExternalFootstepPlanner/Plan.h> // XXX for Foot only
#include <ExternalFootstepPlanner/SE2d.h>

namespace mc_plugin
{
namespace ExternalFootstepPlanner
{

/**
 * @brief For now the request is simply the initial position of the feet, and goal position, along with a maximum
 * allowed computation time
 */
struct Request
{
  SE2d start_left_foot, start_right_foot; ///< Starting state (x;y;theta) for the feet. Should match the state in which
                                          ///< the robot is expected to be once the plan will be applied
  SE2d goal_left_foot, goal_right_foot; ///< Goal state for the feet. This is where you want each foot to be at the end
                                        ///< of the footstep plan.
  Foot support_foot =
      Foot::Right; ///< The plan should start using this foot as support (swing foot will be the opposite foot)
  double allowed_time = 0.5; ///< Max allowed time for the planner to compute a plan
};

} // namespace ExternalFootstepPlanner

} // namespace mc_plugin
