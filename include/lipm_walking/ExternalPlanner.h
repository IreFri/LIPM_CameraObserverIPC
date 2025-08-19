#pragma once
#include <mc_control/MCController.h>

#include <ExternalFootstepPlanner/Plan.h>
#include <ExternalFootstepPlanner/Request.h>
#include <lipm_walking/FootstepPlan.h>
#include <lipm_walking/utils/SE2d.h>

namespace mc_control
{
struct MCController;
} // namespace mc_control

namespace lipm_walking
{

/**
 * @brief Handle requesting/receiving a footstep plan from an external planner
 *
 * For now this is intended to be used along with the mc_plugin_footstep_plan plugin that handles the actual
 * communication with the planner, and conversion to a FootstepPlan.
 */
struct ExternalPlanner
{
  using SE2d = lipm_walking::utils::SE2d;
  using Request = mc_plugin::ExternalFootstepPlanner::Request;
  using Foot = mc_plugin::ExternalFootstepPlanner::Foot;

  enum State
  {
    Standing,
    DoubleSupport,
    SingleSupport
  }; ///< State in which the requested plan applies

  ExternalPlanner(mc_control::MCController & ctl);

  /**
   * @brief External planning configuration.
   *
   * @param config Config in the format of the "external" plan section of LIPMWalking configuration
   */
  void configure(const mc_rtc::Configuration & config);

  bool planningRequested() const
  {
    return ctl_.datastore().call<bool>("ExternalFootstepPlanner::PlanningRequested");
  }

  // clang-format off
  /**
   * @brief Request a plan to the plugin
   *
   * The target is set within the ExternalFootstepPlannerPlugin (from GUI or other inputs).
   *
   * @note Since the planner usually takes time to find and send a solution, a plan typically needs to be requested
   *       with a starting configuration corresponding to a future robot state. E.g when requesting a plan during the
   *       SingleSupport phase, you'll want to request a plan with an initial Left/Right foot configuration
   *       corresponding to the (predicted) landing state in the next DoubleSupport phase. That way the planner will
   *       have time to compute the plan while the current SingleSupport phase executes (current step)
   *       and the requested plan should be available at the start of the next DoubleSupport phase
   *
   * @param state Walking state for which the computed plan is intended
   * @param supportFoot Starting support foot (first swing foot will be the opposite foot)
   * @param start_lf Starting configuration that the left foot will have in "state"
   * @param start_rf Starting configuration for the right foot will have in "state"
   * @param allowed_time Time allowed for the planner to compute a solution.
   *  This should be less than the time available between when the request is made and when it is expected to be used.
   *  Example:
   *  - SingleSupport (0.7s), making a request at t=0.2s during the SingleSupport phase
   *    for a plan intended for the next DoubleSupport phase that is:
   *    requestPlan(DoubleSupport,
   *                <support foot for the next single support phase>,
   *                <left foot state in the next double support phase (landing state)>,
   *                <right foot state in the next double support phase (landing state)>,
   *                0.3)
   *  - This means we have 0.5s available to compute until the DoubleSupport phase.
   *  - Accounting for communication time with the planner, allowed_time<=0.4s would be a reasonable choice.
   *    Also note that setting a time as small as possible while ensuring that the planner will find a solution is
   *    preferable as it gives more time-span available to request a new plan.
   */
  // clang-format on
  void requestPlan(const State state,
                   const Foot supportFoot,
                   const utils::SE2d & start_lf,
                   const utils::SE2d & start_rf,
                   double allowed_time);

  void cancelRequest()
  {
    // FIXME should handle cancelling the request on the planner side
    requested_ = false;
  }

  bool planRequested(State state) const noexcept
  {
    return state_ == state && requested_;
  }

  bool hasPlan(State state) const
  {
    return state_ == state && ctl_.datastore().call<bool>("ExternalFootstepPlanner::HasPlan");
  }

  /**
   * @brief Convert plugin's plan to lipm_walking plan format
   *
   * @return lipm_walking::FootstepPlan The plan to be executed
   */
  std::vector<lipm_walking::Contact> plan();

  State state() const
  {
    return state_;
  }

  /**
   * @brief Offset between the left foot and the world target frame, expressed in world target frame
   * @warning It is the user's responsibility to ensure that the left-to-right foot offset results in a collision-free
   * landing stance.
   * @see rightFootLandingOffset
   *
   * @param offset Offset expressed in world target frame
   */
  void leftFootLandingOffset(const SE2d & offset)
  {
    leftFootLandingOffset_ = offset;
  }

  void rightFootLandingOffset(const SE2d & offset)
  {
    rightFootLandingOffset_ = offset;
  }

  void allowedTimeSingleSupport(const double time)
  {
    allowedTimeSingleSupport_ = time;
  }

  double allowedTimeSingleSupport() const noexcept
  {
    return allowedTimeSingleSupport_;
  }

  void allowedTimeStanding(const double time)
  {
    allowedTimeStanding_ = time;
  }

  double allowedTimeStanding() const noexcept
  {
    return allowedTimeStanding_;
  }

  void activate()
  {
    ctl_.datastore().call<void>("ExternalFootstepPlanner::Activate");
  }

  void deactivate()
  {
    ctl_.datastore().call<void>("ExternalFootstepPlanner::Deactivate");
  }

protected:
  /**
   * @brief Request a plan with a target expressed in world frame
   *
   * @see requestPlan
   */
  void requestPlanWorldPositionTarget(const State state,
                                      const Foot supportFoot,
                                      const utils::SE2d & start_lf,
                                      const utils::SE2d & start_rf,
                                      const utils::SE2d & targetWorld,
                                      double allowed_time);

  /**
   * @brief Request a plan in local frame
   *
   * The local frame is defined as the frame halfway between start_lf and start_rf
   *
   * @see requestPlanWorldPositionTarget
   * @see requestPlan
   */
  void requestPlanLocalPositionTarget(const State state,
                                      const Foot supportFoot,
                                      const utils::SE2d & start_lf,
                                      const utils::SE2d & start_rf,
                                      const utils::SE2d & localTarget,
                                      double allowed_time);

protected:
  mc_control::MCController & ctl_;
  State state_ = State::Standing;
  bool requested_ = false;
  double allowedTimeSingleSupport_ = 0.5;
  double allowedTimeStanding_ = 2.0;

  /**
   * @brief Landing offset of the feet w.r.t target frame, expressed in target frame
   * @warning These default values are intended to reduce the risk of the robot stepping on itself. They must be set
   * per-robot to a collision-free stance.
   * @{
   */
  SE2d leftFootLandingOffset_{0., 0.15, 0.};
  SE2d rightFootLandingOffset_{0, -0.15, 0.};
  /** @} */
};

} // namespace lipm_walking
