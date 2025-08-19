#include <mc_control/MCController.h>
#include <mc_rtc/gui.h>

#include <boost/optional.hpp>

#include <ExternalFootstepPlanner/Plan.h>
#include <lipm_walking/ExternalPlanner.h>

namespace lipm_walking
{

using SE2d = utils::SE2d;

ExternalPlanner::ExternalPlanner(mc_control::MCController & ctl) : ctl_(ctl) {}

void ExternalPlanner::configure(const mc_rtc::Configuration & config)
{
  auto planningConf = config("allowed_planning_time");
  planningConf("single_support", allowedTimeSingleSupport_);
  planningConf("standing", allowedTimeStanding_);

  if(config.has("leftFootLandingOffset"))
  {
    Eigen::Vector3d offset = config("leftFootLandingOffset");
    leftFootLandingOffset_ = utils::SE2d{offset.x(), offset.y(), offset.z()};
  }
  if(config.has("rightFootLandingOffset"))
  {
    Eigen::Vector3d offset = config("rightFootLandingOffset");
    rightFootLandingOffset_ = utils::SE2d{offset.x(), offset.y(), offset.z()};
  }
}

void ExternalPlanner::requestPlan(const State state,
                                  const Foot supportFoot,
                                  const utils::SE2d & start_lf,
                                  const utils::SE2d & start_rf,
                                  double allowed_time)
{
  bool worldTargetChanged = ctl_.datastore().call<bool>("ExternalFootstepPlanner::WorldPositionTargetChanged");
  bool localVelocityTargetChanged = ctl_.datastore().call<bool>("ExternalFootstepPlanner::LocalPositionTargetChanged");
  if(worldTargetChanged && localVelocityTargetChanged)
  {
    mc_rtc::log::warning(
        "[ExternalPlanner] Both target position (world) and velocity target (world) changed, using target position");
    return;
  }
  if(worldTargetChanged)
  { // world position target
    const auto & target = ctl_.datastore().call<const mc_plugin::ExternalFootstepPlanner::SE2d &>(
        "ExternalFootstepPlanner::WorldPositionTarget");
    const auto goal = utils::SE2d{target.x, target.y, target.theta};
    requestPlanWorldPositionTarget(state, supportFoot, start_lf, start_rf, goal, allowed_time);
  }
  else
  { // local target
    const auto & local = ctl_.datastore().call<const mc_plugin::ExternalFootstepPlanner::SE2d &>(
        "ExternalFootstepPlanner::LocalPositionTarget");
    const auto local_goal = utils::SE2d{local.x, local.y, local.theta};
    requestPlanLocalPositionTarget(state, supportFoot, start_lf, start_rf, local_goal, allowed_time);
  }
}

void ExternalPlanner::requestPlanWorldPositionTarget(const State state,
                                                     const Foot supportFoot,
                                                     const utils::SE2d & start_lf,
                                                     const utils::SE2d & start_rf,
                                                     const utils::SE2d & targetWorld,
                                                     double allowed_time)
{
  Request request;
  request.start_left_foot = {start_lf.x, start_lf.y, start_lf.theta};
  request.start_right_foot = {start_rf.x, start_rf.y, start_rf.theta};

  // Compute goal with nominal foot standing width
  auto lfGoal = utils::SE2d(leftFootLandingOffset_.asPTransform() * targetWorld.asPTransform());
  auto rfGoal = utils::SE2d(rightFootLandingOffset_.asPTransform() * targetWorld.asPTransform());

  request.goal_left_foot = {lfGoal.x, lfGoal.y, lfGoal.theta};
  request.goal_right_foot = {rfGoal.x, rfGoal.y, rfGoal.theta};
  request.support_foot = supportFoot;
  request.allowed_time = allowed_time;

  ctl_.datastore().call<void>("ExternalFootstepPlanner::RequestPlan", static_cast<const Request &>(request));
  state_ = state;
  requested_ = true;
}

void ExternalPlanner::requestPlanLocalPositionTarget(const State state,
                                                     const Foot supportFoot,
                                                     const utils::SE2d & start_lf,
                                                     const utils::SE2d & start_rf,
                                                     const utils::SE2d & local_velocity,
                                                     double allowed_time)
{
  // Convert local velocity to global target
  // What should the choice of local frame be here?
  // For now we assume the support foot as the reference local frame
  auto worldSupportFrame = SE2d{sva::interpolate(start_lf.asPTransform(), start_rf.asPTransform(), 0.5)};
  auto goal = utils::SE2d(local_velocity.asPTransform() * worldSupportFrame.asPTransform());
  requestPlanWorldPositionTarget(state, supportFoot, start_lf, start_rf, goal, allowed_time);
}

std::vector<lipm_walking::Contact> ExternalPlanner::plan()
{
  auto convertPlan = [](const mc_plugin::ExternalFootstepPlanner::Plan & ext_plan)
  {
    std::vector<lipm_walking::Contact> contacts;
    unsigned i = 0;
    for(const auto & ext_contact : ext_plan.contacts)
    {
      auto contact = lipm_walking::Contact{};
      // contact.
      // plan.contacts.push_back()
      contact.surfaceName = (ext_contact.foot == Foot::Right ? "RightFootCenter" : "LeftFootCenter");
      utils::SE2d pose2D = {ext_contact.pose.x, ext_contact.pose.y, ext_contact.pose.theta};
      contact.pose = pose2D.asPTransform();
      contact.id = i++;
      contacts.push_back(contact);
    }
    return contacts;
  };

  requested_ = false;
  auto plan = ctl_.datastore().call<mc_plugin::ExternalFootstepPlanner::Plan>("ExternalFootstepPlanner::PopPlan");
  return convertPlan(plan);
}

} // namespace lipm_walking
