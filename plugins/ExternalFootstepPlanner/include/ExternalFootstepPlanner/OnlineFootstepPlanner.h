#pragma once

#include <ExternalFootstepPlanner/ExternalFootstepPlanner.h>
#include <ExternalFootstepPlanner/Plan.h>
#include <future>
#include <mutex>
#include <thread>

namespace mc_plugin
{
namespace ExternalFootstepPlanner
{

/**
 * @brief Base interface to interact with external planners
 */
struct OnlineFootstepPlanner : ExternalFootstepPlanner
{
  OnlineFootstepPlanner();
  ~OnlineFootstepPlanner();

  void configure(const mc_rtc::Configuration & config) override;

  /**
   * @brief
   *
   * @param request Requested parameters for the plan (start, finish, time, etc)
   * @return std::future<boost::optional<Plan>> A future plan to be returned once processed
   */
  void requestPlan(const Request & request) override;
  bool hasPlan() const noexcept override;
  Plan popPlan() override;

  std::string name() const override
  {
    return "OnlineFootstepPlanner";
  }

  bool available() const override
  {
    return available_;
  }

  void activate() override;
  void deactivate() override;

protected:
  void rosThread();

protected:
  bool activated_ = false; ///< Whether the plugin is active (GUI displayed, planner connected, etc)
  bool verbose_ = false; ///< Whether to print debug information
  /**
   * Whether to print warning if requests were ignored (e.g a plan was requested while another one is aready being
   * computed)
   *
   * @note It would be better to have a mechanism on the planner's side to cancel ongoing requests and take into account
   * the latest requests immediately. In the meanwhile, this parameter can be set to 'true' to avoid triggering warning
   * messages when requests are being ignored. Typically this happens when changing the target through the GUI markers
   * (which sends many requests in a short time-span).
   */
  bool ignore_skipped_requests_ = false;

  /**
   * ROS-thread and synchronization with the planner
   * @{
   */
  std::thread rosThread_;
  std::atomic<bool> planRequested_{false};
  std::atomic<bool> planReceived_{false};
  std::atomic<bool> run_{true};
  std::atomic<bool> available_{false}; ///< Whether the planner is available (ROS service connected)
  double rate_ = 30;
  mutable std::mutex requestMutex_;
  Request request_;
  mutable std::mutex planMutex_;
  Plan plan_;

  std::future<boost::optional<Plan>> futurePlan_;
  std::string footstep_service_topic_ =
      "/online_footstep_planner/footstep_generation_srv"; ///< ROS service used to request a new plan
  /** @} */
};

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin
