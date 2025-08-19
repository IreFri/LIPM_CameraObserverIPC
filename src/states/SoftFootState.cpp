#include <mc_rtc/gui/Form.h>

#include <lipm_walking/SwingTraj.h>

#include "SoftFootState.h"

#include <variable_stiffness/connectionFile.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <numeric>
#include <thread>

#include <mc_mujoco/devices/RangeSensor.h>
#include <mc_tasks/CoMTask.h>

#include "libqhullcpp/Qhull.h"
#include "libqhullcpp/QhullPoint.h"
#include "libqhullcpp/QhullVertex.h"
#include "libqhullcpp/QhullVertexSet.h"

#include <boost/numeric/odeint.hpp>

namespace
{

// https://stackoverflow.com/questions/27028226/python-linspace-in-c
template<typename T>
std::vector<T> linspace(T start_in, T end_in, int num_in)
{
  std::vector<T> linspaced;

  double start = static_cast<double>(start_in);
  double end = static_cast<double>(end_in);
  double num = static_cast<double>(num_in);

  if(num == 0)
  {
    return linspaced;
  }
  if(num == 1)
  {
    linspaced.push_back(start);
    return linspaced;
  }

  double delta = (end - start) / (num - 1);

  for(int i = 0; i < num - 1; ++i)
  {
    linspaced.push_back(start + delta * i);
  }
  linspaced.push_back(end); // I want to ensure that start and end are exactly the same as the input
  return linspaced;
}

} // namespace

namespace lipm_walking::states
{

void SoftFootState::start()
{
  auto & ctl = controller();

  ctl.datastore().make_call("SoftFootState::GetState", [this]() -> std::string { return current_state_; });

  with_variable_stiffness_ = config_("with_variable_stiffness", false);
  with_ankle_rotation_ = config_("with_ankle_rotation", false);
  with_foot_adjustment_ = config_("with_foot_adjustment", false);

  nr_phalanxes_ = config_("nr_phalanx", 10);
  phalanx_length_ = foot_length_ / static_cast<double>(nr_phalanxes_);
  use_camera_sensor_ = config_("use_camera_sensor", false);

  range_sensor_names_[Foot::Left] = config_("range_sensors")("left_foot");
  for(const auto & range_sensor_name : range_sensor_names_[Foot::Left])
  {
    range_sensor_data_[Foot::Left][range_sensor_name] = 0.;
  }

  range_sensor_names_[Foot::Right] = config_("range_sensors")("right_foot");
  for(const auto & range_sensor_name : range_sensor_names_[Foot::Right])
  {
    range_sensor_data_[Foot::Right][range_sensor_name] = 0.;
  }

  if(range_sensor_names_[Foot::Left].empty())
  {
    mc_rtc::log::error("[SoftFootState] range_sensor_names_[Foot::Left] is empty");
  }

  if(range_sensor_names_[Foot::Right].empty())
  {
    mc_rtc::log::error("[SoftFootState] range_sensor_names_[Foot::Right] is empty");
  }

  //
  past_foot_pose_[Foot::Right] = Circular_Buffer<sva::PTransformd>(0.5 / ctl.solver().dt()); // 0.5s
  past_foot_pose_[Foot::Left] = Circular_Buffer<sva::PTransformd>(0.5 / ctl.solver().dt()); // 0.5s
  // For logging
  past_foot_pose_control_[Foot::Right] = Circular_Buffer<sva::PTransformd>(0.5 / ctl.solver().dt()); // 0.5s
  past_foot_pose_control_[Foot::Left] = Circular_Buffer<sva::PTransformd>(0.5 / ctl.solver().dt()); // 0.5s

  // Display configuration
  mc_rtc::log::info("[SoftFootState] use_camera_sensor is set to {}", use_camera_sensor_);
  mc_rtc::log::info("[SoftFootState] nr_phalanxes is set to {}", nr_phalanxes_);
  mc_rtc::log::info("[SoftFootState] phalanx_length is set to {}", phalanx_length_);
  mc_rtc::log::info("[SoftFootState] with_variable_stiffness is set to {}", with_variable_stiffness_);
  mc_rtc::log::info("[SoftFootState] with_ankle_rotation is set to {}", with_ankle_rotation_);
  mc_rtc::log::info("[SoftFootState] with_foot_adjustment is set to {}", with_foot_adjustment_);
  mc_rtc::log::info("[SoftFootState] range_sensors/left_foot is set to {}",
                    fmt::join(range_sensor_names_[Foot::Left], ", "));
  mc_rtc::log::info("[SoftFootState] range_sensors/right_foot is set to {}",
                    fmt::join(range_sensor_names_[Foot::Right], ", "));

  // Create client
  client_ = mc_rtc::ROSBridge::get_node_handle()->serviceClient<variable_stiffness::connectionFile>("connectionFile");

  if(nr_remaining_footstep_ == 0.)
  {
    nr_footstep_ = 0.;
  }

  // Initialize map
  foot_data_[Foot::Left] = FootData{};
  foot_data_[Foot::Right] = FootData{};

  ctl.gui()->addElement({"SoftFoot"}, mc_rtc::gui::Label("cost", [this]() { return this->cost_; }));
  ctl.logger().addLogEntry("cost", [this]() { return cost_; });

  ctl.gui()->addElement({"SoftFoot"},
                        mc_rtc::gui::Label("nr_remaining_footstep", [this]() { return this->nr_remaining_footstep_; }));
  ctl.logger().addLogEntry("nr_remaining_footstep", [this]() { return nr_remaining_footstep_; });

  ctl.gui()->addElement({"SoftFoot"}, mc_rtc::gui::Label("nr_footstep", [this]() { return this->nr_footstep_; }));
  ctl.logger().addLogEntry("nr_footstep", [this]() { return nr_footstep_; });

  ctl.gui()->addElement({"SoftFoot"},
                        mc_rtc::gui::Label("PhalangesStiffness", [this]() { const std::lock_guard<std::mutex> lock(variable_stiffness_mutex_); return this->PhalangesStiffness_; }));
  ctl.logger().addLogEntry("PhalangesStiffness", [this]() { const std::lock_guard<std::mutex> lock(variable_stiffness_mutex_); return PhalangesStiffness_; });

  ctl.gui()->addElement({"SoftFoot"},
                        mc_rtc::gui::Label("Air_Pressure", [this]() { const std::lock_guard<std::mutex> lock(variable_stiffness_mutex_); return this->pressure_; }));
  ctl.logger().addLogEntry("Air_Pressure", [this]() { const std::lock_guard<std::mutex> lock(variable_stiffness_mutex_); return pressure_; });

  ctl.gui()->addElement({"SoftFoot"},
                        mc_rtc::gui::Label("Desired_Pressure", [this]() { return this->D_; }));
  ctl.logger().addLogEntry("Desired_Pressure", [this]() { return D_; });

  ctl.gui()->addElement({"SoftFoot"},
                        mc_rtc::gui::Label("WhichFoot", [this]() { return this->WhichFoot_; }));
  ctl.logger().addLogEntry("WhichFoot", [this]() { return WhichFoot_; });

  ctl.gui()->addElement({"SoftFoot"},
                        mc_rtc::gui::Label("Right valvesStatus", [this]() { return this->foot_data_[Foot::Right].valves_status; }));

  ctl.gui()->addElement({"SoftFoot"},
                        mc_rtc::gui::Label("Left valvesStatus", [this]() { return this->foot_data_[Foot::Left].valves_status; }));

  ctl.gui()->addElement({"SoftFoot"},
                        mc_rtc::gui::Label("Monitor_Left", [this]() { return this->ML_; }));
  ctl.logger().addLogEntry("Monitor_Left", [this]() { return ML_; });

  ctl.gui()->addElement({"SoftFoot"},
                        mc_rtc::gui::Label("Monitor_Right", [this]() { return this->MR_; }));
  ctl.logger().addLogEntry("Monitor_Right", [this]() { return MR_; });

  for(const auto & range_sensor_name : range_sensor_names_[Foot::Right])
  {
    ctl.gui()->addElement(
        {"SoftFoot"},
        mc_rtc::gui::Label(range_sensor_name,
                           [this, range_sensor_name]()
                           {
                             const std::lock_guard<std::mutex> lock(range_sensor_mutex_);
                             return controller().robot().device<mc_mujoco::RangeSensor>(range_sensor_name).data();
                           }));
  }

  for(const auto & range_sensor_name : range_sensor_names_[Foot::Left])
  {
    ctl.gui()->addElement(
        {"SoftFoot"},
        mc_rtc::gui::Label(range_sensor_name,
                           [this, range_sensor_name]()
                           {
                             const std::lock_guard<std::mutex> lock(range_sensor_mutex_);
                             return controller().robot().device<mc_mujoco::RangeSensor>(range_sensor_name).data();
                           }));
  }

  ctl.gui()->addElement(
      {"SoftFoot", "Config"},
      mc_rtc::gui::Checkbox(
          "With variable stiffness", [this]() { return with_variable_stiffness_; },
          [this]() { with_variable_stiffness_ = !with_variable_stiffness_; }),
      mc_rtc::gui::Checkbox(
          "With ankle rotation", [this]() { return with_ankle_rotation_; },
          [this]() { with_ankle_rotation_ = !with_ankle_rotation_; }),
      mc_rtc::gui::Checkbox(
          "With position adjustment", [this]() { return with_foot_adjustment_; },
          [this]() { with_foot_adjustment_ = !with_foot_adjustment_; }),
      mc_rtc::gui::Checkbox(
          "Debug ouptut in terminal", [this]() { return debug_output_; }, [this]() { debug_output_ = !debug_output_; }),
      mc_rtc::gui::Checkbox(
          "Use real robot for ground estimation", [this]() { return use_real_robot_for_estimation_; },
          [this]() { use_real_robot_for_estimation_ = !use_real_robot_for_estimation_; }),
      mc_rtc::gui::NumberInput(
          "Add delta delay to estimation [s]", [this]() { return delta_delay_of_estimation_; },
          [this](double v) { delta_delay_of_estimation_ = v; }),
      mc_rtc::gui::NumberInput(
          "Use fixed delay to estimation if not 0. [s]", [this]() { return fixed_delay_of_estimation_; },
          [this](double v) { fixed_delay_of_estimation_ = v; }));

  ctl.gui()->addXYPlot(
      "SoftFoot",
      mc_rtc::gui::plot::Polygon("Ground",
                                 [&ctl, this]()
                                 {
                                   // Construct Ground Polygon
                                   std::vector<std::array<double, 2>> points;
                                   // Get current moving foot
                                   Foot current_moving_foot = getCurrentMovingFoot(ctl);
                                   // Get the segment
                                   const auto & raw_segment = ground_segment_[current_moving_foot].raw;
                                   // Save the selected segment in raw data of ground segment structure
                                   std::transform(raw_segment.begin(), raw_segment.end(), std::back_inserter(points),
                                                  [](const Eigen::Vector3d & v) {
                                                    return std::array<double, 2>{v.x(), v.z()};
                                                  });
                                   // Create polygon
                                   auto polygon =
                                       mc_rtc::gui::plot::PolygonDescription(points, mc_rtc::gui::Color::Magenta);
                                   polygon.closed(false);
                                   return polygon;
                                 }),
      mc_rtc::gui::plot::Polygon("Filtered_Ground",
                                 [&ctl, this]()
                                 {
                                   // Construct Ground Polygon
                                   std::vector<std::array<double, 2>> points;
                                   // Get current moving foot
                                   Foot current_moving_foot = getCurrentMovingFoot(ctl);
                                   // Get the segment
                                   const auto & filtered_segment = ground_segment_[current_moving_foot].filtered;
                                   // Save the selected segment in raw data of ground segment structure
                                   std::transform(filtered_segment.begin(), filtered_segment.end(),
                                                  std::back_inserter(points),
                                                  [](const Eigen::Vector3d & v) {
                                                    return std::array<double, 2>{v.x(), v.z()};
                                                  });
                                   // Create polygon
                                   auto polygon =
                                       mc_rtc::gui::plot::PolygonDescription(points, mc_rtc::gui::Color::Blue);
                                   polygon.closed(false);
                                   return polygon;
                                 }),
      mc_rtc::gui::plot::Polygon("Convex",
                                 [&ctl, this]()
                                 {
                                   // Construct Ground Polygon
                                   std::vector<std::array<double, 2>> points;
                                   // Get current moving foot
                                   Foot current_moving_foot = getCurrentMovingFoot(ctl);
                                   // Get the segment
                                   const auto & convex_segment = ground_segment_[current_moving_foot].convex;
                                   // Save the selected segment in convex data of ground segment structure
                                   std::transform(convex_segment.begin(), convex_segment.end(),
                                                  std::back_inserter(points),
                                                  [](const Eigen::Vector3d & v) {
                                                    return std::array<double, 2>{v.x(), v.z()};
                                                  });
                                   // Create polygon
                                   auto polygon =
                                       mc_rtc::gui::plot::PolygonDescription(points, mc_rtc::gui::Color::Green);
                                   polygon.closed(false);
                                   return polygon;
                                 }),
      mc_rtc::gui::plot::Polygon("Phalanxes",
                                 [&ctl, this]()
                                 {
                                   // Get current moving foot
                                   Foot current_moving_foot = getCurrentMovingFoot(ctl);
                                   // Create polygon
                                   auto polygon = mc_rtc::gui::plot::PolygonDescription(
                                       foot_data_[current_moving_foot].phalanxes, mc_rtc::gui::Color::Red);
                                   polygon.closed(false);
                                   return polygon;
                                 }));

  ctl.gui()->addXYPlot("GroundEstimation",
                       mc_rtc::gui::plot::XY(
                           "LeftFoot", [this]() { return foot_data_[Foot::Left].last_ground.x(); },
                           [this]() { return foot_data_[Foot::Left].last_ground.z(); }, mc_rtc::gui::Color::Red),
                       mc_rtc::gui::plot::XY(
                           "RightFoot", [this]() { return foot_data_[Foot::Right].last_ground.x(); },
                           [this]() { return foot_data_[Foot::Right].last_ground.z(); }, mc_rtc::gui::Color::Blue));

  ctl.gui()->addPlot(
      "Ground",
      mc_rtc::gui::plot::X("t",
                           [this, &ctl]()
                           {
                             static double t = 0.;
                             return t += ctl.solver().dt();
                           }),
      mc_rtc::gui::plot::Y(
          "LeftGround", [this]() { return foot_data_[Foot::Left].last_ground.z(); }, mc_rtc::gui::Color::Red),
      mc_rtc::gui::plot::Y(
          "RightGround", [this]() { return foot_data_[Foot::Right].last_ground.z(); }, mc_rtc::gui::Color::Blue));

  ctl.gui()->addPlot(
      "Stiffness",
      mc_rtc::gui::plot::X("t",
                           [this, &ctl]()
                           {
                             static double t = 0.;
                             return t += ctl.solver().dt();
                           }),
      mc_rtc::gui::plot::Y(
          "Left Foot", [this]() { const std::lock_guard<std::mutex> lock(variable_stiffness_mutex_); return this->PhalangesStiffness_; }, mc_rtc::gui::Color::Blue));

  ctl.gui()->addPlot(
      "Feet Stiffness",
      mc_rtc::gui::plot::X("t",
                           [this, &ctl]()
                           {
                             static double t = 0.;
                             return t += ctl.solver().dt();
                           }),
      mc_rtc::gui::plot::Y(
          "Left Foot", [this]() { const std::lock_guard<std::mutex> lock(variable_stiffness_mutex_); return foot_data_[Foot::Left].k; }, mc_rtc::gui::Color::Blue), 
      mc_rtc::gui::plot::Y(
          "Right Foot", [this]() { const std::lock_guard<std::mutex> lock(variable_stiffness_mutex_); return foot_data_[Foot::Right].k; }, mc_rtc::gui::Color::Red));

  ctl.gui()->addPlot(
      "Feet Pressure",
      mc_rtc::gui::plot::X("t",
                           [this, &ctl]()
                           {
                             static double t = 0.;
                             return t += ctl.solver().dt();
                           }),
      mc_rtc::gui::plot::Y(
          "Left Foot", [this]() { const std::lock_guard<std::mutex> lock(variable_stiffness_mutex_); return foot_data_[Foot::Left].pressure; }, mc_rtc::gui::Color::Blue), 
      mc_rtc::gui::plot::Y(
          "Right Foot", [this]() { const std::lock_guard<std::mutex> lock(variable_stiffness_mutex_); return foot_data_[Foot::Right].pressure; }, mc_rtc::gui::Color::Red));

  ctl.gui()->addPlot(
      "Pressure",
      mc_rtc::gui::plot::X("t",
                           [this, &ctl]()
                           {
                             static double t = 0.;
                             return t += ctl.solver().dt();
                           }),
      mc_rtc::gui::plot::Y(
          "Pressure", [this]() { const std::lock_guard<std::mutex> lock(variable_stiffness_mutex_); return this->pressure_; }, mc_rtc::gui::Color::Blue));

  ctl.gui()->addPlot(
      "WhichFoot",
      mc_rtc::gui::plot::X("t",
                           [this, &ctl]()
                           {
                             static double t = 0.;
                             return t += ctl.solver().dt();
                           }),
      mc_rtc::gui::plot::Y(
          "WhichFoot", [this]() { return this->WhichFoot_; }, mc_rtc::gui::Color::Blue));

  ctl.gui()->addPlot(
      "valvesStatus",
      mc_rtc::gui::plot::X("t",
                           [this, &ctl]()
                           {
                             static double t = 0.;
                             return t += ctl.solver().dt();
                           }),
      mc_rtc::gui::plot::Y(
          "Left valvesStatus", [this]() { return this->foot_data_[Foot::Left].valves_status; }, mc_rtc::gui::Color::Blue),
      mc_rtc::gui::plot::Y(
          "Right valvesStatus", [this]() { return this->foot_data_[Foot::Right].valves_status; }, mc_rtc::gui::Color::Red));

  ctl.gui()->addPlot(
      "Monitor_Left",
      mc_rtc::gui::plot::X("t",
                           [this, &ctl]()
                           {
                             static double t = 0.;
                             return t += ctl.solver().dt();
                           }),
      mc_rtc::gui::plot::Y(
          "Monitor_Left", [this]() { return this->ML_; }, mc_rtc::gui::Color::Blue));

  ctl.gui()->addPlot(
      "Monitor_Right",
      mc_rtc::gui::plot::X("t",
                           [this, &ctl]()
                           {
                             static double t = 0.;
                             return t += ctl.solver().dt();
                           }),
      mc_rtc::gui::plot::Y(
          "Monitor_Right", [this]() { return this->MR_; }, mc_rtc::gui::Color::Blue));

  ctl.gui()->addPlot(
      "Desired_Pressure",
      mc_rtc::gui::plot::X("t",
                           [this, &ctl]()
                           {
                             static double t = 0.;
                             return t += ctl.solver().dt();
                           }),
      mc_rtc::gui::plot::Y(
          "Desired_Pressure", [this]() { return this->D_; }, mc_rtc::gui::Color::Blue));






  //
  ctl.logger().addLogEntry("MyMeasures_continuous_ground_right",
                           [this]() { return foot_data_[Foot::Right].last_ground; });
  ctl.logger().addLogEntry("MyMeasures_continuous_ground_left",
                           [this]() { return foot_data_[Foot::Left].last_ground; });

  // Subscriber for the range sensors
  right_foot_range_sensor_sub_ = mc_rtc::ROSBridge::get_node_handle()->subscribe(
      "right_range_sensor/distance", 1, &SoftFootState::rightFRSCallback, this);
  left_foot_range_sensor_sub_ = mc_rtc::ROSBridge::get_node_handle()->subscribe("left_range_sensor/distance", 1,
                                                                                &SoftFootState::leftFRSCallback, this);
  air_pressure_sub_ = mc_rtc::ROSBridge::get_node_handle()->subscribe("air_pressure", 1, &SoftFootState::airPressureCallback, this);
}

void SoftFootState::runState()
{
  ros::spinOnce();
  variable_stiffness::connectionFile srv;

  // Cast ctl to BaselineWalkingController
  auto & ctl = controller();
  time_ += ctl.solver().dt();

  // Compute cost
  // calculateCost(ctl);

  // Get the current moving foot
  Foot current_moving_foot = getCurrentMovingFoot(ctl);

  // Estimate ground from sensors
  {
    for(const auto & foot : {Foot::Right, Foot::Left})
    {
      auto start = std::chrono::high_resolution_clock::now();
      estimateGround(ctl, foot);
      auto stop = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration<double, std::milli>(stop - start);
    }
    if(debug_output_)
    {
      // It is spamming to much the terminal
      // mc_rtc::log::warning("[SoftFootState] estimateGround(ctl, current_moving_foot) took {:.4f}ms",
      // duration.count());
    }
  }

  // Check if we are in single support or not
  if(ctl.stabilizer_->inDoubleSupport())
  {
    current_state_ = "DoubleSupport";
    foot_data_[Foot::Right].need_reset = true;
    foot_data_[Foot::Left].need_reset = true;
    return;
  }
  else if(current_moving_foot == Foot::Left)
  {
    current_state_ = "Swing";
  }
  else if(current_moving_foot == Foot::Right)
  {
    current_state_ = "Swing";
  }

  // Scope to handle reset/GUI
  {
    const Foot foot = current_moving_foot;
    const std::string name = foot == Foot::Left ? "left" : "right";
    if(foot_data_[current_moving_foot].need_reset)
    {
      ++nr_footstep_;
      {
        auto start = std::chrono::high_resolution_clock::now();
        reset(ctl, current_moving_foot);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration<double, std::milli>(stop - start);
        mc_rtc::log::warning("[SoftFootState] reset(ctl, current_moving_foot) took {:.4f}ms", duration.count());
      }
    }
    else if(!ctl.gui()->hasElement({"SoftFoot"}, name + "_point_ground"))
    {
      // Handle logger and gui
      ctl.logger().addLogEntry("MyMeasures_" + name + "_ground",
                               [this, foot]() { return foot_data_[foot].last_ground; });
      ctl.logger().addLogEntry("MyMeasures_" + name + "_ground_control",
                               [this, foot]() { return foot_data_[foot].last_ground_control; });
      ctl.logger().addLogEntry("MyMeasures_" + name + "_ground_Identity",
                               [this, foot]() { return foot_data_[foot].last_ground_Identity; });
      ctl.logger().addLogEntry("MyMeasures_" + name + "_back_foot_x",
                               [this, foot]() {
                                 return controller().targetContact().pose.translation().x()
                                        + landing_to_foot_middle_offset_ - foot_length_ * 0.5;
                               });
      ctl.logger().addLogEntry("MyMeasures_" + name + "_front_foot_x",
                               [this, foot]() {
                                 return controller().targetContact().pose.translation().x()
                                        + landing_to_foot_middle_offset_ + foot_length_ * 0.5;
                               });
      ctl.logger().addLogEntry("MyMeasures_" + name + "_back_foot_z",
                               [this, foot]() { return controller().targetContact().pose.translation().z(); });
      ctl.logger().addLogEntry("MyMeasures_" + name + "_front_foot_z",
                               [this, foot]() { return controller().targetContact().pose.translation().z(); });
      ctl.logger().addLogEntry("MyMeasures_" + name + "_filtered_x",
                               [this, foot]()
                               {
                                 std::vector<double> d;
                                 std::transform(ground_segment_[foot].filtered.begin(),
                                                ground_segment_[foot].filtered.end(), std::back_inserter(d),
                                                [](const Eigen::Vector3d & v) { return v.x(); });
                                 return d;
                               });
      ctl.logger().addLogEntry("MyMeasures_" + name + "_filtered_y",
                               [this, foot]()
                               {
                                 std::vector<double> d;
                                 std::transform(ground_segment_[foot].filtered.begin(),
                                                ground_segment_[foot].filtered.end(), std::back_inserter(d),
                                                [](const Eigen::Vector3d & v) { return v.z(); });
                                 return d;
                               });
      ctl.logger().addLogEntry("MyMeasures_" + name + "_convex_x",
                               [this, foot]()
                               {
                                 std::vector<double> d;
                                 std::transform(ground_segment_[foot].convex.begin(),
                                                ground_segment_[foot].convex.end(), std::back_inserter(d),
                                                [](const Eigen::Vector3d & v) { return v.x(); });
                                 return d;
                               });
      ctl.logger().addLogEntry("MyMeasures_" + name + "_convex_y",
                               [this, foot]()
                               {
                                 std::vector<double> d;
                                 std::transform(ground_segment_[foot].convex.begin(),
                                                ground_segment_[foot].convex.end(), std::back_inserter(d),
                                                [](const Eigen::Vector3d & v) { return v.z(); });
                                 return d;
                               });
      for(const auto & range_sensor_name : range_sensor_names_[foot])
      {
        ctl.logger().addLogEntry("MyMeasures_" + name + "_" + range_sensor_name + "_range",
                                 [this, foot, range_sensor_name]()
                                 { return range_sensor_data_[foot][range_sensor_name]; });
      }
      ctl.logger().addLogEntry("MyMeasures_" + name + "_k", [this, foot]() { const std::lock_guard<std::mutex> lock(variable_stiffness_mutex_); return foot_data_[foot].k; });
      ctl.logger().addLogEntry("MyMeasures_" + name + "_angle", [this, foot]() { return foot_data_[foot].angle; });
      ctl.logger().addLogEntry("MyMeasures_" + name + "_z_offset",
                               [this, foot]() { return foot_data_[foot].position_offset_z; });
      ctl.logger().addLogEntry("MyMeasures_" + name + "_min_max_phalanxes_angle",
                               [this, foot]() { return foot_data_[foot].min_max_phalanxes_angle; });
      ctl.logger().addLogEntry("MyMeasures_" + name + "_position_offset",
                               [this, foot]() { return foot_data_[foot].position_offset_x; });
      ctl.logger().addLogEntry("MyMeasures_" + name + "_valves_status",
                               [this, foot]() { const std::lock_guard<std::mutex> lock(variable_stiffness_mutex_); return foot_data_[foot].valves_status; });
      ctl.logger().addLogEntry("MyMeasures_" + name + "_phalanxes_x",
                               [this, foot]()
                               {
                                 std::vector<double> d;
                                 std::transform(foot_data_[foot].phalanxes.begin(), foot_data_[foot].phalanxes.end(),
                                                std::back_inserter(d),
                                                [](const std::array<double, 2> & v) { return v[0]; });
                                 return d;
                               });
      ctl.logger().addLogEntry("MyMeasures_" + name + "_phalanxes_y",
                               [this, foot]()
                               {
                                 std::vector<double> d;
                                 std::transform(foot_data_[foot].phalanxes.begin(), foot_data_[foot].phalanxes.end(),
                                                std::back_inserter(d),
                                                [](const std::array<double, 2> & v) { return v[1]; });
                                 return d;
                               });

      ctl.gui()->addElement({"SoftFoot"},
                            mc_rtc::gui::Point3D(name + "_point_ground", {mc_rtc::gui::Color::Green},
                                                 [this, foot]() { return foot_data_[foot].last_ground; }),
                            mc_rtc::gui::Trajectory(name + "_ground", {mc_rtc::gui::Color::Green},
                                                    [this, foot]() { return foot_data_[foot].ground; }));

      ctl.gui()->addElement({"SoftFoot"},
                            mc_rtc::gui::Point3D(name + "_front_foot", {mc_rtc::gui::Color::Red},
                                                 [this, foot]()
                                                 {
                                                   const auto & X_0_p =
                                                       controller().robot().surfacePose(surface_name_[foot]);
                                                   auto p = X_0_p.translation();
                                                   p.x() += landing_to_foot_middle_offset_ + foot_length_ * 0.5;
                                                   return p;
                                                 }),
                            mc_rtc::gui::Point3D(name + "_back_foot", {mc_rtc::gui::Color::Red},
                                                 [this, foot]()
                                                 {
                                                   const auto & X_0_p =
                                                       controller().robot().surfacePose(surface_name_[foot]);
                                                   auto p = X_0_p.translation();
                                                   p.x() += landing_to_foot_middle_offset_ - foot_length_ * 0.5;
                                                   return p;
                                                 }));
    }
  }

  // Returns true if single support has elpased \p ratio of its time
  auto checkSingleSupportTime = [&](double ratio)
  {
    return ctl.swingTraj->t_
           > ctl.swingTraj->startTime_ + (ctl.swingTraj->endTime_ - ctl.swingTraj->startTime_) * ratio;
  };

  auto belowSingleSupportTime = [&](double ratio)
  {
    return ctl.swingTraj->t_
           < ctl.swingTraj->startTime_ + (ctl.swingTraj->endTime_ - ctl.swingTraj->startTime_) * ratio;
  };

  // Open the valves
  if(checkSingleSupportTime(0.05) && belowSingleSupportTime(0.75))
  { 
    // std::cout << "CHECK IF GOING " << std::endl;
    if(with_variable_stiffness_)
    {
      updateVariableStiffness(ctl, current_moving_foot, true);
    }
  }

  // Close the valves
  if(checkSingleSupportTime(0.8))
  {
    // call the server and update the variable stiffness
    if(with_variable_stiffness_)
    {
      updateVariableStiffness(ctl, current_moving_foot, false);
    }
  }

  // Check foot position with respect to desired landing pose
  const auto & ground = foot_data_[current_moving_foot].ground;
  if(ground.empty())
  {
    return;
  }

  sva::PTransformd X_0_landing = ctl.targetContact().pose;
  // If we saw more than the landing pose + half of the foot, we have enough data to perform all the computations
  const bool enough_ground_in_front = ground.back().x() >= X_0_landing.translation().x()
                                                               + landing_to_foot_middle_offset_ + foot_length_ * 0.55
                                                               + extra_to_compute_best_position_;
  // const bool enough_ground_in_back = true;
  const bool enough_ground_in_back = ground.front().x() <= X_0_landing.translation().x()
                                                               + landing_to_foot_middle_offset_ - foot_length_ * 0.55
                                                               - extra_to_compute_best_position_;

  // mc_rtc::log::info("ground.back().x() {} ground.front().x() {} enough_ground_in_front {} enough_ground_in_back {} checkSingleSupportTime(0.5) {}",
  //   ground.back().x(), ground.front().x(), enough_ground_in_front, enough_ground_in_back, checkSingleSupportTime(0.5));

  if(!foot_data_[current_moving_foot].computation_done && enough_ground_in_front && enough_ground_in_back
     && checkSingleSupportTime(0.5))
  {
    mc_rtc::log::success("Accumulated enough data for {}",
                         current_moving_foot == Foot::Right ? "right foot" : "left foot");
    // We need to do these steps only one time
    foot_data_[current_moving_foot].computation_done = true;
    // Extract ground segment from ground data
    {
      auto start = std::chrono::high_resolution_clock::now();
      extractGroundSegment(ctl, current_moving_foot, X_0_landing.translation());
      auto stop = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration<double, std::milli>(stop - start);
      mc_rtc::log::info(
          "[SoftFootState] extractGroundSegment(ctl, current_moving_foot, X_0_landing.translation()) took {:.4f}ms",
          duration.count());
    }
    // Continue only if we have enough data such as the foot is smaller than the estimated ground
    if(!ground_segment_[current_moving_foot].raw.empty())
    {
      if(ground_segment_[current_moving_foot].raw.back().x() - ground_segment_[current_moving_foot].raw.front().x()
             > 0.25 * foot_length_
         || checkSingleSupportTime(0.75))
      {
        // Compute the altitude profile
        {
          auto start = std::chrono::high_resolution_clock::now();
          extractAltitudeProfileFromGroundSegment(current_moving_foot);
          auto stop = std::chrono::high_resolution_clock::now();
          auto duration = std::chrono::duration<double, std::milli>(stop - start);
          mc_rtc::log::info(
              "[SoftFootState] extractAltitudeProfileFromGroundSegment(current_moving_foot) took {:.4f}ms",
              duration.count());
        }

        // Compute convex hull of the segment -> right now it does not work
        {
          auto start = std::chrono::high_resolution_clock::now();
          computeSegmentConvexHull(ctl, current_moving_foot);
          auto stop = std::chrono::high_resolution_clock::now();
          auto duration = std::chrono::duration<double, std::milli>(stop - start);
          mc_rtc::log::info("[SoftFootState] computeSegmentConvexHull(ctl, current_moving_foot) took {:.4f}ms",
                            duration.count());
        }
        // Find best landing position
        if(with_foot_adjustment_)
        {
          // Compute min/max phalanxes angle depending of stiffness
          {
            auto start = std::chrono::high_resolution_clock::now();
            computeMinMaxAngle(current_moving_foot);
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration<double, std::milli>(stop - start);
            mc_rtc::log::info("[SoftFootState] computeMinMaxAngle(current_moving_foot) took {:.4f}ms",
                              duration.count());
          }
          // Compute the landing position
          {
            auto start = std::chrono::high_resolution_clock::now();
            computeFootLandingPosition(current_moving_foot, X_0_landing.translation());
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration<double, std::milli>(stop - start);
            mc_rtc::log::info("[SoftFootState] computeFootLandingPosition(current_moving_foot, "
                              "X_0_landing.translation()) took {:.4f}ms",
                              duration.count());
          }
          // Update landing pose
          X_0_landing = sva::PTransformd(Eigen::Vector3d(foot_data_[current_moving_foot].position_offset_x, 0., 0.))
                        * X_0_landing;
        }
        if(with_ankle_rotation_)
        {
          // Now with the convex hull we can compute the angle
          {
            auto start = std::chrono::high_resolution_clock::now();
            computeFootLandingAngle(current_moving_foot, X_0_landing.translation());
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration<double, std::milli>(stop - start);
            mc_rtc::log::info(
                "[SoftFootState] computeFootLandingAngle(current_moving_foot, X_0_landing.translation()) took {:.4f}ms",
                duration.count());
          }
        }
        // Update targeted pose
        updateFootSwingPose(ctl, current_moving_foot, X_0_landing);
      }
      else
      {
        mc_rtc::log::warning("[SoftFootState] Too small {} > {}",
                             ground_segment_[current_moving_foot].raw.back().x()
                                 - ground_segment_[current_moving_foot].raw.front().x(),
                             foot_length_);
      }
    }
  }
  else if(!foot_data_[current_moving_foot].computation_done)
  {
    // mc_rtc::log::warning("[SoftFootState] Accumulating data -> ground.back().x() {} {}", ground.back().x(),
    // X_0_landing.translation().x() + foot_length_ * 0.5 + extra_to_compute_best_position_);
  }
}

bool SoftFootState::checkTransitions()
{
  output("OK");
  return false;
}

void SoftFootState::teardown()
{
  // Clean up GUI
  auto & ctl = controller();

  ctl.gui()->removeCategory({"SoftFoot"});

  ctl.logger().removeLogEntry("cost");
  ctl.logger().removeLogEntry("PhalangesStiffness");
  ctl.logger().removeLogEntry("Air_Pressure");
  ctl.logger().removeLogEntry("WhichFoot");
}

void SoftFootState::calculateCost(mc_control::fsm::Controller & ctl)
{
  // Cast ctl to BaselineWalkingController
  auto & ctrl = controller();

  double zmp = 0.0;
  // ZMP Error
  {
    // Get ZMP from Centroidal Manager
    const Eigen::Vector3d & zmp_ref = ctrl.pendulum_.zmp();
    const Eigen::Vector3d & zmp_mes = ctrl.stabilizer_->measuredZMP();
    const double zmp_error = (zmp_ref - zmp_mes).norm();

    zmp_.push_back(zmp_error);
    double sum = std::accumulate(zmp_.begin(), zmp_.end(), 0.0);
    double mean = sum / zmp_.size();
    std::vector<double> diff(zmp_.size());
    std::transform(zmp_.begin(), zmp_.end(), diff.begin(), [mean](double x) { return x - mean; });
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / zmp_.size());
    zmp = lambda_zmp_ * mean + lambda_zmp_ * lambda_zmp_ * stdev;
  }

  double CoM = 0.0;
  // CoM error
  {
    const Eigen::Vector3d & CoM_ref = ctrl.pendulum_.com();
    const Eigen::Vector3d & CoM_mes = ctrl.stabilizer_->measuredCoM();
    const double CoM_error = (CoM_ref - CoM_mes).norm();

    CoM_.push_back(CoM_error);
    double sum = std::accumulate(CoM_.begin(), CoM_.end(), 0.0);
    double mean = sum / CoM_.size();
    std::vector<double> diff(CoM_.size());
    std::transform(CoM_.begin(), CoM_.end(), diff.begin(), [mean](double x) { return x - mean; });
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / CoM_.size());
    CoM = lambda_CoM_ * mean + lambda_CoM_ * lambda_CoM_ * stdev;
  }

  double footstep = 0.0;
  // Footstep
  {
    // Current number of steps to do
    nr_remaining_footstep_ = ctrl.plan.contacts().size() - 2 - ctrl.nrFootsteps_;
    footstep = lambda_footstep_ * nr_remaining_footstep_;
  }

  cost_ = -footstep - zmp - CoM;
  // mc_rtc::log::success("Foot_Target_Pose : {}, Foot_Robot_Pose : {}, error_distance : {}",
  // Foot_Target_Pose.translation().head<2>(), Foot_Robot_Pose.translation().head<2>(), error_distance);
  //  mc_rtc::log::info("[SoftFootState] footstep {}", footstep);
}

void SoftFootState::estimateGround(mc_control::fsm::Controller & ctl, const Foot & current_moving_foot)
{
  // From here do not need to worry about which foot it is
  FootData & data = foot_data_[current_moving_foot];

  // Only one range sensor
  const auto & estimated_ground =
      ctl.robot().device<mc_mujoco::RangeSensor>(range_sensor_names_[current_moving_foot][0]).points();

  if(!estimated_ground.empty())
  {
    data.ground = estimated_ground;
  }

  // // Return the parent body of the sensor (phalanx)
  // const std::string& body_of_sensor =
  // ctl.robot().device<mc_mujoco::RangeSensor>(range_sensor_names_[current_moving_foot][0]).parent();
  // // Access the position of body name in world coordinates (phalanx position)
  // sva::PTransformd X_0_ph;
  // if(use_real_robot_for_estimation_)
  // {
  //   X_0_ph = ctl.realRobot().bodyPosW(body_of_sensor);
  // }
  // else
  // {
  //   X_0_ph = ctl.robot().bodyPosW(body_of_sensor);
  // }
  // bool X_0_ph_updated = false;
  // // Keep in memory the current foot pose;  We need to enqueue only one time per run
  // past_foot_pose_[current_moving_foot].enqueue(X_0_ph);
  // // For logging
  // past_foot_pose_control_[current_moving_foot].enqueue(ctl.robot().bodyPosW(body_of_sensor));

  // // Select data/string based on current_moving_foot
  // for(const auto & sensor_name: range_sensor_names_[current_moving_foot])
  // {
  //   double range = 0.;
  //   double delay_time = 0.;
  //   {
  //     const std::lock_guard<std::mutex> lock(range_sensor_mutex_);
  //     range = ctl.robot().device<mc_mujoco::RangeSensor>(sensor_name).data();
  //     delay_time = ctl.robot().device<mc_mujoco::RangeSensor>(sensor_name).time();
  //   }

  //   if(range_sensor_data_[current_moving_foot][sensor_name] != range)
  //   {
  //     // Update data.range
  //     range_sensor_data_[current_moving_foot][sensor_name] = range;

  //     // Returns the transformation from the parent body to the sensor
  //     const sva::PTransformd& X_ph_s = ctl.robot().device<mc_mujoco::RangeSensor>(sensor_name).X_p_s();
  //     size_t delay_iterations;
  //     if(fixed_delay_of_estimation_ != 0.)
  //     {
  //       delay_iterations = (fixed_delay_of_estimation_) / ctl.solver().dt();
  //     }
  //     else
  //     {
  //       delay_iterations = (delay_time  + delta_delay_of_estimation_) / ctl.solver().dt();
  //     }

  //     if(past_foot_pose_[current_moving_foot].full() || (past_foot_pose_[current_moving_foot].size() -
  //     delay_iterations) > 0)
  //     {
  //       const int n_to_remove = past_foot_pose_[current_moving_foot].size() - delay_iterations - 1;
  //       if(!controller().stabilizer_->inDoubleSupport() && debug_output_)
  //       {
  //         mc_rtc::log::info("[SoftFootState] We saved {} / {} foot pose and we need to go back of {} iterations.",
  //         past_foot_pose_[current_moving_foot].size(), past_foot_pose_[current_moving_foot].capacity(),
  //         delay_iterations); mc_rtc::log::info("[SoftFootState] We will remove {} elements.", n_to_remove);
  //         mc_rtc::log::warning("[SoftFootState] Please ensure that past_foot_pose_ is not full i.e size {} < capacity
  //         {}", past_foot_pose_[current_moving_foot].size(), past_foot_pose_[current_moving_foot].capacity());
  //       }

  //       // Dequeue
  //       for(int i = 0; i < n_to_remove; ++i)
  //       {
  //         past_foot_pose_[current_moving_foot].dequeue();
  //         // For logging
  //         past_foot_pose_control_[current_moving_foot].dequeue();
  //       }

  //       auto optionnal_X_0_ph = past_foot_pose_[current_moving_foot].dequeue();
  //       if(optionnal_X_0_ph.has_value())
  //       {
  //         X_0_ph = optionnal_X_0_ph.value();
  //       }
  //       else
  //       {
  //         if(!controller().stabilizer_->inDoubleSupport())
  //         {
  //           mc_rtc::log::error("[SoftFootState] optionnal_X_0_ph is empty; it should not happen");
  //         }
  //         return;
  //       }

  //       sva::PTransformd X_s_m, X_0_m;
  //       // Keep the estimated 3d point for the ground
  //       if(!use_camera_sensor_)
  //       {
  //         const sva::PTransformd X_s_r = sva::PTransformd(Eigen::Vector3d(0, 0, range));
  //         X_s_m = X_s_r;
  //         const sva::PTransformd X_0_r = X_s_r * X_ph_s * X_0_ph;
  //         X_0_m = X_0_r;
  //         data.ground.push_back(X_0_r.translation());
  //         data.last_ground = data.ground.back();
  //       }
  //       else // Using camera sensor
  //       {
  //         const std::vector<Eigen::Vector3d> points =
  //         ctl.robot().device<mc_mujoco::RangeSensor>(sensor_name).points();
  //         // mc_rtc::log::warning("SoftFootState] {} : {}", sensor_name, points.size());
  //         if(points.empty())
  //         {
  //           continue;
  //         }
  //         // Handle extra data
  //         for(size_t i = 0; i < points.size(); ++i)
  //         {
  //           if(points[i] != Eigen::Vector3d::Zero())
  //           {
  //             const sva::PTransformd X_s_p(points[i]);
  //             X_s_m = X_s_p;
  //             const sva::PTransformd X_0_p = X_s_p * X_ph_s * X_0_ph;
  //             X_0_m = X_0_p;
  //             data.ground.push_back(X_0_p.translation());
  //             if(i == 0)
  //             {
  //               data.last_ground = data.ground.back();
  //             }
  //           }
  //         }
  //       }

  //       {
  //         X_0_ph.rotation() = Eigen::Matrix3d::Identity();
  //         data.last_ground_Identity = (X_s_m * X_ph_s * X_0_ph).translation();
  //       }

  //       {
  //         // For logging
  //         auto optionnal_X_0_ph = past_foot_pose_control_[current_moving_foot].dequeue();
  //         if(optionnal_X_0_ph.has_value())
  //         {
  //           X_0_ph = optionnal_X_0_ph.value();
  //           data.last_ground_control = (X_s_m * X_ph_s * X_0_ph).translation();
  //         }
  //       }

  //       if(!controller().stabilizer_->inDoubleSupport() && debug_output_)
  //       {
  //         mc_rtc::log::info("[SoftFootState] The estimation of ground altitude is {} [m]", X_0_m.translation().z());
  //       }

  //       std::sort(data.ground.begin(), data.ground.end(),
  //         [](const Eigen::Vector3d & a, const Eigen::Vector3d & b)
  //         {
  //           return a.x() < b.x();
  //         }
  //       );

  //       if(data.ground.size() > 500)
  //       {
  //         const auto& ground = data.ground;

  //         std::vector<Eigen::Vector3d> new_ground;
  //         for(size_t i = 0; i < ground.size() - 1; ++i)
  //         {
  //           const Eigen::Vector3d& p = ground[i];
  //           Eigen::Vector3d sum = p;
  //           size_t j = i + 1;
  //           while(j < ground.size() && std::abs(ground[j].x() - p.x()) < 0.0025)
  //           {
  //             sum += ground[j];
  //             ++ j;
  //           }
  //           new_ground.push_back(sum / static_cast<double>(j - i));
  //           i = j - 1;
  //         }
  //         new_ground.push_back(ground.back());

  //         data.ground = new_ground;

  //         if(debug_output_)
  //         {
  //           mc_rtc::log::warning("[SoftFootState] After erasing, the size of ground is {}", data.ground.size());
  //         }
  //       }
  //     }

  //   }
  // }
}

void SoftFootState::extractGroundSegment(mc_control::fsm::Controller & ctl,
                                         const Foot & current_moving_foot,
                                         const Eigen::Vector3d & landing)
{
  FootData & data = foot_data_[current_moving_foot];
  const std::vector<Eigen::Vector3d> & ground = data.ground;

  // Find beginning and ending of segment to extract
  const auto begin_iterator = std::find_if(ground.begin(), ground.end(),
                                           [&](const Eigen::Vector3d & v)
                                           {
                                             return v.x() >= landing.x() + landing_to_foot_middle_offset_
                                                                 - foot_length_ * 0.5 - extra_to_compute_best_position_;
                                           });
  const auto end_iterator = std::find_if(ground.begin(), ground.end(),
                                         [&](const Eigen::Vector3d & v)
                                         {
                                           return v.x() >= landing.x() + landing_to_foot_middle_offset_
                                                               + foot_length_ * 0.5 + extra_to_compute_best_position_;
                                         });

  // Save the selected segment in raw data of ground segment structure
  ground_segment_[current_moving_foot].raw.clear();
  std::vector<Eigen::Vector3d> raw;
  std::transform(begin_iterator, end_iterator, std::back_inserter(raw), [](const Eigen::Vector3d & v) { return v; });

  // Check if there are missing data in collected data due to obstruction
  // Interpolate points in-between point from convex hull
  auto lerp = [](const Eigen::Vector3d & A, const Eigen::Vector3d & B, double t)
  {
    const Eigen::Vector3d vec = B * t + A * (1. - t);
    return vec;
  };

  if(!raw.empty())
  {
    // If yes, interpolate data
    for(size_t i = 0; i < raw.size() - 1; ++i)
    {
      ground_segment_[current_moving_foot].raw.push_back(raw[i]);
      // Check if next element is far
      if(raw[i + 1].x() - raw[i].x() > 0.01)
      {
        // We need to interpolate data
        for(double t = 0.; t <= 1.0; t += 0.05)
        {
          ground_segment_[current_moving_foot].raw.push_back(lerp(raw[i], raw[i + 1], t));
        }
      }
    }

    // Filter the trajectory
    {
      const auto & raw = ground_segment_[current_moving_foot].raw;
      auto & filtered = ground_segment_[current_moving_foot].filtered;
      const double ePow = 1. - std::exp(-0.005 * 2 * M_PI * 5.);
      filtered.push_back(raw.front());
      for(size_t i = 1; i < raw.size(); ++i)
      {
        filtered.emplace_back(raw[i].x(), raw[i].y(), filtered.back().z() + (raw[i].z() - filtered.back().z()) * ePow);
      }

      // Add small variation noise
      const double noise = 0.00075;
      double direction = 1.0;
      for(auto & f : filtered)
      {
        f.z() += direction * noise;
        direction *= -1.;
      }
    }
  }

  mc_rtc::log::info("[SoftFootState] ground_segment_[{}].raw.size() {}",
                    current_moving_foot == Foot::Left ? "Left" : "Right",
                    ground_segment_[current_moving_foot].raw.size());
  mc_rtc::log::info("[SoftFootState] ground_segment_[{}].filtered.size() {}",
                    current_moving_foot == Foot::Left ? "Left" : "Right",
                    ground_segment_[current_moving_foot].filtered.size());

  // Add trajectory
  if(ground_segment_[current_moving_foot].raw.size() > 5)
  {
    const std::string name = current_moving_foot == Foot::Left ? "left" : "right";
    ctl.gui()->addElement({"SoftFoot"}, mc_rtc::gui::Trajectory(name + "_segment", {mc_rtc::gui::Color::Red},
                                                                [this, current_moving_foot]()
                                                                { return ground_segment_[current_moving_foot].raw; }));
    ctl.gui()->addElement({"SoftFoot"},
                          mc_rtc::gui::Trajectory(name + "_segment_filtered", {mc_rtc::gui::Color::Magenta},
                                                  [this, current_moving_foot]()
                                                  { return ground_segment_[current_moving_foot].filtered; }));
  }
}

void SoftFootState::extractAltitudeProfileFromGroundSegment(const Foot & current_moving_foot)
{
  // Get the segment
  const auto & raw_segment = ground_segment_[current_moving_foot].raw;
  // Save the selected segment in raw data of ground segment structure
  {
    const std::lock_guard<std::mutex> lock(variable_stiffness_mutex_);
    std::transform(raw_segment.begin(), raw_segment.end(), std::back_inserter(foot_data_[current_moving_foot].altitude),
                  [](const Eigen::Vector3d & v) { return v.z(); });
  }
}

void SoftFootState::updateVariableStiffness(mc_control::fsm::Controller & ctl,
                                        const Foot & current_moving_foot, bool open_valve)
{
  {
    const std::lock_guard<std::mutex> lock(variable_stiffness_mutex_);
    if(foot_data_[current_moving_foot].altitude.empty())
    {
      // mc_log::error:("[SoftFootState::updateVariableStiffness] The ground is not estimated");
      return;
    }
  }

  // Here we detach a thread to use the service so it is not to slow
  std::thread([this, &ctl, current_moving_foot, open_valve]()
    {
      variable_stiffness::connectionFile srv;
      srv.request.profile = foot_data_[current_moving_foot].altitude;

      if(open_valve)
      {
        const std::lock_guard<std::mutex> lock(variable_stiffness_mutex_);
        srv.request.valvesStatus = 1; // Open the solenoid
        foot_data_[current_moving_foot].valves_status = 1;
        mc_rtc::log::error("Open for {}", current_moving_foot == Foot::Right ? "Right" : "Left");
      }
      else
      {
        const std::lock_guard<std::mutex> lock(variable_stiffness_mutex_);
        srv.request.valvesStatus = 0; // Close the solenoid
        foot_data_[current_moving_foot].valves_status = 0;
        foot_data_[current_moving_foot].altitude.clear();
        mc_rtc::log::error("Close for {}", current_moving_foot == Foot::Right ? "Right" : "Left");
      }

      if(current_moving_foot == Foot::Left)
      {
        srv.request.WhichFoot = 0;
        WhichFoot_ = 0;
      }
      else
      {
        srv.request.WhichFoot = 1;
        WhichFoot_ = 1;
      }

      // if(client_.call(srv) && foot_data_[current_moving_foot].valves_status == 1)
      int valves_status;
      {
        const std::lock_guard<std::mutex> lock(variable_stiffness_mutex_);
        valves_status = foot_data_[current_moving_foot].valves_status;
      }
      mc_rtc::log::info("Calling the server and the valve status is {}", valves_status);
      if(client_.call(srv))
      {
        // const std::lock_guard<std::mutex> lock(variable_stiffness_mutex_);
        // mc_rtc::log::success("[SoftFootState] We udpate the sole stiffness with {}", srv.response.stiffness, srv.response.pressure);
        // foot_data_[current_moving_foot].k = srv.response.stiffness;

        // // Solution to modify the variable stiffness
        // auto stiffnessToAngle = [this](double VarStiff)
        // {
        //   double angle_low = 0;
        //   double angle_high = 1;
        //   double stiffness_low = 0;
        //   double stiffness_high = 100;
        //   return angle_low + (VarStiff - stiffness_low) * (angle_high - angle_low) / (stiffness_high - stiffness_low);
        // };
        // auto postureTask = ctl.getPostureTask(ctl.robot().name());
        // // Reset stifness for both feet gains
        // postureTask->jointGains(ctl.solver(),
        //                         {tasks::qp::JointGains("R_VARSTIFF", 350), tasks::qp::JointGains("L_VARSTIFF", 350)});
        // // Set computed stiffness for current moving foot
        // PhalangesStiffness_ = foot_data_[current_moving_foot].k;
        // pressure_ = srv.response.pressure;
        // postureTask->target({{variable_stiffness_jointname_[current_moving_foot],
        //                       std::vector<double>{stiffnessToAngle(foot_data_[current_moving_foot].k)}}});
        const std::lock_guard<std::mutex> lock(variable_stiffness_mutex_);
        mc_rtc::log::success("[SoftFootState] We udpate the sole stiffness with {} and pressure {}", srv.response.stiffness, srv.response.pressure);
        foot_data_[current_moving_foot].k = srv.response.stiffness;
        foot_data_[current_moving_foot].pressure = srv.response.pressure;

        //Set computed stiffness for current moving foot
        PhalangesStiffness_ = foot_data_[current_moving_foot].k;
        pressure_ = srv.response.pressure;

      }
      else
      {
        mc_rtc::log::error("[SoftFootState] Failed to call service connectionFile to compute the variable stiffness");
      }

      mc_rtc::log::info("Finish to handled stiffness changes");
    }).detach();
}

void SoftFootState::computeSegmentConvexHull(mc_control::fsm::Controller & ctl, const Foot & current_moving_foot)
{
  const auto & raw_segment = ground_segment_[current_moving_foot].filtered;

  const std::vector<Eigen::Vector3d> convex_hull = computeConvexHull(raw_segment);

  // For display
  std::vector<Eigen::Vector3d> convex_display;
  for(size_t i = 0; i < convex_hull.size(); ++i)
  {
    convex_display.push_back(Eigen::Vector3d(convex_hull[i].x(), 0., convex_hull[i].y()));
  }

  // Add trajectory
  const std::string name = current_moving_foot == Foot::Left ? "left" : "right";
  ctl.gui()->addElement({"SoftFoot"}, mc_rtc::gui::Trajectory(name + "_convex_display", {mc_rtc::gui::Color::Blue},
                                                              [this, convex_display]() { return convex_display; }));

  ground_segment_[current_moving_foot].convex = convex_hull;

  // Add trajectory
  ctl.gui()->addElement({"SoftFoot"}, mc_rtc::gui::Trajectory(name + "_convex_segment", {mc_rtc::gui::Color::Magenta},
                                                              [this, current_moving_foot]()
                                                              { return ground_segment_[current_moving_foot].convex; }));
}

std::vector<Eigen::Vector3d> SoftFootState::computeConvexHull(const std::vector<Eigen::Vector3d> & data)
{
  std::vector<Eigen::Vector3d> ret;

  if(data.empty())
  {
    mc_rtc::log::error("[SoftFootState::computeConvexHull] data is empty, we can not compute the convex hull ...");
  }

  // Build the input for qhull
  std::vector<double> points_in;
  double min = std::numeric_limits<double>::max();
  double max = std::numeric_limits<double>::lowest();
  // Skip some points
  for(size_t i = 0; i < data.size(); i += 2)
  {
    const auto & p = data[i];
    points_in.push_back(p.x());
    points_in.push_back(p.z());

    min = std::min(min, p.z());
    max = std::max(max, p.z());
  }

  std::vector<Eigen::Vector2d> convex_hull;
  if(max - min > 0.001)
  {
    try
    {
      // Run qhull
      orgQhull::Qhull qhull;
      qhull.runQhull("", 2, points_in.size() / 2, points_in.data(), "QJ");

      orgQhull::QhullFacet face = qhull.firstFacet();
      orgQhull::QhullVertex v;
      for(size_t i = 0; i < qhull.facetCount(); ++i)
      {
        face = face.nextFacet2d(&v);
        convex_hull.emplace_back(v.point().coordinates()[0], v.point().coordinates()[1]);
      }

      // Select only the upper part of the convex hull
      size_t idx_min_x = std::numeric_limits<size_t>::max();
      double min_x = std::numeric_limits<double>::max();
      size_t idx_max_x = std::numeric_limits<size_t>::lowest();
      double max_x = std::numeric_limits<double>::lowest();
      for(size_t i = 0; i < convex_hull.size(); ++i)
      {
        if(convex_hull[i].x() < min_x)
        {
          min_x = convex_hull[i].x();
          idx_min_x = i;
        }

        if(convex_hull[i].x() > max_x)
        {
          max_x = convex_hull[i].x();
          idx_max_x = i;
        }
      }
      //
      std::vector<Eigen::Vector2d> selected_part_0;
      double max_part_0 = std::numeric_limits<double>::lowest();
      for(size_t i = idx_min_x; i <= idx_max_x; ++i)
      {
        selected_part_0.push_back(convex_hull[i]);
        max_part_0 = selected_part_0.back().y() > max_part_0 ? selected_part_0.back().y() : max_part_0;
      }
      //
      std::vector<Eigen::Vector2d> selected_part_1;
      double max_part_1 = std::numeric_limits<double>::lowest();
      int idx = idx_max_x;
      while(idx != idx_min_x)
      {
        selected_part_1.push_back(convex_hull[idx]);
        idx = (idx + 1) % convex_hull.size();
        max_part_1 = selected_part_1.back().y() > max_part_0 ? selected_part_1.back().y() : max_part_0;
      }
      selected_part_1.push_back(convex_hull[idx_min_x]);
      //
      if(max_part_0 > max_part_1)
      {
        convex_hull = selected_part_0;
      }
      else
      {
        convex_hull = selected_part_1;
      }
      // Sort alongside x
      std::sort(convex_hull.begin(), convex_hull.end(),
                [](const Eigen::Vector2d & a, const Eigen::Vector2d & b) { return a.x() < b.x(); });
    }
    catch(std::exception & e)
    {
      mc_rtc::log::error("[SoftFootState] Error during qhull ! {}", e.what());
      convex_hull.push_back(Eigen::Vector2d(data.front().x(), data.front().z()));
      convex_hull.push_back(Eigen::Vector2d(data.back().x(), data.back().z()));
    }
  }
  else
  {
    mc_rtc::log::error("[SoftFootState] Too flat to run qhull !");
    convex_hull.push_back(Eigen::Vector2d(data.front().x(), data.front().z()));
    convex_hull.push_back(Eigen::Vector2d(data.back().x(), data.back().z()));
  }

  // Interpolate points in-between point from convex hull
  auto lerp = [](const Eigen::Vector2d & A, const Eigen::Vector2d & B, double y, double t)
  {
    const Eigen::Vector2d vec = B * t + A * (1. - t);
    return Eigen::Vector3d(vec.x(), y, vec.y());
  };

  for(size_t i = 0; i < convex_hull.size() - 1; ++i)
  {
    for(double t = 0.; t <= 1.0; t += 0.1)
    {
      ret.push_back(lerp(convex_hull[i], convex_hull[i + 1], data.front().y(), t));
    }
  }

  return ret;
}

void SoftFootState::computeMinMaxAngle(const Foot & current_moving_foot)
{
  if(nr_phalanxes_ > 1.)
  {
    const double MASS_PHALANX = 0.2;
    const double MASS_ROBOT = 43.;
    const double nr_phalanxes = static_cast<double>(nr_phalanxes_);
    double k = 0.;
    {
      const std::lock_guard<std::mutex> lock(variable_stiffness_mutex_);
      k = foot_data_[current_moving_foot].k;
    }
    const double d = 2.;
    const double f0 = MASS_ROBOT * 9.81 * (foot_length_ / 2.);
    const double I = (10. * MASS_PHALANX * phalanx_length_ * phalanx_length_) / 3.;

    auto odefcn = [=](const std::vector<double> & x, std::vector<double> & dx, const double t)
    {
      dx[0] = x[1];
      dx[1] = (-(nr_phalanxes - 1.) * k * x[0] - (nr_phalanxes - 1.) * d * x[1] + f0) / I;
    };

    double angle = 0.;
    auto observer = [&angle](const std::vector<double> & x, const double t)
    {
      angle = x[0]; // * 180. / M_PI;
    };

    typedef boost::numeric::odeint::runge_kutta_dopri5<std::vector<double>> stepper_type;
    const double dt = 0.1;
    std::vector<double> x(2);
    x[0] = 0.0;
    x[1] = 0.0;
    boost::numeric::odeint::integrate_const(boost::numeric::odeint::make_dense_output<stepper_type>(1E-6, 1E-3), odefcn,
                                            x, 0.0, 10.0, dt, observer);

    angle = angle / (nr_phalanxes - 1.);

    mc_rtc::log::info("[SoftFootState] Min/max angle is {} [rad]", angle);

    foot_data_[current_moving_foot].min_max_phalanxes_angle = angle;
  }
  else
  {
    foot_data_[current_moving_foot].min_max_phalanxes_angle = M_PI;
  }
}

void SoftFootState::computeFootLandingPosition(const Foot & current_moving_foot,
                                               const Eigen::Vector3d & default_landing_pos)
{
  //
  auto computeThetaBasedOnQhull = [&](const Eigen::Vector3d & start, const Eigen::Vector3d & end)
  {
    const auto & ground = ground_segment_[current_moving_foot].filtered;

    std::vector<Eigen::Vector3d> ground_under_phalanx;

    const auto begin_iterator = std::find_if(ground.begin(), ground.end(),
                                             [&](const Eigen::Vector3d & v) { return v.x() >= start.x() - 0.01; });
    const auto end_iterator =
        std::find_if(ground.begin(), ground.end(), [&](const Eigen::Vector3d & v) { return v.x() >= end.x() + 0.01; });
    std::transform(begin_iterator, end_iterator, std::back_inserter(ground_under_phalanx),
                   [](const Eigen::Vector3d & v) { return v; });

    if(ground_under_phalanx.empty())
    {
      return 0.;
    }

    const std::vector<Eigen::Vector3d> convex = computeConvexHull(ground_under_phalanx);

    if(convex.empty())
    {
      mc_rtc::log::error("[SoftFootState::computeFootLandingPosition] convex hull is empty");
      return 0.;
    }

    const auto start_convex_iterator =
        std::find_if(convex.begin(), convex.end(), [&](const Eigen::Vector3d & v) { return v.x() >= start.x(); });
    const auto end_convex_iterator =
        std::find_if(convex.begin(), convex.end(), [&](const Eigen::Vector3d & v) { return v.x() >= end.x(); });

    if(start_convex_iterator == convex.begin() || end_convex_iterator == convex.end())
    {
      return 0.;
    }

    const Eigen::Vector3d & p_0 = *start_convex_iterator;
    const Eigen::Vector3d & p_1 = *end_convex_iterator;

    const double dz = p_1.z() - p_0.z();
    const double dx = p_1.x() - p_0.x();

    const double theta = std::atan(dz / dx);

    return theta;
  };

  //
  auto whichPhalanxItIs = [&](const Eigen::Vector3d & landing_pos,
                              const Eigen::Vector3d & highest_point_on_convex) -> size_t
  {
    const auto & convex = ground_segment_[current_moving_foot].convex;

    // Compute phalanxes pos
    const double start_x =
        (sva::PTransformd(Eigen::Vector3d(-foot_length_ * 0.5, 0., 0.)) * landing_pos).translation().x();
    const double end_x =
        (sva::PTransformd(Eigen::Vector3d(+foot_length_ * 0.5, 0., 0.)) * landing_pos).translation().x();

    double ratio = (highest_point_on_convex.x() - start_x) / phalanx_length_;
    return static_cast<size_t>(ratio);
  };

  //
  auto addLeftPhalanxToPhalanxes = [&](const Eigen::Vector3d & phalanx_pos, double theta)
  {
    const std::array<double, 2> start = {phalanx_pos.x(), phalanx_pos.z()};
    foot_data_[current_moving_foot].phalanxes.push_back(start);

    const std::array<double, 2> end = {phalanx_pos.x() + phalanx_length_ * std::cos(theta),
                                       phalanx_pos.z() + phalanx_length_ * std::sin(theta)};
    foot_data_[current_moving_foot].phalanxes.push_back(end);
  };
  //
  auto addRightPhalanxToPhalanxes = [&](const Eigen::Vector3d & phalanx_pos, double theta)
  {
    const std::array<double, 2> start = {phalanx_pos.x() - phalanx_length_ * std::cos(theta),
                                         phalanx_pos.z() - phalanx_length_ * std::sin(theta)};
    foot_data_[current_moving_foot].phalanxes.push_back(start);

    const std::array<double, 2> end = {phalanx_pos.x(), phalanx_pos.z()};
    foot_data_[current_moving_foot].phalanxes.push_back(end);
  };

  //
  auto computeSide = [&](int start, int end, double direction, const Eigen::Vector3d & phalanx_pos, double theta,
                         std::vector<double> & ret_phalanxes_theta, std::vector<Eigen::Vector2d> & ret_contact_points,
                         bool switch_direction = false) -> std::vector<Eigen::Vector3d>
  {
    std::vector<Eigen::Vector3d> ret;
    if(start == end)
    {
      return ret;
    }
    const std::vector<Eigen::Vector3d> & ground = ground_segment_[current_moving_foot].filtered;

    // std::cout << "phalanx_pos " << phalanx_pos.transpose() << std::endl;

    Eigen::Vector3d next_phalanx_side_pos(phalanx_pos.x() + direction * (0.5 * phalanx_length_) * std::cos(theta),
                                          phalanx_pos.y(),
                                          phalanx_pos.z() + direction * (0.5 * phalanx_length_) * std::sin(theta));

    if(switch_direction)
    {
      direction = -direction;
    }

    // std::cout << "phalanx_pos_side " << next_phalanx_side_pos.transpose() << std::endl;

    // Create linspace of theta to evaluate using the foot_data_[current_moving_foot].min_max_phalanxes_angle
    const double min_max_phalanxes_angle =
        std::min(M_PI * 0.5, foot_data_[current_moving_foot].min_max_phalanxes_angle);
    const double start_theta = direction * min_max_phalanxes_angle - direction * 0.01;
    const double end_theta = -direction * min_max_phalanxes_angle + direction * 0.01;
    // const double start_theta = direction * M_PI * 0.5 - direction * 0.01;
    // const double end_theta = -direction * M_PI * 0.5  + direction * 0.01;
    // std::cout << "direction " << direction << std::endl;
    std::cout << "start_theta " << start_theta << " end_theta " << end_theta << " - "
              << static_cast<int>(min_max_phalanxes_angle * 180. / M_PI) << std::endl;
    const std::vector<double> evaluated_thetas =
        linspace(start_theta, end_theta, static_cast<int>(min_max_phalanxes_angle * 180. / M_PI)); // [rad]
    //
    std::vector<int> phalanx_idxs;
    if(end > 2)
    {
      phalanx_idxs = linspace(start, end, std::abs(end - start) + 1);
    }
    else
    {
      phalanx_idxs = {0};
    }
    for(const auto & phalanx_i : phalanx_idxs)
    {
      double theta_intersection = 0.;
      bool is_intersection = false;
      Eigen::Vector2d contact_found;

      if(phalanx_i < nr_phalanxes_)
      {
        Eigen::Vector3d phalanx_pos_left;
        Eigen::Vector3d phalanx_pos_right;

        std::vector<Eigen::Vector3d> ground_under_phalanx;
        // Find beginning and ending of segment to extract to select ground under phalanx
        if(direction == -1)
        {
          phalanx_pos_right = next_phalanx_side_pos;
          phalanx_pos_left =
              Eigen::Vector3d(phalanx_pos_right.x() - phalanx_length_, phalanx_pos_right.y(), phalanx_pos_right.z());
        }
        else
        {
          phalanx_pos_left = next_phalanx_side_pos;
          phalanx_pos_right =
              Eigen::Vector3d(phalanx_pos_left.x() + phalanx_length_, phalanx_pos_left.y(), phalanx_pos_left.z());
        }
        const auto begin_iterator = std::find_if(
            ground.begin(), ground.end(), [&](const Eigen::Vector3d & v) { return v.x() >= phalanx_pos_left.x(); });
        const auto end_iterator = std::find_if(
            ground.begin(), ground.end(), [&](const Eigen::Vector3d & v) { return v.x() >= phalanx_pos_right.x(); });
        std::transform(begin_iterator, end_iterator, std::back_inserter(ground_under_phalanx),
                       [](const Eigen::Vector3d & v) { return v; });
        // std::cout << ground_under_phalanx.size() << std::endl;
        if(!ground_under_phalanx.empty())
        {
          for(const auto & ev_theta : evaluated_thetas)
          {
            // std::cout << "phalanx_pos_left " << phalanx_pos_left.transpose() << " phalanx_pos_right " <<
            // phalanx_pos_right.transpose() << std::endl;
            is_intersection =
                evaluateTheta(ev_theta, direction, next_phalanx_side_pos, ground_under_phalanx, contact_found);
            if(is_intersection)
            {
              const double step_theta = 2. * M_PI / 180.;
              const double start_theta = ev_theta + direction * step_theta;
              const double end_theta = ev_theta - direction * step_theta;
              const std::vector<double> evaluated_precise_thetas = linspace(start_theta, end_theta, 10); // [rad]
              for(const auto & ev_pre_theta : evaluated_precise_thetas)
              {
                is_intersection =
                    evaluateTheta(ev_pre_theta, direction, next_phalanx_side_pos, ground_under_phalanx, contact_found);
                if(is_intersection)
                {
                  // std::cout << "ev_pre_theta: " << ev_pre_theta << std::endl;
                  theta_intersection = ev_pre_theta;
                  break;
                }
              }
              break;
            }
          }
        }

        if(is_intersection)
        {
          // std::cout << "intersection found" << std::endl;
          ret_contact_points.push_back(contact_found);
        }
        else
        {
          std::cout << "no intersection found" << std::endl;
          theta_intersection = 0.;
        }

        // Update pos
        next_phalanx_side_pos =
            Eigen::Vector3d(next_phalanx_side_pos.x() + direction * phalanx_length_ * std::cos(theta_intersection),
                            next_phalanx_side_pos.y(),
                            next_phalanx_side_pos.z() + direction * phalanx_length_ * std::sin(theta_intersection));

        // std::cout << next_phalanx_side_pos.transpose() << std::endl;

        ret_phalanxes_theta.push_back(theta_intersection);
        ret.push_back(next_phalanx_side_pos);
        // break;
      }
    }

    return ret;
  };

  const std::vector<double> position_offsets_x = {0., -0.015, -0.01, -0.005, 0.005, 0.01, 0.015};
  // const std::vector<double> position_offsets_x = {0.};
  const auto & convex = ground_segment_[current_moving_foot].convex;
  double maximized_distance_between_phalanxes = std::numeric_limits<double>::lowest();
  for(const auto & position_offset_x : position_offsets_x)
  {
    // Compute landing pos to evaluate
    const sva::PTransformd landing_pos =
        sva::PTransformd(Eigen::Vector3d(position_offset_x, 0., 0.)) * default_landing_pos;
    // Get min and max foot x
    const double min_x_foot =
        (sva::PTransformd(Eigen::Vector3d(-foot_length_ * 0.5 + landing_to_foot_middle_offset_, 0., 0.)) * landing_pos)
            .translation()
            .x();
    const double max_x_foot =
        (sva::PTransformd(Eigen::Vector3d(+foot_length_ * 0.5 + landing_to_foot_middle_offset_, 0., 0.)) * landing_pos)
            .translation()
            .x();
    // Find highest point on hull
    Eigen::Vector3d highest_point_on_convex(0., 0., std::numeric_limits<double>::lowest());
    for(const auto & pos : convex)
    {
      if(pos.x() >= min_x_foot && pos.x() <= max_x_foot && pos.z() > highest_point_on_convex.z())
      {
        highest_point_on_convex = pos;
      }
    }

    // std::cout << "Highest point on convex hull is " << highest_point_on_convex.transpose() << std::endl;
    // Compute which n-th phalanx will land on the highest point
    size_t n_phalanx = whichPhalanxItIs(landing_pos.translation(), highest_point_on_convex);
    // Keep highest altitude
    foot_data_[current_moving_foot].position_offset_z =
        std::max(foot_data_[current_moving_foot].position_offset_z, highest_point_on_convex.z());

    // std::cout << "min_x_foot " << min_x_foot << " max_x_foot " << max_x_foot << std::endl;
    Eigen::Vector3d phalanx_pos =
        (sva::PTransformd(Eigen::Vector3d(
             -foot_length_ * 0.5 + landing_to_foot_middle_offset_ + (n_phalanx + 0.5) * phalanx_length_, 0., 0.))
         * landing_pos)
            .translation();
    phalanx_pos.z() = highest_point_on_convex.z();

    mc_rtc::log::warning("[SoftFootState] computeFootLandingPosition position_offset_z {}",
                         foot_data_[current_moving_foot].position_offset_z);
    mc_rtc::log::warning("[SoftFootState] computeFootLandingPosition highest_point_on_convex {}",
                         highest_point_on_convex.z());

    // std::cout << "phalanx_pos_left " << phalanx_pos_left.transpose() << std::endl;
    // std::cout << "phalanx_pos_right " << phalanx_pos_right.transpose() << std::endl;

    // std::cout << "n_phalanx " << n_phalanx << " nr_phalanxes_ " << nr_phalanxes_ << std::endl;
    Eigen::Vector3d phalanx_pos_right;
    std::vector<double> right_side_phalanxes_theta;
    std::vector<Eigen::Vector2d> right_contact_points;
    std::vector<Eigen::Vector3d> right_side_phalanxes_pos;

    Eigen::Vector3d phalanx_pos_left;
    std::vector<double> left_side_phalanxes_theta;
    std::vector<Eigen::Vector2d> left_contact_points;
    std::vector<Eigen::Vector3d> left_side_phalanxes_pos;

    double distance_between_phalanxes = 0.;

    if(nr_phalanxes_ > 2)
    {
      // std::cout << "The n-th phalanx is " <<  n_phalanx << std::endl;
      // phalanx_pos is the middle of the phalanx
      const Eigen::Vector3d phalanx_pos_start =
          (sva::PTransformd(Eigen::Vector3d(
               -foot_length_ * 0.5 + landing_to_foot_middle_offset_ + n_phalanx * phalanx_length_, 0., 0.))
           * landing_pos)
              .translation();
      const Eigen::Vector3d phalanx_pos_end =
          (sva::PTransformd(Eigen::Vector3d(
               -foot_length_ * 0.5 + landing_to_foot_middle_offset_ + (n_phalanx + 1) * phalanx_length_, 0., 0.))
           * landing_pos)
              .translation();

      // Compute landing theta at this point for the n-th phalanx
      const double theta = computeThetaBasedOnQhull(phalanx_pos_start, phalanx_pos_end);
      // std::cout << "theta " << theta << std::endl;
      // std::cout << highest_point_on_convex.transpose() << std::endl;
      // std::cout << "phalanx_pos " << phalanx_pos.transpose() << std::endl;
      // std::cout << "start phalanx " << min_x_foot + phalanx_length_ * n_phalanx << std::endl;

      phalanx_pos_left = Eigen::Vector3d(phalanx_pos.x() - (0.5 * phalanx_length_) * std::cos(theta), phalanx_pos.y(),
                                         phalanx_pos.z() - (0.5 * phalanx_length_) * std::sin(theta));
      phalanx_pos_right = Eigen::Vector3d(phalanx_pos.x() + (0.5 * phalanx_length_) * std::cos(theta), phalanx_pos.y(),
                                          phalanx_pos.z() + (0.5 * phalanx_length_) * std::sin(theta));

      // Compute right part
      // std::cout << "Right side" << std::endl;
      right_side_phalanxes_pos = computeSide(n_phalanx, nr_phalanxes_, +1., phalanx_pos, theta,
                                             right_side_phalanxes_theta, right_contact_points);

      // Compute left part
      // std::cout << "Left side" << std::endl;
      left_side_phalanxes_pos =
          computeSide(n_phalanx, 0, -1., phalanx_pos, theta, left_side_phalanxes_theta, left_contact_points);

      if(left_contact_points.size() > 0 && right_contact_points.size() > 0)
      {
        distance_between_phalanxes = right_contact_points.back().x() - left_contact_points.back().x();
      }
      else if(left_contact_points.size() > 0)
      {
        distance_between_phalanxes = phalanx_pos.x() - left_contact_points.back().x();
      }
      else if(right_contact_points.size() > 0)
      {
        distance_between_phalanxes = right_contact_points.back().x() - phalanx_pos.x();
      }
    }
    else
    {
      // std::cout << "The n-th phalanx is " <<  n_phalanx << std::endl;
      // phalanx_pos is the middle of the phalanx
      const Eigen::Vector3d phalanx_pos_start =
          (sva::PTransformd(Eigen::Vector3d(
               -foot_length_ * 0.5 + landing_to_foot_middle_offset_ + n_phalanx * phalanx_length_, 0., 0.))
           * landing_pos)
              .translation();
      const Eigen::Vector3d phalanx_pos_end =
          (sva::PTransformd(Eigen::Vector3d(
               -foot_length_ * 0.5 + landing_to_foot_middle_offset_ + (n_phalanx + 1) * phalanx_length_, 0., 0.))
           * landing_pos)
              .translation();

      // Compute landing theta at this point for the n-th phalanx
      const double theta = 0.;

      // double theta = 0.;

      phalanx_pos_left = Eigen::Vector3d(phalanx_pos.x() - (0.5 * phalanx_length_) * std::cos(theta), phalanx_pos.y(),
                                         phalanx_pos.z() - (0.5 * phalanx_length_) * std::sin(theta));
      phalanx_pos_right = Eigen::Vector3d(phalanx_pos.x() + (0.5 * phalanx_length_) * std::cos(theta), phalanx_pos.y(),
                                          phalanx_pos.z() + (0.5 * phalanx_length_) * std::sin(theta));

      if(std::abs(highest_point_on_convex.x() - phalanx_pos_right.x())
         > std::abs(highest_point_on_convex.x() - phalanx_pos_left.x()))
      {
        distance_between_phalanxes = std::abs(highest_point_on_convex.x() - phalanx_pos_right.x());
      }
      else
      {
        distance_between_phalanxes = std::abs(highest_point_on_convex.x() - phalanx_pos_left.x());
      }

      std::cout << "distance_between_phalanxes " << distance_between_phalanxes << std::endl;
    }

    if(foot_data_[current_moving_foot].phalanxes.empty()
       || distance_between_phalanxes > maximized_distance_between_phalanxes)
    {
      maximized_distance_between_phalanxes = distance_between_phalanxes;
      foot_data_[current_moving_foot].position_offset_x = position_offset_x;
      mc_rtc::log::info("[SoftFootState] New maximized found for {}", position_offset_x);
      mc_rtc::log::info("[SoftFootState] distance_between_phalanxes {}", distance_between_phalanxes);
      foot_data_[current_moving_foot].phalanxes.clear();
      // Fill phalanxes
      for(int i = left_side_phalanxes_pos.size() - 1; i >= 0; --i)
      {
        addLeftPhalanxToPhalanxes(left_side_phalanxes_pos[i], left_side_phalanxes_theta[i]);
      }

      const std::array<double, 2> start = {phalanx_pos_left.x(), phalanx_pos_left.z()};
      foot_data_[current_moving_foot].phalanxes.push_back(start);

      const std::array<double, 2> end = {phalanx_pos_right.x(), phalanx_pos_right.z()};
      foot_data_[current_moving_foot].phalanxes.push_back(end);

      for(int i = 0; i < right_side_phalanxes_pos.size(); ++i)
      {
        addRightPhalanxToPhalanxes(right_side_phalanxes_pos[i], right_side_phalanxes_theta[i]);
      }
    }
  }
}

bool SoftFootState::evaluateTheta(double theta,
                                  double direction,
                                  const Eigen::Vector3d & pos,
                                  const std::vector<Eigen::Vector3d> & ground_under_phalanx,
                                  Eigen::Vector2d & output)
{
  // To check if the intersection belongs to the segment we are evaluating
  auto isPointOnSegment = [](const Eigen::Vector2d & A, const Eigen::Vector2d & B, const Eigen::Vector2d & C) -> bool
  {
    const Eigen::Vector2d AB = B - A;
    const Eigen::Vector2d AC = C - A;

    // Check the they are collinear
    if(std::abs(AB.x() * AC.y() - AC.x() * AB.y()) < 1e-3)
    {
      // Check if C is in between A and B
      const double dot_AB = AB.dot(AB);
      const double dot_AC = AB.dot(AC);

      // // std::cout << "dot_AB " << dot_AB << " dot_AC " << dot_AC << std::endl;
      if(dot_AC == 0. || dot_AC == dot_AB || (dot_AC > 0. && dot_AC < dot_AB))
      {
        // // std::cout << "Point on segment" << A.transpose() << " " << B.transpose() << " " << C.transpose() <<
        // std::endl;
        return true;
      }
    }
    return false;
  };

  //
  const Eigen::Vector2d start_offset(pos.x() + direction * 0.005 * std::cos(theta),
                                     pos.z() + direction * 0.005 * std::sin(theta));

  const Eigen::Vector2d end(pos.x() + direction * phalanx_length_ * std::cos(theta),
                            pos.z() + direction * phalanx_length_ * std::sin(theta));

  // Compute line equation
  const double slope = (end.y() - pos.z()) / (end.x() - pos.x());
  const double intercept = end.y() - slope * end.x();

  auto f = [slope, intercept](double x) -> double { return slope * x + intercept; };

  // std::cout << "-----------------------" << std::endl;
  // std::cout << "For theta: " << theta << std::endl;
  // // std::cout << pos.z() << " = " << f(pos.x()) << " -- " << end.y() << " = " << f(end.x()) << std::endl;
  // std::cout << "start_offset " << start_offset.transpose() << " end " << end.transpose() << std::endl;
  // std::cout << "gxs " << ground_under_phalanx.front().x() << " gxe " << ground_under_phalanx.back().x() << std::endl;
  // Compute points for line that matches
  double previous_diff_y = f(ground_under_phalanx[0].x()) - ground_under_phalanx[0].z();
  for(size_t i = 1; i < ground_under_phalanx.size(); ++i)
  {
    const double line_y = f(ground_under_phalanx[i].x());
    const double diff_y = line_y - ground_under_phalanx[i].z();
    // std::cout.precision(4);
    // std::cout << ground_under_phalanx[i].x() << " (" << line_y << " - " << ground_under_phalanx[i].z() << " = " <<
    // diff_y << ") "; We check if it is a possible 0
    if(previous_diff_y * diff_y <= 0.)
    {
      // Check if the point is on the segment
      // // std::cout << "Found a zero at " << i << std::endl;
      // // std::cout << "Ground " << Eigen::Vector2d(ground_under_phalanx[i].x(),
      // ground_under_phalanx[i].z()).transpose() << std::endl;
      output = Eigen::Vector2d(ground_under_phalanx[i].x(), ground_under_phalanx[i].z());
      if(isPointOnSegment(start_offset, end, output))
      {
        return true;
      }
    }
    previous_diff_y = diff_y;
  }
  // std::cout << std::endl;

  return false;
}

void SoftFootState::computeFootLandingAngle(const Foot & current_moving_foot, const Eigen::Vector3d & landing)
{
  // Get closest point from landing in convex
  const auto & convex = ground_segment_[current_moving_foot].convex;
  auto convex_iterator =
      std::find_if(convex.begin(), convex.end(), [&](const Eigen::Vector3d & v) { return v.x() >= landing.x(); });

  const Eigen::Vector3d & p_1 = *convex_iterator;
  convex_iterator = --convex_iterator;
  const Eigen::Vector3d & p_0 = *(convex_iterator);

  double dz = p_1.z() - p_0.z();
  double dx = p_1.x() - p_0.x();

  const double bound_for_angle = 25. * M_PI / 180.; // [rad]
  // foot_data_[current_moving_foot].angle = std::max(std::min(-std::atan(dz / dx), bound_for_angle), -bound_for_angle);

  foot_data_[current_moving_foot].angle = -std::atan(dz / dx);
  if(foot_data_[current_moving_foot].angle < -bound_for_angle)
  {
    mc_rtc::log::warning("[SoftFootState::computeFootLandingAngle] Clamp computed angle {} to {}",
                         foot_data_[current_moving_foot].angle, -bound_for_angle);
    foot_data_[current_moving_foot].angle = -bound_for_angle;
  }
  else if(foot_data_[current_moving_foot].angle > bound_for_angle)
  {
    mc_rtc::log::warning("[SoftFootState::computeFootLandingAngle] Clamp computed angle {} to {}",
                         foot_data_[current_moving_foot].angle, bound_for_angle);
    foot_data_[current_moving_foot].angle = bound_for_angle;
  }

  Eigen::Vector3d highest_point_on_convex(0., 0., std::numeric_limits<double>::lowest());
  const double min_x_foot =
      (sva::PTransformd(Eigen::Vector3d(-foot_length_ * 0.5 + landing_to_foot_middle_offset_, 0., 0.))
       * sva::PTransformd(landing))
          .translation()
          .x();
  const double max_x_foot =
      (sva::PTransformd(Eigen::Vector3d(+foot_length_ * 0.5 + landing_to_foot_middle_offset_, 0., 0.))
       * sva::PTransformd(landing))
          .translation()
          .x();
  for(const auto & pos : convex)
  {
    if(pos.x() >= min_x_foot && pos.x() <= max_x_foot && pos.z() > highest_point_on_convex.z())
    {
      highest_point_on_convex = pos;
    }
  }
  // Keep highest altitude
  foot_data_[current_moving_foot].position_offset_z = highest_point_on_convex.z();
  // foot_data_[current_moving_foot].position_offset_z = p_1.z();
}

void SoftFootState::updateFootSwingPose(mc_control::fsm::Controller & ctl,
                                        const Foot & current_moving_foot,
                                        const sva::PTransformd &)
{
  auto & ctrl = controller();
  // Get the angle
  double desired_angle = foot_data_[current_moving_foot].angle * 1.05;
  if(desired_angle > 0.05)
  {
    desired_angle *= 1.25;
  }
  if(!with_ankle_rotation_)
  {
    desired_angle = 0.;
  }
  double desired_offset_position_x = foot_data_[current_moving_foot].position_offset_x;
  if(desired_angle > -0.05)
  {
    desired_offset_position_x += 0.025;
  }
  if(desired_angle > 0.05)
  {
    desired_offset_position_x += 0.015;
  }
  if(!with_foot_adjustment_)
  {
    desired_offset_position_x = 0.;
  }
  // double desired_offset_position_z = foot_data_[current_moving_foot].position_offset_z * 1.15 + 0.0;
  double desired_offset_position_z = -0.005;

  // if(desired_angle > 0.05)
  // {
  //   // desired_offset_position_z = desired_offset_position_z * 1.30 + 0.015;
  //   desired_offset_position_z = desired_offset_position_z * 1.30 + 0.01;
  // }
  // if(!with_foot_adjustment_ && !with_ankle_rotation_)
  // {
  //   desired_offset_position_z = 0.;
  // }

  mc_rtc::log::info("[SoftFootState] position_offset_x for landing {}", desired_offset_position_x);
  mc_rtc::log::info("[SoftFootState] position_offset_z for landing {}", desired_offset_position_z);
  mc_rtc::log::info("[SoftFootState] angle {} [rad] {} [deg]", desired_angle, desired_angle * 180. / M_PI);

  if(ctrl.swingTrajType != "DefaultSwingFoot")
  {
    ctrl.swingTraj->update(desired_angle, desired_offset_position_x, desired_offset_position_z);
    ctrl.plan.updateTargetContact(ctrl.swingTraj->endPose_);
  }
  else
  {
    mc_rtc::log::warning("[SoftFootState] The swing trajectory is '{}', if you want an update please use "
                         "'LandingSearch' or 'CubicSplineSimple'",
                         ctrl.swingTrajType);
  }
}

void SoftFootState::reset(mc_control::fsm::Controller & ctl, const Foot & foot)
{
  foot_data_[foot].need_reset = false;
  foot_data_[foot].computation_done = false;
  foot_data_[foot].min_max_phalanxes_angle = 0.;
  foot_data_[foot].position_offset_x = 0.;
  foot_data_[foot].position_offset_z = 0.;

  {
    const std::lock_guard<std::mutex> lock(variable_stiffness_mutex_);
    if(ctl.robot().hasJoint("L_VARSTIFF"))
    {
      auto angleToStiffness = [this](double angle)
      {
        double angle_low = 0;
        double angle_high = 1;
        double stiffness_low = 0;
        double stiffness_high = 100;
        return stiffness_low + (angle - angle_low) * (stiffness_high - stiffness_low) / (angle_high - angle_low);
      };
      foot_data_[foot].k = angleToStiffness(
          ctl.robot().q()[ctl.robot().jointIndexByName(foot == Foot::Left ? "L_VARSTIFF" : "R_VARSTIFF")][0]);
    }
    else
    {
      foot_data_[foot].k = 0.;
    }
  }

  // Delete all the data that are behind the foot
  const auto X_0_p = ctl.robot().surfacePose(surface_name_[foot]);
  // Before to do so, we need to sort ground as it is unsorted and the elements order are time-based
  auto & ground = foot_data_[foot].ground;

  const auto ground_iterator = std::find_if(ground.begin(), ground.end(),
                                            [&](const Eigen::Vector3d & v)
                                            {
                                              return v.x() >= X_0_p.translation().x() + landing_to_foot_middle_offset_
                                                                  - 0.5 * foot_length_
                                                                  - extra_to_compute_best_position_;
                                            });
  // Delete from the beginning of the vector up to the back part of the foot
  if(ground_iterator != ground.begin())
  {
    ground.erase(ground.begin(), ground_iterator);
  }

  // Reset altitude
  foot_data_[foot].altitude.clear();
  foot_data_[foot].phalanxes.clear();

  // Reset ground segment data
  ground_segment_[foot].raw.clear();
  ground_segment_[foot].filtered.clear();
  ground_segment_[foot].convex.clear();


  // Get name for the feet
  const Foot other_foot = foot == Foot::Left ? Foot::Right : Foot::Left;
  const std::string other_name = other_foot == Foot::Left ? "left" : "right";
  const std::string name = foot == Foot::Left ? "left" : "right";

  // Make sure to close the valve
  // if(foot_data_[other_foot].valves_status == 1)
  // {
    // call the server and update the variable stiffness
  if(with_variable_stiffness_)
  {
    mc_rtc::log::error("Close for {}", other_foot == Foot::Right ? "Right" : "Left");
    updateVariableStiffness(ctl, other_foot, false);
  }
  // }

  // Reset logger
  ctl.logger().removeLogEntry("MyMeasures_" + other_name + "_range");
  for(const auto & range_sensor_name : range_sensor_names_[foot])
  {
    ctl.logger().removeLogEntry("MyMeasures_" + name + "_" + range_sensor_name + "_range");
  }
  ctl.logger().removeLogEntry("MyMeasures_" + other_name + "_ground");
  ctl.logger().removeLogEntry("MyMeasures_" + other_name + "_ground_control");
  ctl.logger().removeLogEntry("MyMeasures_" + other_name + "_ground_Identity");
  ctl.logger().removeLogEntry("MyMeasures_" + other_name + "_back_foot_x");
  ctl.logger().removeLogEntry("MyMeasures_" + other_name + "_front_foot_x");
  ctl.logger().removeLogEntry("MyMeasures_" + other_name + "_back_foot_z");
  ctl.logger().removeLogEntry("MyMeasures_" + other_name + "_front_foot_z");
  ctl.logger().removeLogEntry("MyMeasures_" + other_name + "_filtered_x");
  ctl.logger().removeLogEntry("MyMeasures_" + other_name + "_convex_x");
  ctl.logger().removeLogEntry("MyMeasures_" + other_name + "_filtered_y");
  ctl.logger().removeLogEntry("MyMeasures_" + other_name + "_convex_y");
  ctl.logger().removeLogEntry("MyMeasures_" + other_name + "_k");
  ctl.logger().removeLogEntry("MyMeasures_" + other_name + "_angle");
  ctl.logger().removeLogEntry("MyMeasures_" + other_name + "_z_offset");
  ctl.logger().removeLogEntry("MyMeasures_" + other_name + "_min_max_phalanxes_angle");
  ctl.logger().removeLogEntry("MyMeasures_" + other_name + "_position_offset");
  ctl.logger().removeLogEntry("MyMeasures_" + other_name + "_valves_status");
  ctl.logger().removeLogEntry("MyMeasures_" + other_name + "_phalanxes_x");
  ctl.logger().removeLogEntry("MyMeasures_" + other_name + "_phalanxes_y");

  // Reset GUI
  ctl.gui()->removeElement({"SoftFoot"}, other_name + "_point_ground");
  ctl.gui()->removeElement({"SoftFoot"}, other_name + "_ground");
  ctl.gui()->removeElement({"SoftFoot"}, other_name + "_back_foot");
  ctl.gui()->removeElement({"SoftFoot"}, other_name + "_front_foot");

  ctl.gui()->removeElement({"SoftFoot"}, other_name + "_segment");
  ctl.gui()->removeElement({"SoftFoot"}, name + "_segment");

  ctl.gui()->removeElement({"SoftFoot"}, other_name + "_segment_filtered");
  ctl.gui()->removeElement({"SoftFoot"}, name + "_segment_filtered");

  ctl.gui()->removeElement({"SoftFoot"}, other_name + "_convex_segment");
  ctl.gui()->removeElement({"SoftFoot"}, name + "_convex_segment");

  ctl.gui()->removeElement({"SoftFoot"}, other_name + "_convex_display");
  ctl.gui()->removeElement({"SoftFoot"}, name + "_convex_display");
}

void SoftFootState::rightFRSCallback(const std_msgs::Float64::ConstPtr & data)
{
  // Select data/string based on current_moving_foot
  // const std::string sensor_name = range_sensor_names_[Foot::Right];
  // const std::lock_guard<std::mutex> lock(range_sensor_mutex_);
  // controller().robot().device<mc_mujoco::RangeSensor>(sensor_name).update(data->data * 0.001, time_);
}

void SoftFootState::leftFRSCallback(const std_msgs::Float64::ConstPtr & data)
{
  // Select data/string based on current_moving_foot
  // const std::string sensor_name = range_sensor_names_[Foot::Left];
  // const std::lock_guard<std::mutex> lock(range_sensor_mutex_);
  // controller().robot().device<mc_mujoco::RangeSensor>(sensor_name).update(data->data * 0.001, time_);
}

void SoftFootState::airPressureCallback(const std_msgs::Float64MultiArray::ConstPtr & data)
{
  double ML = data->data[0];
  double MR = data->data[1];
  double D = data->data[2];
  ML_ = ML;
  MR_ = MR;
  D_ = D;
}

} // namespace lipm_walking::states

EXPORT_SINGLE_STATE("LIPMWalking::SoftFoot", lipm_walking::states::SoftFootState)
