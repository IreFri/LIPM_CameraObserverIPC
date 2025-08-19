#include <lipm_walking/observer/CameraSensor.h>

#include <mc_control/MCController.h>
#include <mc_mujoco/devices/RangeSensor.h>
#include <mc_observers/ObserverMacros.h>
#include <mc_rtc/version.h>

#include <chrono>
#include <fstream>

namespace lipm_walking
{

CameraSensor::CameraSensor(const std::string & type, double dt) : mc_observers::Observer(type, dt) {}

void CameraSensor::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  data_ = CameraSensorShared::get(name_.c_str());
  if(data_->client.isAlive())
  {
    mc_rtc::log::error_and_throw("[CameraSensor::{}] Previous client (pid: {}) is still alive", name_,
                                 data_->client.pid);
  }
  data_->newClient(getpid());
  if(!data_->server.isAlive())
  {
    mc_rtc::log::critical("[CameraSensor::{}] Server is not running yet?", name_);
  }
  robot_name_ = config("robot", ctl.robot().name());

  if(config.has("sensor_name"))
  {
    sensor_name_ = static_cast<std::string>(config("sensor_name"));
  }
  else
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>(
        "[CameraSensor::{}] 'sensor_name' is mandatory in the configuration.", name_);
  }

  if(config.has("desired_state"))
  {
    // camera_serial_ = static_cast<std::string>(config("camera_serial"));
    desired_state_ = static_cast<std::string>(config("desired_state"));
  }
  else
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>(
        "[CameraSensor::{}] 'desired_state' is mandatory in the configuration; it can be 'LeftSwing' or 'RightSwing'",
        name_);
  }

  mc_rtc::log::info("[CameraSensor::{}] 'sensor_name' is {}", name_, sensor_name_);

  desc_ = fmt::format("{} (sensor_name={})", name_, sensor_name_);
}

void CameraSensor::reset(const mc_control::MCController & ctl)
{
  if(!estimation_loop_.joinable())
  {
    mc_rtc::log::info("[CameraSensor::{}] Starting estimation thread", name_);
    startGroundEstimation(const_cast<mc_control::MCController &>(ctl));
  }
  // Nothing to do here
}

bool CameraSensor::run(const mc_control::MCController &)
{
  // Nothing to do here

  return true;
}

void CameraSensor::updateServerOnline()
{
  bool was_online = serverOnline_;
  serverOnline_ = data_->server.isAlive();
  if(serverOnline_ && !was_online)
  {
    mc_rtc::log::success("[CameraSensor::{}] Server is back online", name_);
  }
  if(!serverOnline_ && was_online)
  {
    lastServerOfflineMessage_t_ = t_;
    mc_rtc::log::critical("[CameraSensor::{}] Server is offline", name_);
  }
  if(!serverOnline_ && t_ > lastServerOfflineMessage_t_ + 1.0)
  {
    lastServerOfflineMessage_t_ = t_;
    mc_rtc::log::critical("[CameraSensor::{}] Server is still offline", name_);
  }
}

void CameraSensor::update(mc_control::MCController & ctl)
{
  t_ += ctl.solver().dt();
  publish_plot(*ctl.gui());
  updateServerOnline();
  if(new_ground_data_)
  {
    ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->points_mtx);
    ground_points_.resize(data_->ground_points.size());
    for(size_t i = 0; i < data_->ground_points.size(); ++i)
    {
      ground_points_[i] = data_->ground_points[i];
    }
    // std::cout << "data_->aligned_points.size() " << data_->aligned_points.size() << std::endl;
    aligned_points_.resize(data_->aligned_points.size());
    for(size_t i = 0; i < data_->aligned_points.size(); ++i)
    {
      aligned_points_[i] = data_->aligned_points[i];
    }
    selected_points_.resize(data_->selected_points.size());
    for(size_t i = 0; i < data_->selected_points.size(); ++i)
    {
      selected_points_[i] = data_->selected_points[i];
    }
    if(!ground_points_.empty())
    {
      ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(sensor_name_).update(ground_points_, t_);
    }
    new_ground_data_ = false;
  }
}

void CameraSensor::publish_plot_data(std::vector<std::array<double, 2>> & points)
{
  points.reserve(ground_points_.size());
  for(const auto & p : ground_points_)
  {
    points.push_back({p.x(), p.z()});
  }
}

void CameraSensor::addToLogger(const mc_control::MCController &, mc_rtc::Logger & logger, const std::string & category)
{
  MC_RTC_LOG_HELPER(category + "_online", serverOnline_);
  logger.addLogEntry(category + "_range", this, [this]() -> double { return points_.empty() ? 0. : points_[0].z(); });
  logger.addLogEntry(category + "_camera_points_x", this,
                     [this]() -> std::vector<double>
                     {
                       std::vector<double> d;
                       std::transform(points_.begin(), points_.end(), std::back_inserter(d),
                                      [](const Eigen::Vector3d & v) { return v.x(); });
                       return d;
                     });
  logger.addLogEntry(category + "_camera_points_y", this,
                     [this]() -> std::vector<double>
                     {
                       std::vector<double> d;
                       std::transform(points_.begin(), points_.end(), std::back_inserter(d),
                                      [](const Eigen::Vector3d & v) { return v.y(); });
                       return d;
                     });
  logger.addLogEntry(category + "_camera_points_z", this,
                     [this]() -> std::vector<double>
                     {
                       std::vector<double> d;
                       std::transform(points_.begin(), points_.end(), std::back_inserter(d),
                                      [](const Eigen::Vector3d & v) { return v.z(); });
                       return d;
                     });
  logger.addLogEntry(category + "_ground_points_x", this,
                     [this]() -> std::vector<double>
                     {
                       std::vector<double> d;
                       std::transform(ground_points_.begin(), ground_points_.end(), std::back_inserter(d),
                                      [](const Eigen::Vector3d & v) { return v.x(); });
                       return d;
                     });
  logger.addLogEntry(category + "_ground_points_y", this,
                     [this]() -> std::vector<double>
                     {
                       std::vector<double> d;
                       std::transform(ground_points_.begin(), ground_points_.end(), std::back_inserter(d),
                                      [](const Eigen::Vector3d & v) { return v.y(); });
                       return d;
                     });
  logger.addLogEntry(category + "_ground_points_z", this,
                     [this]() -> std::vector<double>
                     {
                       std::vector<double> d;
                       std::transform(ground_points_.begin(), ground_points_.end(), std::back_inserter(d),
                                      [](const Eigen::Vector3d & v) { return v.z(); });
                       return d;
                     });
  logger.addLogEntry(category + "_world_points_x", this,
                     [this]() -> std::vector<double>
                     {
                       std::vector<double> d;
                       std::transform(points_.begin(), points_.end(), std::back_inserter(d),
                                      [this](const Eigen::Vector3d & v)
                                      {
                                        const sva::PTransformd X_s_p(v);
                                        const sva::PTransformd X_0_p = X_s_p * data_->X_b_s * data_->X_0_b;
                                        return X_0_p.translation().x();
                                      });
                       return d;
                     });
  logger.addLogEntry(category + "_world_points_y", this,
                     [this]() -> std::vector<double>
                     {
                       std::vector<double> d;
                       std::transform(points_.begin(), points_.end(), std::back_inserter(d),
                                      [this](const Eigen::Vector3d & v)
                                      {
                                        const sva::PTransformd X_s_p(v);
                                        const sva::PTransformd X_0_p = X_s_p * data_->X_b_s * data_->X_0_b;
                                        return X_0_p.translation().y();
                                      });
                       return d;
                     });
  logger.addLogEntry(category + "_world_points_z", this,
                     [this]() -> std::vector<double>
                     {
                       std::vector<double> d;
                       std::transform(points_.begin(), points_.end(), std::back_inserter(d),
                                      [this](const Eigen::Vector3d & v)
                                      {
                                        const sva::PTransformd X_s_p(v);
                                        const sva::PTransformd X_0_p = X_s_p * data_->X_b_s * data_->X_0_b;
                                        return X_0_p.translation().z();
                                      });
                       return d;
                     });
}

void CameraSensor::removeFromLogger(mc_rtc::Logger & logger, const std::string &)
{
  logger.removeLogEntries(this);
}

void CameraSensor::addToGUI(const mc_control::MCController & ctl,
                            mc_rtc::gui::StateBuilder & gui,
                            const std::vector<std::string> & category)
{
  gui.addElement(
      category,
      mc_rtc::gui::Checkbox(
          "Online", [this]() { return serverOnline_; }, []() {}),
      mc_rtc::gui::Button("Reset ground",
                          [this]()
                          {
                            ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->points_mtx);
                            data_->reset_ground = true;
                            ground_points_.clear();
                          }),
      mc_rtc::gui::Button("Show ground profile",
                          [this]()
                          {
                            if(publish_plot_ || published_plot_)
                            {
                              mc_rtc::log::warning("Already publishing");
                            }
                            publish_plot_ = true;
                          }),
      mc_rtc::gui::Label("Range [m]", [this]() { return points_.empty() ? 0. : points_[0].z(); }),
      mc_rtc::gui::Point3D("Point [3D]", [this]() { return points_.empty() ? Eigen::Vector3d::Zero() : points_[0]; }),
      mc_rtc::gui::Label("nr points", [this]() { return points_.size(); }),
      mc_rtc::gui::Transform(fmt::format("X_0_{}", name_),
                             [this, &ctl]()
                             {
                               const std::string & body_name =
                                   ctl.robot().device<mc_mujoco::RangeSensor>(sensor_name_).parent();
                               const sva::PTransformd X_0_p = ctl.realRobot().bodyPosW(body_name);
                               return ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(sensor_name_).X_p_s()
                                      * X_0_p;
                             }),
      mc_rtc::gui::ArrayInput(
          "Rotation [deg]", {"r", "p", "y"},
          [this, &ctl]() -> Eigen::Vector3d
          {
            return mc_rbdyn::rpyFromMat(
                       ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(sensor_name_).X_p_s().rotation())
                .unaryExpr([](double x) { return x * 180. / M_PI; });
          },
          [this, &ctl](const Eigen::Vector3d & new_rpy)
          {
            const sva::PTransformd current_X_p_s =
                ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(sensor_name_).X_p_s();
            const sva::PTransformd new_X_p_s(
                mc_rbdyn::rpyToMat(new_rpy.unaryExpr([](double x) { return x * M_PI / 180.; })),
                current_X_p_s.translation());
            const_cast<mc_control::MCController &>(ctl)
                .robot(robot_name_)
                .device<mc_mujoco::RangeSensor>(sensor_name_)
                .X_p_s(new_X_p_s);
          }),
      mc_rtc::gui::ArrayInput(
          "Translation", {"x", "y", "z"},
          [this, &ctl]()
          { return ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(sensor_name_).X_p_s().translation(); },
          [this, &ctl](const Eigen::Vector3d & new_translation)
          {
            const sva::PTransformd current_X_p_s =
                ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(sensor_name_).X_p_s();
            const sva::PTransformd new_X_p_s(current_X_p_s.rotation(), new_translation);
            const_cast<mc_control::MCController &>(ctl)
                .robot(robot_name_)
                .device<mc_mujoco::RangeSensor>(sensor_name_)
                .X_p_s(new_X_p_s);
          }),
      mc_rtc::gui::Transform(
          "X_p_s",
          [this, &ctl]()
          {
            const std::string & body_name = ctl.robot().device<mc_mujoco::RangeSensor>(sensor_name_).parent();
            const sva::PTransformd X_0_p = ctl.realRobot().bodyPosW(body_name);
            return ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(sensor_name_).X_p_s() * X_0_p;
          },
          [this, &ctl](const sva::PTransformd & X_0_s)
          {
            const std::string & body_name = ctl.robot().device<mc_mujoco::RangeSensor>(sensor_name_).parent();
            const sva::PTransformd X_0_p = ctl.realRobot().bodyPosW(body_name);
            const sva::PTransformd new_X_p_s = X_0_s * X_0_p.inv();
            const_cast<mc_control::MCController &>(ctl)
                .robot(robot_name_)
                .device<mc_mujoco::RangeSensor>(sensor_name_)
                .X_p_s(new_X_p_s);
          }));

  std::vector<std::string> points_category = category;
  points_category.push_back("points");
  // gui.addElement(
  //     points_category,
  //     mc_rtc::gui::Trajectory("Points",
  //                             [this, &ctl]()
  //                             {
  //                               std::vector<Eigen::Vector3d> points;
  //                               {
  //                                 points = points_;
  //                               }

  //                               if(points.empty())
  //                               {
  //                                 return std::vector<Eigen::Vector3d>{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  //                               }

  //                               const std::string & body_of_sensor =
  //                                   ctl.robot().device<mc_mujoco::RangeSensor>(sensor_name_).parent();
  //                               // Access the position of body name in world coordinates (phalanx position)
  //                               sva::PTransformd X_0_ph = ctl.realRobot().bodyPosW(body_of_sensor);
  //                               // Returns the transformation from the parent body to the sensor
  //                               const sva::PTransformd & X_ph_s =
  //                                   ctl.robot().device<mc_mujoco::RangeSensor>(sensor_name_).X_p_s();
  //                               // Keep the estimated 3d point for the ground
  //                               std::vector<Eigen::Vector3d> traj;
  //                               for(const auto & point : points)
  //                               {
  //                                 const sva::PTransformd X_0_m = sva::PTransformd(point) * X_ph_s * X_0_ph;
  //                                 traj.push_back(X_0_m.translation());
  //                               }
  //                               return traj;
  //                             }));

  gui.addElement(
      points_category,
      mc_rtc::gui::Trajectory("Ground reconstructed",
                              {mc_rtc::gui::Color::Magenta},
                              [this, &ctl]()
                              {
                                std::vector<Eigen::Vector3d> points;
                                {
                                  ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->points_mtx);
                                  points = ground_points_;
                                }

                                if(points.empty())
                                {
                                  return std::vector<Eigen::Vector3d>{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
                                }

                                return points;
                              }));
  gui.addElement(
      points_category,
      mc_rtc::gui::Trajectory("Selected points to align",
                              {mc_rtc::gui::Color::Blue},
                              [this, &ctl]()
                              {
                                std::vector<Eigen::Vector3d> points;
                                {
                                  ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->points_mtx);
                                  points = selected_points_;
                                }

                                if(points.empty())
                                {
                                  return std::vector<Eigen::Vector3d>{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
                                }

                                return points;
                              }));

  gui.addElement(
      points_category,
      mc_rtc::gui::Trajectory("Align points with picth and t_z",
                              {mc_rtc::gui::Color::Gray},
                              [this, &ctl]()
                              {
                                std::vector<Eigen::Vector3d> points;
                                {
                                  ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->points_mtx);
                                  points = aligned_points_;
                                }

                                if(points.empty())
                                {
                                  return std::vector<Eigen::Vector3d>{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
                                }

                                return points;
                              }));
}

void CameraSensor::publish_plot(mc_rtc::gui::StateBuilder & gui)
{
  auto plot_name = [this]() { return fmt::format("CameraSensor::{}::GroundProfile", name_); };
  if(published_plot_)
  {
    published_plot_ = false;
    gui.removePlot(plot_name());
  }
  if(publish_plot_)
  {
    gui.addXYPlot(plot_name(),
                  mc_rtc::gui::plot::XYChunk(
                      "profile", [this](auto & points) { publish_plot_data(points); }, mc_rtc::gui::Color::Red));
    publish_plot_ = false;
    published_plot_ = true;
  }
}

void CameraSensor::startGroundEstimation(mc_control::MCController & ctl)
{
  if(estimation_loop_.joinable())
  {
    estimation_loop_.join();
  }

  estimation_loop_ = std::thread(
      [this, &ctl]()
      {
        // Return the parent body of the sensor (phalanx)
        const std::string & body_of_sensor = ctl.robot().device<mc_mujoco::RangeSensor>(sensor_name_).parent();
        data_->X_b_s = ctl.robot().device<mc_mujoco::RangeSensor>(sensor_name_).X_p_s();

        while(!stop_loop_.load())
        {
          while(data_->server.isAlive())
          {
            ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->mtx);
            if(data_->data_ready->timed_wait(lck, pclock::universal_time() + ptime::seconds(1)))
            {
               {
                ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->points_mtx);
                // std::cout << "Getting data from the server " << std::endl;
                points_.resize(data_->points.size());
                // std::cout << "New points with size of " << points_.size() << std::endl;
                for(size_t i = 0; i < data_->points.size(); ++i)
                {
                  points_[i] = data_->points[i];
                }
              }
              data_->skip = false;
              if(ctl.datastore().has("SoftFootState::GetState"))
              {
                const std::string state = ctl.datastore().call<std::string>("SoftFootState::GetState");
                if(!(state == desired_state_ || state == "DoubleSupport"))
                {
                  data_->skip = true;
                }
              }
              else
              {
                data_->skip = true;
              }
            }
            if(data_->skip)
            {
              data_->compute_ready->notify_all();
              continue;
            }
            data_->fz = ctl.robot().bodyForceSensor(body_of_sensor).force().z();
            data_->X_0_b = ctl.realRobot().bodyPosW(body_of_sensor);
            data_->compute_ready->notify_all();
            if(data_->result_ready->timed_wait(lck, pclock::universal_time() + ptime::seconds(1)))
            {
              new_ground_data_ = true;
            }
          }
          data_->client_idle.notify_all();
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
      });
}

} // namespace lipm_walking
EXPORT_OBSERVER_MODULE("CameraSensor", lipm_walking::CameraSensor)
