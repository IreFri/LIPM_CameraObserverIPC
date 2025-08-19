/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <vector>
#include <string>
#include <exception>
#include <fstream>
#include <memory>
#include <thread>

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rtc/gui.h>
#include <mc_rtc/log/FlatLog.h>
#include <mc_rtc/logging.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rbdyn/Robots.h>

#include <mc_mujoco/devices/RangeSensor.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

struct LogRobot
{
  struct Configuration
  {
    /** RobotModule used for robots initialization */
    mc_rbdyn::RobotModulePtr rm;
    /** Timestep of the log */
    double dt;
    /** Identifier and TF prefix */
    std::string id;
    /** Configuration entry in the log */
    std::string configuration;
    /** Encoders entry in the log */
    std::string encoders;
    /** PTransfomd entry used for floating-base estimation */
    std::string base;
    /** If not empty, use this as floating-base rotation */
    std::string base_rotation;
    /** If not empty, use this as floating-base translation */
    std::string base_translation;
    /** If true, interpret base_rotation as an IMU reading */
    bool base_rotation_is_imu = false;
  };

  /** Create a LogRobot with the provided configuration */
  LogRobot(const Configuration & config)
  : config_(config), robots_(mc_rbdyn::loadRobot(*config.rm))
  {
  }


  /** Update the published robot state */
  void update(const mc_rtc::log::FlatLog & log, size_t i)
  {
    q = log.get(config_.configuration, i, q);
    encoders = log.get(config_.encoders, i, encoders);
    if(config_.base.size())
    {
      base = log.get(config_.base, i, base);
    }
    if(config_.base_translation.size())
    {
      base.translation() = log.get(config_.base_translation, i, base.translation());
    }
    if(config_.base_rotation.size())
    {
      if(config_.base_rotation_is_imu)
      {
        if(i == 0)
        {
          X_imu0_0 = {log.get<Eigen::Quaterniond>(config_.base_rotation, i, {1, 0, 0, 0})};
          X_imu0_0 = X_imu0_0.inv();
        }
        sva::PTransformd X_0_imu = {log.get<Eigen::Quaterniond>(config_.base_rotation, i, {1, 0, 0, 0})};
        base.rotation() = (X_0_imu * X_imu0_0).rotation();
      }
      else
      {
        base.rotation() = Eigen::Matrix3d(log.get(config_.base_rotation, i, Eigen::Quaterniond(base.rotation())));
      }
    }
    auto & robot = robots_->robot();
    for(size_t refIdx = 0; refIdx < robot.refJointOrder().size(); ++refIdx)
    {
      const auto & jN = robot.refJointOrder()[refIdx];
      if(robot.hasJoint(jN))
      {
        auto jIndex = robot.jointIndexByName(jN);
        if(robot.mbc().q[jIndex].size() == 1)
        {
          robot.mbc().q[jIndex][0] = q[refIdx];
        }
      }
    }
    robot.posW(base);
  }

  /** Access underlying robot */
  inline const mc_rbdyn::Robot & robot() const
  {
    return robots_->robot();
  }

private:
  Configuration config_;
  std::shared_ptr<mc_rbdyn::Robots> robots_;
  sva::PTransformd base;
  std::vector<double> q;
  std::vector<double> encoders;
  // Inverse of initial IMU orientation
  sva::PTransformd X_imu0_0;
};


struct LogExplorer
{
public:
  LogExplorer(const std::string & logfile, mc_rbdyn::RobotModulePtr mod, double dt)
  : mod_(mod), dt_(dt)
  {
    log_.load(logfile);
    if(log_.has("t"))
    {
      const auto & t = log_.getRaw<double>("t");
      if(t.size() > 2 && t[0] && t[1])
      {
        dt_ = *t[1] - *t[0];
        mc_rtc::log::info("[LogExplorer] Log timestep: {}", dt_);
      }
    }

    LogRobot::Configuration conf;
    conf.rm = mod;
    conf.dt = dt_;
    conf.encoders = "qIn";

    // Only use real roobt
    {
      conf.id = "real";
      conf.configuration = "qIn";
      if(log_.has("realRobot_posW"))
      {
        conf.base = "realRobot_posW";
        robot_.reset(new LogRobot(conf));
      }
      else if(log_.has("rpyIn"))
      {
        conf.base = "ff";
        conf.base_rotation = "rpyIn";
        conf.base_rotation_is_imu = true;
        robot_.reset(new LogRobot(conf));
      }
      else
      {
        robot_.reset(nullptr);
      }
    }
  }

  bool setupTimeSection(const std::string & t)
  {
    size_t i = 0;
    double t_i = 0;
    double t_ii = 0;
    do
    {
      t_i = log_.get<double>(t, i, 0);
      t_ii = log_.get<double>(t, i + 1, 0);
    } while(t_i == t_ii && ++i < log_.size() - 1);
    if(i + 1 == log_.size())
    {
      return false;
    }
    if(fabs(t_ii - t_i - dt_) > 1e-6)
    {
      i = i + 1;
    }
    size_t start_i = i;
    do
    {
      t_i = log_.get<double>(t, i, 0);
      t_ii = log_.get<double>(t, i + 1, 0);
    } while(fabs(t_ii - t_i - dt_) < 1e-6 && ++i < log_.size() - 1);
    if(i == start_i)
    {
      return false;
    }
    min_i_ = start_i;
    cur_i_ = min_i_;
    max_i_ = i;
    min_t_ = log_.get<double>(t, min_i_, 0);
    cur_t_ = min_t_;
    max_t_ = log_.get<double>(t, max_i_, 0);
    t_data = t;

    return true;
  }

  bool run()
  {
    robot_->update(log_, cur_i_);

    if(cur_i_ >= max_i_)
    {
      return false;
    }
    if(cur_i_ < min_i_)
    {
      cur_i_ = max_i_;
    }
    cur_t_ = log_.get("t", cur_i_, cur_t_);

    ++ cur_i_;

    return true;
  }

  const size_t& cur_i() const
  {
    return cur_i_;
  }

  const mc_rtc::log::FlatLog& log() const
  {
    return log_;
  }

  inline const mc_rbdyn::Robot & robot() const
  {
    return robot_->robot();
  }

public:
  std::shared_ptr<mc_rbdyn::RobotModule> mod_;

  /** Log data */
  mc_rtc::log::FlatLog log_;

  /** Time parameters */
  double dt_;

  /* min/max/current playback index/time */
  double min_t_ = 0;
  size_t min_i_ = 0;
  double max_t_ = 0;
  size_t max_i_ = 0;
  double cur_t_ = 0;
  size_t cur_i_ = 0;

  /* Current time data */
  std::string t_data;

  std::unique_ptr<LogRobot> robot_;
};


int main(int argc, char * argv[])
{
  if(argc == 1 || argc > 3)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("It requires one or two arguments: /path/to/bin [optionnal] name of time section to use");
  }

  const std::string log_path(argv[1]);

  std::string time_section_name("t");
  if(argc == 3)
  {
    time_section_name = std::string(argv[2]);
  }

  mc_rbdyn::RobotModulePtr mod;
  {
    std::vector<std::string> robot_params = {"HRP4CR"};
    if(robot_params.size() == 1)
    {
      mod = mc_rbdyn::RobotLoader::get_robot_module(robot_params[0]);
    }
    else if(robot_params.size() == 2)
    {
      mod = mc_rbdyn::RobotLoader::get_robot_module(robot_params[0], robot_params[1]);
    }
    else if(robot_params.size() == 3)
    {
      mod = mc_rbdyn::RobotLoader::get_robot_module(robot_params[0], robot_params[1], robot_params[2]);
    }
    else
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("log_visualization cannot handle the robot_params it was given");
    }
  }

  mc_rtc::log::info("Replaying log_: {}", log_path);
  double dt = 0.005;

  // What do we want to compute
  std::vector<Eigen::Vector3d> left_points;
  std::vector<Eigen::Vector3d> right_points;

  // Define what we want to log
  using Policy = mc_rtc::Logger::Policy;
  mc_rtc::Logger logger(Policy::THREADED, "/tmp", "export");
  logger.start("data", dt);

  logger.addLogEntry("left_points_x",
    [&left_points]
    {
      std::vector<double> x;
      std::transform(left_points.begin(), left_points.end(), std::back_inserter(x),
        [](const Eigen::Vector3d & left_p) { return left_p.x(); });
      return x;
    });

  logger.addLogEntry("left_points_y",
    [&left_points]
    {
      std::vector<double> y;
      std::transform(left_points.begin(), left_points.end(), std::back_inserter(y),
        [](const Eigen::Vector3d & left_p) { return left_p.y(); });
      return y;
    });

  logger.addLogEntry("left_points_z",
    [&left_points]
    {
      std::vector<double> z;
      std::transform(left_points.begin(), left_points.end(), std::back_inserter(z),
        [](const Eigen::Vector3d & left_p) { return left_p.z(); });
      return z;
    });

  logger.addLogEntry("right_points_x",
    [&right_points]
    {
      std::vector<double> x;
      std::transform(right_points.begin(), right_points.end(), std::back_inserter(x),
        [](const Eigen::Vector3d & right_p) { return right_p.x(); });
      return x;
    });

  logger.addLogEntry("right_points_y",
    [&right_points]
    {
      std::vector<double> y;
      std::transform(right_points.begin(), right_points.end(), std::back_inserter(y),
        [](const Eigen::Vector3d & right_p) { return right_p.y(); });
      return y;
    });

  logger.addLogEntry("right_points_z",
    [&right_points]
    {
      std::vector<double> z;
      std::transform(right_points.begin(), right_points.end(), std::back_inserter(z),
        [](const Eigen::Vector3d & right_p) { return right_p.z(); });
      return z;
    });

  sva::PTransformd right_X_0_ph = sva::PTransformd::Identity();
  sva::PTransformd right_X_ph_s = sva::PTransformd::Identity();
  sva::PTransformd right_X_0_s = sva::PTransformd::Identity();
  sva::PTransformd right_X_0_f = sva::PTransformd::Identity();
  sva::PTransformd right_X_0_fc = sva::PTransformd::Identity();

  sva::PTransformd left_X_0_ph = sva::PTransformd::Identity();
  sva::PTransformd left_X_ph_s = sva::PTransformd::Identity();
  sva::PTransformd left_X_0_s = sva::PTransformd::Identity();
  sva::PTransformd left_X_0_f = sva::PTransformd::Identity();
  sva::PTransformd left_X_0_fc = sva::PTransformd::Identity();

  logger.addLogEntry("right_X_0_ph", [&right_X_0_ph]{ return right_X_0_ph; });
  logger.addLogEntry("right_X_ph_s", [&right_X_ph_s]{ return right_X_ph_s; });
  logger.addLogEntry("right_X_0_s", [&right_X_0_s]{ return right_X_0_s; });
  logger.addLogEntry("right_X_0_f", [&right_X_0_f]{ return right_X_0_f; });
  // logger.addLogEntry("right_X_0_fcontinuous", [&right_X_0_fc]{ return right_X_0_fc; });

  logger.addLogEntry("left_X_0_ph", [&left_X_0_ph]{ return left_X_0_ph; });
  logger.addLogEntry("left_X_ph_s", [&left_X_ph_s]{ return left_X_ph_s; });
  logger.addLogEntry("left_X_0_s", [&left_X_0_s]{ return left_X_0_s; });
  logger.addLogEntry("left_X_0_f", [&left_X_0_f]{ return left_X_0_f; });
  // logger.addLogEntry("left_X_0_fcontinuous", [&left_X_0_fc]{ return left_X_0_fc; });


  // Replay
  LogExplorer appli(log_path, mod, dt);
  appli.setupTimeSection(time_section_name);

  std::vector<double> left_previous_points_x;
  std::vector<double> left_previous_points_y;
  std::vector<double> left_previous_points_z;

  std::vector<double> right_previous_points_x;
  std::vector<double> right_previous_points_y;
  std::vector<double> right_previous_points_z;

  std::vector<double> left_points_x = {0.};
  std::vector<double> left_points_y = {0.};
  std::vector<double> left_points_z = {0.};

  std::vector<double> right_points_x = {0.};
  std::vector<double> right_points_y = {0.};
  std::vector<double> right_points_z = {0.};

  logger.addLogEntry("right_camera_points_x", [&right_points_x]{ return right_points_x; });
  logger.addLogEntry("right_camera_points_y", [&right_points_y]{ return right_points_y; });
  logger.addLogEntry("right_camera_points_z", [&right_points_z]{ return right_points_z; });

  logger.addLogEntry("left_camera_points_x", [&left_points_x]{ return left_points_x; });
  logger.addLogEntry("left_camera_points_y", [&left_points_y]{ return left_points_y; });
  logger.addLogEntry("left_camera_points_z", [&left_points_z]{ return left_points_z; });

  while(true)
  {
    if(!appli.run())
    {
      break;
    }
    // Do computations
    const auto& robot = appli.robot();
    const auto& log = appli.log();
    const auto& cur_i = appli.cur_i();

    // Read the log
    std::vector<double> left_points_x = log.get<std::vector<double>>("Observers_LIPMWalkingObserverPipeline_CameraLeft_points_x", cur_i, {});
    std::vector<double> left_points_y = log.get<std::vector<double>>("Observers_LIPMWalkingObserverPipeline_CameraLeft_points_y", cur_i, {});
    std::vector<double> left_points_z = log.get<std::vector<double>>("Observers_LIPMWalkingObserverPipeline_CameraLeft_points_z", cur_i, {});

    std::vector<double> right_points_x = log.get<std::vector<double>>("Observers_LIPMWalkingObserverPipeline_CameraRight_points_x", cur_i, {});
    std::vector<double> right_points_y = log.get<std::vector<double>>("Observers_LIPMWalkingObserverPipeline_CameraRight_points_y", cur_i, {});
    std::vector<double> right_points_z = log.get<std::vector<double>>("Observers_LIPMWalkingObserverPipeline_CameraRight_points_z", cur_i, {});


    // LEFT FOOT
    // Initialize previous points
    if(left_previous_points_x.empty() || left_points_x.empty())
    {
      left_previous_points_x = left_points_x;
      left_previous_points_y = left_points_y;
      left_previous_points_z = left_points_z;
      continue;
    }

    left_X_0_fc = robot.surfacePose("LeftFootCenter");

    // Check if new data
    if(left_previous_points_x.front() != left_points_x.front())
    {
      left_previous_points_x = left_points_x;
      left_previous_points_y = left_points_y;
      left_previous_points_z = left_points_z;

      const std::string& left_body_of_sensor = robot.device<mc_mujoco::RangeSensor>("LeftFootCameraSensor").parent();
      // Access the position of body name in world coordinates (phalanx position)
      left_X_0_ph = robot.bodyPosW(left_body_of_sensor);
      // Returns the transformation from the parent body to the sensor
      left_X_ph_s = robot.device<mc_mujoco::RangeSensor>("LeftFootCameraSensor").X_p_s();
      left_X_0_f = robot.surfacePose("LeftFootCenter");
      //
      if(log.get<sva::ForceVecd>("LeftFootForceSensor", cur_i, sva::ForceVecd::Zero()).force().z() < 20)
      {

      }
      else
      {
        // We are in Support and we cancel the rotation of the foot !
        // TODO: Need to add this to the controller to in SoftFootState.cpp
        left_X_0_ph.rotation() = Eigen::Matrix3d::Identity();
      }

      // Keep the sensor pose when we receive a new set of data
      left_X_0_s = left_X_ph_s * left_X_0_ph;
    }

    // Compute the 3D point in world frame
    left_points.clear();
    for(size_t i = 0; i < left_points_x.size(); ++i)
    {
      Eigen::Vector3d left_pos(left_points_x[i], left_points_y[i], left_points_z[i]);
      // From camera frame to world frame
      const sva::PTransformd left_X_0_m = sva::PTransformd(left_pos) * left_X_0_s;
      left_points.push_back(left_X_0_m.translation());
    }


    // RIGHT FOOT
    // Initialize previous points
    if(right_previous_points_x.empty() || right_points_x.empty())
    {
      right_previous_points_x = right_points_x;
      right_previous_points_y = right_points_y;
      right_previous_points_z = right_points_z;
      continue;
    }

    right_X_0_fc = robot.surfacePose("RightFootCenter");

    // Check if new data
    if(right_previous_points_x.front() != right_points_x.front())
    {
      right_previous_points_x = right_points_x;
      right_previous_points_y = right_points_y;
      right_previous_points_z = right_points_z;

      const std::string& right_body_of_sensor = robot.device<mc_mujoco::RangeSensor>("RightFootCameraSensor").parent();
      // Access the position of body name in world coordinates (phalanx position)
      right_X_0_ph = robot.bodyPosW(right_body_of_sensor);
      // Returns the transformation from the parent body to the sensor
      right_X_ph_s = robot.device<mc_mujoco::RangeSensor>("RightFootCameraSensor").X_p_s();
      right_X_0_f = robot.surfacePose("RightFootCenter");
      //
      if(log.get<sva::ForceVecd>("RightFootForceSensor", cur_i, sva::ForceVecd::Zero()).force().z() < 20)
      {

      }
      else
      {
        // We are in Support and we cancel the rotation of the foot !
        // TODO: Need to add this to the controller to in SoftFootState.cpp
        right_X_0_ph.rotation() = Eigen::Matrix3d::Identity();
      }

      // Keep the sensor pose when we receive a new set of data
      right_X_0_s = right_X_ph_s * right_X_0_ph;
    }

    // Compute the 3D point in world frame
    right_points.clear();
    for(size_t i = 0; i < right_points_x.size(); ++i)
    {
      Eigen::Vector3d right_pos(right_points_x[i], right_points_y[i], right_points_z[i]);
      // From camera frame to world frame
      const sva::PTransformd right_X_0_m = sva::PTransformd(right_pos) * right_X_0_s;
      right_points.push_back(right_X_0_m.translation());
    }



    // Log
    logger.log();
  }

  mc_rtc::log::info("Done !");

  return 0;
}
