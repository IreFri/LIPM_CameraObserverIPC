/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <vector>
#include <string>
#include <exception>
#include <fstream>
#include <memory>
#include <thread>
#include <deque>
#include <chrono>

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rtc/gui.h>
#include <mc_rtc/log/FlatLog.h>
#include <mc_rtc/logging.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rbdyn/Robots.h>

#include <mc_mujoco/devices/RangeSensor.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <open3d/Open3D.h>

namespace sva
{

using PTransformd = PTransform<double>;

}

namespace
{

/** We redefined sva::Rot. functions to make them work with non-scalar types */

template<typename T>
inline Eigen::Matrix3<T> RotX(T theta)
{
  T s = sin(theta);
  T c = cos(theta);
  return (Eigen::Matrix3<T>() << T(1), T(0), T(0), T(0), c, s, T(0), -s, c).finished();
}

template<typename T>
inline Eigen::Matrix3<T> RotY(T theta)
{
  T s = sin(theta);
  T c = cos(theta);
  return (Eigen::Matrix3<T>() << c, T(0), -s, T(0), T(1), T(0), s, T(0), c).finished();
}

template<typename T>
inline Eigen::Matrix3<T> RotZ(T theta)
{
  T s = sin(theta);
  T c = cos(theta);
  return (Eigen::Matrix3<T>() << c, s, T(0), -s, c, T(0), T(0), T(0), T(1)).finished();
}

template<typename T>
Eigen::Matrix3<T> rpyToMat(const T & r, const T & p, const T & y)
{
  return RotX<T>(r) * RotY<T>(p) * RotZ<T>(y);
}

struct PitchZCostFunctor
{
  PitchZCostFunctor(const sva::PTransformd & X_s_p, const sva::PTransformd & X_0_b, const sva::PTransformd & X_b_s)
  : X_s_p_(X_s_p), X_0_b_(X_0_b), X_b_s_(X_b_s)
  {
  }

  template<typename T>
  bool operator()(const T * const pitch, const T * const t_z, T * residual) const
  {
    sva::PTransform<T> X_0_b = X_0_b_;
    sva::PTransform<T> X_b_b(RotY(pitch[0]), Eigen::Vector3<T>(T(0), T(0), t_z[0]));
    sva::PTransform<T> X_b_s = X_b_s_;
    sva::PTransform<T> X_s_p = X_s_p_;

    sva::PTransform<T> X_0_p = (X_s_p * X_b_s * X_b_b * X_0_b);

    residual[0] = X_0_p.translation().z();

    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction * Create(const sva::PTransformd & X_s_p,
                                      const sva::PTransformd & X_0_b,
                                      const sva::PTransformd & X_b_s)
  {
    return (new ceres::AutoDiffCostFunction<PitchZCostFunctor, 1, 1, 1>(new PitchZCostFunctor(X_s_p, X_0_b, X_b_s)));
  }

private:
  const sva::PTransformd X_s_p_;
  const sva::PTransformd X_0_b_;
  const sva::PTransformd X_b_s_;
};

struct XCostFunctor
{
  XCostFunctor(const Eigen::Vector3d & p, const std::vector<Eigen::Vector3d> & obstacles)
  : p_(p),
    obstacles_(obstacles)
  {
  }

  template<typename T>
  bool operator()(const T * const t_x, T * residual) const
  {
    const Eigen::Vector3<T> translation(t_x[0], T(0.), T(0.));
    const Eigen::Vector3<T> new_p = p_.cast<T>() + translation;

    T min_dist = T(1000.);
    for(const auto & point: obstacles_)
    {
      T dist = (point.cast<T>() - new_p).norm();
      if(dist < min_dist)
      {
        min_dist = dist;
      }
    }

    // residual[0] = T(1000.);

    // if(min_dist < T(0.05))
    {
      residual[0] = min_dist;
    }


    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction * Create(const Eigen::Vector3d & p,
                                      const std::vector<Eigen::Vector3d> & obstacles)
  {
    return (new ceres::AutoDiffCostFunction<XCostFunctor, 1, 1>(new XCostFunctor(p, obstacles)));
  }

private:
  const Eigen::Vector3d p_;
  const std::vector<Eigen::Vector3d> obstacles_;
};

}

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
    /** PTransfomd entry used for doubleing-base estimation */
    std::string base;
    /** If not empty, use this as doubleing-base rotation */
    std::string base_rotation;
    /** If not empty, use this as doubleing-base translation */
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

    // Only use control roobt
    // {
    //   conf.id = "control";
    //   conf.configuration = "qOut";
    //   conf.base = "ff";
    //   robot_.reset(new LogRobot(conf));
    // }
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

struct FootDataHolder
{
public:
  // sva::PTransformd
  sva::PTransformd X_0_f = sva::PTransformd::Identity();
  sva::PTransformd X_0_s = sva::PTransformd::Identity();
  sva::PTransformd X_0_b = sva::PTransformd::Identity();
  sva::PTransformd _X_0_b = sva::PTransformd::Identity();
  sva::PTransformd X_b_s = sva::PTransformd::Identity();

  // Robot relative name
  std::string surface_name;
  std::string force_sensor_name;
  std::string camera_sensor_name;

  // Log relative name
  std::string path_to_points;
  std::string path_to_ground_points;

  // Data
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector3d> icp_points;
  std::vector<Eigen::Vector3d> last_icp_points;
  std::vector<Eigen::Vector3d> previous_points;
  std::vector<Eigen::Vector3d> camera_points;
  std::vector<Eigen::Vector3d> corrected_points;
  std::vector<std::vector<Eigen::Vector3d>> ground_points_grouped_;

  std::vector<Eigen::Vector3d> new_camera_points_;
  std::vector<Eigen::Vector3d> new_ground_points_;
  std::vector<Eigen::Vector3d> pre_new_camera_points_;
  std::vector<Eigen::Vector3d> pre_new_ground_points_;
  std::vector<Eigen::Vector3d> ground_points_;
  std::vector<Eigen::Vector3d> corrected_ground_points_;
  std::vector<Eigen::Vector3d> logged_ground_points_;
  std::vector<Eigen::Vector3d> historic_points_;
  std::deque<std::vector<Eigen::Vector3d>> live_ground_points_;

  std::vector<double> points_x;
  std::vector<double> points_y;
  std::vector<double> points_z;

  std::vector<double> previous_points_x;
  std::vector<double> previous_points_y;
  std::vector<double> previous_points_z;

  // For Open3D ICP
  std::shared_ptr<open3d::geometry::PointCloud> source;
  std::shared_ptr<open3d::geometry::PointCloud> source_transformed;
  std::shared_ptr<open3d::geometry::PointCloud> reference_source;


  // For Ceres
  double pitch_ = 0.;
  double t_z_ = 0.;

  std::vector<double> corrected_points_x;
  std::vector<double> corrected_points_y;
  std::vector<double> corrected_points_z;

  std::vector<double> every_foot_t_z;
  std::vector<double> every_pitch;
  std::vector<double> every_t_z;

  // For plotting
  std::vector<double> last_points_x;
  std::vector<double> last_points_y;
  std::vector<double> last_points_z;

  std::vector<double> last_icp_points_x;
  std::vector<double> last_icp_points_y;
  std::vector<double> last_icp_points_z;

  std::vector<double> avg_points_x;
  std::vector<double> avg_points_y;
  std::vector<double> avg_points_z;

  std::vector<double> foot_x;
  std::vector<double> foot_y;
  std::vector<double> foot_z;

  std::vector<double> icp_points_x;
  std::vector<double> icp_points_y;
  std::vector<double> icp_points_z;
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
  double dt = 0.005f;

  // What do we want to compute
  FootDataHolder right_foot;
  right_foot.surface_name = "RightFootCenter";
  right_foot.camera_sensor_name = "RightFootCameraSensor";
  right_foot.force_sensor_name = "RightFootForceSensor";
  right_foot.path_to_points = "Observers_LIPMWalkingObserverPipeline_CameraRight_camera_points";
  right_foot.path_to_ground_points = "Observers_LIPMWalkingObserverPipeline_CameraRight_ground_points";
  right_foot.source = std::make_shared<open3d::geometry::PointCloud>();
  right_foot.source_transformed = std::make_shared<open3d::geometry::PointCloud>();
  right_foot.reference_source = std::make_shared<open3d::geometry::PointCloud>();

  FootDataHolder left_foot;
  left_foot.surface_name = "LeftFootCenter";
  left_foot.camera_sensor_name = "LeftFootCameraSensor";
  left_foot.force_sensor_name = "LeftFootForceSensor";
  left_foot.path_to_points = "Observers_LIPMWalkingObserverPipeline_CameraLeft_camera_points";
  left_foot.path_to_ground_points = "Observers_LIPMWalkingObserverPipeline_CameraLeft_ground_points";
  left_foot.source = std::make_shared<open3d::geometry::PointCloud>();
  left_foot.source_transformed = std::make_shared<open3d::geometry::PointCloud>();
  left_foot.reference_source = std::make_shared<open3d::geometry::PointCloud>();

  // Replay
  LogExplorer appli(log_path, mod, dt);

  // FUNCTIONS
  auto fill = [](std::shared_ptr<open3d::geometry::PointCloud> & pc, const std::vector<Eigen::Vector3d> & data)
  {
    pc->points_.resize(data.size());
    for(size_t i = 0; i < data.size(); ++i)
    {
      pc->points_[i].x() = data[i].x();
      pc->points_[i].y() = 0.;
      pc->points_[i].z() = data[i].z();
    }
  };

  auto updateDataForPlot = [](FootDataHolder& foot_holder)
  {
    double x = std::accumulate(foot_holder.points.begin(), foot_holder.points.end(), 0.f, [](double a, const Eigen::Vector3d& v) { return a + v.x(); });
    double y = std::accumulate(foot_holder.points.begin(), foot_holder.points.end(), 0.f, [](double a, const Eigen::Vector3d& v) { return a + v.y(); });
    double z = std::accumulate(foot_holder.points.begin(), foot_holder.points.end(), 0.f, [](double a, const Eigen::Vector3d& v) { return a + v.z(); });

    foot_holder.avg_points_x.push_back(x / static_cast<double>(foot_holder.points.size()));
    foot_holder.avg_points_y.push_back(y / static_cast<double>(foot_holder.points.size()));
    foot_holder.avg_points_z.push_back(z / static_cast<double>(foot_holder.points.size()));

    std::transform(foot_holder.camera_points.begin(), foot_holder.camera_points.end(), std::back_inserter(foot_holder.corrected_points_x), [](const Eigen::Vector3d & p) { return p.x(); });
    std::transform(foot_holder.camera_points.begin(), foot_holder.camera_points.end(), std::back_inserter(foot_holder.corrected_points_y), [](const Eigen::Vector3d & p) { return p.y(); });
    std::transform(foot_holder.camera_points.begin(), foot_holder.camera_points.end(), std::back_inserter(foot_holder.corrected_points_z), [](const Eigen::Vector3d & p) { return p.z(); });

    std::transform(foot_holder.points.begin(), foot_holder.points.end(), std::back_inserter(foot_holder.points_x), [](const Eigen::Vector3d & p) { return p.x(); });
    std::transform(foot_holder.points.begin(), foot_holder.points.end(), std::back_inserter(foot_holder.points_y), [](const Eigen::Vector3d & p) { return p.y(); });
    std::transform(foot_holder.points.begin(), foot_holder.points.end(), std::back_inserter(foot_holder.points_z), [](const Eigen::Vector3d & p) { return p.z(); });

    foot_holder.last_points_x.clear();
    foot_holder.last_points_y.clear();
    foot_holder.last_points_z.clear();
    std::transform(foot_holder.points.begin(), foot_holder.points.end(), std::back_inserter(foot_holder.last_points_x), [](const Eigen::Vector3d & p) { return p.x(); });
    std::transform(foot_holder.points.begin(), foot_holder.points.end(), std::back_inserter(foot_holder.last_points_y), [](const Eigen::Vector3d & p) { return p.y(); });
    std::transform(foot_holder.points.begin(), foot_holder.points.end(), std::back_inserter(foot_holder.last_points_z), [](const Eigen::Vector3d & p) { return p.z(); });

    foot_holder.last_icp_points_x.clear();
    foot_holder.last_icp_points_y.clear();
    foot_holder.last_icp_points_z.clear();
    std::transform(foot_holder.last_icp_points.begin(), foot_holder.last_icp_points.end(), std::back_inserter(foot_holder.last_icp_points_x), [](const Eigen::Vector3d & p) { return p.x(); });
    std::transform(foot_holder.last_icp_points.begin(), foot_holder.last_icp_points.end(), std::back_inserter(foot_holder.last_icp_points_y), [](const Eigen::Vector3d & p) { return p.y(); });
    std::transform(foot_holder.last_icp_points.begin(), foot_holder.last_icp_points.end(), std::back_inserter(foot_holder.last_icp_points_z), [](const Eigen::Vector3d & p) { return p.z(); });

    std::transform(foot_holder.icp_points.begin(), foot_holder.icp_points.end(), std::back_inserter(foot_holder.icp_points_x), [](const Eigen::Vector3d & p) { return p.x(); });
    std::transform(foot_holder.icp_points.begin(), foot_holder.icp_points.end(), std::back_inserter(foot_holder.icp_points_y), [](const Eigen::Vector3d & p) { return p.y(); });
    std::transform(foot_holder.icp_points.begin(), foot_holder.icp_points.end(), std::back_inserter(foot_holder.icp_points_z), [](const Eigen::Vector3d & p) { return p.z(); });

    foot_holder.foot_x.push_back(foot_holder.X_0_b.translation().x());
    foot_holder.foot_y.push_back(foot_holder.X_0_b.translation().y());
    foot_holder.foot_z.push_back(foot_holder.X_0_b.translation().z());
  };

  auto extract = [&appli](FootDataHolder& foot_holder, size_t cur_i_offset = 0) -> void
  {

    const auto& log = appli.log();
    const auto& cur_i = appli.cur_i();
    appli.robot_->update(log, cur_i - cur_i_offset);
    const auto& robot = appli.robot();

    foot_holder.X_0_f = robot.surfacePose(foot_holder.surface_name);

    std::vector<double> _points_x = log.get<std::vector<double>>(foot_holder.path_to_points + "_x", cur_i, {});
    std::vector<double> _points_y = log.get<std::vector<double>>(foot_holder.path_to_points + "_y", cur_i, {});
    std::vector<double> _points_z = log.get<std::vector<double>>(foot_holder.path_to_points + "_z", cur_i, {});

    std::vector<double> points_x;
    std::transform(_points_x.begin(), _points_x.end(), std::back_inserter(points_x), [](double v) { return static_cast<double>(v); });

    std::vector<double> points_y;

    std::transform(_points_y.begin(), _points_y.end(), std::back_inserter(points_y), [](double v) { return static_cast<double>(v); });
    std::vector<double> points_z;

    std::transform(_points_z.begin(), _points_z.end(), std::back_inserter(points_z), [](double v) { return static_cast<double>(v); });

    // Initialize previous points
    if(foot_holder.previous_points_x.empty() || points_x.empty())
    {
      foot_holder.previous_points_x = points_x;
      foot_holder.previous_points_y = points_y;
      foot_holder.previous_points_z = points_z;
      return;
    }

    // Check if new data
    if(foot_holder.previous_points_x.front() != points_x.front())
    {
      foot_holder.previous_points_x = points_x;
      foot_holder.previous_points_y = points_y;
      foot_holder.previous_points_z = points_z;

      const std::string& body_of_sensor = robot.device<mc_mujoco::RangeSensor>(foot_holder.camera_sensor_name).parent();
      // Access the position of body name in world coordinates (balanx position)
      foot_holder.X_0_b = robot.bodyPosW(body_of_sensor);
      // Returns the transformation from the parent body to the sensor
      foot_holder.X_b_s = robot.device<mc_mujoco::RangeSensor>(foot_holder.camera_sensor_name).X_p_s();

      // Keep the sensor pose when we receive a new set of data
      foot_holder.X_0_s = foot_holder.X_b_s * foot_holder.X_0_b;
    }
    else
    {
      if(!foot_holder.points.empty())
      {
        foot_holder.previous_points = foot_holder.points;
      }
      foot_holder.points.clear();
      foot_holder.camera_points.clear();
      return;
    }

    foot_holder.points.clear();
    foot_holder.camera_points.clear();
    // Compute the 3D point in world frame
    for(size_t i = 0; i < points_x.size(); ++i)
    {
      Eigen::Vector3d pos(points_x[i], points_y[i], points_z[i]);
      // From camera frame to world frame
      const sva::PTransformd X_s_p(pos);
      const sva::PTransformd X_0_p = X_s_p * foot_holder.X_0_s;
      // if(X_s_p.translation().x() < 0.05 && X_s_p.translation().z() < 0.30)
      {
        foot_holder.camera_points.push_back(pos);
        foot_holder.points.push_back(X_0_p.translation());
      }
    }

    {
      std::vector<double> _points_x = log.get<std::vector<double>>(foot_holder.path_to_ground_points + "_x", cur_i, {});
      std::vector<double> _points_y = log.get<std::vector<double>>(foot_holder.path_to_ground_points + "_y", cur_i, {});
      std::vector<double> _points_z = log.get<std::vector<double>>(foot_holder.path_to_ground_points + "_z", cur_i, {});

      std::vector<double> points_x;
      std::transform(_points_x.begin(), _points_x.end(), std::back_inserter(points_x), [](double v) { return static_cast<double>(v); });

      std::vector<double> points_y;

      std::transform(_points_y.begin(), _points_y.end(), std::back_inserter(points_y), [](double v) { return static_cast<double>(v); });
      std::vector<double> points_z;

      std::transform(_points_z.begin(), _points_z.end(), std::back_inserter(points_z), [](double v) { return static_cast<double>(v); });

      foot_holder.logged_ground_points_.clear();
      // Compute the 3D point in world frame
      for(size_t i = 0; i < points_x.size(); ++i)
      {
        Eigen::Vector3d pos(points_x[i], points_y[i], points_z[i]);
        foot_holder.logged_ground_points_.push_back(pos);
      }
    }


    appli.robot_->update(log, cur_i);
  };

  open3d::visualization::Visualizer visualizer;
  if(!visualizer.CreateVisualizerWindow("Ground Estimation", 1280, 720, 50, 50))
  {
    return false;
  }

  visualizer.GetRenderOption().point_show_normal_ = false;
  visualizer.GetRenderOption().mesh_show_wireframe_ = false;
  visualizer.GetRenderOption().mesh_show_back_face_ = false;

  // // Open3D Rendering Loop
  visualizer.BuildUtilities();
  visualizer.UpdateWindowTitle();

  // Offset for viewing
  Eigen::Matrix4d plus_offset_view = Eigen::Matrix4d::Identity();
  plus_offset_view.block<3, 1>(0, 3) = Eigen::Vector3d(0., 0.005, 0.);

  Eigen::Matrix4d minus_offset_view = Eigen::Matrix4d::Identity();
  minus_offset_view.block<3, 1>(0, 3) = Eigen::Vector3d(0., -0.02, 0.);

  std::shared_ptr<open3d::geometry::PointCloud> reference_0(new open3d::geometry::PointCloud);
  for(size_t i = 0; i < 150; ++i)
  {
    reference_0->points_.push_back(Eigen::Vector3d(i * 0.01, 0., 0.));
  }

  reference_0->PaintUniformColor(Eigen::Vector3d(0., 1., 1.));
  // // visualizer.AddGeometry(reference_0);

  std::shared_ptr<open3d::geometry::PointCloud> foot_pose(new open3d::geometry::PointCloud);

  appli.setupTimeSection(time_section_name);

  size_t counter = 0;
  size_t step_counter = 0;
  while(true)
  {
    if(!appli.run())
    {
      break;
    }

    const auto& log = appli.log();
    const auto& cur_i = appli.cur_i();

    const std::string state = log.get<std::string>("Executor_LIPMWalking::WalkInternal", cur_i, "None");
    if(state == "None" || state == "LIPMWalking::Standing" )
    {
      continue;
    }

    // if(state == "None" || state == "LIPMWalking::Standing")
    // {
    //   continue;
    // }

    bool stop = false;

    // auto _foot_holder = std::reference_wrapper<FootDataHolder>{left_foot};
    // auto _foot_holder = std::reference_wrapper<FootDataHolder>{right_foot};
    for(auto& _foot_holder: std::vector<std::reference_wrapper<FootDataHolder>>{right_foot, left_foot})
    {
      auto& foot_holder = _foot_holder.get();

      // 1- Get the point as X_0_p
      extract(foot_holder, 0);

      if(foot_holder.camera_points.empty())
      {
        continue;
      }

      ++ counter;

      std::cout << "counter: " << counter << std::endl;

      // size_t choice_counter = 53;

      // if(counter == choice_counter)
      //   break;

      // if(counter < choice_counter - 3)
      // {
      //   continue;
      // }

      {
        foot_holder.pre_new_camera_points_ = foot_holder.camera_points;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //// plot_3d
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      auto plot_3d = [&] (const std::vector<Eigen::Vector3d> & vec, size_t nr)
      {
        static std::default_random_engine re;
        std::uniform_real_distribution<double> unif(0., 1.);

        size_t nr_plots = 15;
        std::shared_ptr<open3d::geometry::PointCloud> pc(new open3d::geometry::PointCloud);
        pc->points_ = vec;

        for(size_t o = 0; o < counter * nr_plots + nr; ++o)
        {
          pc->Transform(plus_offset_view);
          pc->Transform(plus_offset_view);
        }
        pc->PaintUniformColor(Eigen::Vector3d(unif(re), unif(re), unif(re)));
        // visualizer.AddGeometry(pc);
      };

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //// fromCameraToWorldFrameWithAlignement
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      auto fromCameraToWorldFrameWithAlignement = [&foot_holder](const std::vector<Eigen::Vector3d> & camera_points)
      {
        std::vector<Eigen::Vector3d> ground_points;

        for(int i = camera_points.size() - 1; i >= 0; --i)
        {
          const sva::PTransformd X_s_p(camera_points[i]);
          sva::PTransformd X_0_p = X_s_p * foot_holder.X_b_s * foot_holder._X_0_b;
          X_0_p.translation().y() = 0.;
          ground_points.push_back(X_0_p.translation());
        }


        std::shared_ptr<open3d::geometry::PointCloud> pc(new open3d::geometry::PointCloud);
        pc->points_ = ground_points;
        pc = pc->VoxelDownSample(0.003);
        ground_points = pc->points_;
        std::sort(ground_points.begin(), ground_points.end(), [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });
        return ground_points;
      };

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //// selectOnlyGrounds
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      auto selectOnlyGrounds = [&](const std::vector<Eigen::Vector3d> & ground_points, double threshold_deg, double threshold_z)
      {
        std::vector<Eigen::Vector3d> ground;
        std::vector<Eigen::Vector3d> not_ground;

        std::shared_ptr<open3d::geometry::PointCloud> pc(new open3d::geometry::PointCloud);
        pc->points_ = ground_points;
        for(const auto & p: ground_points)
        {
          pc->points_.push_back(Eigen::Vector3d(p.x(), 0.002, p.z() - 0.00025));
          pc->points_.push_back(Eigen::Vector3d(p.x(), -0.002, p.z() - 0.00025));
          pc->points_.push_back(Eigen::Vector3d(p.x(), 0.004, p.z() + 0.00025));
          pc->points_.push_back(Eigen::Vector3d(p.x(), -0.004, p.z() + 0.00025));
        }

        pc->EstimateNormals();

        if(pc->normals_.empty())
        {
          return ground;
        }

        try
        {
          pc->OrientNormalsTowardsCameraLocation(foot_holder._X_0_b.translation());
        }
        catch(const std::exception &)
        {
          return ground;
        }

        const double sum_z = std::accumulate(ground_points.begin(), ground_points.end(), 0., [](double a, const Eigen::Vector3d& v) { return a + v.z(); });
        const double mean_z = sum_z / static_cast<double>(ground_points.size() + 1);

        const auto n_z = Eigen::Vector3d::UnitZ();
        for(size_t i = 0; i < ground_points.size(); ++i)
        {
          const auto & n_0 = pc->normals_[i];
          const double angle = std::acos(n_0.dot(n_z) / (n_0.norm() * n_z.norm()));

          if(std::abs(angle * 180. / M_PI) < threshold_deg && pc->points_[i].z() < mean_z + threshold_z)
          {
            ground.push_back(pc->points_[i]);
          }
          else
          {
            not_ground.push_back(pc->points_[i]);
          }
        }

        std::sort(ground.begin(), ground.end(), [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });
        std::sort(not_ground.begin(), not_ground.end(), [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });

        std::vector<std::vector<Eigen::Vector3d>> group_ground;
        group_ground.push_back(std::vector<Eigen::Vector3d>{});
        if(!ground.empty())
        {
          group_ground.back().push_back(ground.front());
          for(size_t i = 1; i < ground.size(); ++i)
          {
            if((ground[i] - group_ground.back().back()).norm() > 0.005)
            {
              group_ground.push_back(std::vector<Eigen::Vector3d>{});
            }
            group_ground.back().push_back(ground[i]);
          }
        }

        std::vector<std::vector<Eigen::Vector3d>> group_not_ground;
        group_not_ground.push_back(std::vector<Eigen::Vector3d>{});

        if(!not_ground.empty())
        {
          group_not_ground.back().push_back(not_ground.front());
          for(size_t i = 1; i < not_ground.size(); ++i)
          {
            if((not_ground[i] - group_not_ground.back().back()).norm() > 0.005)
            {
              group_not_ground.push_back(std::vector<Eigen::Vector3d>{});
            }
            group_not_ground.back().push_back(not_ground[i]);
          }
        }

        if(!not_ground.empty() && !ground.empty())
        {
          // Case 0: We remove a ground group if it is just after a not ground group
          std::vector<size_t> idxs_to_remove;
          for(size_t i = 0; i < group_ground.size(); ++i)
          {
            for(size_t j = 0; j < group_not_ground.size(); ++j)
            {
              if((group_ground[i].front() - group_not_ground[j].back()).norm() < 0.005)
              {
                idxs_to_remove.push_back(i);
                break;
              }
            }
          }

          std::reverse(idxs_to_remove.begin(), idxs_to_remove.end());
          for(const auto & idx: idxs_to_remove)
          {
            group_ground.erase(group_ground.begin() + idx);
          }
        }


        for(const auto & group: group_ground)
        {
          plot_3d(group, 10);
        }

        for(const auto & group: group_not_ground)
        {
          plot_3d(group, 11);
        }

        ground.clear();
        for(const auto & group: group_ground)
        {
          if(!group.empty())
          {
            ground.insert(ground.end(), group.begin(), group.end());
          }
        }

        return ground;
      };

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //// selectObstacles
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      auto selectObstacles = [&](std::vector<Eigen::Vector3d> & ground_points, bool with_erase = false)
      {
        std::vector<int> idxs_to_erase;

        std::vector<Eigen::Vector3d> res;
        for(int i = 0; i < static_cast<int>(ground_points.size()) - 1; ++ i)
        {
          const auto & p = ground_points[i];
          if(p.z() > 0.003)
          {
            res.push_back(p);
            if(std::abs(ground_points[i + 1].x() - p.x()) > 0.02)
            {
              idxs_to_erase.push_back(i);
            }
          }
        }

        if(with_erase)
        {
          for(const auto & index: idxs_to_erase)
          {
            auto from = ground_points.begin() + index - 2;
            if(index - 2 < 0)
            {
              from = ground_points.begin();
            }
            auto to = ground_points.begin() + index;
            if(to == ground_points.begin())
            {
              ground_points.erase(ground_points.begin());
            }
            else
            {
              ground_points.erase(from, ground_points.begin() + index);
            }
          }
        }

        return res;
      };

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //// computePitchAndTzWithCeres
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      auto computePitchAndTzWithCeres = [&foot_holder](const std::vector<Eigen::Vector3d> & selected_line, double & pitch, double & t_z)
      {
        pitch = 0.;
        t_z = 0.;

        ceres::Problem problem;
        for(const auto& T_0_p: selected_line)
        {
          const sva::PTransformd X_0_p(T_0_p);
          const sva::PTransformd X_s_p = X_0_p * (foot_holder.X_b_s * foot_holder._X_0_b).inv();
          ceres::CostFunction* cost_function = PitchZCostFunctor::Create(X_s_p, foot_holder._X_0_b, foot_holder.X_b_s);
          problem.AddResidualBlock(cost_function,  new ceres::CauchyLoss(0.5), &pitch, &t_z);
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
      };

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //// reAlignGround
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      auto reAlignGround = [&foot_holder](const std::vector<Eigen::Vector3d> & ground_points, double pitch, double t_z)
      {
        std::vector<Eigen::Vector3d> _ground_points;
        const sva::PTransformd X_b_b(sva::RotY(pitch).cast<double>(), Eigen::Vector3d(0., 0., t_z));
        for(const auto& T_0_p: ground_points)
        {
          // From camera frame to world frame
          const sva::PTransformd _X_0_p(T_0_p);
          const sva::PTransformd X_s_p = _X_0_p * (foot_holder.X_b_s * foot_holder._X_0_b).inv();
          const sva::PTransformd X_0_p = X_s_p * foot_holder.X_b_s * X_b_b * foot_holder._X_0_b;
          _ground_points.push_back(X_0_p.translation());
        }

        return _ground_points;
      };

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //// cuttoffZ
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      auto cuttoffZ = [&foot_holder](const std::vector<Eigen::Vector3d> & ground_points)
      {
        std::vector<Eigen::Vector3d> _ground_points;
        for(const auto& T_0_p: ground_points)
        {
          if(T_0_p.z() > -0.0015)
          {
            _ground_points.push_back(T_0_p);

          }
        }
        return _ground_points;
      };

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //// denoise
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      auto denoise = [&](std::vector<Eigen::Vector3d> points)
      {
        std::function<std::vector<int>(const std::vector<Eigen::Vector3d> &, const open3d::geometry::KDTreeFlann &, std::vector<int> &, int, double)> knn_fire_exploration;

        knn_fire_exploration = [&](const std::vector<Eigen::Vector3d> & points, const open3d::geometry::KDTreeFlann & tree, std::vector<int> & indices_done, int idx, double th) -> std::vector<int>
        {
          std::vector<int> res;

          if(std::find(indices_done.begin(), indices_done.end(), idx) != indices_done.end())
          {
            return res;
          }

          indices_done.push_back(idx);

          const auto & point = points[idx];

          const double current_threshold = th;
          std::vector<int> indices;
          std::vector<double> distances;
          tree.SearchRadius<Eigen::Vector3d>(point, current_threshold, indices, distances);

          for(const auto & _idx : indices)
          {
            if(std::find(indices_done.begin(), indices_done.end(), _idx) == indices_done.end())
            {
              res.push_back(_idx);
              const std::vector<int> tmp_res = knn_fire_exploration(points, tree, indices_done, _idx, th);
              res.insert(res.end(), tmp_res.begin(), tmp_res.end());
            }
          }

          return res;
        };

        std::vector<std::vector<Eigen::Vector3d>> groups;
        groups.push_back(std::vector<Eigen::Vector3d>{});
        // Split in "big groups" by using "0.01cm"
        {
          std::shared_ptr<open3d::geometry::PointCloud> pc(new open3d::geometry::PointCloud);
          pc->points_ = points;
          open3d::geometry::KDTreeFlann tree(*pc);

          std::vector<int> indices_done;
          for(size_t i = 0; i < pc->points_.size(); ++i)
          {
            if(std::find(indices_done.begin(), indices_done.end(), i) != indices_done.end())
            {
              continue;
            }
            auto indices = knn_fire_exploration(pc->points_, tree, indices_done, i, 0.01);

            if(!groups.back().empty())
            {
              groups.push_back(std::vector<Eigen::Vector3d>{});
            }

            for(const auto & idx: indices)
            {
              if(indices.size() > 2)
              {
                groups.back().push_back(pc->points_[idx]);
              }
            }
          }
        }


        std::vector<Eigen::Vector3d> res;
        // Keep the latest which is the more far away
        double threshold = 0.004;
        for(size_t j = 0; j < groups.size(); ++j)
        {
          const auto & group = groups[j];

          if(group.empty())
          {
            continue;
          }

          std::shared_ptr<open3d::geometry::PointCloud> pc(new open3d::geometry::PointCloud);
          pc->points_ = group;
          open3d::geometry::KDTreeFlann tree(*pc);

          std::vector<int> indices_done;
          for(size_t i = 0; i < pc->points_.size(); ++i)
          {
            if(std::find(indices_done.begin(), indices_done.end(), i) != indices_done.end())
            {
              continue;
            }
            auto indices = knn_fire_exploration(pc->points_, tree, indices_done, i, threshold);

            std::vector<Eigen::Vector3d> segmentation;
            for(const auto & idx: indices)
            {
              if(indices.size() > 2)
              {
                segmentation.push_back(pc->points_[idx]);
              }
            }

            if(!segmentation.empty())
            {
              double sum_distance = 0.;
              std::sort(segmentation.begin(), segmentation.end(), [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });
              for(size_t k = 0; k < segmentation.size() - 1; ++k)
              {
                const auto & p0 = segmentation[k];
                const auto & p1 = segmentation[k + 1];
                sum_distance += (p1 - p0).norm();
              }

              const double estimated_distance = segmentation.back().x() - segmentation.front().x();
              if(sum_distance > 0.02 && estimated_distance > 0.005)
              {
                if(std::abs(estimated_distance - sum_distance) < 0.015)
                {
                  size_t idx_start = 0;
                  while(idx_start < segmentation.size() && (segmentation[idx_start].x() - segmentation.front().x()) < 0.01)
                  {
                    ++ idx_start;
                  }
                  segmentation.erase(segmentation.begin(), segmentation.begin() + idx_start);
                }
                res.insert(res.end(), segmentation.begin(), segmentation.end());
              }

            }
          }

          threshold += 0.002;
        }

        std::sort(res.begin(), res.end(), [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });

        // size_t idx_start = 0;
        // while(idx_start < res.size() && (res[idx_start].x() - res.front().x()) < 0.20)
        // {
        //   ++ idx_start;
        // }
        // res.erase(res.begin() + idx_start, res.end());

        return res;
      };


      {
        auto start = std::chrono::high_resolution_clock::now();

        foot_holder._X_0_b = foot_holder.X_0_b;
        foot_holder._X_0_b.translation().y() = 0.;

        const Eigen::Vector3d rpy = mc_rbdyn::rpyFromMat(foot_holder.X_0_b.rotation());
        foot_holder._X_0_b.rotation() = mc_rbdyn::rpyToMat(Eigen::Vector3d(rpy(0), 0., 0.));

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        mc_rtc::log::info("Took {} ms for [Extract X_0_b]", static_cast<double>(duration.count()) / 1000.);
      }


      auto start_computation = std::chrono::high_resolution_clock::now();
      {
        auto start = std::chrono::high_resolution_clock::now();
        plot_3d(foot_holder.pre_new_camera_points_, 1);

        foot_holder.pre_new_ground_points_ = fromCameraToWorldFrameWithAlignement(foot_holder.pre_new_camera_points_);

        // plot_3d(foot_holder.pre_new_ground_points_, 1);

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        mc_rtc::log::info("Took {} ms for [From Camera to World frame]", static_cast<double>(duration.count()) / 1000.);
      }
      {
        auto stop_computation = std::chrono::high_resolution_clock::now();
        auto duration_computation = std::chrono::duration_cast<std::chrono::microseconds>(stop_computation - start_computation);
        if(duration_computation.count() / 1000. > 33.0)
        {
          // continue;
        }
      }

      {
        auto start = std::chrono::high_resolution_clock::now();

        foot_holder.pre_new_ground_points_ = denoise(foot_holder.pre_new_ground_points_);
        plot_3d(foot_holder.pre_new_ground_points_, 2);

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        mc_rtc::log::info("Took {} ms for [Denoise]", static_cast<double>(duration.count()) / 1000.);
      }
      {
        auto stop_computation = std::chrono::high_resolution_clock::now();
        auto duration_computation = std::chrono::duration_cast<std::chrono::microseconds>(stop_computation - start_computation);
        if(duration_computation.count() / 1000. > 33.0)
        {
          // continue;
        }
      }

      if(foot_holder.pre_new_ground_points_.empty())
      {
        mc_rtc::log::error("Not enough data to continue");
        continue;
      }

      {
        auto start = std::chrono::high_resolution_clock::now();

        foot_holder.new_ground_points_ = foot_holder.pre_new_ground_points_;

        std::array<double, 3> threshold_degrees = {45., 15., 15.};
        std::array<double, 3> threshold_z = {0.05, 0.01, 0.002};

        bool is_success = false;
        for(size_t i = 0; i < threshold_degrees.size(); ++i)
        {
          const std::vector<Eigen::Vector3d> ground = selectOnlyGrounds(foot_holder.new_ground_points_, threshold_degrees[i], threshold_z[i]);

          if(!ground.empty())
          {
            // plot_3d(ground, -1 - i);
            plot_3d(foot_holder.new_ground_points_, 3);

            computePitchAndTzWithCeres(ground, foot_holder.pitch_, foot_holder.t_z_);

            if(std::abs(foot_holder.pitch_ * 180. / M_PI) > 25. || std::abs(foot_holder.t_z_) > 0.10)
            {
              continue;
            }

            is_success = true;
            foot_holder.new_ground_points_ = reAlignGround(foot_holder.new_ground_points_, foot_holder.pitch_, foot_holder.t_z_);

          }
        }

        foot_holder.new_ground_points_ = cuttoffZ(foot_holder.new_ground_points_);

        plot_3d(foot_holder.new_ground_points_, 3);

        if(!is_success)
        {
          mc_rtc::log::error("Could not perform the alignement");
          continue;
        }

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        mc_rtc::log::info("Took {} ms for [Perform alignement]", static_cast<double>(duration.count()) / 1000.);
      }
      {
        auto stop_computation = std::chrono::high_resolution_clock::now();
        auto duration_computation = std::chrono::duration_cast<std::chrono::microseconds>(stop_computation - start_computation);
        if(duration_computation.count() / 1000. > 33.0)
        {
          // continue;
        }
      }

      {
        auto start = std::chrono::high_resolution_clock::now();

        std::shared_ptr<open3d::geometry::PointCloud> target(new open3d::geometry::PointCloud);
        target->points_ = foot_holder.ground_points_;

        foot_pose->points_.push_back(Eigen::Vector3d(foot_holder._X_0_b.translation().x(), foot_holder._X_0_b.translation().y(), foot_holder._X_0_b.translation().z()));

        const auto front = Eigen::Vector3d(foot_holder._X_0_b.translation().x() + 0.10, foot_holder._X_0_b.translation().y() - 0.10, -0.10);
        const auto back = Eigen::Vector3d(Eigen::Vector3d(foot_holder._X_0_b.translation().x() + 0.55, foot_holder._X_0_b.translation().x() + 0.10, 0.10));
        target = target->Crop(open3d::geometry::AxisAlignedBoundingBox(front, back));

        std::sort(foot_holder.new_ground_points_.begin(), foot_holder.new_ground_points_.end(), [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });
        size_t idx_start = 0;
        while(idx_start < foot_holder.new_ground_points_.size() && (foot_holder.new_ground_points_[idx_start].x() - foot_holder.new_ground_points_.front().x()) < 0.15)
        {
          ++ idx_start;
        }
        foot_holder.new_ground_points_.erase(foot_holder.new_ground_points_.begin() + idx_start, foot_holder.new_ground_points_.end());

        std::vector<Eigen::Vector3d> src_obstacles = selectObstacles(foot_holder.new_ground_points_, true);
        std::vector<Eigen::Vector3d> target_obstacles = selectObstacles(target->points_);

        plot_3d(target_obstacles, 4);
        plot_3d(src_obstacles, 5);

        if(!src_obstacles.empty() && !target_obstacles.empty())
        {
          double t_x = 0.;
          ceres::Problem problem;

          if(target_obstacles.size() > src_obstacles.size())
          {
            for(const auto& T_0_p: src_obstacles)
            {
              ceres::CostFunction* cost_function = XCostFunctor::Create(T_0_p, target_obstacles);
              problem.AddResidualBlock(cost_function,  new ceres::CauchyLoss(0.5), &t_x);
            }
          }
          else
          {
            for(const auto& T_0_p: target_obstacles)
            {
              ceres::CostFunction* cost_function = XCostFunctor::Create(T_0_p, src_obstacles);
              problem.AddResidualBlock(cost_function,  new ceres::CauchyLoss(0.5), &t_x);
            }
          }

          ceres::Solver::Options options;
          options.linear_solver_type = ceres::DENSE_QR;
          ceres::Solver::Summary summary;
          ceres::Solve(options, &problem, &summary);

          if(!(target_obstacles.size() > src_obstacles.size()))
          {
            t_x = -t_x;
          }

          if(std::abs(t_x) > 0.025)
          {
            mc_rtc::log::error("Too much error in x: {}", t_x);
            foot_holder.new_ground_points_.clear();
          }

          if(!foot_holder.new_ground_points_.empty())
          {
            for(auto & p: foot_holder.new_ground_points_)
            {
              p.x() += t_x;
            }
          }
        }
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        mc_rtc::log::info("Took {} ms for [X Alignement]", static_cast<double>(duration.count()) / 1000.);
      }

      plot_3d(foot_holder.new_ground_points_, 6);

      {
        auto start = std::chrono::high_resolution_clock::now();

        if(foot_holder.historic_points_.empty())
        {
          foot_holder.historic_points_ = foot_holder.new_ground_points_;
        }

        if(!foot_holder.new_ground_points_.empty())
        {
          if(foot_holder.live_ground_points_.size() == 3)
          {
            foot_holder.live_ground_points_.pop_front();
          }
          foot_holder.live_ground_points_.push_back(foot_holder.new_ground_points_);
        }

        if(foot_holder.live_ground_points_.front().front().x() < foot_holder.new_ground_points_.front().x())
        {
          size_t idx_start = 0;
          while(idx_start < foot_holder.historic_points_.size() && foot_holder.new_ground_points_.front().x() > foot_holder.historic_points_[idx_start].x())
          {
            ++ idx_start;
          }
          foot_holder.historic_points_.erase(foot_holder.historic_points_.begin() + idx_start, foot_holder.historic_points_.end());
          for(const auto & points: foot_holder.live_ground_points_)
          {
            foot_holder.historic_points_.insert(foot_holder.historic_points_.end(), points.begin(), points.end());
          }
          foot_holder.historic_points_.insert(foot_holder.historic_points_.end(), foot_holder.new_ground_points_.begin(), foot_holder.new_ground_points_.end());
          std::sort(foot_holder.historic_points_.begin(), foot_holder.historic_points_.end(), [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });
        }

        foot_holder.ground_points_ = foot_holder.historic_points_;
        for(const auto & points: foot_holder.live_ground_points_)
        {
          foot_holder.ground_points_.insert(foot_holder.ground_points_.end(), points.begin(), points.end());
        }

        std::shared_ptr<open3d::geometry::PointCloud> pc(new open3d::geometry::PointCloud);
        pc->points_ = foot_holder.ground_points_;
        pc = pc->VoxelDownSample(0.002);
        foot_holder.ground_points_ = pc->points_;
        std::sort(foot_holder.ground_points_.begin(), foot_holder.ground_points_.end(), [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });

        foot_holder.corrected_ground_points_.clear();
        for(const auto & p: foot_holder.ground_points_)
        {
          foot_holder.corrected_ground_points_.push_back(Eigen::Vector3d(p.x(), foot_holder.X_0_b.translation().y(), p.z()));
        }

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        mc_rtc::log::info("Took {} ms for [Historic and update]", static_cast<double>(duration.count()) / 1000.);
      }

      auto stop_computation = std::chrono::high_resolution_clock::now();
      auto duration_computation = std::chrono::duration_cast<std::chrono::microseconds>(stop_computation - start_computation);
      mc_rtc::log::info("Took {} ms for [Everything]", static_cast<double>(duration_computation.count()) / 1000.);

      plot_3d(foot_holder.historic_points_, 12);

      size_t kkk = 7;
      for(const auto & points: foot_holder.live_ground_points_)
      {
        std::shared_ptr<open3d::geometry::PointCloud> red(new open3d::geometry::PointCloud);
        red->points_ = points;
        for(size_t o = 0; o < counter * 15 + kkk; ++o)
        {
          red->Transform(plus_offset_view);
          red->Transform(plus_offset_view);
        }
        red->PaintUniformColor(Eigen::Vector3d(1., 0., 0.));
        ++ kkk;
        visualizer.AddGeometry(red);
      }

      // std::shared_ptr<open3d::geometry::PointCloud> red(new open3d::geometry::PointCloud);
      std::shared_ptr<open3d::geometry::PointCloud> green(new open3d::geometry::PointCloud);

      // red->points_ = foot_holder.new_ground_points_;
      green->points_ = foot_holder.ground_points_;

      // green = green->VoxelDownSample(0.01);

      for(size_t o = 0; o < counter * 15 + 13; ++o)
      {
        green->Transform(plus_offset_view);
        green->Transform(plus_offset_view);
        // red->Transform(plus_offset_view);
      }
      // red->Transform(plus_offset_view);

      // red->PaintUniformColor(Eigen::Vector3d(1., 0., 0.));
      green->PaintUniformColor(Eigen::Vector3d(0., 1., 0.));


      // // visualizer.AddGeometry(red);
      // visualizer.AddGeometry(green);

      if(stop)
      {
        break;
      }

    }
  }

  mc_rtc::log::info("Done !");

  std::shared_ptr<mc_rtc::Logger> logger = std::make_shared<mc_rtc::Logger>(mc_rtc::Logger::Policy::NON_THREADED, "/tmp", "export");
  logger->start("res", 0.005);

  logger->addLogEntry("Observers_LIPMWalkingObserverPipeline_CameraLeft_ground_points_x",
                     [&]() -> std::vector<double>
                     {
                       std::vector<double> d;
                       std::transform(left_foot.corrected_ground_points_.begin(), left_foot.corrected_ground_points_.end(), std::back_inserter(d),
                                      [](const Eigen::Vector3d & v) { return v.x(); });
                       return d;
                     });
  logger->addLogEntry("Observers_LIPMWalkingObserverPipeline_CameraLeft_ground_points_y",
                     [&]() -> std::vector<double>
                     {
                       std::vector<double> d;
                       std::transform(left_foot.corrected_ground_points_.begin(), left_foot.corrected_ground_points_.end(), std::back_inserter(d),
                                      [](const Eigen::Vector3d & v) { return v.y(); });
                       return d;
                     });
  logger->addLogEntry("Observers_LIPMWalkingObserverPipeline_CameraLeft_ground_points_z",
                     [&]() -> std::vector<double>
                     {
                       std::vector<double> d;
                       std::transform(left_foot.corrected_ground_points_.begin(), left_foot.corrected_ground_points_.end(), std::back_inserter(d),
                                      [](const Eigen::Vector3d & v) { return v.z(); });
                       return d;
                     });

  logger->addLogEntry("Observers_LIPMWalkingObserverPipeline_CameraRight_ground_points_x",
                     [&]() -> std::vector<double>
                     {
                       std::vector<double> d;
                       std::transform(right_foot.corrected_ground_points_.begin(), right_foot.corrected_ground_points_.end(), std::back_inserter(d),
                                      [](const Eigen::Vector3d & v) { return v.x(); });
                       return d;
                     });
  logger->addLogEntry("Observers_LIPMWalkingObserverPipeline_CameraRight_ground_points_y",
                     [&]() -> std::vector<double>
                     {
                       std::vector<double> d;
                       std::transform(right_foot.corrected_ground_points_.begin(), right_foot.corrected_ground_points_.end(), std::back_inserter(d),
                                      [](const Eigen::Vector3d & v) { return v.y(); });
                       return d;
                     });
  logger->addLogEntry("Observers_LIPMWalkingObserverPipeline_CameraRight_ground_points_z",
                     [&]() -> std::vector<double>
                     {
                       std::vector<double> d;
                       std::transform(right_foot.corrected_ground_points_.begin(), right_foot.corrected_ground_points_.end(), std::back_inserter(d),
                                      [](const Eigen::Vector3d & v) { return v.z(); });
                       return d;
                     });

  logger->log();

  // visualizer.AddGeometry(left_foot.reference_source);

  foot_pose->PaintUniformColor(Eigen::Vector3d(1., 0.75, 0.));
  // visualizer.AddGeometry(foot_pose);

  std::shared_ptr<open3d::geometry::PointCloud> right_ground_selection(new open3d::geometry::PointCloud);
  std::shared_ptr<open3d::geometry::PointCloud> left_ground_selection(new open3d::geometry::PointCloud);

  left_ground_selection->points_ = left_foot.ground_points_;
  left_ground_selection->PaintUniformColor(Eigen::Vector3d(1., 0.25, 0.75));
  // visualizer.AddGeometry(left_ground_selection);

  right_ground_selection->points_ = right_foot.ground_points_;
  right_ground_selection->PaintUniformColor(Eigen::Vector3d(1., 0.25, 0.75));
  // visualizer.AddGeometry(right_ground_selection);

  std::shared_ptr<open3d::geometry::PointCloud> ref_right_ground_selection(new open3d::geometry::PointCloud);
  std::shared_ptr<open3d::geometry::PointCloud> ref_left_ground_selection(new open3d::geometry::PointCloud);

  ref_left_ground_selection->points_ = left_foot.logged_ground_points_;
  ref_left_ground_selection->PaintUniformColor(Eigen::Vector3d(0., 0.0, 1.0));
  // visualizer.AddGeometry(ref_left_ground_selection);

  ref_right_ground_selection->points_ = right_foot.logged_ground_points_;
  ref_right_ground_selection->PaintUniformColor(Eigen::Vector3d(0., 0.0, 1.0));
  // visualizer.AddGeometry(ref_right_ground_selection);


  // while(visualizer.PollEvents())
  // {
  //   visualizer.UpdateGeometry();
  //   // Set render flag as dirty anyways, because when we use callback
  //   // functions, we assume something has been changed in the callback
  //   // and the redraw event should be triggered.
  //   visualizer.UpdateRender();
  // }

  return 0;
}
