#pragma once

#include <lipm_walking/observer/CameraSensorShared.h>

#include <mc_rtc/Configuration.h>

#include <open3d/Open3D.h>

#include <thread>

#include <mc_rtc/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>

namespace lipm_walking
{

struct CameraSensorServer
{
  /** Initialize the server and kernel configuration */
  CameraSensorServer(const char * name, const mc_rtc::Configuration & config);

  ~CameraSensorServer();

  /** Starts the acquisition thread and the computation thread
   *
   * Then run forever until stop is called
   */
  void run();

  void stop();

private:
  std::string name_;
  /** SHM */
  CameraSensorShared * data_;
  /** Acquisition and computation parameters */
  std::string camera_serial_;
  std::string path_to_preset_;
  std::string path_to_replay_data_;
  size_t kernel_size_ = 3;
  float kernel_threshold_ = 0.005f;
  float outlier_threshold_ = 0.01f;

  /** Acquisition and computation data */
  std::mutex points_mtx_;
  std::vector<Eigen::Vector3d> points_;
  std::vector<Eigen::Vector3d> new_camera_points_;
  std::vector<Eigen::Vector3d> pre_new_ground_points_;
  std::vector<Eigen::Vector3d> new_ground_points_;
  std::vector<Eigen::Vector3d> ground_points_;
  std::vector<Eigen::Vector3d> historic_points_;
  std::deque<std::vector<Eigen::Vector3d>> live_ground_points_;
  sva::PTransformd X_0_b_;

  double previous_pitch_ = 0.;
  double previous_t_z_ = 0.;

  std::atomic<bool> run_ = false;

  std::atomic<bool> pipeline_started_ = false;

  std::thread acquisition_th_;
  void acquisition();

  std::thread computation_th_;
  void computation();

  void do_computation();

  // ROS part to publish the image
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher image_pub_color_;
  cv::Mat image_rgb_;
  rs2::colorizer color_map_;
  cv::Mat image_colorized_depth_;
};

} // namespace lipm_walking
