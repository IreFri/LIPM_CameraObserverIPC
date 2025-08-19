#include "CameraSensorServer.h"

#include "CameraSensorCeresFunctions.h"

// Include RealSense Cross Platform API
#include <librealsense2/rs_advanced_mode.hpp>

#include <fstream>
#include <thread>

#include <signal.h>
#include <unistd.h>

#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

namespace lipm_walking
{

CameraSensorServer::CameraSensorServer(const char * name, const mc_rtc::Configuration & config)
: it_(ros::NodeHandle())
{
  name_ = name;
  data_ = CameraSensorShared::get(name);
  if(data_->server.isAlive())
  {
    mc_rtc::log::error_and_throw("[CameraSensorServer] Another server (pid: {}) is already handling {}",
                                 data_->server.pid, name);
  }
  data_->newServer(getpid());

  auto serial = config("camera_serial");
  if(serial.isString())
  {
    camera_serial_ = serial.operator std::string();
  }
  else
  {
    camera_serial_ = std::to_string(serial.operator uint64_t());
  }

  if(config.has("path_to_preset"))
  {
    path_to_preset_ = config("path_to_preset").operator std::string();
  }

  if(config.has("kernel_size"))
  {
    kernel_size_ = config("kernel_size");
    kernel_size_ = kernel_size_ < 3 ? 3 : kernel_size_;
  }

  if(config.has("kernel_threshold"))
  {
    kernel_threshold_ = static_cast<float>(static_cast<double>(config("kernel_threshold")));
    kernel_threshold_ = kernel_threshold_ < 0 ? 0.005f : kernel_threshold_;
  }

  if(config.has("outlier_threshold"))
  {
    outlier_threshold_ = static_cast<float>(static_cast<double>(config("outlier_threshold")));
    outlier_threshold_ = outlier_threshold_ < 0 ? 0.01f : outlier_threshold_;
  }

  if(config.has("path_to_replay_data"))
  {
    path_to_replay_data_ = static_cast<std::string>(config("path_to_replay_data"));
  }

  image_pub_ = it_.advertise("/camera/" + name_, 1);
  image_pub_color_ = it_.advertise("/camera_color/" + name_, 1);

  mc_rtc::log::info("[CameraSensorServer::{}] 'camera_serial' is {}", name_, camera_serial_);
  mc_rtc::log::info("[CameraSensorServer::{}] 'path_to_preset' is {}", name_, path_to_preset_);
  mc_rtc::log::info("[CameraSensorServer::{}] 'kernel_size' is {}", name_, kernel_size_);
  mc_rtc::log::info("[CameraSensorServer::{}] 'kernel_threshold' is {}", name_, kernel_threshold_);
  mc_rtc::log::info("[CameraSensorServer::{}] 'outlier_threshold' is {}", name_, outlier_threshold_);
}

CameraSensorServer::~CameraSensorServer()
{
  stop();
}

void CameraSensorServer::run()
{
  if(acquisition_th_.joinable())
  {
    mc_rtc::log::error_and_throw("[CameraSensorServer::{}] Already running, call stop() first", name_);
  }
  run_ = true;
  acquisition_th_ = std::thread([this]() { acquisition(); });
  computation_th_ = std::thread([this]() { computation(); });
}

void CameraSensorServer::stop()
{
  run_ = false;
  if(computation_th_.joinable())
  {
    computation_th_.join();
  }
  if(acquisition_th_.joinable())
  {
    if(pipeline_started_)
    {
      acquisition_th_.join();
    }
    else
    {
      acquisition_th_.detach();
      mc_rtc::log::critical("The acquisition pipeline has not started yet, this might not exit gracefully");
    }
  }
}

void CameraSensorServer::acquisition()
{
  rs2::config cfg;
  // Check if we read from a .bag or from a camera
  if(!path_to_replay_data_.empty())
  {
    mc_rtc::log::info("[CameraSensor] We will use the recorded depth at {}", path_to_replay_data_);
    cfg.enable_device_from_file(path_to_replay_data_);
  }
  else
  {
    cfg.enable_device(camera_serial_);
  }

  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
  //
  auto prof = pipe.start(cfg);
  pipeline_started_ = true;

  // Load preset
  if(!path_to_preset_.empty())
  {
    std::ifstream t(path_to_preset_);
    std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    rs2::device dev = prof.get_device();
    auto advanced = dev.as<rs400::advanced_mode>();
    advanced.load_json(str);
  }

  auto depthToPoint = [](const rs2_intrinsics & intrinsics, const std::array<float, 2> & pixel,
                         float depth) -> Eigen::Vector3d
  {
    Eigen::Vector3f point;
    rs2_deproject_pixel_to_point(point.data(), &intrinsics, pixel.data(), depth);
    return point.cast<double>();
  };

  // Decimation filter reduces the amount of data (while preserving best samples)
  rs2::decimation_filter dec;
  // If the demo is too slow, make sure you run in Release (-DCMAKE_BUILD_TYPE=Release)
  // but you can also increase the following parameter to decimate depth more (reducing quality)
  dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 4);
  // HDR Merge
  rs2::hdr_merge hdr;
  // Threshold filter
  rs2::threshold_filter thresh(0.5f, 0.40f);
  // Define transformations from and to Disparity domain
  rs2::disparity_transform depth2disparity;
  // Define spatial filter (edge-preserving)
  rs2::spatial_filter spat;
  spat.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2.f);
  spat.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5f);
  spat.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.f);
  spat.set_option(RS2_OPTION_HOLES_FILL, 0);
  rs2::disparity_transform disparity2depth(false);

  while(run_)
  {
    auto frames = pipe.wait_for_frames();

    // Get depth frame
    auto frame = frames.get_depth_frame();
    frame = frame.apply_filter(dec);
    frame = frame.apply_filter(hdr);
    frame = frame.apply_filter(thresh);
    frame = frame.apply_filter(depth2disparity);
    frame = frame.apply_filter(spat);
    frame = frame.apply_filter(disparity2depth);

    rs2_intrinsics intrinsics = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    const size_t height = static_cast<size_t>(frame.get_height());
    const size_t width = static_cast<size_t>(frame.get_width());

    mc_rtc::log::info("h {} w {}", height, width);

    const size_t half_height = static_cast<size_t>(static_cast<double>(height) * 0.5);
    const size_t half_width = static_cast<size_t>(static_cast<double>(width) * 0.5);

    std::array<float, 2> pixel;
    const size_t half_kernel_size = kernel_size_ / 2;
    std::vector<Eigen::Vector3d> points;
    for(size_t i = half_kernel_size; i < width - half_kernel_size - 50; ++i)
    {
      pixel[0] = static_cast<float>(i);
      for(int j = -1; j <= 1; ++j)
      {
        pixel[1] = static_cast<float>(half_height + j);

        const float depth = frame.get_distance(static_cast<int>(pixel[0]), static_cast<int>(pixel[1]));
        Eigen::Vector3d point = depthToPoint(intrinsics, pixel, depth);
        if(point.z() != 0)
        {
          // point.y() = 0.;
          points.push_back(point);
        }
      }
    }

    {
      const std::lock_guard<std::mutex> lock(points_mtx_);
      points_ = points;
    }

    {
      ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->points_mtx);
      std::cout << "Updating data_->points" << std::endl;
      data_->points.resize(points.size());
      for(size_t i = 0; i < points.size(); ++i)
      {
        data_->points[i] = points[i];
      }
      std::cout << "New size is " << data_->points.size() << std::endl;
    }

    if(data_->client.isAlive())
    {
      data_->data_ready->notify_all();
    }

    // Publish in ROS
    {
      // Allocate the image
      // if(image_colorized_depth_.size() != cv::Size(width, height))
      // {
      //   image_colorized_depth_ = cv::Mat(height, width, CV_8UC3);
      // }

      // image_colorized_depth_.data = (uint8_t*)color_map_.process(frame).get_data();
      if(image_colorized_depth_.size() != cv::Size(width, height))
      {
        image_colorized_depth_ = cv::Mat(height, width, CV_8UC3);
      }

      uint16_t* ptr = (uint16_t*)frame.get_data();
      int stride = frame.as<rs2::video_frame>().get_stride_in_bytes();

      for(size_t i = width - half_kernel_size - 100; i < width; ++i)
      {
        for(int j = 0; j < height; ++j)
        {
          ptr[j * stride + i] = 0;
        }
      }

      image_colorized_depth_.data = (uint8_t*)color_map_.process(frame).get_data();
      // if (frame.is<rs2::depth_frame>())
      // {
      //     image = fix_depth_scale(image, _depth_scaled_image[stream]);
      // }
      sensor_msgs::Image img_msg;

      // Convert the CV::Mat into a ROS image message (1 copy is done here)
      cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8 , image_colorized_depth_).toImageMsg(img_msg);

      // Convert OpenCV Mat to ROS Image
      img_msg.header.frame_id = "/camera/"+name_;
      img_msg.height = height;
      img_msg.width = width;
      img_msg.is_bigendian = false;
      img_msg.step = width * image_colorized_depth_.elemSize();

      // Transfer the unique pointer ownership to the RMW
      image_pub_.publish(img_msg);
      {
        auto frame = frames.get_color_frame();

        const size_t height = static_cast<size_t>(frame.get_height());
        const size_t width = static_cast<size_t>(frame.get_width());

        if(image_rgb_.size() != cv::Size(width,height))
        {
          image_rgb_ = cv::Mat(height, width, CV_8UC3);
        }

        image_rgb_.data = (uint8_t*)frame.get_data();

        sensor_msgs::Image img_msg;
        // Convert the CV::Mat into a ROS image message (1 copy is done here)
        cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, image_rgb_).toImageMsg(img_msg);

        // Convert OpenCV Mat to ROS Image
        img_msg.header.frame_id = "/camera/"+name_;
        img_msg.height = height;
        img_msg.width = width;
        img_msg.is_bigendian = false;
        img_msg.step = width * image_rgb_.elemSize();

        //Transfer the unique pointer ownership to the RMW
        image_pub_color_.publish(img_msg);
      }

      ros::spinOnce();
    }
  }
}

void CameraSensorServer::computation()
{
  bool client_was_alive = data_->client.isAlive();
  while(run_)
  {
    while(data_->client.isAlive())
    {
      if(!client_was_alive)
      {
        mc_rtc::log::success("[CameraSensorServer::{}] Client connected", name_);
        client_was_alive = true;
      }
      ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->mtx);
      if(data_->compute_ready->timed_wait(lck, pclock::universal_time() + ptime::seconds(1)))
      {
        // mc_rtc::log::error("Here");
        do_computation();
      }
    }
    if(client_was_alive)
    {
      client_was_alive = false;
      mc_rtc::log::critical("[CameraSensorServer::{}] Client disconnected", name_);
    }
    // Wait for the client to become available
    data_->server_idle.notify_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

void CameraSensorServer::do_computation()
{
  if(data_->skip)
  {
    data_->result_ready->notify_all();
    return;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //// fromCameraToWorldFrameWithAlignement
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  auto fromCameraToWorldFrameWithAlignement = [this](const std::vector<Eigen::Vector3d> & camera_points)
  {
    std::vector<Eigen::Vector3d> ground_points;

    for(int i = camera_points.size() - 1; i >= 0; --i)
    {
      const sva::PTransformd X_s_p(camera_points[i]);
      sva::PTransformd X_0_p = X_s_p * data_->X_b_s * X_0_b_;
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
  auto selectOnlyGrounds = [this](const std::vector<Eigen::Vector3d> & ground_points, double threshold_deg, double threshold_z)
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
      pc->OrientNormalsTowardsCameraLocation(X_0_b_.translation());
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
  auto computePitchAndTzWithCeres = [this](const std::vector<Eigen::Vector3d> & selected_line, double & pitch, double & t_z)
  {
    pitch = 0.;
    t_z = 0.;

    ceres::Problem problem;
    for(const auto& T_0_p: selected_line)
    {
      const sva::PTransformd X_0_p(T_0_p);
      const sva::PTransformd X_s_p = X_0_p * (data_->X_b_s * X_0_b_).inv();
      ceres::CostFunction* cost_function = PitchZCostFunctor::Create(X_s_p, X_0_b_, data_->X_b_s);
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
  auto reAlignGround = [this](const std::vector<Eigen::Vector3d> & ground_points, double pitch, double t_z)
  {
    std::vector<Eigen::Vector3d> _ground_points;
    const sva::PTransformd X_b_b(sva::RotY(pitch).cast<double>(), Eigen::Vector3d(0., 0., t_z));
    for(const auto& T_0_p: ground_points)
    {
      // From camera frame to world frame
      const sva::PTransformd _X_0_p(T_0_p);
      const sva::PTransformd X_s_p = _X_0_p * (data_->X_b_s * X_0_b_).inv();
      const sva::PTransformd X_0_p = X_s_p * data_->X_b_s * X_b_b * X_0_b_;
      _ground_points.push_back(X_0_p.translation());
    }

    return _ground_points;
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //// cuttoffZ
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  auto cuttoffZ = [](const std::vector<Eigen::Vector3d> & ground_points)
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
  auto denoise = [this](std::vector<Eigen::Vector3d> points)
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
    ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->points_mtx);
    if(data_->reset_ground)
    {
      ground_points_.clear();
      pre_new_ground_points_.clear();
      data_->ground_points.clear();
      data_->reset_ground = false;
    }
  }

  {
    mc_rtc::log::info("Acquired {} raw points", points_.size());
    const std::lock_guard<std::mutex> lock(points_mtx_);
    new_camera_points_ = points_;
    new_ground_points_.reserve(new_camera_points_.size());
  }

  if(new_camera_points_.empty())
  {
    data_->result_ready->notify_all();
    return;
  }

  auto start_computation = std::chrono::high_resolution_clock::now();
  {
    auto start = std::chrono::high_resolution_clock::now();

    X_0_b_ = data_->X_0_b;
    X_0_b_.translation().y() = 0.;

    const Eigen::Vector3d rpy = mc_rbdyn::rpyFromMat(data_->X_0_b.rotation());
    X_0_b_.rotation() = mc_rbdyn::rpyToMat(Eigen::Vector3d(rpy(0), 0., 0.));

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::info("Took {} ms for [Extract X_0_b]", static_cast<double>(duration.count()) / 1000.);
  }

  {
    auto start = std::chrono::high_resolution_clock::now();

    pre_new_ground_points_ = fromCameraToWorldFrameWithAlignement(new_camera_points_);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::info("Took {} ms for [From Camera to World frame]", static_cast<double>(duration.count()) / 1000.);
  }
  {
    auto stop_computation = std::chrono::high_resolution_clock::now();
    auto duration_computation = std::chrono::duration_cast<std::chrono::microseconds>(stop_computation - start_computation);
    if(duration_computation.count() / 1000. > 33.0)
    {
      mc_rtc::log::error("Timeout");
      // data_->result_ready->notify_all();
      // return;
    }
  }

  {
    auto start = std::chrono::high_resolution_clock::now();

    pre_new_ground_points_ = denoise(pre_new_ground_points_);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::info("Took {} ms for [Denoise]", static_cast<double>(duration.count()) / 1000.);
  }
  {
    auto stop_computation = std::chrono::high_resolution_clock::now();
    auto duration_computation = std::chrono::duration_cast<std::chrono::microseconds>(stop_computation - start_computation);
    if(duration_computation.count() / 1000. > 33.0)
    {
      mc_rtc::log::error("Timeout");
      // data_->result_ready->notify_all();
      // return;
    }
  }

   // If not enough points
  if(pre_new_ground_points_.empty())
  {
    mc_rtc::log::error("Not enough data to continue");
    data_->result_ready->notify_all();
    return;
  }

  {
    auto start = std::chrono::high_resolution_clock::now();

    new_ground_points_ = pre_new_ground_points_;

    std::array<double, 3> threshold_degrees = {45., 15., 5.};
    std::array<double, 3> threshold_z = {0.05, 0.01, 0.002};

    bool is_success = false;
    for(size_t i = 0; i < threshold_degrees.size(); ++i)
    {
      const std::vector<Eigen::Vector3d> ground = selectOnlyGrounds(new_ground_points_, threshold_degrees[i], threshold_z[i]);

      if(!ground.empty())
      {
        computePitchAndTzWithCeres(ground, previous_pitch_, previous_t_z_);

        if(std::abs(previous_pitch_ * 180. / M_PI) > 25. || std::abs(previous_t_z_) > 0.10)
        {
          continue;
        }

        is_success = true;
        new_ground_points_ = reAlignGround(new_ground_points_, previous_pitch_, previous_t_z_);
      }
    }

    new_ground_points_ = cuttoffZ(new_ground_points_);

    if(!is_success)
    {
      mc_rtc::log::error("Could not perform the alignement");
      data_->result_ready->notify_all();
      return;
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::info("Took {} ms for [Perform alignement]", static_cast<double>(duration.count()) / 1000.);
  }

  {
    auto start = std::chrono::high_resolution_clock::now();

    std::shared_ptr<open3d::geometry::PointCloud> target(new open3d::geometry::PointCloud);
    target->points_ = ground_points_;

    const auto front = Eigen::Vector3d(X_0_b_.translation().x() + 0.10, X_0_b_.translation().y() - 0.10, -0.10);
    const auto back = Eigen::Vector3d(Eigen::Vector3d(X_0_b_.translation().x() + 0.55, X_0_b_.translation().x() + 0.10, 0.10));
    target = target->Crop(open3d::geometry::AxisAlignedBoundingBox(front, back));

    std::sort(new_ground_points_.begin(), new_ground_points_.end(), [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });
    size_t idx_start = 0;
    while(idx_start < new_ground_points_.size() && (new_ground_points_[idx_start].x() - new_ground_points_.front().x()) < 0.15)
    {
      ++ idx_start;
    }
    new_ground_points_.erase(new_ground_points_.begin() + idx_start, new_ground_points_.end());

    std::vector<Eigen::Vector3d> src_obstacles = selectObstacles(new_ground_points_, true);
    std::vector<Eigen::Vector3d> target_obstacles = selectObstacles(target->points_);

    if(src_obstacles.size() > 5 && target_obstacles.size() > 5)
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
        new_ground_points_.clear();
      }

      if(!new_ground_points_.empty())
      {
        for(auto & p: new_ground_points_)
        {
          p.x() += t_x;
        }
      }
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    mc_rtc::log::info("Took {} ms for [X Alignement]", static_cast<double>(duration.count()) / 1000.);
  }

  if(historic_points_.empty())
  {
    historic_points_ = new_ground_points_;
  }

  if(!new_ground_points_.empty())
  {
    if(live_ground_points_.size() == 3)
    {
      live_ground_points_.pop_front();
    }
    live_ground_points_.push_back(new_ground_points_);
  }

  if(live_ground_points_.front().front().x() < new_ground_points_.front().x())
  {
    size_t idx_start = 0;
    while(idx_start < historic_points_.size() && new_ground_points_.front().x() > historic_points_[idx_start].x())
    {
      ++ idx_start;
    }

    size_t size = std::min(size_t(20), live_ground_points_.front().size());
    if(idx_start + size < historic_points_.size())
    {
      double sum_diff = 0.;
      for(size_t i = 0; i < size; ++i)
      {
        sum_diff += std::abs((historic_points_.begin() + idx_start + i)->z() - live_ground_points_.front()[i].z());
      }
      sum_diff /= static_cast<double>(size);

      if(sum_diff > 0.005)
      {
        idx_start += 20;
        idx_start = std::min(idx_start, historic_points_.size());
      }
    }

    historic_points_.erase(historic_points_.begin() + idx_start, historic_points_.end());
    for(const auto & points: live_ground_points_)
    {
      historic_points_.insert(historic_points_.end(), points.begin(), points.end());
    }
    historic_points_.insert(historic_points_.end(), new_ground_points_.begin(), new_ground_points_.end());
    std::sort(historic_points_.begin(), historic_points_.end(), [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });
  }

  ground_points_ = historic_points_;
  for(const auto & points: live_ground_points_)
  {
    ground_points_.insert(ground_points_.end(), points.begin(), points.end());
  }

  std::shared_ptr<open3d::geometry::PointCloud> pc(new open3d::geometry::PointCloud);
  pc->points_ = ground_points_;
  pc = pc->VoxelDownSample(0.002);
  ground_points_ = pc->points_;
  std::sort(ground_points_.begin(), ground_points_.end(), [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return a.x() < b.x(); });

  auto stop_computation = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop_computation - start_computation);
  mc_rtc::log::info("Took {} ms for [E V E R Y T H I N G]", static_cast<double>(duration.count()) / 1000.);

  {
    ipc::scoped_lock<ipc::interprocess_mutex> lck(data_->points_mtx);
    const auto & points = ground_points_;
    data_->ground_points.resize(points.size());
    for(size_t i = 0; i < points.size(); ++i)
    {
      const auto & p = points[i];
      data_->ground_points[i] = Eigen::Vector3d(p.x(), data_->X_0_b.translation().y(), p.z());
    }
    mc_rtc::log::info("The estimated ground contains {} points", data_->ground_points.size());
  }
  data_->result_ready->notify_all();
}

} // namespace lipm_walking

std::mutex cv_mtx;
std::condition_variable condv;
std::atomic<bool> stop_program{false};

void signal_callback_handler(int)
{
  stop_program = true;
  condv.notify_all();
}

int main(int argc, char * argv[])
{
  if(argc < 3)
  {
    mc_rtc::log::info("[usage] {} <ControllerConfig.yaml> <CameraName>", argv[0]);
    return 1;
  }
  mc_rtc::Configuration ctl_cfg(argv[1]);
  std::string camera = argv[2];
  if(!ctl_cfg.has("ObserverPipelines"))
  {
    mc_rtc::log::error_and_throw("Must have ObserverPipelines in the configuration");
  }
  ros::init(argc, argv, camera);
  auto observer_pipelines = ctl_cfg("ObserverPipelines");
  auto observers =
      [&]()
  {
    if(observer_pipelines.isArray())
    {
      mc_rtc::log::warning("Multiple pipelines in configuration, taking the first one");
      return observer_pipelines[0]("observers");
    }
    return observer_pipelines("observers");
  }()
          .
      operator std::vector<mc_rtc::Configuration>();
  auto camera_cfg = [&]()
  {
    for(const auto & o : observers)
    {
      if(o.has("name") && o("name").operator std::string() == camera)
      {
        return o("config");
      }
    }
    mc_rtc::log::error_and_throw("No configuration for camera {} found in {}", camera, argv[1]);
  }();
  lipm_walking::CameraSensorServer server(camera.c_str(), camera_cfg);
  mc_rtc::log::info("[CameraServer::{}] Running, press ctrl+c to stop", camera);
  server.run();
  signal(SIGINT, signal_callback_handler);
  std::unique_lock<std::mutex> lck(cv_mtx);
  condv.wait(lck, []() { return stop_program.load(); });
  server.stop();
  return 0;
}
