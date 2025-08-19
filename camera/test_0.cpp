#include <iostream>
#include <chrono>
#include <unordered_map>
#include <algorithm>
#include <vector>

#include <mc_control/ControllerServer.h>
#include <mc_rtc/gui.h>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

constexpr size_t KERNEL_SIZE = 1;
constexpr size_t NR_RANGES = 50;
constexpr double DT = 0.005;

int main(int argc, char *argv[])
{
  //
  rs2::config cfg;
  // Check if we have a file
  std::string path = "";
  if(argc == 2)
  {
    path = argv[1];
    mc_rtc::log::info("We will use the recorded depth at {}", path);
    cfg.enable_device_from_file(path);
  }
  else
  {
    cfg.enable_stream(RS2_STREAM_DEPTH); // Enable default depth
  }


  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
  //
  pipe.start(cfg);

  auto get_sensor_name = [](const rs2::sensor& sensor)
  {
      // Sensors support additional information, such as a human readable name
      if (sensor.supports(RS2_CAMERA_INFO_NAME))
          return sensor.get_info(RS2_CAMERA_INFO_NAME);
      else
          return "Unknown Sensor";
  };

  // Get intrinsics parameters

  rs2::sensor sensor = pipe.get_active_profile().get_device().query_sensors()[0];

  mc_rtc::log::info("Getting the intrisics parameters of {}", get_sensor_name(sensor));
  rs2::stream_profile depth_stream_profile = sensor.get_stream_profiles()[0];
  mc_rtc::log::info("It is a {} stream", depth_stream_profile.stream_type());
  rs2_intrinsics intrinsics = depth_stream_profile.as<rs2::video_stream_profile>().get_intrinsics();

  // Decimation filter reduces the amount of data (while preserving best samples)
  rs2::decimation_filter dec;
  // If the demo is too slow, make sure you run in Release (-DCMAKE_BUILD_TYPE=Release)
  // but you can also increase the following parameter to decimate depth more (reducing quality)
  dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
  // Define transformations from and to Disparity domain
  rs2::disparity_transform depth2disparity;
  rs2::disparity_transform disparity2depth(false);
  // Define spatial filter (edge-preserving)
  rs2::spatial_filter spat;
  // Enable hole-filling
  // Hole filling is an agressive heuristic and it gets the depth wrong many times
  // However, this demo is not built to handle holes
  // (the shortest-path will always prefer to "cut" through the holes since they have zero 3D distance)
  spat.set_option(RS2_OPTION_HOLES_FILL, 5); // 5 = fill all the zero pixels
  // Define temporal filter
  rs2::temporal_filter temp;

  // Create GUI to check the depth
  mc_rtc::gui::StateBuilder gui;

  double t = 0.;
  double range = 0.;
  double plot_depth = 0.;

  gui.addPlot(
    "RangeSensor",
    mc_rtc::gui::plot::X("t", [&t]() { return t; }),
    mc_rtc::gui::plot::Y("range", [&range]() { return range; }, mc_rtc::gui::Color::Red),
    mc_rtc::gui::plot::Y("depth", [&plot_depth]() { return plot_depth; }, mc_rtc::gui::Color::Blue)
  );

  Eigen::Vector3d point3d = Eigen::Vector3d::Zero();
  using Policy = mc_rtc::Logger::Policy;

  mc_rtc::Logger logger(Policy::THREADED, "/tmp", "camera");
  logger.start("logger", DT);
  logger.addLogEntry("point", [&point3d] { return point3d; });


  std::unique_ptr<mc_control::ControllerServer> server;
  server.reset(new mc_control::ControllerServer(DT, 10 * DT, {"ipc:///tmp/mc_rtc_pub.ipc"}, {"ipc:///tmp/mc_rtc_rep.ipc"}));

  auto start = std::chrono::high_resolution_clock::now();
  while(true)
  {
    // Wait for the next set of frames from the camera
    for(size_t i = 0; i < 50; ++i)
    {
      auto frames = pipe.wait_for_frames();
    }

    auto frames = pipe.wait_for_frames();

    // Get depth frame
    auto depth = frames.get_depth_frame();
    // Decimation will reduce the resultion of the depth image,
    // closing small holes and speeding-up the algorithm
    depth = depth.apply_filter(dec);
    // To make sure far-away objects are filtered proportionally
    // we try to switch to disparity domain
    depth = depth.apply_filter(depth2disparity);
    // Apply spatial filtering
    // depth = depth.apply_filter(spat);
    // Apply temporal filtering
    depth = depth.apply_filter(temp);
    // If we are in disparity domain, switch back to depth
    depth = depth.apply_filter(disparity2depth);

    const size_t height = depth.get_height();
    const size_t width = depth.get_width();

    const size_t half_height = static_cast<size_t>(height * 0.5);
    const size_t half_width = static_cast<size_t>(width * 0.5);

    std::array<float, 2> pixel = {half_width, 0.};
    mc_rtc::log::error("START");
    for(size_t i = 0; i < height; i += 1)
    {
      pixel[1] = i;
      std::array<float, 3> point = {0., 0., 0.};
      const float _depth = depth.get_distance(half_width, i);
      if(_depth > 0.0 && _depth < 2.0)
      {
        plot_depth = static_cast<double>(_depth);
        rs2_deproject_pixel_to_point(point.data(), &intrinsics, pixel.data(), _depth);
        point3d[0] = point[0];
        point3d[1] = point[1];
        point3d[2] = point[2];
        range = point[2];
        mc_rtc::log::info("i {} range {} depth {}", i, range, _depth);
        // Little hack
        const auto end = std::chrono::high_resolution_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(end - start).count();
        t += 0.005;
        start = std::chrono::high_resolution_clock::now();

        server->handle_requests(gui);
        server->publish(gui);
        logger.log();
      }
    }
    mc_rtc::log::error("END");
    return -1;

    // Measure elapsed time
  }

  return 0; // success
}