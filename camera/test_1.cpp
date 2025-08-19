// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include "example.hpp" // Include short list of convenience functions for rendering
#include <atomic>
#include <fstream>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rs_advanced_mode.hpp>
#include <map>
#include <string>
#include <thread>
#include <Eigen/Core>

// Helper function for getting data from the queues and updating the view
void update_data(rs2::frame_queue & data,
                 rs2::frame & depth,
                 rs2::points & points,
                 rs2::pointcloud & pc,
                 glfw_state & view,
                 rs2::colorizer & color_map);

void update_data_and_compute(rs2::frame_queue & data,
                 std::vector<rs2::vertex> & points);

rs2_intrinsics intrinsics;

int main(int argc, char * argv[])
try
{
  // Create a simple OpenGL window for rendering:
  window app(1280, 720, "RealSense Post Processing Example");

  // Construct objects to manage view state
  glfw_state original_view_orientation{};
  glfw_state filtered_view_orientation{};
  glfw_state side_view_orientation{};

  filtered_view_orientation.yaw = 0; // -25
  filtered_view_orientation.pitch = 0; // +5

  original_view_orientation.yaw = 0; // -25
  original_view_orientation.pitch = 0; // +5

  side_view_orientation.yaw = 75.; // 75.
  side_view_orientation.pitch = -15.; // -15.

  // Declare pointcloud objects, for calculating pointclouds and texture mappings
  rs2::pointcloud original_pc;
  rs2::pointcloud filtered_pc;

  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
  rs2::config cfg;
  // Use a configuration object to request only depth from the pipeline
  // cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
  // std::string serial = "230322275639";
  // std::string serial = "230422273358";
  // cfg.enable_device(serial);
  std::string cfg_path = "/home/jrluser/my_workspace/controller/lipm_walking_controller_softfoot/etc/D405_ShortRangePreset.JSON";
  // Start streaming with the above configuration
  // pipe.start(cfg);
  auto prof = pipe.start(cfg);
  //
  std::ifstream t(cfg_path);
  std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
  rs2::device dev = prof.get_device();
  auto advanced = dev.as<rs400::advanced_mode>();
  advanced.load_json(str);

  // intrinsics.width /= 4.f;     /**< Width of the image in pixels */
  // intrinsics.height /= 4.f;    /**< Height of the image in pixels */
  // intrinsics.ppx /= 4.f;       /**< Horizontal coordinate of the principal point of the image, as a pixel offset from the left edge */
  // intrinsics.ppy /= 4.f;       /**< Vertical coordinate of the principal point of the image, as a pixel offset from the top edge */
  // intrinsics.fx /= 4.f;        /**< Focal length of the image plane, as a multiple of pixel width */
  // intrinsics.fy /= 4.f;        /**< Focal length of the image plane, as a multiple of pixel height */

  // Declare filters
  rs2::decimation_filter dec;
  dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 4);
  rs2::hdr_merge hdr;
  rs2::threshold_filter thresh(0.5f, 0.40f);
  rs2::disparity_transform depth2disparity;
  rs2::spatial_filter spat;
  spat.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2.f);
  spat.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5f);
  spat.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.f);
  spat.set_option(RS2_OPTION_HOLES_FILL, 0);
  rs2::disparity_transform disparity2depth(false);

  // Declaring two concurrent queues that will be used to enqueue and dequeue frames from different threads
  rs2::frame_queue original_data;
  rs2::frame_queue filtered_data;
  rs2::frame_queue selected_data;

  // Declare depth colorizer for pretty visualization of depth data
  rs2::colorizer color_map;

  // Atomic boolean to allow thread safe way to stop the thread
  std::atomic_bool stopped(false);

  // Create a thread for getting frames from the device and process them
  // to prevent UI thread from blocking due to long computations.
  std::thread processing_thread(
      [&]()
      {
        while(!stopped) // While application is running
        {
          rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
          rs2::frame depth_frame = data.get_depth_frame(); // Take the depth frame from the frameset
          if(!depth_frame) // Should not happen but if the pipeline is configured differently
            return; //  it might not provide depth and we don't want to crash

          // Get intrinsics parameters

          rs2::frame filtered = depth_frame; // Does not copy the frame, only adds a reference

          /* Apply filters.
          The implemented flow of the filters pipeline is in the following order:
          1. apply decimation filter
          2. apply threshold filter
          3. transform the scene into disparity domain
          4. apply spatial filter
          5. apply temporal filter
          6. revert the results back (if step Disparity filter was applied
          to depth domain (each post processing block is optional and can be applied independantly).
          */
          filtered = filtered.apply_filter(dec);
          intrinsics = filtered.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
          filtered = filtered.apply_filter(hdr);
          filtered = filtered.apply_filter(thresh);
          filtered = filtered.apply_filter(depth2disparity);
          filtered = filtered.apply_filter(spat);
          filtered = filtered.apply_filter(disparity2depth);

          // Push filtered & original data to their respective queues
          // Note, pushing to two different queues might cause the application to display
          //  original and filtered pointclouds from different depth frames
          //  To make sure they are synchronized you need to push them together or add some
          //  synchronization mechanisms
          filtered_data.enqueue(filtered);
          selected_data.enqueue(filtered);
          original_data.enqueue(depth_frame);
        }
      });

  // Declare objects that will hold the calculated pointclouds and colored frames
  // We save the last set of data to minimize flickering of the view
  rs2::frame colored_depth;
  rs2::frame colored_filtered;
  rs2::points original_points;
  rs2::points filtered_points;
  std::vector<rs2::vertex> selected_points;

  // Save the time of last frame's arrival
  auto last_time = std::chrono::high_resolution_clock::now();

  // We'll use rotation_velocity to rotate the pointcloud for a better view of the filters effects
  float rotation_velocity = 0.3f;

  while(app)
  {
    float w = static_cast<float>(app.width());
    float h = static_cast<float>(app.height());

    // Try to get new data from the queues and update the view with new texture
    update_data(original_data, colored_depth, original_points, original_pc, original_view_orientation, color_map);
    update_data(filtered_data, colored_filtered, filtered_points, filtered_pc, filtered_view_orientation, color_map);
    // update_data(filtered_data, colored_filtered, filtered_points, filtered_pc, side_view_orientation, color_map);
    update_data_and_compute(selected_data, selected_points);

    // Draw the pointclouds of the original and the filtered frames (if the are available already)
    if(colored_depth && original_points)
    {
      glViewport(int(w) / 2, int(h) / 2, int(w) / 2, int(h) / 2);
      draw_pointcloud(int(w) / 2.f, int(h) / 2.f, original_view_orientation, original_points, int(w) / 2.f / 640.f);
    }
    if(colored_filtered && filtered_points)
    {
      glViewport(int(w) / 2, 0, int(w) / 2, int(h) / 2);
      draw_pointcloud(int(w) / 2.f, int(h) / 2.f, filtered_view_orientation, filtered_points, int(w) / 2.f / 240.f);
    }
    if(colored_filtered && filtered_points)
    {
      // glViewport(0, int(h) / 2, int(w) / 2, int(h) / 2);
      glViewport(0, 0, int(w) / 2, int(h));
      draw_pointcloud(int(w) / 2.f, int(h) / 2.f, filtered_view_orientation, filtered_points, selected_points, int(w) / 2.f / 240.f);
    }
    // if(colored_filtered && filtered_points)
    // {
    //   glViewport(0, 0, int(w) / 2, int(h) / 2);
    //   draw_pointcloud(int(w) / 2.f, int(h) / 2.f, side_view_orientation, filtered_points, selected_points, int(w) / 2.f / 240.f);
    // }
    // Update time of current frame's arrival
    auto curr = std::chrono::high_resolution_clock::now();

    // Time interval which must pass between movement of the pointcloud
    const std::chrono::milliseconds rotation_delta(40);

    // In order to calibrate the velocity of the rotation to the actual displaying speed, rotate
    //  pointcloud only when enough time has passed between frames
    if(curr - last_time > rotation_delta)
    {
      if(filtered_view_orientation.yaw < -30 || filtered_view_orientation.yaw > -20)
      {
        rotation_velocity = -rotation_velocity;
      }
      original_view_orientation.yaw += rotation_velocity;
      filtered_view_orientation.yaw += rotation_velocity;
      side_view_orientation.yaw += rotation_velocity;
      last_time = curr;
    }
  }

  // Signal the processing thread to stop, and join
  // (Not the safest way to join a thread, please wrap your threads in some RAII manner)
  stopped = true;
  processing_thread.join();

  return EXIT_SUCCESS;
}
catch(const rs2::error & e)
{
  std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    "
            << e.what() << std::endl;
  return EXIT_FAILURE;
}
catch(const std::exception & e)
{
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}

void update_data(rs2::frame_queue & data,
                 rs2::frame & colorized_depth,
                 rs2::points & points,
                 rs2::pointcloud & pc,
                 glfw_state & view,
                 rs2::colorizer & color_map)
{
  rs2::frame f;
  if(data.poll_for_frame(&f)) // Try to take the depth and points from the queue
  {
    points = pc.calculate(f); // Generate pointcloud from the depth data
    colorized_depth = color_map.process(f); // Colorize the depth frame with a color map
    pc.map_to(colorized_depth); // Map the colored depth to the point cloud
    view.tex.upload(colorized_depth); //  and upload the texture to the view (without this the view will be B&W)
  }
}

void update_data_and_compute(rs2::frame_queue & data,
                 std::vector<rs2::vertex> & points)
{
  size_t kernel_size_ = 5;
  const int half_kernel_size = static_cast<int>(kernel_size_/2);
  float kernel_threshold_ = 0.005f;
  float outlier_threshold_ = 0.01f;

  auto applyKernel = [&](const std::array<float, 2>& pixel, const rs2::depth_frame& frame) -> float
  {
    const float pixel_depth = frame.get_distance(static_cast<int>(pixel[0]), static_cast<int>(pixel[1]));
    float sum_depth = 0.f;
    float counter = 0.f;
    for(int i = -half_kernel_size; i < half_kernel_size; ++i)
    {
      for(int j = -half_kernel_size; j < half_kernel_size; ++j)
      {
        const std::array<float, 2> ppixel = {pixel[0] + static_cast<float>(i), pixel[1] + static_cast<float>(j)};
        const float ddepth = frame.get_distance(static_cast<int>(ppixel[0]), static_cast<int>(ppixel[1]));
        if(std::abs(ddepth - pixel_depth) < kernel_threshold_)
        {
          sum_depth += ddepth;
          counter += 1.f;
        }
        if(std::abs(ddepth - pixel_depth) > outlier_threshold_)
        {
          return 0.f;
        }
      }
    }
    return sum_depth / counter;
  };

  auto depthToPoint = [](const std::array<float, 2>& pixel, float depth) -> Eigen::Vector3d
    {
      Eigen::Vector3f point;
      rs2_deproject_pixel_to_point(point.data(), &intrinsics, pixel.data(), depth);
      return point.cast<double>();
    };

  rs2::frame f;
  if(data.poll_for_frame(&f)) // Try to take the depth and points from the queue
  {
    points.clear();

    auto frame = f.as<rs2::depth_frame>();

    const size_t height = frame.get_height();
    const size_t width = frame.get_width();

    const size_t half_height = static_cast<size_t>(static_cast<double>(height) * 0.5);
    const size_t half_width = static_cast<size_t>(static_cast<double>(width) * 0.5);

    std::array<float, 2> pixel;
    size_t offset = 0;
    for(size_t i = half_kernel_size - offset; i < width - half_kernel_size - offset; ++i)
    {
      pixel[0] = static_cast<float>(i);
      pixel[1] = static_cast<float>(half_height);

      const float depth = applyKernel(pixel, frame);
      const Eigen::Vector3d point = depthToPoint(pixel, depth);
      // TODO: Add a post check for noisy data?
      if(point.z() != 0)
      {
        rs2::vertex vertex;
        vertex.x = point.x();
        vertex.y = point.y();
        vertex.z = point.z();
        points.push_back(vertex);
      }
    }
  }
}