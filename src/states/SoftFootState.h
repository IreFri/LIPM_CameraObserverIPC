#pragma once

#include <lipm_walking/State.h>

#include <mc_rtc/ros.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <mutex>
#include <memory>


// https://github.com/embeddedartistry/embedded-resources/blob/master/examples/cpp/circular_buffer.cpp
template<class T>
class Circular_Buffer
{
public:
	Circular_Buffer() = default;

  Circular_Buffer(size_t max_size)
  : TElemCount(max_size),
    buf_(TElemCount)
  {}

	void enqueue(T item) noexcept
	{
		buf_[head_] = item;

		if(full_)
		{
			tail_ = (tail_ + 1) % TElemCount;
		}

		head_ = (head_ + 1) % TElemCount;

		full_ = head_ == tail_;
	}

	std::optional<T> dequeue() const noexcept
	{
		if(empty())
		{
			return std::nullopt;
		}

		// Read data and advance the tail (we now have a free space)
		auto val = buf_[tail_];
		full_ = false;
		tail_ = (tail_ + 1) % TElemCount;

		return val;
	}

	void reset() noexcept
	{
		head_ = tail_;
		full_ = false;
	}

  void resize(size_t new_size) noexcept
	{
    TElemCount = new_size;
    buf_ = std::vector<T>(TElemCount);
    reset();
	}

	bool empty() const noexcept
	{
		// if head and tail are equal, we are empty
		return (!full_ && (head_ == tail_));
	}

	bool full() const noexcept
	{
		// If tail is ahead the head by 1, we are full
		return full_;
	}

	size_t capacity() const noexcept
	{
		return TElemCount;
	}

	size_t size() const noexcept
	{
		size_t size = TElemCount;

		if(!full_)
		{
			if(head_ >= tail_)
			{
				size = head_ - tail_;
			}
			else
			{
				size = TElemCount + head_ - tail_;
			}
		}

		return size;
	}

private:
  mutable size_t TElemCount;
	mutable std::vector<T> buf_;
	mutable size_t head_ = 0;
	mutable size_t tail_ = 0;
	mutable bool full_ = 0;
};

namespace lipm_walking::states
{

enum class Foot
{
  //! Left foot
  Left = 0,

  //! Right foot
  Right
};




/** \brief FSM state to calculate ground profile and calculate sole stiffness online. */
struct SoftFootState : State
{
public:
  void configure(const mc_rtc::Configuration & config) override
  {
    config_.load(config);
  }

  /** \brief Start. */
  void start() override;

  /** \brief checkTransitions. */
  bool checkTransitions() override;

  /** \brief runState. */
  void runState() override;

  /** \brief Teardown. */
  void teardown() override;

protected:

  void calculateCost(mc_control::fsm::Controller & ctl);

  void estimateGround(mc_control::fsm::Controller & ctl, const Foot & current_moving_foot);

  void extractGroundSegment(mc_control::fsm::Controller & ctl, const Foot & current_moving_foot, const Eigen::Vector3d & landing);

  void extractAltitudeProfileFromGroundSegment(const Foot & current_moving_foot);

  void updateVariableStiffness(mc_control::fsm::Controller & ctl, const Foot & current_moving_foot, bool open_valve);

  void computeSegmentConvexHull(mc_control::fsm::Controller & ctl, const Foot & current_moving_foot);

  std::vector<Eigen::Vector3d> computeConvexHull(const std::vector<Eigen::Vector3d>& data);

  void computeMinMaxAngle(const Foot & current_moving_foot);

  void computeFootLandingPosition(const Foot & current_moving_foot, const Eigen::Vector3d & landing);

  void computeFootLandingAngle(const Foot & current_moving_foot, const Eigen::Vector3d & landing);

  void updateFootSwingPose(mc_control::fsm::Controller & ctl, const Foot & current_moving_foot, const sva::PTransformd & X_0_landing);

  Foot getCurrentMovingFoot(mc_control::fsm::Controller & ctl)
  {
    Foot current_moving_foot;
    if(controller().supportContact().surfaceName == "LeftFootCenter")
    {
      current_moving_foot = Foot::Right;
    }
    else
    {
      current_moving_foot = Foot::Left;
    }
    return current_moving_foot;
  }

  bool evaluateTheta(double theta, double direction, const Eigen::Vector3d& pos, const std::vector<Eigen::Vector3d>& ground, Eigen::Vector2d& output);

protected:
  // Used to compute the cost to understand which is the best stiffness
  double lambda_zmp_ = 3.0;
  std::vector<double> zmp_;

  double lambda_CoM_ = 3.0;
  std::vector<double> CoM_;

  double nr_remaining_footstep_ = 0.0;
  double nr_footstep_ = 0.0;
  double lambda_footstep_ = 10.0;

  double cost_ = 0.0;
  double PhalangesStiffness_ = 0.0;
  double pressure_ = 0.0;
  double ML_ = 0.0;
  double MR_ = 0.0;
  int WhichFoot_;
  double D_;

  double extra_to_compute_best_position_ = 0.01;

  // FootData contains data used to estimate the ground profile
  struct FootData
  {
    std::vector<Eigen::Vector3d> ground;
    Eigen::Vector3d last_ground;
    Eigen::Vector3d last_ground_control;
    Eigen::Vector3d last_ground_Identity;
    std::vector<std::array<double, 2>> phalanxes;
    std::vector<double> altitude;
    double k = 0;
    double pressure = 0; //17 for HS
    double angle;
    double min_max_phalanxes_angle;
    double position_offset_x;
    double position_offset_z;
    bool need_reset;
    bool computation_done;
    int valves_status = 0;
  };
  std::unordered_map<Foot, FootData> foot_data_;

  // Data for GroundSegment to compute best position/orientation
  struct GroundSegment
  {
    std::vector<Eigen::Vector3d> raw; // means all the data
    std::vector<Eigen::Vector3d> filtered;
    std::vector<Eigen::Vector3d> convex;
  };
  std::unordered_map<Foot, GroundSegment> ground_segment_;

  std::unordered_map<Foot, std::string> variable_stiffness_jointname_ = {
    {Foot::Left, "L_VARSTIFF"},
    {Foot::Right, "R_VARSTIFF"}
  };

  std::unordered_map<Foot, std::string> surface_name_ = {
    {Foot::Left, "LeftFootCenter"},
    {Foot::Right, "RightFootCenter"}
  };

  // Initialized by .yaml
  std::unordered_map<Foot, std::vector<std::string>> range_sensor_names_;
  std::unordered_map<Foot, std::unordered_map<std::string, double>> range_sensor_data_;

  // Keep preivous poses for ground estimation
  double delta_delay_of_estimation_ = 0.;
  double fixed_delay_of_estimation_ = 0.;
  bool use_real_robot_for_estimation_ = true;
  std::unordered_map<Foot, Circular_Buffer<sva::PTransformd>> past_foot_pose_;
  std::unordered_map<Foot, Circular_Buffer<sva::PTransformd>> past_foot_pose_control_;

  // TODO: Ugly hardcoded value
  // double foot_length_ = 0.27742;
  double foot_length_ = 0.15;
  double landing_to_foot_middle_offset_ = 0.0;
  size_t nr_phalanxes_;
  double phalanx_length_;
  double time_ = 0.; // controller time

  // Reset data
  void reset(mc_control::fsm::Controller & ctl, const Foot & foot);

  // Client is here to call the service to compute the stiffness based on the ground profile
  ros::ServiceClient client_;

  // options
  bool with_variable_stiffness_ = false;
  bool with_ankle_rotation_ = false;
  bool with_foot_adjustment_ = false;
  bool use_camera_sensor_ = false;

  // Subscriber to update the range sensor
  ros::Subscriber right_foot_range_sensor_sub_;
  void rightFRSCallback(const std_msgs::Float64::ConstPtr& data);

  ros::Subscriber left_foot_range_sensor_sub_;
  void leftFRSCallback(const std_msgs::Float64::ConstPtr& data);

  // Subscriber to update the air pressure for variable stiffness
  ros::Subscriber air_pressure_sub_;
  void airPressureCallback(const std_msgs::Float64MultiArray::ConstPtr& data);

  std::mutex range_sensor_mutex_;
  std::mutex variable_stiffness_mutex_;

  bool debug_output_ = false;

  std::string current_state_;

};

} // namespace lipm_walking::states
