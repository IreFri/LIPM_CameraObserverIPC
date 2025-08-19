#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/NumberInput.h>

#include <TrajColl/CubicSpline.h>

#include <lipm_walking/swing/SwingTrajCubicSplineSimple.h>

using namespace BWC;

void SwingTrajCubicSplineSimple::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  SwingTraj::Configuration::load(mcRtcConfig);

  mcRtcConfig("withdrawDurationRatio", withdrawDurationRatio);
  mcRtcConfig("withdrawOffset", withdrawOffset);
  mcRtcConfig("approachDurationRatio", approachDurationRatio);
  mcRtcConfig("approachOffset", approachOffset);
  mcRtcConfig("swingOffset", swingOffset);
}

void SwingTrajCubicSplineSimple::loadDefaultConfig(const mc_rtc::Configuration & mcRtcConfig)
{
  defaultConfig_.load(mcRtcConfig);
}

void SwingTrajCubicSplineSimple::addConfigToGUI(mc_rtc::gui::StateBuilder & gui,
                                                const std::vector<std::string> & category)
{
  gui.addElement(
      category,
      mc_rtc::gui::NumberInput(
          "withdrawDurationRatio", []() { return defaultConfig_.withdrawDurationRatio; },
          [](double v) { defaultConfig_.withdrawDurationRatio = v; }),
      mc_rtc::gui::ArrayInput(
          "withdrawOffset", {"x", "y", "z"}, []() -> const Eigen::Vector3d & { return defaultConfig_.withdrawOffset; },
          [](const Eigen::Vector3d & v) { defaultConfig_.withdrawOffset = v; }),
      mc_rtc::gui::NumberInput(
          "approachDurationRatio", []() { return defaultConfig_.approachDurationRatio; },
          [](double v) { defaultConfig_.approachDurationRatio = v; }),
      mc_rtc::gui::ArrayInput(
          "approachOffset", {"x", "y", "z"}, []() -> const Eigen::Vector3d & { return defaultConfig_.approachOffset; },
          [](const Eigen::Vector3d & v) { defaultConfig_.approachOffset = v; }),
      mc_rtc::gui::ArrayInput(
          "swingOffset", {"x", "y", "z"}, []() -> const Eigen::Vector3d & { return defaultConfig_.swingOffset; },
          [](const Eigen::Vector3d & v) { defaultConfig_.swingOffset = v; }));
}

void SwingTrajCubicSplineSimple::removeConfigFromGUI(mc_rtc::gui::StateBuilder & gui,
                                                     const std::vector<std::string> & category)
{
  gui.removeCategory(category);
}


SwingTrajCubicSplineSimple::SwingTrajCubicSplineSimple(const sva::PTransformd & startPose,
                                                       const sva::PTransformd & endPose,
                                                       double startTime,
                                                       double endTime,
                                                       const TaskGain & taskGain,
                                                       const mc_rtc::Configuration & mcRtcConfig)
: SwingTraj(startPose, endPose, startTime, endTime, taskGain, mcRtcConfig),
  posFunc_(std::make_shared<TrajColl::PiecewiseFunc<Eigen::Vector3d>>()),
  rotFunc_(std::make_shared<TrajColl::CubicInterpolator<Eigen::Matrix3d, Eigen::Vector3d>>())
{
  config_.load(mcRtcConfig);

  posFunc_->clearFuncs();
  rotFunc_->clearPoints();

  double withdrawDuration = config_.withdrawDurationRatio * (endTime_ - startTime_);
  double approachDuration = config_.approachDurationRatio * (endTime_ - startTime_);

  const double new_angle_in_deg = -5.0;
  const Eigen::Matrix3d R = mc_rbdyn::rpyToMat(0., new_angle_in_deg * M_PI / 180., 0.);
  const sva::PTransformd R_offset(R, Eigen::Vector3d::Zero());

  TrajColl::BoundaryConstraint<Eigen::Vector3d> zeroVelBC(TrajColl::BoundaryConstraintType::Velocity,
                                                          Eigen::Vector3d::Zero());
  TrajColl::BoundaryConstraint<Eigen::Vector3d> zeroAccelBC(TrajColl::BoundaryConstraintType::Acceleration,
                                                            Eigen::Vector3d::Zero());

  // Spline to withdraw foot
  // Pos
  std::map<double, Eigen::Vector3d> withdrawPosWaypoints = {
      {startTime_, startPose_.translation()},
      {startTime_ + withdrawDuration, (sva::PTransformd(config_.withdrawOffset) * startPose_).translation()}};
  auto withdrawPosSpline =
      std::make_shared<TrajColl::CubicSpline<Eigen::Vector3d>>(3, zeroVelBC, zeroAccelBC, withdrawPosWaypoints);
  withdrawPosSpline->calcCoeff();
  posFunc_->appendFunc(startTime_ + withdrawDuration, withdrawPosSpline);
  // Rot HERE I CHANGED
  rotFunc_->appendPoint(std::make_pair(startTime_, startPose_.rotation().transpose()));
  // rotFunc_->appendPoint(std::make_pair(startTime_ + withdrawDuration, startPose_.rotation().transpose()));
  rotFunc_->appendPoint(std::make_pair(startTime_ + withdrawDuration, (R_offset * startPose_).rotation().transpose()));

  // Spline to approach foot
  // Pos
  std::map<double, Eigen::Vector3d> approachPosWaypoints = {
      {endTime_ - approachDuration, (sva::PTransformd(config_.approachOffset) * endPose_).translation()},
      {endTime_, endPose_.translation()}};
  auto approachPosSpline =
      std::make_shared<TrajColl::CubicSpline<Eigen::Vector3d>>(3, zeroAccelBC, zeroVelBC, approachPosWaypoints);
  approachPosSpline->calcCoeff();
  posFunc_->appendFunc(endTime_, approachPosSpline);
  // Rot HERE I CHANGED
  // rotFunc_->appendPoint(std::make_pair(endTime_ - approachDuration, endPose_.rotation().transpose()));
  rotFunc_->appendPoint(std::make_pair(endTime_ - approachDuration, (R_offset * endPose_).rotation().transpose()));
  rotFunc_->appendPoint(std::make_pair(endTime_, endPose_.rotation().transpose()));

  // Spline to swing foot
  // Pos
  std::map<double, Eigen::Vector3d> swingPosWaypoints = {
      *withdrawPosWaypoints.rbegin(),
      {0.5 * (startTime_ + endTime_),
       (sva::PTransformd(config_.swingOffset) * sva::interpolate(startPose_, endPose_, 0.5)).translation()},
      *approachPosWaypoints.begin()};
  auto swingPosSpline = std::make_shared<TrajColl::CubicSpline<Eigen::Vector3d>>(
      3,
      TrajColl::BoundaryConstraint<Eigen::Vector3d>(TrajColl::BoundaryConstraintType::Velocity,
                                                    withdrawPosSpline->derivative(startTime_ + withdrawDuration, 1)),
      TrajColl::BoundaryConstraint<Eigen::Vector3d>(TrajColl::BoundaryConstraintType::Velocity,
                                                    approachPosSpline->derivative(endTime_ - approachDuration, 1)),
      swingPosWaypoints);
  swingPosSpline->calcCoeff();
  posFunc_->appendFunc(endTime_ - approachDuration, swingPosSpline);
  // Rot
  rotFunc_->calcCoeff();
}

void SwingTrajCubicSplineSimple::update(double pitch, double x_offset, double z_offset)
{
  const sva::PTransformd newEndPose = sva::PTransformd(Eigen::Vector3d(x_offset, 0., z_offset)) * endPose_;
  endPose_ = newEndPose;
  Eigen::Matrix3d rotOffset = mc_rbdyn::rpyToMat(Eigen::Vector3d(0., pitch, 0.));
  endPose_ = sva::PTransformd(rotOffset) * endPose_;
  compute();
}

void SwingTrajCubicSplineSimple::compute()
{
  // Keep current timing
  // t_ = t; in pose/vel/accel

  // Get current pose on the trajectory
  std::cout << "Update with: " << t_ << std::endl;
  std::cout << "endTime_: " << endTime_ << std::endl;
  
  const sva::PTransformd X_0_P((*rotFunc_)(t_).transpose(), (*posFunc_)(t_));
  
  std::cout << (*posFunc_)(t_).transpose() << std::endl;
  
  const Eigen::Vector3d V_0_P = posFunc_->derivative(t_, 1);

  std::cout << V_0_P.transpose() << std::endl;

  // Reset Funcs
  posFunc_->clearFuncs();
  rotFunc_->clearPoints();

  std::map<double, Eigen::Vector3d> approachPosWaypoints = {
    {t_, X_0_P.translation()}, // Current pos at current time t_
    {endTime_, endPose_.translation()}};

  auto approachPosSpline = std::make_shared<TrajColl::CubicSpline<Eigen::Vector3d>>(
    3,
    TrajColl::BoundaryConstraint<Eigen::Vector3d>(TrajColl::BoundaryConstraintType::Velocity, V_0_P),
    TrajColl::BoundaryConstraint<Eigen::Vector3d>(TrajColl::BoundaryConstraintType::Velocity, Eigen::Vector3d::Zero()),
    approachPosWaypoints);

  // We need to update both Func (pos and rot)
  approachPosSpline->calcCoeff();
  posFunc_->appendFunc(endTime_, approachPosSpline);
  // Rot
  rotFunc_->appendPoint(std::make_pair(t_, X_0_P.rotation().transpose()));
  rotFunc_->appendPoint(std::make_pair(t_ + (endTime_ - t_) * 0.5, endPose_.rotation().transpose()));
  rotFunc_->appendPoint(std::make_pair(endTime_, endPose_.rotation().transpose()));
  rotFunc_->calcCoeff();

  std::cout << (*posFunc_)(t_).transpose() << std::endl;
}

sva::PTransformd SwingTrajCubicSplineSimple::pose(double t) const
{
  if(touchDownTime_ > 0 && t >= touchDownTime_)
  {
    t = touchDownTime_;
  }
  t_ = t;
  return sva::PTransformd((*rotFunc_)(t).transpose(), (*posFunc_)(t));
}

sva::MotionVecd SwingTrajCubicSplineSimple::vel(double t) const
{
  if(touchDownTime_ > 0 && t >= touchDownTime_)
  {
    t_ = touchDownTime_;
    return sva::MotionVecd::Zero();
  }
  else
  {
    t_ = t;
    return sva::MotionVecd(rotFunc_->derivative(t, 1), posFunc_->derivative(t, 1));
  }
}

sva::MotionVecd SwingTrajCubicSplineSimple::accel(double t) const
{
  if(touchDownTime_ > 0 && t >= touchDownTime_)
  {
    t_ = touchDownTime_;
    return sva::MotionVecd::Zero();
  }
  else
  {
    t_ = t;
    return sva::MotionVecd(rotFunc_->derivative(t, 2), posFunc_->derivative(t, 2));
  }
}