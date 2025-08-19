#pragma once
#include <Eigen/Core>
#include <ostream>

namespace mc_plugin
{

namespace ExternalFootstepPlanner
{

struct SE2d
{
  double x = 0;
  double y = 0;
  double theta = 0;
  SE2d() {}
  SE2d(double x, double y, double theta) : x(x), y(y), theta(theta) {}
  SE2d(const Eigen::Vector3d & v) : x(v.x()), y(v.y()), theta(v.z()) {}
};

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin

inline std::ostream & operator<<(std::ostream & os, const mc_plugin::ExternalFootstepPlanner::SE2d & se2d)
{
  os << se2d.x << ", " << se2d.y << ", " << se2d.theta;
  return os;
}
