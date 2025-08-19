#include <iostream>
#include <chrono>
#include <unordered_map>
#include <algorithm>
#include <vector>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

constexpr size_t NR_POLYNOMIAL_COEFFS = 4;

namespace
{

template<size_t polynomial_coeff_size>
struct CostFunctor
{
  CostFunctor(double x, double y)
  : x_(x),
    y_(y)
  {}

  template <typename T>
  bool operator()(const T* const polynomial_coeff, T* residuals) const
  {
    T _x = T(x_);
    T _y = T(y_);
    T sum_poly = polynomial_coeff[0];
    T x_pow = _x;
    for(size_t i = 1; i < polynomial_coeff_size; ++i)
    {
      sum_poly = sum_poly + (polynomial_coeff[i] * x_pow);
      x_pow = x_pow * _x;
    }
    residuals[0] = _y - sum_poly;
    return true;
  }

   // Factory to hide the construction of the CostFunction object from
   // the client code.
   static ceres::CostFunction* Create(double x, double y)
   {
     return (new ceres::AutoDiffCostFunction<CostFunctor, 1, 4>(new CostFunctor(x, y)));
   }

private:
  const double x_;
  const double y_;
};

}

double fn(double x, const std::array<double, NR_POLYNOMIAL_COEFFS>& coeff)
{
  double res = coeff[0];
  double x_pow = x;
  for(size_t i = 1; i < NR_POLYNOMIAL_COEFFS; ++i)
  {
    res += coeff[i] * x_pow;
    x_pow = x_pow * x;
  }
  return res;
}


int main(int argc, char *argv[])
{
  // Define x where it is the measured distance
  std::vector<double> x = {0., 98., 176.,365., 420., 480.};
  // Define y where it is the theoritical distance/ground truth distance
  std::vector<double> y = {0., 100., 200., 300., 400., 500.};

  // Polynimial coffecients to find

  std::array<double, NR_POLYNOMIAL_COEFFS> coeff;
  for(size_t i = 0; i < NR_POLYNOMIAL_COEFFS; ++i)
  {
    coeff[i] = 0.;
  }

  ceres::Problem problem;
  for(size_t i = 0; i < x.size(); ++i)
  {
    ceres::CostFunction* cost_function = CostFunctor<NR_POLYNOMIAL_COEFFS>::Create(x[i], y[i]);
    problem.AddResidualBlock(cost_function,  new ceres::CauchyLoss(0.5), coeff.data());
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::vector<std::string> coeff_name = {"a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m", "n", "o", "p", "q", "r", "s", "t", "u", "v", "w", "x", "y", "z"};
  for(size_t i = 0; i < NR_POLYNOMIAL_COEFFS; ++i)
  {
    std::cout << "double " << coeff_name[i] << " = " << coeff[i] << ";" << std::endl;
  }

  for(double x = 0.; x < 550.0; x += 5.0)
  {
    std::cout << x << " - " << fn(x, coeff) << std::endl;
  }

  return 0; // success
}