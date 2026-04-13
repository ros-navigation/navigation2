// Copyright (c) 2025 Nav2 Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CLOTHOID_PLANNER__CLOTHOID_MATH_HPP_
#define CLOTHOID_PLANNER__CLOTHOID_MATH_HPP_

#include <array>
#include <cmath>

namespace nav2_geometric_planners
{

/**
 * @brief Evaluate generalised Fresnel integrals C(a,b,c) and S(a,b,c).
 *
 * Computes:
 *   C = integral_0^1  cos( (a*t^2)/2 + b*t + c ) dt
 *   S = integral_0^1  sin( (a*t^2)/2 + b*t + c ) dt
 *
 * Uses a Gauss–Legendre 20-point quadrature for accurate evaluation.
 *
 * @param a Quadratic coefficient.
 * @param b Linear coefficient.
 * @param c Constant offset.
 * @param C Output cosine integral.
 * @param S Output sine integral.
 */
inline void fresnelCS(double a, double b, double c, double & C, double & S)
{
  // 20-point Gauss-Legendre nodes and weights on [-1, 1]
  static const std::array<double, 10> nodes = {
    0.0765265211334973, 0.2277858511416451,
    0.3737060887154195, 0.5108670019508271,
    0.6360536807265150, 0.7463064833401990,
    0.8391169718222188, 0.9122344282513259,
    0.9639719272779138, 0.9931285991850949
  };
  static const std::array<double, 10> weights = {
    0.1527533871307258, 0.1491729864726037,
    0.1420961093183820, 0.1316886384491766,
    0.1181945319615184, 0.1019301198172404,
    0.0832767415767048, 0.0626720483341091,
    0.0406014298003869, 0.0176140071391521
  };

  C = 0.0;
  S = 0.0;

  // Map quadrature from [-1,1] to [0,1]: t = (1 + xi) / 2
  for (int i = 0; i < 10; ++i) {
    const double t_pos = 0.5 * (1.0 + nodes[i]);
    const double t_neg = 0.5 * (1.0 - nodes[i]);
    const double w = 0.5 * weights[i];

    const double arg_pos = (a * t_pos * t_pos) / 2.0 + b * t_pos + c;
    const double arg_neg = (a * t_neg * t_neg) / 2.0 + b * t_neg + c;

    C += w * (std::cos(arg_pos) + std::cos(arg_neg));
    S += w * (std::sin(arg_pos) + std::sin(arg_neg));
  }
}

/**
 * @brief A single clothoid segment parameterised by arc length.
 *
 * The curvature varies linearly: kappa(s) = kappa0_ + dkappa_ * s.
 * Consequently, heading: theta(s) = theta0_ + kappa0_*s + dkappa_*s^2/2.
 */
struct ClothoidSegment
{
  double x0_{0};      ///< Start x-coordinate.
  double y0_{0};      ///< Start y-coordinate.
  double theta0_{0};  ///< Start heading (radians).
  double kappa0_{0};  ///< Initial curvature.
  double dkappa_{0};  ///< Rate of curvature change per unit arc length.
  double length_{0};  ///< Total arc length of the segment.

  /**
   * @brief Build a G1-continuous clothoid connecting two poses.
   *
   * Uses a Newton–Raphson solver to find (length, kappa0, dkappa) such that
   * the clothoid starting at (x0,y0,theta0) reaches (x1,y1,theta1).
   *
   * @return True if the solver converged; false otherwise.
   */
  bool buildG1(
    double x0, double y0, double theta0,
    double x1, double y1, double theta1)
  {
    x0_ = x0;
    y0_ = y0;
    theta0_ = theta0;

    // Transform endpoint to canonical frame (start = origin, theta0 = 0).
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double phi = std::atan2(dy, dx);
    const double d = std::hypot(dx, dy);

    if (d < 1e-10) {
      length_ = 0.0;
      kappa0_ = 0.0;
      dkappa_ = 0.0;
      return true;
    }

    // Angles relative to chord direction.
    const double alpha = theta0 - phi;
    const double beta = theta1 - phi;
    const double dtheta = theta1 - theta0;

    // Initial guess: unit-length clothoid with half-angle parameterisation.
    double L = d;
    double k0 = (6.0 * alpha + 2.0 * beta) / L;
    double dk = (-12.0 * alpha - 6.0 * beta) / (L * L) + 6.0 * dtheta / (L * L);

    // Newton iterations.
    for (int iter = 0; iter < 20; ++iter) {
      const double a = dk * L * L;
      const double b = k0 * L;

      double Cf{}, Sf{}, Cf2{}, Sf2{};
      fresnelCS(a, b, theta0, Cf, Sf);       // [0,1] integral scaled by L
      fresnelCS(a, b + a, theta0, Cf2, Sf2);  // derivative helper

      // Residuals: x_end - x1, y_end - y1, theta_end - theta1 (normalised by d).
      const double x_end = x0 + L * Cf;
      const double y_end = y0 + L * Sf;
      const double th_end = theta0 + k0 * L + dk * L * L / 2.0;

      const double rx = x_end - x1;
      const double ry = y_end - y1;
      const double rth = th_end - theta1;

      if (std::abs(rx) < 1e-9 && std::abs(ry) < 1e-9 && std::abs(rth) < 1e-9) {
        break;
      }

      // Approximate Jacobian by finite differences (robust for our use case).
      const double eps = 1e-6;

      double Cf_dL{}, Sf_dL{};
      const double a_dL = dk * (L + eps) * (L + eps);
      const double b_dL = k0 * (L + eps);
      fresnelCS(a_dL, b_dL, theta0, Cf_dL, Sf_dL);
      const double dxdL = (( L + eps) * Cf_dL - L * Cf) / eps;
      const double dydL = (( L + eps) * Sf_dL - L * Sf) / eps;
      const double dthdL = k0 + dk * L;

      double Cf_dk0{}, Sf_dk0{};
      fresnelCS(a, b + eps * L, theta0, Cf_dk0, Sf_dk0);
      const double dxdk0 = L * (Cf_dk0 - Cf) / eps;
      const double dydk0 = L * (Sf_dk0 - Sf) / eps;
      const double dthdk0 = L;

      double Cf_ddk{}, Sf_ddk{};
      fresnelCS(a + eps * L * L, b, theta0, Cf_ddk, Sf_ddk);
      const double dxddk = L * (Cf_ddk - Cf) / eps;
      const double dyddk = L * (Sf_ddk - Sf) / eps;
      const double dthddk = L * L / 2.0;

      // Solve 3x3 linear system J * delta = -r via Cramer's rule.
      const double det =
        dxdL  * (dydk0 * dthddk - dyddk * dthdk0) -
        dxdk0 * (dydL  * dthddk - dyddk * dthdL) +
        dxddk * (dydL  * dthdk0 - dydk0 * dthdL);

      if (std::abs(det) < 1e-20) {break;}

      const double dL = -(
        rx    * (dydk0 * dthddk - dyddk * dthdk0) -
        dxdk0 * (ry    * dthddk - dyddk * rth) +
        dxddk * (ry    * dthdk0 - dydk0 * rth)) / det;

      const double dk0 = -(
        dxdL  * (ry    * dthddk - dyddk * rth) -
        rx    * (dydL  * dthddk - dyddk * dthdL) +
        dxddk * (dydL  * rth   - ry    * dthdL)) / det;

      const double ddk = -(
        dxdL  * (dydk0 * rth   - ry    * dthdk0) -
        dxdk0 * (dydL  * rth   - ry    * dthdL) +
        rx    * (dydL  * dthdk0 - dydk0 * dthdL)) / det;

      L  += dL;
      k0 += dk0;
      dk += ddk;

      if (L < 1e-10) {L = 1e-10;}
    }

    length_ = L;
    kappa0_ = k0;
    dkappa_ = dk;
    return true;
  }

  /**
   * @brief Evaluate Cartesian position at arc length s.
   * @param s Arc length from start (clamped to [0, length_]).
   * @param x Output x-coordinate.
   * @param y Output y-coordinate.
   */
  void eval(double s, double & x, double & y) const
  {
    if (length_ < 1e-10) {x = x0_; y = y0_; return;}
    s = std::max(0.0, std::min(s, length_));

    const double a = dkappa_ * s * s;
    const double b = kappa0_ * s;
    double C{}, S{};
    fresnelCS(a, b, theta0_, C, S);

    x = x0_ + s * C;
    y = y0_ + s * S;
  }

  /**
   * @brief Evaluate heading (radians) at arc length s.
   * @param s Arc length from start.
   * @return Heading angle in radians.
   */
  double theta(double s) const
  {
    return theta0_ + kappa0_ * s + dkappa_ * s * s / 2.0;
  }

  /**
   * @brief Evaluate curvature at arc length s.
   * @param s Arc length from start.
   * @return Signed curvature (rad/m).
   */
  double kappa(double s) const
  {
    return kappa0_ + dkappa_ * s;
  }
};

}  // namespace nav2_geometric_planners

#endif  // CLOTHOID_PLANNER__CLOTHOID_MATH_HPP_
