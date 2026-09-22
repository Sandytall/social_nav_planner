// Small angle helpers (MASTER_PROMPT §51: clear interfaces, minimal state).
#ifndef SOCIAL_NAV_CORE__ANGLES_HPP_
#define SOCIAL_NAV_CORE__ANGLES_HPP_

#include <cmath>

namespace social_nav_core
{

/// Wrap an angle to (-pi, pi]. Handles arbitrarily large magnitudes.
inline double normalizeAngle(double angle)
{
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle <= 0.0) {
    angle += 2.0 * M_PI;
  }
  return angle - M_PI;
}

/// Smallest signed difference a - b, wrapped to (-pi, pi].
inline double shortestAngularDistance(double from, double to)
{
  return normalizeAngle(to - from);
}

}  // namespace social_nav_core

#endif  // SOCIAL_NAV_CORE__ANGLES_HPP_
