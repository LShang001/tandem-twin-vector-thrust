#pragma once

#include <cmath>

inline bool InsComputeAbsoluteBaroAltitude(
    const double takeoff_origin_altitude_m,
    const float relative_altitude_up_m,
    double *absolute_altitude_m)
{
  if (absolute_altitude_m == nullptr ||
      !std::isfinite(takeoff_origin_altitude_m) ||
      !std::isfinite(relative_altitude_up_m))
  {
    return false;
  }

  *absolute_altitude_m =
      takeoff_origin_altitude_m + static_cast<double>(relative_altitude_up_m);
  return std::isfinite(*absolute_altitude_m);
}

inline float InsRelativeHeightUpFromNedDown(const float relative_down_m)
{
  return -relative_down_m;
}

inline float InsVerticalVelocityUpFromNedDown(const float velocity_down_mps)
{
  return -velocity_down_mps;
}

inline double InsTakeoffOriginAltitudeFromCurrent(
    const double current_absolute_altitude_m,
    const double takeoff_to_current_down_m)
{
  return current_absolute_altitude_m + takeoff_to_current_down_m;
}
