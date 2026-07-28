#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <limits>

#include "ins_altitude_reference.h"

namespace
{
void require(bool condition, const char *message)
{
  if (!condition)
  {
    std::fprintf(stderr, "FAILED: %s\n", message);
    std::exit(1);
  }
}

bool nearlyEqual(double lhs, double rhs, double tolerance = 1.0e-6)
{
  return std::fabs(lhs - rhs) <= tolerance;
}
} // namespace

int main()
{
  double absolute_baro_altitude_m = 0.0;
  require(InsComputeAbsoluteBaroAltitude(123.4, 2.5f,
                                         &absolute_baro_altitude_m),
          "finite barometer observation should be accepted");
  require(nearlyEqual(absolute_baro_altitude_m, 125.9),
          "relative barometer height should be anchored to takeoff altitude");

  require(nearlyEqual(InsRelativeHeightUpFromNedDown(-4.25f), 4.25f),
          "negative NED down should become positive control height");
  require(nearlyEqual(InsVerticalVelocityUpFromNedDown(0.8f), -0.8f),
          "positive NED down velocity should become negative climb rate");

  require(nearlyEqual(InsTakeoffOriginAltitudeFromCurrent(152.0, -12.0),
                      140.0),
          "GNSS reanchor should preserve the existing takeoff-relative height");

  require(!InsComputeAbsoluteBaroAltitude(
              std::numeric_limits<double>::quiet_NaN(), 1.0f,
              &absolute_baro_altitude_m),
          "invalid origin altitude should be rejected");
  require(!InsComputeAbsoluteBaroAltitude(
              123.4, std::numeric_limits<float>::infinity(),
              &absolute_baro_altitude_m),
          "invalid relative barometer height should be rejected");
  require(!InsComputeAbsoluteBaroAltitude(123.4, 1.0f, nullptr),
          "null output pointer should be rejected");

  std::puts("[PASS] altitude reference tests");
  return 0;
}
