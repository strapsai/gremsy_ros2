// Tests for the MOUNT_ORIENTATION validity gate. No gimbal, no ROS, no MAVLink:
//   g++ -std=c++17 -I../../libs test_attitude_validity.cpp -o t && ./t
#include "gremsy_attitude_validity.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

using namespace gremsy;

static int checks = 0;
#define CHECK(cond) do { ++checks; if(!(cond)) { \
  printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); return 1; } } while(0)

// Real samples captured off /gimbal_orientation on spiritnx3, 2026-09-14, with
// the aircraft disarmed and the gimbal at rest. (roll, pitch, yaw) degrees.
static const float kRealSamples[][3] = {
  {0.0509069561958313f,  6.403238773345947f,  0.52734375f},
  {0.052483875304460526f,6.404466152191162f,  0.1318359375f},
  {0.05611775815486908f, 6.402574062347412f, -3.01025390625f},
  {0.05556464567780495f, 6.405303001403809f, -3.076171875f},
};

int main()
{
  const std::uint8_t GIMBAL = 154, GIMBAL2 = 171, GIMBAL6 = 175;
  const std::uint8_t CAMERA = 100, AUTOPILOT = 1, USER2 = 26;

  // ---- the sender whitelist ------------------------------------------------
  CHECK(sender_is_gimbal(GIMBAL));
  CHECK(sender_is_gimbal(GIMBAL2));
  CHECK(sender_is_gimbal(GIMBAL6));
  CHECK(!sender_is_gimbal(CAMERA));
  CHECK(!sender_is_gimbal(AUTOPILOT));
  CHECK(!sender_is_gimbal(USER2));
  CHECK(!sender_is_gimbal(170));   // just below GIMBAL2
  CHECK(!sender_is_gimbal(176));   // just above GIMBAL6
  CHECK(!sender_is_gimbal(153));   // just below GIMBAL
  CHECK(!sender_is_gimbal(0));

  // ---- the three causes, each in isolation --------------------------------
  CHECK(classify_attitude(CAMERA, 1.0f, 2.0f, 3.0f) == AttitudeVerdict::RejectSender);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  CHECK(classify_attitude(GIMBAL, nan,  2.0f, 3.0f) == AttitudeVerdict::RejectNotFinite);
  CHECK(classify_attitude(GIMBAL, 1.0f, nan,  3.0f) == AttitudeVerdict::RejectNotFinite);
  CHECK(classify_attitude(GIMBAL, 1.0f, 2.0f, nan ) == AttitudeVerdict::RejectNotFinite);
  CHECK(classify_attitude(GIMBAL, inf,  2.0f, 3.0f) == AttitudeVerdict::RejectNotFinite);
  CHECK(classify_attitude(GIMBAL, 0.0f, 0.0f, 0.0f) == AttitudeVerdict::RejectZeroTriple);
  CHECK(classify_attitude(GIMBAL,-0.0f,-0.0f,-0.0f) == AttitudeVerdict::RejectZeroTriple);

  // Sender is checked first, so a zeroed set from a non-gimbal reports the
  // more specific cause -- which is what makes the tallies interpretable.
  CHECK(classify_attitude(CAMERA, 0.0f, 0.0f, 0.0f) == AttitudeVerdict::RejectSender);

  // ---- a zero on ONE axis is a real pose and must survive ------------------
  CHECK(classify_attitude(GIMBAL, 0.0f, 6.4f, 0.5f) == AttitudeVerdict::Accept);
  CHECK(classify_attitude(GIMBAL, 0.05f, 0.0f, 0.5f) == AttitudeVerdict::Accept);
  CHECK(classify_attitude(GIMBAL, 0.05f, 6.4f, 0.0f) == AttitudeVerdict::Accept);
  // Nadir: pitch -90 with a level, north-pointing camera. Two exact zeros.
  CHECK(classify_attitude(GIMBAL, 0.0f, -90.0f, 0.0f) == AttitudeVerdict::Accept);

  // ---- the load-bearing claim: 0/0/0 NEVER reaches the topic ---------------
  // Exhaustive over every component id MAVLink can express.
  for (int c = 0; c <= 255; ++c) {
    CHECK(classify_attitude((std::uint8_t)c, 0.0f, 0.0f, 0.0f) != AttitudeVerdict::Accept);
    CHECK(classify_attitude((std::uint8_t)c,-0.0f, 0.0f,-0.0f) != AttitudeVerdict::Accept);
  }

  // ---- real readings must still be accepted --------------------------------
  for (const auto& s : kRealSamples) {
    CHECK(classify_attitude(GIMBAL, s[0], s[1], s[2]) == AttitudeVerdict::Accept);
  }

  // ---- how close does a REAL resting sample get to the zero triple? --------
  // The check can only misfire if all three axes read exactly 0.0f at once.
  printf("\nreal resting samples vs the zero triple:\n");
  float min_roll = 1e9f, min_pitch = 1e9f;
  for (const auto& s : kRealSamples) {
    printf("  roll %+.9f  pitch %+.9f  yaw %+.9f\n", s[0], s[1], s[2]);
    min_roll  = std::fmin(min_roll,  std::fabs(s[0]));
    min_pitch = std::fmin(min_pitch, std::fabs(s[1]));
  }
  const float ulp_roll = std::nextafterf(min_roll, 1e9f) - min_roll;
  printf("\n  closest |roll|  to zero: %.9f deg  (%.0f ulp away, 1 ulp = %.3g deg)\n",
         min_roll, min_roll / ulp_roll, ulp_roll);
  printf("  closest |pitch| to zero: %.9f deg\n", min_pitch);
  printf("  => at rest the camera carries a standing bias on both axes; pitch alone\n"
         "     sits %.2f deg off level, so a true reading is nowhere near 0/0/0.\n", min_pitch);
  CHECK(min_roll  > 0.0f);
  CHECK(min_pitch > 0.0f);

  printf("\n%d checks passed\n", checks);
  return 0;
}
