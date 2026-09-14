// Is one MOUNT_ORIENTATION sample fit to publish?
//
// Split out of payloadSdkInterface.cpp and kept free of every dependency --
// no MAVLink headers, no SDK types -- so the rule that decides whether the
// camera's attitude is real can be tested without a gimbal, the same bargain
// gimbal_pointing.py and servo_arbiter.py make on the ROS2 side.
//
// The problem it solves: MOUNT_ORIENTATION is four floats and a timestamp.
// It has NO flags field and NO validity bits. Its only in-band "this is not
// real" marker is NaN, per the message's own field comments ("set to NaN for
// invalid"). So a component that sends 0.0 is, to the letter of the protocol,
// asserting that the gimbal is perfectly level -- indistinguishable from a
// true reading. Several components share the payload's MAVLink link and more
// than one emits this message; the ones that are not the gimbal send zeroed
// fields. That leaves three independent things to check, none of which
// subsumes the others.
#pragma once

#include <cmath>
#include <cstdint>

namespace gremsy {

/// Why the sample was refused. Kept distinct rather than a bool so the caller
/// can count each cause separately -- which is what tells you later whether
/// the zero-triple check is still earning its place. See classify_attitude.
enum class AttitudeVerdict {
  Accept,
  RejectSender,       ///< not the gimbal component
  RejectNotFinite,    ///< NaN/Inf: the protocol's own "invalid" marker
  RejectZeroTriple,   ///< exactly 0/0/0 -- a zeroed field set, not a pose
};

/// MAVLink MAV_COMP_ID_GIMBAL (154) and MAV_COMP_ID_GIMBAL2..GIMBAL6
/// (171..175). Spelled numerically so this header needs no mavlink include;
/// payloadSdkInterface.cpp static_asserts these against the real enum, so the
/// two cannot drift apart silently.
constexpr bool sender_is_gimbal(std::uint8_t compid)
{
  return compid == 154 || (compid >= 171 && compid <= 175);
}

/// Decide one sample. Order matters: sender first (it is the cheapest and the
/// most specific), then the protocol's NaN marker, then the zero triple.
///
/// The zero test is an EXACT float comparison, deliberately, and an epsilon
/// here would be a bug. Measured on spiritnx3 2026-09-14 by commanding the
/// gimbal to exactly level and holding it 25 s: of 143 samples, 123 were real
/// and NOT ONE had any axis exactly 0.0 -- the tightest was pitch at 0.0075
/// deg, with roll 0.071 and yaw 9.80 deg out. The other 20 were exact zero
/// triples. So the two populations are cleanly separated, and an epsilon loose
/// enough to catch the zeros would discard a genuinely level camera, which is
/// the normal case in level flight. Only a field set that was never written is
/// exactly zero on all three axes at once.
///
/// That measurement also settles why this check is not redundant with the
/// sender whitelist. The binary running on the aircraft at the time ALREADY
/// had the compid filter, and 14% of samples were still zero triples -- so the
/// zeros arrive from inside the gimbal component id range and no sender-based
/// rule can remove them.
inline AttitudeVerdict classify_attitude(std::uint8_t compid,
                                         float roll, float pitch, float yaw)
{
  if (!sender_is_gimbal(compid)) {
    return AttitudeVerdict::RejectSender;
  }
  if (!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw)) {
    return AttitudeVerdict::RejectNotFinite;
  }
  if (roll == 0.0f && pitch == 0.0f && yaw == 0.0f) {
    return AttitudeVerdict::RejectZeroTriple;
  }
  return AttitudeVerdict::Accept;
}

/// For logs.
inline const char* to_string(AttitudeVerdict v)
{
  switch (v) {
    case AttitudeVerdict::Accept:          return "accept";
    case AttitudeVerdict::RejectSender:    return "not-gimbal-compid";
    case AttitudeVerdict::RejectNotFinite: return "nan-or-inf";
    case AttitudeVerdict::RejectZeroTriple:return "exact-zero-triple";
  }
  return "?";
}

}  // namespace gremsy
