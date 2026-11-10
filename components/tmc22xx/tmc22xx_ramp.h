#pragma once

#include <cstdint>

// Integer-only trapezoidal ramp planner (D. Austin, "Generate stepper-motor speed
// profiles in real time", 2005). One call per step, no floats, no 64 bit division.
// Kept free of any platform dependency so it can be unit tested on a host.
//
// Units: step interval "c" in timer ticks with 8 fractional bits (Q24.8).

namespace esphome::tmc22xx {

#ifndef TMC22XX_RAMP_ATTR
#define TMC22XX_RAMP_ATTR
#endif

static const uint32_t RAMP_TICK_HZ = 4000000UL;    // 250 ns timer resolution
static const uint32_t RAMP_C_MAX = 0x3FFFFFFFUL;   // keeps 2*c + remainder inside 32 bit
static const int32_t RAMP_DIST_INFINITE = 0x3FFFFFFF;

enum RampPhase : uint8_t { RAMP_ACCEL = 0, RAMP_CRUISE = 1, RAMP_DECEL = 2 };
enum RampResult : uint8_t { RAMP_CONTINUE = 0, RAMP_FINISHED = 1, RAMP_REVERSED = 2 };

struct RampState {
  // --- profile, written by the task inside a critical section ---
  uint32_t c_min{RAMP_TICK_HZ / 1000 * 256};  // interval at max speed
  uint32_t c0_accel{RAMP_C_MAX};              // first interval when starting from standstill
  uint32_t accel_over_decel_q16{65536};       // converts accel ramp steps -> decel ramp steps
  uint32_t decel_over_accel_q16{65536};
  // --- targets ---
  int32_t target{0};
  bool continuous{false};
  bool stop_request{false};
  int8_t continuous_dir{1};
  // --- live state, owned by the ISR while running ---
  int32_t position{0};
  uint32_t c{RAMP_C_MAX};  // current interval
  uint32_t n{0};           // ramp index in steps (accel units, or decel units while RAMP_DECEL)
  uint32_t rest{0};        // division remainder carried to the next step (keeps long ramps exact)
  int8_t dir{1};
  RampPhase phase{RAMP_ACCEL};
};

static inline uint32_t TMC22XX_RAMP_ATTR ramp_scale_q16(uint32_t n, uint32_t ratio_q16) {
  const uint64_t r = (static_cast<uint64_t>(n) * ratio_q16) >> 16;
  return r > 0x7FFFFFFFULL ? 0x7FFFFFFFUL : static_cast<uint32_t>(r);
}

/// One step up the ramp: c(n) = c(n-1) - 2*c(n-1) / (4n + 1), remainder carried over.
static inline void TMC22XX_RAMP_ATTR ramp_accel_step(RampState &s) {
  s.n++;
  const uint32_t den = 4 * s.n + 1;
  const uint32_t num = 2 * s.c + s.rest;
  const uint32_t q = num / den;
  s.rest = num - q * den;
  s.c -= q;
}

/// One step down the ramp, the exact inverse of ramp_accel_step(). Requires s.n > 0.
static inline void TMC22XX_RAMP_ATTR ramp_decel_step(RampState &s) {
  const uint32_t den = 4 * s.n - 1;
  const uint32_t num = 2 * s.c + s.rest;
  const uint32_t q = num / den;
  s.rest = num - q * den;
  s.c += q;
  if (s.c > RAMP_C_MAX)
    s.c = RAMP_C_MAX;
  s.n--;
}

/// Prepare the state for a start from standstill. `dir` must already be set.
static inline void TMC22XX_RAMP_ATTR ramp_restart(RampState &s) {
  s.n = 0;
  s.rest = 0;
  if (s.c0_accel <= s.c_min) {
    // Requested speed is below the speed of the very first ramp step: no ramp needed.
    s.c = s.c_min;
    s.phase = RAMP_CRUISE;
  } else {
    s.c = s.c0_accel;
    s.phase = RAMP_ACCEL;
  }
}

/// Called right after a step has been issued (position already updated).
/// Computes the interval until the next step in s.c.
static inline RampResult TMC22XX_RAMP_ATTR ramp_after_step(RampState &s) {
  // Remaining distance measured in the current direction of travel.
  int32_t dist;
  if (s.continuous) {
    dist = (s.continuous_dir == s.dir) ? RAMP_DIST_INFINITE : -1;
  } else {
    dist = (s.target - s.position) * s.dir;
  }
  // Steps needed to come to a stop from the current speed.
  const uint32_t stop_steps = (s.phase == RAMP_DECEL) ? s.n : ramp_scale_q16(s.n, s.accel_over_decel_q16);

  if (s.stop_request || dist <= static_cast<int32_t>(stop_steps)) {
    if (s.phase != RAMP_DECEL) {
      s.n = stop_steps;
      s.rest = 0;
      s.phase = RAMP_DECEL;
    }
    if (s.n == 0) {
      // Standstill reached.
      if (s.stop_request || dist == 0)
        return RAMP_FINISHED;
      // Target is behind us (it was moved while running): turn around.
      s.dir = static_cast<int8_t>(-s.dir);
      ramp_restart(s);
      return RAMP_REVERSED;
    }
    ramp_decel_step(s);
    return RAMP_CONTINUE;
  }

  if (s.c < s.c_min) {
    // Faster than the (lowered) speed limit: brake down to it.
    if (s.phase != RAMP_DECEL) {
      s.n = stop_steps;
      s.rest = 0;
      s.phase = RAMP_DECEL;
    }
    if (s.n > 0)
      ramp_decel_step(s);
    if (s.c >= s.c_min || s.n == 0) {
      s.c = s.c_min;
      s.n = ramp_scale_q16(s.n, s.decel_over_accel_q16);
      s.rest = 0;
      s.phase = RAMP_CRUISE;
    }
    return RAMP_CONTINUE;
  }

  if (s.phase == RAMP_DECEL) {
    // Deceleration no longer required (target extended): continue on the acceleration ramp.
    s.n = ramp_scale_q16(s.n, s.decel_over_accel_q16);
    s.rest = 0;
    s.phase = RAMP_ACCEL;
  }
  if (s.c > s.c_min) {
    if (s.n == 0 && s.c != s.c0_accel) {
      // Premature standstill caused by rounding: restart the ramp cleanly.
      ramp_restart(s);
      return RAMP_CONTINUE;
    }
    // Accelerate only if we can still stop in time after one more acceleration step.
    const uint32_t stop_next = ramp_scale_q16(s.n + 1, s.accel_over_decel_q16);
    if (dist - 1 >= static_cast<int32_t>(stop_next)) {
      ramp_accel_step(s);
      if (s.c <= s.c_min) {
        s.c = s.c_min;
        s.phase = RAMP_CRUISE;
      } else {
        s.phase = RAMP_ACCEL;
      }
    }
    // else: hold the current speed until the deceleration point is reached
  } else {
    s.phase = RAMP_CRUISE;
  }
  return RAMP_CONTINUE;
}

}  // namespace esphome::tmc22xx
