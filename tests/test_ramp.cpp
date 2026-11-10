// Host-side test of the integer ramp planner and the UART CRC.
// Build: g++ -std=c++17 -O2 -I../components/tmc22xx test_ramp.cpp -o test_ramp && ./test_ramp
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include "tmc22xx_ramp.h"

using namespace esphome::tmc22xx;

static int failures = 0;
#define CHECK(cond, ...) \
  do { \
    if (!(cond)) { \
      failures++; \
      printf("FAIL %s:%d: ", __FILE__, __LINE__); \
      printf(__VA_ARGS__); \
      printf("\n"); \
    } \
  } while (0)

static uint8_t crc8(const uint8_t *data, size_t len) {  // same algorithm as TMCUart::crc8
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    uint8_t byte = data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (((crc >> 7) ^ (byte & 0x01)) != 0) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc <<= 1;
      }
      byte >>= 1;
    }
  }
  return crc;
}

static void set_profile(RampState &s, double v, double a, double d) {
  s.c_min = (uint32_t)(RAMP_TICK_HZ * 256.0 / v);
  double c0 = 0.676 * RAMP_TICK_HZ * 256.0 * std::sqrt(2.0 / a);
  s.c0_accel = c0 >= RAMP_C_MAX ? RAMP_C_MAX : (uint32_t) c0;
  // same clamp as StepGenerator::set_profile()
  const double ratio = std::fmin(std::fmax(a / d, 1.0 / 256.0), 256.0);
  s.accel_over_decel_q16 = (uint32_t)(ratio * 65536.0);
  s.decel_over_accel_q16 = (uint32_t)(65536.0 / ratio);
}

struct Sim {
  RampState s;
  double t{0};          // seconds
  double v_peak{0};     // steps/s
  double a_peak{0};     // steps/s^2 (from consecutive intervals)
  long steps{0};
  int reversals{0};
  bool running{false};
  int win_steps{0};
  double win_t{0}, win_v{0};

  void start_move(int32_t target) {
    s.target = target;
    s.continuous = false;
    s.stop_request = false;
    if (!running && target != s.position) {
      s.dir = target > s.position ? 1 : -1;
      ramp_restart(s);
      running = true;
    }
  }
  // Issue one step; returns false when the motion finished.
  bool step() {
    const double c_prev = s.c / 256.0 / RAMP_TICK_HZ;
    t += c_prev;  // the interval preceding this step
    s.position += s.dir;
    steps++;
    RampResult r = ramp_after_step(s);
    if (r == RAMP_FINISHED) {
      running = false;
      return false;
    }
    if (r == RAMP_REVERSED)
      reversals++;
    const double c_next = s.c / 256.0 / RAMP_TICK_HZ;
    const double v = 1.0 / c_next;
    if (v > v_peak)
      v_peak = v;
    // Acceleration averaged over 256 steps (single intervals only show tick quantization).
    if (r == RAMP_REVERSED) {
      win_steps = 0;
    } else if (steps > 3) {
      if (win_steps == 0) {
        win_t = t;
        win_v = v;
      }
      if (++win_steps == 256) {
        const double a = std::fabs(v - win_v) / (t + c_next - win_t);
        if (a > a_peak)
          a_peak = a;
        win_steps = 0;
      }
    }
    return true;
  }
  void run_to_end(long max_steps = 50000000) {
    while (running && steps < max_steps)
      step();
  }
};

static double trapezoid_time(double dist, double v, double a, double d) {
  const double d_acc = v * v / (2 * a), d_dec = v * v / (2 * d);
  if (d_acc + d_dec <= dist)
    return v / a + v / d + (dist - d_acc - d_dec) / v;
  const double vp = std::sqrt(2 * dist * a * d / (a + d));
  return vp / a + vp / d;
}

int main() {
  // --- CRC: request datagrams documented for node address 0 ---
  const uint8_t gconf[] = {0x05, 0x00, 0x00}, ifcnt[] = {0x05, 0x00, 0x02}, ioin[] = {0x05, 0x00, 0x06};
  CHECK(crc8(gconf, 3) == 0x48, "crc gconf %02X", crc8(gconf, 3));
  CHECK(crc8(ifcnt, 3) == 0x8F, "crc ifcnt %02X", crc8(ifcnt, 3));
  CHECK(crc8(ioin, 3) == 0x6F, "crc ioin %02X", crc8(ioin, 3));

  // --- point to point moves over a parameter grid ---
  const double speeds[] = {1, 50, 800, 4000, 20000, 100000};
  const double accels[] = {1, 100, 2000, 50000, 1000000};
  const int32_t dists[] = {1, 2, 3, 7, 100, 3200, 50000, 400000};
  int cases = 0;
  for (double v : speeds)
    for (double a : accels)
      for (double d : accels)
        for (int32_t dist : dists)
          for (int sign : {1, -1}) {
            if (trapezoid_time(dist, v, a, d) > 3000)
              continue;  // skip absurdly long runs
            if (a / d > 256 || d / a > 256)
              continue;  // outside the supported accel/decel ratio
            Sim sim;
            set_profile(sim.s, v, a, d);
            sim.s.position = 1000;
            sim.start_move(1000 + sign * dist);
            sim.run_to_end();
            cases++;
            CHECK(!sim.running, "did not finish v=%g a=%g d=%g dist=%d", v, a, d, dist);
            CHECK(sim.s.position == 1000 + sign * dist, "wrong end position v=%g a=%g d=%g dist=%d: %d", v, a, d, dist,
                  sim.s.position - 1000);
            CHECK(sim.steps == dist, "overshoot v=%g a=%g d=%g dist=%d steps=%ld", v, a, d, dist, sim.steps);
            CHECK(sim.reversals == 0, "reversal v=%g a=%g d=%g dist=%d", v, a, d, dist);
            CHECK(sim.v_peak <= v * 1.001, "speed limit exceeded v=%g a=%g d=%g dist=%d peak=%g", v, a, d, dist,
                  sim.v_peak);
            if (dist >= 3200) {
              const double ideal = trapezoid_time(dist, v, a, d);
              CHECK(std::fabs(sim.t - ideal) / ideal < 0.06, "duration v=%g a=%g d=%g dist=%d: %.4f vs ideal %.4f", v,
                    a, d, dist, sim.t, ideal);
              CHECK(sim.a_peak <= std::fmax(a, d) * 1.1, "accel v=%g a=%g d=%g dist=%d peak=%g", v, a, d, dist,
                    sim.a_peak);
            }
          }
  printf("point-to-point cases: %d\n", cases);

  // --- retarget while moving: extend, shorten (overshoot + return), reverse ---
  {
    Sim sim;
    set_profile(sim.s, 4000, 8000, 8000);
    sim.start_move(20000);
    for (int i = 0; i < 5000; i++)
      sim.step();
    sim.start_move(40000);  // extend
    sim.run_to_end();
    CHECK(sim.s.position == 40000 && sim.reversals == 0, "extend: pos=%d rev=%d", sim.s.position, sim.reversals);

    sim.start_move(0);
    for (int i = 0; i < 20000; i++)
      sim.step();
    sim.start_move(sim.s.position - 10);  // closer than the stopping distance (1000 steps)
    const int32_t goal = sim.s.target;
    sim.run_to_end();
    CHECK(sim.s.position == goal && sim.reversals == 1, "shorten: pos=%d goal=%d rev=%d", sim.s.position, goal,
          sim.reversals);

    sim.reversals = 0;
    sim.start_move(30000);
    for (int i = 0; i < 3000; i++)
      sim.step();
    sim.start_move(-500);  // opposite direction
    sim.run_to_end();
    CHECK(sim.s.position == -500 && sim.reversals == 1, "reverse: pos=%d rev=%d", sim.s.position, sim.reversals);
  }

  // --- ramped stop and continuous run ---
  {
    Sim sim;
    set_profile(sim.s, 2000, 4000, 1000);
    sim.s.continuous = true;
    sim.s.continuous_dir = -1;
    sim.s.dir = -1;
    ramp_restart(sim.s);
    sim.running = true;
    for (int i = 0; i < 10000; i++)
      sim.step();
    CHECK(sim.s.phase == RAMP_CRUISE && sim.s.c == sim.s.c_min, "continuous run not at cruise");
    const int32_t at = sim.s.position;
    sim.s.stop_request = true;
    sim.run_to_end();
    const int32_t stop_dist = at - sim.s.position;
    // v^2 / (2 d) = 2000 steps
    CHECK(!sim.running && std::abs(stop_dist - 2000) < 60, "ramped stop distance %d", stop_dist);
  }

  // --- speed limit lowered / raised while cruising ---
  {
    Sim sim;
    set_profile(sim.s, 8000, 16000, 16000);
    sim.start_move(200000);
    for (int i = 0; i < 20000; i++)
      sim.step();
    sim.s.c_min = (uint32_t)(RAMP_TICK_HZ * 256.0 / 2000);  // slower
    for (int i = 0; i < 20000; i++)
      sim.step();
    CHECK(sim.s.c == sim.s.c_min && sim.s.phase == RAMP_CRUISE, "did not settle at lowered speed");
    sim.s.c_min = (uint32_t)(RAMP_TICK_HZ * 256.0 / 6000);  // faster again
    for (int i = 0; i < 20000; i++)
      sim.step();
    CHECK(sim.s.c == sim.s.c_min, "did not settle at raised speed");
    sim.run_to_end();
    CHECK(sim.s.position == 200000 && sim.reversals == 0, "speed change: pos=%d", sim.s.position);
  }

  printf(failures == 0 ? "ALL TESTS PASSED\n" : "%d FAILURES\n", failures);
  return failures == 0 ? 0 : 1;
}
