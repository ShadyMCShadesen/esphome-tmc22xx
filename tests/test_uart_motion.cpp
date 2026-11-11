// Host simulation of the UART (VACTUAL + INDEX) motion backend against a model of the driver.
// Build (from the tests directory), one line:
//   g++ -std=c++20 -O2 -Istubs -I../components/tmc22xx test_uart_motion.cpp
//       ../components/tmc22xx/tmc22xx_vactual.cpp -o test_uart_motion
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include "tmc22xx_registers.h"
#include "tmc22xx_vactual.h"

using namespace esphome;
using namespace esphome::tmc22xx;

int g_log_errors = 0;
static int failures = 0;
#define CHECK(cond, ...) \
  do { \
    if (!(cond)) { \
      failures++; \
      printf("FAIL line %d: ", __LINE__); \
      printf(__VA_ARGS__); \
      printf("\n"); \
    } \
  } while (0)

// ---- model of the world -----------------------------------------------------
static double g_now_us = 0;       // simulated time
static double g_clock_error = 1;  // driver's internal clock vs nominal
static double g_velocity = 0;     // steps/s the driver is generating right now (signed)
static double g_fraction = 0;     // progress towards the next step
static long g_true_pos = 0;       // real motor position in steps
static InternalGPIOPin g_index;
static bool g_uart_dead = false;
static bool g_index_broken = false;
static long g_switch_at = 0;  // trigger input is active at positions <= this value
static bool g_switch_enabled = false;

uint32_t esphome::micros() { return static_cast<uint32_t>(g_now_us); }
bool ISRInternalGPIOPin::digital_read() { return g_switch_enabled && g_true_pos <= g_switch_at; }

// Let time pass; the driver emits one INDEX edge per step.
static void advance(double us) {
  const double end = g_now_us + us;
  while (g_velocity != 0.0) {
    const double rate = std::fabs(g_velocity);            // steps/s
    const double to_next = (1.0 - g_fraction) / rate * 1e6;  // us until the next step
    if (g_now_us + to_next > end)
      break;
    g_now_us += to_next;
    g_fraction = 0;
    g_true_pos += g_velocity > 0 ? 1 : -1;
    if (!g_index_broken)
      g_index.fire();
  }
  if (g_velocity != 0.0)
    g_fraction += (end - g_now_us) * std::fabs(g_velocity) / 1e6;
  g_now_us = end;
}

static void apply_vactual(uint32_t raw) {
  int32_t v = static_cast<int32_t>(raw << 8) >> 8;  // sign extend 24 bit
  g_velocity = v * (static_cast<double>(TMC_FCLK_HZ) / 16777216.0) * g_clock_error;
  if (g_velocity == 0.0)
    g_fraction = 0;
}

// Fake UART: a write datagram takes 0.7 ms on the wire, verification another 1.8 ms.
bool TMCUart::write(uint8_t reg, uint32_t value) {
  if (g_uart_dead) {
    advance(15000);
    return false;
  }
  advance(700);
  if (reg == REG_VACTUAL)
    apply_vactual(value);
  advance(1800);
  return true;
}
void TMCUart::write_unverified(uint8_t reg, uint32_t value) {
  if (g_uart_dead) {
    advance(700);
    return;
  }
  advance(700);
  if (reg == REG_VACTUAL)
    apply_vactual(value);
}
bool TMCUart::read(uint8_t, uint32_t *) { return false; }

// ---- helpers ----------------------------------------------------------------
static unsigned g_seed = 12345;
static double rnd() {
  g_seed = g_seed * 1103515245u + 12345u;
  return ((g_seed >> 8) & 0xFFFF) / 65536.0;
}
// Main loop period: usually 0.3..2.5 ms, sometimes blocked for a long time (display update, WiFi).
static double loop_gap() { return rnd() < 0.03 ? 20000 + rnd() * 15000 : 300 + rnd() * 2200; }

static double run_until_idle(UartMotion &m, double timeout_s) {
  const double start = g_now_us;
  while (m.is_running() && g_now_us - start < timeout_s * 1e6) {
    advance(loop_gap());
    m.loop();
  }
  return (g_now_us - start) / 1e6;
}

static double trapezoid_time(double dist, double v, double a, double d) {
  const double d_acc = v * v / (2 * a), d_dec = v * v / (2 * d);
  if (d_acc + d_dec <= dist)
    return v / a + v / d + (dist - d_acc - d_dec) / v;
  const double vp = std::sqrt(2 * dist * a * d / (a + d));
  return vp / a + vp / d;
}

int main() {
  static TMCUart uart;
  static UartMotion m;
  m.setup(&uart, &g_index);

  // --- point to point: counted position must match the real motor and the target ---
  int cases = 0;
  long worst_overshoot = 0;
  for (double clock : {0.95, 1.0, 1.05})
    for (double v : {200.0, 2000.0, 8000.0, 20000.0})
      for (double a : {1000.0, 4000.0, 40000.0})
        for (long dist : {1L, 2L, 10L, 100L, 1600L, 20000L})
          for (int sign : {1, -1}) {
            g_clock_error = clock;
            m.set_profile(v, a, a);
            const long start_true = g_true_pos;
            const int32_t start = m.get_position();
            const int32_t target = start + sign * dist;
            long peak = 0;
            m.move_to(target);
            const double begin = g_now_us;
            while (m.is_running() && g_now_us - begin < 300e6) {
              advance(loop_gap());
              m.loop();
              const long over = (g_true_pos - start_true) * sign - dist;
              if (over > peak)
                peak = over;
            }
            const double took = (g_now_us - begin) / 1e6;
            cases++;
            if (peak > worst_overshoot)
              worst_overshoot = peak;
            CHECK(!m.is_running(), "did not finish v=%g a=%g dist=%ld", v, a, dist);
            CHECK(m.get_position() == target, "counter off target v=%g a=%g dist=%ld: %d", v, a, dist,
                  m.get_position() - target);
            CHECK(g_true_pos - start_true == sign * dist, "motor off target v=%g a=%g dist=%ld: %ld", v, a, dist,
                  g_true_pos - start_true - sign * dist);
            if (dist >= 1600) {
              const double ideal = trapezoid_time(dist, v, a, a);
              CHECK(took < ideal * 1.5 + 0.5, "too slow v=%g a=%g dist=%ld: %.2fs vs ideal %.2fs", v, a, dist, took,
                    ideal);
            }
          }
  printf("point-to-point cases: %d, worst transient overshoot: %ld step(s)\n", cases, worst_overshoot);

  // --- ramped stop, continuous run with reversal ---
  g_clock_error = 1.0;
  m.set_profile(4000, 8000, 8000);
  m.run(true);
  for (int i = 0; i < 2000; i++) {
    advance(loop_gap());
    m.loop();
  }
  CHECK(m.is_cruising(), "continuous run did not reach cruise");
  m.run(false);
  for (int i = 0; i < 3000; i++) {
    advance(loop_gap());
    m.loop();
  }
  CHECK(m.is_running() && g_velocity < 0, "reversal failed (v=%g)", g_velocity);
  m.stop();
  run_until_idle(m, 10);
  CHECK(!m.is_running() && g_velocity == 0.0, "ramped stop failed");

  // --- homing trigger: overshoot must be reported exactly ---
  for (double v : {500.0, 2000.0, 6000.0}) {
    m.set_profile(v, 20000, 20000);
    const long offset = m.get_position() - g_true_pos;  // counter vs motor
    g_switch_at = g_true_pos - 3000;
    g_switch_enabled = true;
    m.arm_trigger(&g_index, 0, false);
    m.move_to(m.get_position() - 100000);
    run_until_idle(m, 120);
    CHECK(m.trigger_fired(), "trigger did not fire at v=%g", v);
    const long past = g_switch_at - g_true_pos;  // steps beyond the switch point (direction is negative)
    CHECK(m.trigger_overshoot() == -past, "overshoot v=%g: reported %d, real %ld", v, m.trigger_overshoot(), -past);
    CHECK(m.get_position() - g_true_pos == offset, "counter lost steps during homing at v=%g", v);
    printf("homing at %g steps/s: stopped %ld step(s) past the switch, reported %d\n", v, past,
           -m.trigger_overshoot());
    m.disarm_trigger();
    g_switch_enabled = false;
  }

  // --- faults: missing INDEX pulses, driver not answering the stop command ---
  g_log_errors = 0;
  g_index_broken = true;
  m.set_profile(2000, 4000, 4000);
  m.move_to(m.get_position() + 5000);
  const double fault_time = run_until_idle(m, 10);
  CHECK(!m.is_running() && g_velocity == 0.0 && fault_time < 1.0, "INDEX fault not handled (%.2fs)", fault_time);
  CHECK(g_log_errors > 0, "INDEX fault not reported");
  g_index_broken = false;

  m.move_to(m.get_position() + 50000);
  for (int i = 0; i < 300; i++) {
    advance(loop_gap());
    m.loop();
  }
  g_uart_dead = true;
  m.emergency_stop();
  CHECK(m.stop_failed() && m.is_running(), "lost stop command not flagged");
  g_uart_dead = false;
  run_until_idle(m, 5);
  CHECK(!m.is_running() && !m.stop_failed() && g_velocity == 0.0, "stop not retried after UART recovered");

  printf(failures == 0 ? "ALL TESTS PASSED\n" : "%d FAILURES\n", failures);
  return failures == 0 ? 0 : 1;
}
