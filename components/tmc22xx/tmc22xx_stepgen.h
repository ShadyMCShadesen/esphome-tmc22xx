#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP32

#include <cstdint>
#include <driver/gptimer.h>
#include <freertos/FreeRTOS.h>
#include "esphome/core/gpio.h"
#include "esphome/core/hal.h"
#include "tmc22xx_motion.h"

// The planner runs inside the timer ISR and must stay in IRAM.
#define TMC22XX_RAMP_ATTR IRAM_ATTR
#include "tmc22xx_ramp.h"

namespace esphome::tmc22xx {

/// Hard upper bound for the step rate; see README for realistic values.
static const uint32_t STEPGEN_MAX_HZ = 100000;

/// Step pulse generator: one GPTimer alarm interrupt per step.
/// The ISR issues the pulse, counts the position and plans the next interval.
class StepGenerator : public MotionBackend {
 public:
  /// Allocates the hardware timer. Pins must already be configured as outputs.
  bool setup(InternalGPIOPin *step_pin, InternalGPIOPin *dir_pin);

  /// Speeds in steps/s, accelerations in steps/s^2. Safe to call while moving.
  void set_profile(float max_speed, float acceleration, float deceleration) override;
  void set_invert_direction(bool invert) { this->invert_dir_ = invert; }

  void move_to(int32_t target) override;
  void run(bool forward) override;
  /// Decelerate to standstill using the configured ramp.
  void stop() override;
  /// Stop issuing steps immediately (no ramp).
  void emergency_stop() override;

  void set_position(int32_t position) override;
  int32_t get_position() override;
  int32_t get_target() override;
  bool is_running() const override { return this->running_; }
  bool is_cruising() const override { return this->running_ && this->ramp_.phase == RAMP_CRUISE; }

  /// Arms an input that halts the motion from within the ISR (end stop, DIAG).
  /// The pin has to read active for debounce_us before the trigger fires.
  void arm_trigger(InternalGPIOPin *pin, uint32_t debounce_us, bool cruise_only) override;
  void disarm_trigger() override;
  bool trigger_fired() const override { return this->trig_fired_; }

 protected:
  static bool timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *arg);
  void on_alarm_(gptimer_handle_t timer);
  void halt_from_isr_(gptimer_handle_t timer);
  void start_timer_();
  void write_dir_();

  gptimer_handle_t timer_{nullptr};
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  InternalGPIOPin *dir_pin_{nullptr};
  ISRInternalGPIOPin step_isr_pin_;
  ISRInternalGPIOPin dir_isr_pin_;
  ISRInternalGPIOPin trig_isr_pin_;

  RampState ramp_;
  float acceleration_{1000.0f};
  float deceleration_{1000.0f};
  uint32_t frac_{0};          // fractional tick accumulator (Q8)
  uint32_t last_ticks_{0};    // interval that led to the current alarm
  uint32_t trig_debounce_ticks_{0};
  uint32_t trig_accum_{0};
  volatile bool running_{false};
  volatile bool trig_fired_{false};
  bool trig_armed_{false};
  bool trig_active_{false};
  bool trig_cruise_only_{false};
  bool invert_dir_{false};
};

}  // namespace esphome::tmc22xx

#endif  // USE_ESP32
