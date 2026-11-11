#pragma once

#include "esphome/core/defines.h"

#ifdef USE_TMC22XX_VACTUAL

#include "esphome/core/gpio.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "tmc22xx_motion.h"
#include "tmc22xx_uart.h"

namespace esphome::tmc22xx {

/// Motion without STEP/DIR: the driver's internal pulse generator is commanded through the
/// VACTUAL register and every generated microstep is counted on the INDEX pin (GCONF.index_step).
/// The ramp is produced by periodic velocity updates; the braking curve follows the counted
/// position, so the tolerance of the driver's internal clock does not affect positioning.
class UartMotion : public MotionBackend {
 public:
  void setup(TMCUart *uart, InternalGPIOPin *index_pin);
  /// Speed (steps/s) used for starting and for the final approach to the target.
  void set_creep_speed(float steps_per_s) { this->creep_ = steps_per_s; }

  void set_profile(float max_speed, float acceleration, float deceleration) override;
  void move_to(int32_t target) override;
  void run(bool forward) override;
  void stop() override;
  void emergency_stop() override;
  void set_position(int32_t position) override;
  int32_t get_position() override { return this->pos_; }
  int32_t get_target() override { return this->target_; }
  bool is_running() const override { return this->running_; }
  bool is_cruising() const override { return this->running_ && this->cruising_; }
  void arm_trigger(InternalGPIOPin *pin, uint32_t debounce_us, bool cruise_only) override;
  void disarm_trigger() override;
  bool trigger_fired() const override { return this->trig_fired_; }
  int32_t trigger_overshoot() const override { return this->trig_fired_ ? this->pos_ - this->trig_pos_ : 0; }
  void loop() override;
  bool stop_failed() const override { return this->stop_failed_; }

 protected:
  static void index_isr(UartMotion *self);
  void start_(int8_t dir);
  void halt_();
  void finish_();
  bool write_velocity_(float steps_per_s, bool verified);
  float creep_speed_() const { return this->creep_ < this->max_speed_ ? this->creep_ : this->max_speed_; }

  TMCUart *uart_{nullptr};
  HighFrequencyLoopRequester high_freq_;
  ISRInternalGPIOPin trig_isr_pin_;

  float max_speed_{1000.0f};
  float acceleration_{1000.0f};
  float deceleration_{1000.0f};
  float creep_{100.0f};
  float v_cur_{0.0f};  // commanded speed, steps/s (magnitude)
  int32_t target_{0};
  int32_t last_pos_{0};
  uint32_t last_update_us_{0};
  uint32_t last_pulse_us_{0};
  uint32_t trig_debounce_us_{0};
  uint8_t corrections_{0};
  int8_t continuous_dir_{1};
  bool running_{false};
  bool continuous_{false};
  bool stop_request_{false};
  bool stop_failed_{false};
  bool trig_cruise_only_{false};

  // shared with the INDEX interrupt
  volatile int32_t pos_{0};
  volatile int32_t trig_pos_{0};
  volatile uint32_t trig_since_us_{0};
  volatile int8_t dir_{1};
  volatile bool cruising_{false};
  volatile bool trig_armed_{false};
  volatile bool trig_active_{false};
  volatile bool trig_fired_{false};
};

}  // namespace esphome::tmc22xx

#endif  // USE_TMC22XX_VACTUAL
