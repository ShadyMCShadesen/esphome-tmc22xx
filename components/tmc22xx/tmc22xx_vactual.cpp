#include "tmc22xx_vactual.h"

#ifdef USE_TMC22XX_VACTUAL

#include <algorithm>
#include <cmath>
#include "esphome/core/log.h"
#include "tmc22xx_registers.h"

namespace esphome::tmc22xx {

static const char *const TAG = "tmc22xx.uart_motion";

static const uint32_t UPDATE_INTERVAL_US = 5000;  // velocity update period while ramping
static const uint32_t PULSE_TIMEOUT_US = 300000;  // no INDEX edge for this long while moving = fault
static const uint8_t MAX_CORRECTIONS = 3;         // short creep moves to remove a residual error
static const float MAX_SPEED_HZ = 100000.0f;

void UartMotion::setup(TMCUart *uart, InternalGPIOPin *index_pin) {
  this->uart_ = uart;
  // With GCONF.index_step = 1 the INDEX output toggles once per microstep.
  index_pin->attach_interrupt(UartMotion::index_isr, this, gpio::INTERRUPT_ANY_EDGE);
}

void IRAM_ATTR UartMotion::index_isr(UartMotion *self) {
  self->pos_ = self->pos_ + self->dir_;
  if (!self->trig_armed_)
    return;
  if ((!self->trig_cruise_only_ || self->cruising_) && self->trig_isr_pin_.digital_read()) {
    const uint32_t now = micros();
    if (!self->trig_active_) {
      self->trig_active_ = true;
      self->trig_since_us_ = now;
    }
    if (now - self->trig_since_us_ >= self->trig_debounce_us_) {
      self->trig_armed_ = false;
      self->trig_pos_ = self->pos_;
      self->trig_fired_ = true;
    }
  } else {
    self->trig_active_ = false;
  }
}

void UartMotion::set_profile(float max_speed, float acceleration, float deceleration) {
  this->max_speed_ = std::clamp(max_speed, 1.0f, MAX_SPEED_HZ);
  this->acceleration_ = std::max(acceleration, 1.0f);
  this->deceleration_ = std::max(deceleration, 1.0f);
}

bool UartMotion::write_velocity_(float steps_per_s, bool verified) {
  // v [usteps/s] = VACTUAL * f_CLK / 2^24, signed 24 bit
  const int32_t vactual =
      static_cast<int32_t>(lroundf(steps_per_s * this->dir_ * 16777216.0f / static_cast<float>(TMC_FCLK_HZ)));
  const uint32_t raw = static_cast<uint32_t>(vactual) & 0x00FFFFFFUL;
  if (verified)
    return this->uart_->write(REG_VACTUAL, raw);
  this->uart_->write_unverified(REG_VACTUAL, raw);
  return true;
}

void UartMotion::start_(int8_t dir) {
  this->dir_ = dir;
  this->v_cur_ = this->creep_speed_();
  this->cruising_ = false;
  this->stop_request_ = false;
  if (!this->write_velocity_(this->v_cur_, true)) {
    ESP_LOGE(TAG, "Cannot start: driver did not accept VACTUAL");
    this->v_cur_ = 0.0f;
    this->finish_();
    return;
  }
  const uint32_t now = micros();
  this->last_update_us_ = now;
  this->last_pulse_us_ = now;
  this->last_pos_ = this->pos_;
  if (!this->running_) {
    this->running_ = true;
    this->high_freq_.start();
  }
}

void UartMotion::finish_() {
  this->target_ = this->pos_;
  this->continuous_ = false;
  this->stop_request_ = false;
  this->cruising_ = false;
  if (this->running_) {
    this->running_ = false;
    this->high_freq_.stop();
  }
}

void UartMotion::halt_() {
  if (!this->uart_->write(REG_VACTUAL, 0)) {
    if (!this->stop_failed_)
      ESP_LOGE(TAG, "Driver did not accept the stop command - retrying");
    this->stop_failed_ = true;  // loop() keeps retrying
    return;
  }
  this->stop_failed_ = false;
  this->v_cur_ = 0.0f;
  this->cruising_ = false;
}

void UartMotion::move_to(int32_t target) {
  this->target_ = target;
  this->continuous_ = false;
  this->stop_request_ = false;
  this->corrections_ = 0;
  if (!this->running_ && target != this->pos_)
    this->start_(target > this->pos_ ? 1 : -1);
}

void UartMotion::run(bool forward) {
  this->continuous_ = true;
  this->continuous_dir_ = forward ? 1 : -1;
  this->stop_request_ = false;
  if (!this->running_)
    this->start_(this->continuous_dir_);
}

void UartMotion::stop() {
  if (this->running_)
    this->stop_request_ = true;
}

void UartMotion::emergency_stop() {
  if (!this->running_)
    return;
  this->halt_();
  if (!this->stop_failed_)
    this->finish_();
}

void UartMotion::set_position(int32_t position) {
  InterruptLock lock;
  const int32_t delta = position - this->pos_;
  this->pos_ = position;
  this->trig_pos_ = this->trig_pos_ + delta;
  this->last_pos_ = position;
  if (this->running_ && !this->continuous_) {
    this->target_ += delta;
  } else {
    this->target_ = position;
  }
}

void UartMotion::arm_trigger(InternalGPIOPin *pin, uint32_t debounce_us, bool cruise_only) {
  this->trig_armed_ = false;
  this->trig_isr_pin_ = pin->to_isr();
  this->trig_debounce_us_ = debounce_us;
  this->trig_cruise_only_ = cruise_only;
  this->trig_active_ = false;
  this->trig_fired_ = false;
  this->trig_armed_ = true;
}

void UartMotion::disarm_trigger() {
  this->trig_armed_ = false;
  this->trig_fired_ = false;
}

void UartMotion::loop() {
  if (!this->running_)
    return;
  if (this->stop_failed_) {
    // Previous stop command got lost: nothing else matters until the motor stands still.
    this->halt_();
    if (!this->stop_failed_)
      this->finish_();
    return;
  }
  if (this->trig_fired_) {
    this->halt_();
    if (!this->stop_failed_)
      this->finish_();
    return;
  }

  const uint32_t now = micros();
  const int32_t pos = this->pos_;
  if (pos != this->last_pos_) {
    this->last_pos_ = pos;
    this->last_pulse_us_ = now;
  } else {
    const uint32_t period_us = static_cast<uint32_t>(4000000.0f / std::max(this->v_cur_, 1.0f));
    if (now - this->last_pulse_us_ > std::max(PULSE_TIMEOUT_US, period_us)) {
      ESP_LOGE(TAG, "No pulses on the INDEX pin while moving - check the INDEX wiring / driver supply");
      this->halt_();
      if (!this->stop_failed_)
        this->finish_();
      return;
    }
  }

  // Remaining distance in the current direction of travel (negative: target is behind us).
  int32_t remaining;
  if (this->continuous_) {
    remaining = this->continuous_dir_ == this->dir_ ? INT32_MAX : -1;
  } else {
    remaining = (this->target_ - pos) * this->dir_;
  }
  const bool braking = this->stop_request_ || remaining < 0;
  const float creep = this->creep_speed_();

  if (!braking && remaining == 0) {
    // Arrived. Pulses issued before the stop command took effect are corrected below.
    this->halt_();
  } else if (braking && this->v_cur_ <= creep) {
    this->halt_();
  }
  if (this->stop_failed_)
    return;
  if (this->v_cur_ == 0.0f) {
    // Standing still: finished, or turn around / remove a residual error at creep speed.
    const int32_t error = this->target_ - this->pos_;
    if (this->stop_request_) {
      this->finish_();
    } else if (this->continuous_) {
      this->start_(this->continuous_dir_);
    } else if (error != 0 && this->corrections_ < MAX_CORRECTIONS) {
      this->corrections_++;
      this->start_(error > 0 ? 1 : -1);
    } else {
      if (error != 0)
        ESP_LOGW(TAG, "Stopped %d step(s) off target", static_cast<int>(error));
      this->finish_();
    }
    return;
  }

  if (now - this->last_update_us_ < UPDATE_INTERVAL_US)
    return;
  const float dt = (now - this->last_update_us_) * 1e-6f;
  this->last_update_us_ = now;

  float v;
  if (braking) {
    v = std::max(this->v_cur_ - this->deceleration_ * dt, creep);
  } else {
    v = std::min(this->v_cur_ + this->acceleration_ * dt, this->max_speed_);
    if (!this->continuous_) {
      // Braking curve v = sqrt(2 * d * s) on the measured position, looking one update ahead.
      const float ahead = static_cast<float>(remaining) - this->v_cur_ * (UPDATE_INTERVAL_US * 1e-6f);
      v = std::min(v, sqrtf(2.0f * this->deceleration_ * std::max(ahead, 0.0f)));
    }
    v = std::max(v, creep);
  }
  this->cruising_ = !braking && v >= this->max_speed_ - 0.5f;
  if (fabsf(v - this->v_cur_) >= 0.5f) {
    this->v_cur_ = v;
    this->write_velocity_(v, false);
  }
}

}  // namespace esphome::tmc22xx

#endif  // USE_TMC22XX_VACTUAL
