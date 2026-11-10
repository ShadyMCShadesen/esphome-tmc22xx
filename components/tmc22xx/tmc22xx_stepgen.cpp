#include "tmc22xx_stepgen.h"

#ifdef USE_ESP32

#include <algorithm>
#include <cmath>
#include "esphome/core/log.h"

namespace esphome::tmc22xx {

static const char *const TAG = "tmc22xx.stepgen";

// DIR is written by the task right before the timer starts; this also is the DIR setup time.
static const uint32_t START_DELAY_TICKS = RAMP_TICK_HZ / 20000;  // 50 us
// Minimum distance between "now" and a newly programmed alarm.
static const uint32_t ALARM_GUARD_TICKS = RAMP_TICK_HZ / 250000;  // 4 us

bool StepGenerator::setup(InternalGPIOPin *step_pin, InternalGPIOPin *dir_pin) {
  this->dir_pin_ = dir_pin;
  this->step_isr_pin_ = step_pin->to_isr();
  this->dir_isr_pin_ = dir_pin->to_isr();

  gptimer_config_t config = {};
  config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  config.direction = GPTIMER_COUNT_UP;
  config.resolution_hz = RAMP_TICK_HZ;
  config.intr_priority = 3;  // highest level that may still run C handlers
  esp_err_t err = gptimer_new_timer(&config, &this->timer_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "gptimer_new_timer failed: %s", esp_err_to_name(err));
    return false;
  }
  gptimer_event_callbacks_t callbacks = {};
  callbacks.on_alarm = StepGenerator::timer_isr;
  err = gptimer_register_event_callbacks(this->timer_, &callbacks, this);
  if (err == ESP_OK)
    err = gptimer_enable(this->timer_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "gptimer setup failed: %s", esp_err_to_name(err));
    gptimer_del_timer(this->timer_);
    this->timer_ = nullptr;
    return false;
  }
  return true;
}

void StepGenerator::set_profile(float max_speed, float acceleration, float deceleration) {
  max_speed = std::clamp(max_speed, 1.0f, static_cast<float>(STEPGEN_MAX_HZ));
  acceleration = std::max(acceleration, 1.0f);
  deceleration = std::max(deceleration, 1.0f);

  const uint32_t c_min = static_cast<uint32_t>(static_cast<float>(RAMP_TICK_HZ) * 256.0f / max_speed);
  // First interval: c0 = 0.676 * f * sqrt(2 / a) (Austin, eq. 15 incl. correction factor).
  const float c0 = 0.676f * static_cast<float>(RAMP_TICK_HZ) * 256.0f * sqrtf(2.0f / acceleration);
  const float ratio = std::clamp(acceleration / deceleration, 1.0f / 256.0f, 256.0f);

  portENTER_CRITICAL(&this->mux_);
  if (this->running_) {
    // Keep v^2 = 2*a*n invariant so the remaining ramp stays consistent.
    const float rescale = (this->ramp_.phase == RAMP_DECEL) ? this->deceleration_ / deceleration
                                                            : this->acceleration_ / acceleration;
    this->ramp_.n = static_cast<uint32_t>(static_cast<float>(this->ramp_.n) * rescale);
    this->ramp_.rest = 0;
  }
  this->ramp_.c_min = c_min;
  this->ramp_.c0_accel = c0 >= static_cast<float>(RAMP_C_MAX) ? RAMP_C_MAX : static_cast<uint32_t>(c0);
  this->ramp_.accel_over_decel_q16 = static_cast<uint32_t>(ratio * 65536.0f);
  this->ramp_.decel_over_accel_q16 = static_cast<uint32_t>(65536.0f / ratio);
  portEXIT_CRITICAL(&this->mux_);
  this->acceleration_ = acceleration;
  this->deceleration_ = deceleration;
}

void StepGenerator::write_dir_() { this->dir_pin_->digital_write((this->ramp_.dir > 0) != this->invert_dir_); }

void StepGenerator::start_timer_() {
  this->write_dir_();
  this->last_ticks_ = START_DELAY_TICKS;
  gptimer_alarm_config_t alarm = {};
  alarm.alarm_count = START_DELAY_TICKS;
  alarm.reload_count = 0;
  alarm.flags.auto_reload_on_alarm = true;
  gptimer_set_raw_count(this->timer_, 0);
  gptimer_set_alarm_action(this->timer_, &alarm);
  gptimer_start(this->timer_);
}

void StepGenerator::move_to(int32_t target) {
  if (this->timer_ == nullptr)
    return;
  bool start = false;
  portENTER_CRITICAL(&this->mux_);
  this->ramp_.target = target;
  this->ramp_.continuous = false;
  this->ramp_.stop_request = false;
  if (!this->running_ && target != this->ramp_.position) {
    this->ramp_.dir = target > this->ramp_.position ? 1 : -1;
    ramp_restart(this->ramp_);
    this->frac_ = 0;
    this->running_ = true;
    start = true;
  }
  portEXIT_CRITICAL(&this->mux_);
  if (start)
    this->start_timer_();
}

void StepGenerator::run(bool forward) {
  if (this->timer_ == nullptr)
    return;
  bool start = false;
  portENTER_CRITICAL(&this->mux_);
  this->ramp_.continuous = true;
  this->ramp_.continuous_dir = forward ? 1 : -1;
  this->ramp_.stop_request = false;
  if (!this->running_) {
    this->ramp_.dir = this->ramp_.continuous_dir;
    ramp_restart(this->ramp_);
    this->frac_ = 0;
    this->running_ = true;
    start = true;
  }
  portEXIT_CRITICAL(&this->mux_);
  if (start)
    this->start_timer_();
}

void StepGenerator::stop() {
  portENTER_CRITICAL(&this->mux_);
  if (this->running_)
    this->ramp_.stop_request = true;
  portEXIT_CRITICAL(&this->mux_);
}

void StepGenerator::emergency_stop() {
  portENTER_CRITICAL(&this->mux_);
  const bool was_running = this->running_;
  this->running_ = false;
  this->ramp_.target = this->ramp_.position;
  this->ramp_.continuous = false;
  this->ramp_.stop_request = false;
  portEXIT_CRITICAL(&this->mux_);
  if (was_running) {
    gptimer_stop(this->timer_);
    this->step_isr_pin_.digital_write(false);
  }
}

void StepGenerator::set_position(int32_t position) {
  portENTER_CRITICAL(&this->mux_);
  const int32_t delta = position - this->ramp_.position;
  this->ramp_.position = position;
  if (this->running_ && !this->ramp_.continuous) {
    this->ramp_.target += delta;  // keep the remaining distance
  } else {
    this->ramp_.target = position;
  }
  portEXIT_CRITICAL(&this->mux_);
}

int32_t StepGenerator::get_position() {
  portENTER_CRITICAL(&this->mux_);
  const int32_t position = this->ramp_.position;
  portEXIT_CRITICAL(&this->mux_);
  return position;
}

int32_t StepGenerator::get_target() {
  portENTER_CRITICAL(&this->mux_);
  const int32_t target = this->ramp_.target;
  portEXIT_CRITICAL(&this->mux_);
  return target;
}

void StepGenerator::arm_trigger(InternalGPIOPin *pin, uint32_t debounce_us, bool cruise_only) {
  const ISRInternalGPIOPin isr_pin = pin->to_isr();
  portENTER_CRITICAL(&this->mux_);
  this->trig_isr_pin_ = isr_pin;
  this->trig_debounce_ticks_ = debounce_us * (RAMP_TICK_HZ / 1000000UL);
  this->trig_cruise_only_ = cruise_only;
  this->trig_active_ = false;
  this->trig_accum_ = 0;
  this->trig_fired_ = false;
  this->trig_armed_ = true;
  portEXIT_CRITICAL(&this->mux_);
}

void StepGenerator::disarm_trigger() {
  portENTER_CRITICAL(&this->mux_);
  this->trig_armed_ = false;
  this->trig_fired_ = false;
  portEXIT_CRITICAL(&this->mux_);
}

void IRAM_ATTR StepGenerator::halt_from_isr_(gptimer_handle_t timer) {
  gptimer_stop(timer);
  this->running_ = false;
  this->ramp_.target = this->ramp_.position;
  this->ramp_.continuous = false;
  this->ramp_.stop_request = false;
}

void IRAM_ATTR StepGenerator::on_alarm_(gptimer_handle_t timer) {
  if (this->trig_armed_) {
    if ((!this->trig_cruise_only_ || this->ramp_.phase == RAMP_CRUISE) && this->trig_isr_pin_.digital_read()) {
      if (this->trig_active_) {
        this->trig_accum_ += this->last_ticks_;
      } else {
        this->trig_active_ = true;
        this->trig_accum_ = 0;
      }
      if (this->trig_accum_ >= this->trig_debounce_ticks_) {
        this->trig_armed_ = false;
        this->trig_fired_ = true;
        this->halt_from_isr_(timer);
        return;
      }
    } else {
      this->trig_active_ = false;
    }
  }

  // Rising edge now; the falling edge follows after planning (>= 100 ns required by the TMC22xx).
  this->step_isr_pin_.digital_write(true);
  this->ramp_.position += this->ramp_.dir;
  const RampResult result = ramp_after_step(this->ramp_);
  if (result == RAMP_FINISHED) {
    this->halt_from_isr_(timer);
    this->step_isr_pin_.digital_write(false);
    return;
  }
  if (result == RAMP_REVERSED)
    this->dir_isr_pin_.digital_write((this->ramp_.dir > 0) != this->invert_dir_);

  const uint32_t acc = this->ramp_.c + this->frac_;
  this->frac_ = acc & 0xFF;
  uint64_t ticks = acc >> 8;
  // The counter was reloaded to 0 at the alarm, so intervals do not accumulate ISR latency.
  uint64_t now = 0;
  gptimer_get_raw_count(timer, &now);
  if (ticks < now + ALARM_GUARD_TICKS)
    ticks = now + ALARM_GUARD_TICKS;
  gptimer_alarm_config_t alarm = {};
  alarm.alarm_count = ticks;
  alarm.reload_count = 0;
  alarm.flags.auto_reload_on_alarm = true;
  gptimer_set_alarm_action(timer, &alarm);
  this->last_ticks_ = static_cast<uint32_t>(ticks);
  this->step_isr_pin_.digital_write(false);
}

bool IRAM_ATTR StepGenerator::timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *arg) {
  auto *self = static_cast<StepGenerator *>(arg);
  portENTER_CRITICAL_ISR(&self->mux_);
  if (self->running_)
    self->on_alarm_(timer);
  portEXIT_CRITICAL_ISR(&self->mux_);
  return false;  // no task was woken
}

}  // namespace esphome::tmc22xx

#endif  // USE_ESP32
