#include "tmc22xx.h"

#ifdef USE_ESP32

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include "esphome/core/log.h"
#include "tmc22xx_registers.h"

namespace esphome::tmc22xx {

static const char *const TAG = "tmc22xx";

// Chopper defaults (identical to the reset value 0x10000053 of CHOPCONF).
static const uint8_t CHOP_TOFF = 3;
static const uint8_t CHOP_HSTRT = 5;
static const uint8_t CHOP_HEND = 0;
static const uint8_t CHOP_TBL = 2;
static const uint8_t IHOLD_DELAY = 4;  // current ramp-down speed after TPOWERDOWN

// ---------------------------------------------------------------------------
// Setup / configuration
// ---------------------------------------------------------------------------

int32_t TMC22XX::to_steps_(float units) const { return static_cast<int32_t>(lroundf(units * this->steps_per_unit_)); }

void TMC22XX::setup() {
  // In UART control these pins are optional and only held low: a floating STEP input would make
  // the driver step on noise whenever it stands still.
  if (this->step_pin_ != nullptr) {
    this->step_pin_->setup();
    this->step_pin_->digital_write(false);
  }
  if (this->dir_pin_ != nullptr) {
    this->dir_pin_->setup();
    this->dir_pin_->digital_write(false);
  }
  if (this->enable_pin_ != nullptr) {
    this->enable_pin_->setup();
    this->enable_pin_->digital_write(false);  // logical "not enabled"
  }
  if (this->ms1_pin_ != nullptr)
    this->ms1_pin_->setup();
  if (this->ms2_pin_ != nullptr)
    this->ms2_pin_->setup();
  this->apply_ms_pins_();
#ifdef USE_TMC22XX_STALLGUARD
  if (this->diag_pin_ != nullptr)
    this->diag_pin_->setup();
#endif
#ifdef USE_TMC22XX_HOMING
  if (this->endstop_pin_ != nullptr)
    this->endstop_pin_->setup();
#endif

#ifdef USE_TMC22XX_VACTUAL
  if (this->uart_control_) {
    this->index_pin_->setup();
    this->uart_motion_.setup(&this->uart_, this->index_pin_);
    if (this->creep_speed_ > 0.0f)
      this->uart_motion_.set_creep_speed(this->creep_speed_ * this->steps_per_unit_);
    this->motion_ = &this->uart_motion_;
  }
#endif
  if (!this->uart_control_) {
    if (!this->gen_.setup(this->step_pin_, this->dir_pin_)) {
      this->mark_failed(LOG_STR("no hardware timer available"));
      return;
    }
    // With UART the direction is inverted inside the driver (GCONF.shaft), otherwise on the DIR pin.
    this->gen_.set_invert_direction(this->invert_direction_ && !this->uart_mode_);
  }
  this->ready_ = true;
  this->update_profile_();

#ifdef USE_TMC22XX_UART
  if (this->uart_mode_) {
    this->compute_current_scale_();
    if (!this->configure_driver_())
      this->comm_failed_();
  }
#endif
  if (this->enable_on_boot_)
    this->enable();

  this->publish_position_();
  this->publish_flags_();
#ifdef USE_TMC22XX_HOMING
  if (this->has_homing_ && this->home_on_boot_) {
    // Give the motor supply and the driver some time to come up.
    this->set_timeout(2000, [this]() { this->home(); });
  }
#endif
}

void TMC22XX::dump_config() {
  ESP_LOGCONFIG(TAG,
                "TMC22xx:\n"
                "  Driver: %s\n"
                "  Mode: %s\n"
                "  Microsteps: %u (interpolation: %s)\n"
                "  Steps per unit: %.4f\n"
                "  Max speed: %.2f units/s\n"
                "  Acceleration: %.2f units/s^2\n"
                "  Deceleration: %.2f units/s^2\n"
                "  Invert direction: %s",
                this->driver_type_ == DRIVER_TMC2209 ? "TMC2209" : "TMC2208", this->uart_mode_ ? "UART" : "pins",
                this->microsteps_, YESNO(this->interpolation_ && this->uart_mode_), this->steps_per_unit_,
                this->max_speed_, this->acceleration_, this->deceleration_, YESNO(this->invert_direction_));
  ESP_LOGCONFIG(TAG, "  Control: %s", this->uart_control_ ? "UART (VACTUAL + INDEX)" : "STEP/DIR");
#ifdef USE_TMC22XX_VACTUAL
  LOG_PIN("  INDEX pin: ", this->index_pin_);
#endif
  LOG_PIN("  STEP pin: ", this->step_pin_);
  LOG_PIN("  DIR pin: ", this->dir_pin_);
  LOG_PIN("  EN pin: ", this->enable_pin_);
  LOG_PIN("  MS1 pin: ", this->ms1_pin_);
  LOG_PIN("  MS2 pin: ", this->ms2_pin_);
  if (this->has_min_position_)
    ESP_LOGCONFIG(TAG, "  Soft limit min: %.3f", this->min_position_);
  if (this->has_max_position_)
    ESP_LOGCONFIG(TAG, "  Soft limit max: %.3f", this->max_position_);
  if (this->idle_disable_ms_ != 0)
    ESP_LOGCONFIG(TAG, "  Disable when idle for: %" PRIu32 " ms", this->idle_disable_ms_);
#ifdef USE_TMC22XX_UART
  if (this->uart_mode_) {
    ESP_LOGCONFIG(TAG,
                  "  UART address: %u\n"
                  "  R_sense: %.3f Ohm\n"
                  "  Run current: %.0f mA (IRUN=%u)\n"
                  "  Hold current: %.0f mA (IHOLD=%u), vsense=%u\n"
                  "  Standstill delay: %" PRIu32 " ms\n"
                  "  Chopper: %s\n"
                  "  StealthChop threshold: %.2f units/s (TPWMTHRS=%" PRIu32 ")\n"
                  "  Poll interval: %" PRIu32 " ms\n"
                  "  Driver configured: %s (CRC errors: %u, timeouts: %u)",
                  this->uart_.get_address(), this->rsense_, this->run_current_ma_, this->irun_,
                  this->hold_current_ma_, this->ihold_, this->vsense_ ? 1 : 0, this->standstill_delay_ms_,
                  this->spreadcycle_ ? "SpreadCycle" : "StealthChop", this->stealthchop_threshold_,
                  this->tpwmthrs_value_(), this->poll_interval_ms_, YESNO(this->driver_configured_),
                  this->uart_.crc_error_count(), this->uart_.timeout_count());
  }
#endif
#ifdef USE_TMC22XX_STALLGUARD
  if (this->driver_type_ == DRIVER_TMC2209) {
    ESP_LOGCONFIG(TAG,
                  "  StallGuard threshold (SGTHRS): %u\n"
                  "  Stop on stall: %s",
                  this->stallguard_threshold_, YESNO(this->stop_on_stall_));
    LOG_PIN("  DIAG pin: ", this->diag_pin_);
  }
#endif
#ifdef USE_TMC22XX_HOMING
  if (this->has_homing_) {
    ESP_LOGCONFIG(TAG,
                  "  Homing:\n"
                  "    Source: %s\n"
                  "    Direction: %s\n"
                  "    Speed: %.2f units/s, slow speed: %.2f units/s\n"
                  "    Backoff: %.3f, max travel: %.3f\n"
                  "    Position at reference: %.3f\n"
                  "    Timeout: %" PRIu32 " ms, debounce: %" PRIu32 " us\n"
                  "    Home on boot: %s",
                  this->homing_source_ == HOMING_SOURCE_STALLGUARD ? "StallGuard (DIAG)" : "end stop",
                  this->homing_positive_ ? "positive" : "negative", this->homing_speed_, this->homing_slow_speed_,
                  this->homing_backoff_, this->homing_max_travel_, this->homing_position_, this->homing_timeout_ms_,
                  this->homing_debounce_us_, YESNO(this->home_on_boot_));
    if (this->has_park_position_)
      ESP_LOGCONFIG(TAG, "    Park position: %.3f", this->homing_park_position_);
    LOG_PIN("    End stop pin: ", this->endstop_pin_);
  }
#endif
  if (this->is_failed())
    ESP_LOGE(TAG, "  Setup failed: no hardware timer available for step generation");
}

void TMC22XX::update_profile_() {
  if (!this->ready_)
    return;
  this->motion_->set_profile(this->max_speed_ * this->steps_per_unit_, this->acceleration_ * this->steps_per_unit_,
                         this->deceleration_ * this->steps_per_unit_);
}

void TMC22XX::set_max_speed(float speed) {
  this->max_speed_ = speed;
  if (!this->is_homing())
    this->update_profile_();
}

void TMC22XX::set_acceleration(float acceleration) {
  this->acceleration_ = acceleration;
  if (!this->is_homing())
    this->update_profile_();
}

void TMC22XX::set_deceleration(float deceleration) {
  this->deceleration_ = deceleration;
  if (!this->is_homing())
    this->update_profile_();
}

void TMC22XX::apply_ms_pins_() {
  if (this->ms1_pin_ == nullptr || this->ms2_pin_ == nullptr)
    return;
  // MS2/MS1 = 00 -> 1/8 and 11 -> 1/16 on both chips.
  // TMC2208: 01 -> 1/2, 10 -> 1/4.   TMC2209: 01 -> 1/32, 10 -> 1/64.
  bool ms1 = false, ms2 = false;
  switch (this->microsteps_) {
    case 2:
    case 32:
      ms1 = true;
      break;
    case 4:
    case 64:
      ms2 = true;
      break;
    case 16:
      ms1 = ms2 = true;
      break;
    default:  // 8
      break;
  }
  this->ms1_pin_->digital_write(ms1);
  this->ms2_pin_->digital_write(ms2);
}

void TMC22XX::set_microsteps(uint16_t microsteps) {
  if (microsteps == 0 || microsteps > 256 || (microsteps & (microsteps - 1)) != 0) {
    ESP_LOGW(TAG, "Invalid microstep setting %u (must be a power of two, 1..256)", microsteps);
    return;
  }
  if (!this->uart_mode_ && this->ready_) {
    const bool valid = this->driver_type_ == DRIVER_TMC2209
                           ? (microsteps == 8 || microsteps == 16 || microsteps == 32 || microsteps == 64)
                           : (microsteps == 2 || microsteps == 4 || microsteps == 8 || microsteps == 16);
    if (!valid || this->ms1_pin_ == nullptr) {
      ESP_LOGW(TAG, "Microsteps %u cannot be selected via MS1/MS2", microsteps);
      return;
    }
  }
  this->microsteps_ = microsteps;
  if (!this->ready_)
    return;
  this->apply_ms_pins_();
#ifdef USE_TMC22XX_UART
  if (this->uart_mode_ && this->driver_configured_) {
    this->write_(REG_CHOPCONF, this->chopconf_value_());
    this->write_(REG_TPWMTHRS, this->tpwmthrs_value_());
  }
#endif
}

// ---------------------------------------------------------------------------
// Motion API
// ---------------------------------------------------------------------------

void TMC22XX::start_steps_(int32_t target) {
  if (!this->enabled_)
    this->enable();
  this->idle_pending_ = false;
  this->moving_ = true;
  this->motion_->move_to(target);
}

void TMC22XX::move_to(float position) {
  if (!this->ready_)
    return;
  if (this->is_homing()) {
    ESP_LOGW(TAG, "Move ignored: homing in progress");
    return;
  }
  if (this->has_max_position_ && position > this->max_position_) {
    ESP_LOGW(TAG, "Target %.3f limited to soft limit %.3f", position, this->max_position_);
    position = this->max_position_;
  }
  if (this->has_min_position_ && position < this->min_position_) {
    ESP_LOGW(TAG, "Target %.3f limited to soft limit %.3f", position, this->min_position_);
    position = this->min_position_;
  }
#ifdef USE_TMC22XX_STALLGUARD
  this->stall_prepare_();
#endif
#ifdef USE_TMC22XX_NUMBER
  if (this->target_number_ != nullptr)
    this->target_number_->publish_state(position);
#endif
  ESP_LOGD(TAG, "Move to %.3f", position);
  this->start_steps_(this->to_steps_(position));
}

void TMC22XX::move_by(float distance) { this->move_to(this->get_position() + distance); }

void TMC22XX::run(bool forward) {
  if (!this->ready_)
    return;
  if (this->is_homing()) {
    ESP_LOGW(TAG, "Run ignored: homing in progress");
    return;
  }
  // With a soft limit in the direction of travel a continuous run becomes a move to that limit.
  if (forward && this->has_max_position_) {
    this->move_to(this->max_position_);
    return;
  }
  if (!forward && this->has_min_position_) {
    this->move_to(this->min_position_);
    return;
  }
#ifdef USE_TMC22XX_STALLGUARD
  this->stall_prepare_();
#endif
  if (!this->enabled_)
    this->enable();
  ESP_LOGD(TAG, "Run %s", forward ? "forward" : "backward");
  this->idle_pending_ = false;
  this->moving_ = true;
  this->motion_->run(forward);
}

void TMC22XX::stop() {
#ifdef USE_TMC22XX_HOMING
  if (this->is_homing()) {
    this->homing_fail_(LOG_STR("aborted by stop request"));
    return;
  }
#endif
  this->motion_->stop();
}

void TMC22XX::emergency_stop() {
#ifdef USE_TMC22XX_HOMING
  if (this->is_homing()) {
    this->homing_fail_(LOG_STR("aborted by emergency stop"));
    return;
  }
#endif
  this->motion_->emergency_stop();
}

void TMC22XX::set_position(float position) {
  this->motion_->set_position(this->to_steps_(position));
  this->publish_position_();
}

void TMC22XX::enable() {
  this->enabled_ = true;
  if (this->enable_pin_ != nullptr) {
    this->enable_pin_->digital_write(true);
    return;
  }
#ifdef USE_TMC22XX_UART
  if (this->uart_mode_ && this->driver_configured_)
    this->write_(REG_CHOPCONF, this->chopconf_value_());
#endif
}

void TMC22XX::disable() {
  if (this->motion_->is_running())
    this->motion_->emergency_stop();
  this->enabled_ = false;
  this->idle_pending_ = false;
  if (this->enable_pin_ != nullptr) {
    this->enable_pin_->digital_write(false);
    return;
  }
#ifdef USE_TMC22XX_UART
  // Without EN pin the power stage is switched off through TOFF = 0.
  if (this->uart_mode_ && this->driver_configured_)
    this->write_(REG_CHOPCONF, this->chopconf_value_());
#endif
}

void TMC22XX::publish_position_() {
#ifdef USE_TMC22XX_SENSOR
  if (this->position_sensor_ != nullptr)
    this->position_sensor_->publish_state(this->get_position());
#endif
}

void TMC22XX::publish_flags_() {
#ifdef USE_TMC22XX_BINARY_SENSOR
  if (this->moving_binary_sensor_ != nullptr)
    this->moving_binary_sensor_->publish_state(this->moving_);
  if (this->homed_binary_sensor_ != nullptr)
    this->homed_binary_sensor_->publish_state(this->homed_);
  if (this->homing_binary_sensor_ != nullptr)
    this->homing_binary_sensor_->publish_state(this->is_homing());
#endif
}

void TMC22XX::on_motion_end_() {
  this->last_stop_ms_ = millis();
  this->idle_pending_ = true;
  this->publish_position_();
#ifdef USE_TMC22XX_HOMING
  if (this->is_homing()) {
    this->homing_on_motion_end_();
    return;
  }
#endif
#ifdef USE_TMC22XX_STALLGUARD
  if (this->stop_on_stall_ && this->motion_->trigger_fired())
    this->stall_event_();
  this->motion_->disarm_trigger();
#endif
  ESP_LOGD(TAG, "Move complete at %.3f", this->get_position());
  this->move_complete_cb_.call();
}

void TMC22XX::loop() {
  if (!this->ready_)
    return;
  const uint32_t now = millis();
  this->motion_->loop();
  if (this->motion_->stop_failed() && this->enabled_ && this->enable_pin_ != nullptr) {
    // The driver keeps turning the motor and cannot be reached: cut the power stage via EN.
    ESP_LOGE(TAG, "Stop command not acknowledged, disabling the driver through the EN pin");
    this->enable_pin_->digital_write(false);
    this->enabled_ = false;
  }
  const bool running = this->motion_->is_running();

  if (this->moving_ && !running) {
    this->moving_ = false;
    this->on_motion_end_();  // may start the next homing phase
  }
  if (this->moving_ != this->last_moving_published_) {
    this->last_moving_published_ = this->moving_;
    this->publish_flags_();
  }

  if (running && now - this->last_report_ms_ >= this->report_interval_ms_) {
    this->last_report_ms_ = now;
    this->publish_position_();
  }

#ifdef USE_TMC22XX_HOMING
  if (this->is_homing() && now - this->homing_started_ms_ >= this->homing_timeout_ms_)
    this->homing_fail_(LOG_STR("timeout"));
#endif

#ifdef USE_TMC22XX_STALLGUARD
  // Monitoring only (no automatic stop): DIAG is evaluated while cruising, where StallGuard is valid.
  if (running && !this->stop_on_stall_ && !this->stall_reported_ && !this->is_homing() &&
      this->diag_pin_ != nullptr && this->motion_->is_cruising() && this->diag_pin_->digital_read()) {
    this->stall_event_();
  }
#endif

  if (this->idle_pending_ && this->idle_disable_ms_ != 0 && !this->moving_ && this->enabled_ &&
      now - this->last_stop_ms_ >= this->idle_disable_ms_) {
    ESP_LOGD(TAG, "Idle timeout, disabling driver");
    this->disable();
  }

#ifdef USE_TMC22XX_UART
  // Diagnostics are skipped during homing; a poll costs about 2-3 ms of loop time.
  if (this->uart_mode_ && !this->is_homing() && now - this->last_poll_ms_ >= this->poll_interval_ms_) {
    this->last_poll_ms_ = now;
    this->poll_driver_();
  }
#endif
}

// ---------------------------------------------------------------------------
// UART driver configuration and diagnostics
// ---------------------------------------------------------------------------
#ifdef USE_TMC22XX_UART

bool TMC22XX::write_(uint8_t reg, uint32_t value) {
  if (this->uart_.write(reg, value))
    return true;
  ESP_LOGW(TAG, "Write to register 0x%02X failed (%s)", reg,
           this->uart_.last_error() == UartError::TIMEOUT
               ? "no reply"
               : this->uart_.last_error() == UartError::CRC ? "CRC error" : "IFCNT did not increment");
  return false;
}

void TMC22XX::compute_current_scale_() {
  // I_rms = (CS + 1) / 32 * V_FS / (R_sense + 20 mOhm) / sqrt(2), V_FS = 325 mV (vsense=0) or 180 mV (vsense=1)
  const float k = 32.0f * 1.41421f * (this->rsense_ + 0.02f) / 1000.0f;
  float cs = k * this->run_current_ma_ / 0.325f - 1.0f;
  this->vsense_ = false;
  if (cs < 16.0f) {
    // Use the high sensitivity range for better current resolution.
    this->vsense_ = true;
    cs = k * this->run_current_ma_ / 0.180f - 1.0f;
  }
  const float vfs = this->vsense_ ? 0.180f : 0.325f;
  this->irun_ = static_cast<uint8_t>(std::clamp(lroundf(cs), 0L, 31L));
  const float hold = k * this->hold_current_ma_ / vfs - 1.0f;
  this->ihold_ = static_cast<uint8_t>(std::clamp(lroundf(hold), 0L, static_cast<long>(this->irun_)));
}

uint32_t TMC22XX::gconf_value_(bool force_stealthchop) const {
  // Internal reference instead of VREF pin (I_scale_analog = 0), PDN_UART used for UART only,
  // microstep resolution from MRES instead of the MS1/MS2 pins.
  uint32_t gconf = GCONF_PDN_DISABLE | GCONF_MSTEP_REG_SELECT | GCONF_MULTISTEP_FILT;
  if (this->uart_control_)
    gconf |= GCONF_INDEX_STEP;  // INDEX reports the steps of the internal pulse generator
  if (this->spreadcycle_ && !force_stealthchop)
    gconf |= GCONF_EN_SPREADCYCLE;
  if (this->invert_direction_)
    gconf |= GCONF_SHAFT;
  return gconf;
}

uint32_t TMC22XX::chopconf_value_() const {
  uint8_t mres = 0;  // 0 = 256 microsteps ... 8 = full step
  for (uint16_t ms = this->microsteps_; ms < 256; ms <<= 1)
    mres++;
  const bool output_on = this->enabled_ || this->enable_pin_ != nullptr;
  uint32_t value = (output_on ? CHOP_TOFF : 0) | (static_cast<uint32_t>(CHOP_HSTRT) << CHOPCONF_HSTRT_SHIFT) |
                   (static_cast<uint32_t>(CHOP_HEND) << CHOPCONF_HEND_SHIFT) |
                   (static_cast<uint32_t>(CHOP_TBL) << CHOPCONF_TBL_SHIFT) |
                   (static_cast<uint32_t>(mres) << CHOPCONF_MRES_SHIFT);
  if (this->vsense_)
    value |= CHOPCONF_VSENSE;
  if (this->interpolation_)
    value |= CHOPCONF_INTPOL;
  return value;
}

uint32_t TMC22XX::ihold_irun_value_() const {
  return (static_cast<uint32_t>(this->ihold_) << IHOLD_SHIFT) | (static_cast<uint32_t>(this->irun_) << IRUN_SHIFT) |
         (static_cast<uint32_t>(IHOLD_DELAY) << IHOLDDELAY_SHIFT);
}

uint32_t TMC22XX::tpwmthrs_value_() const {
  if (this->stealthchop_threshold_ <= 0.0f)
    return 0;
  // TSTEP = f_CLK / (step frequency in 1/256 microsteps); StealthChop is active while TSTEP >= TPWMTHRS.
  const float steps_per_s = this->stealthchop_threshold_ * this->steps_per_unit_;
  const float tstep = static_cast<float>(TMC_FCLK_HZ) * this->microsteps_ / (256.0f * steps_per_s);
  return static_cast<uint32_t>(std::clamp(tstep, 1.0f, 1048575.0f));
}

bool TMC22XX::configure_driver_() {
  this->uart_.invalidate();
  uint32_t ioin = 0;
  if (!this->uart_.read(REG_IOIN, &ioin))
    return false;
  const uint8_t version = ioin >> IOIN_VERSION_SHIFT;
  const uint8_t expected = this->driver_type_ == DRIVER_TMC2209 ? IOIN_VERSION_TMC2209 : IOIN_VERSION_TMC2208;
  if (version != expected) {
    ESP_LOGW(TAG, "IOIN reports version 0x%02X, expected 0x%02X - check the 'driver' option", version, expected);
  }
  ESP_LOGD(TAG, "IOIN=0x%08" PRIX32 " (version 0x%02X, ENN=%u, MS1=%u, MS2=%u)", ioin, version,
           (ioin & IOIN_ENN) ? 1 : 0, (ioin & IOIN_MS1) ? 1 : 0, (ioin & IOIN_MS2) ? 1 : 0);

  uint32_t tpowerdown = (this->standstill_delay_ms_ * 1000UL + 10922UL) / 21845UL;  // units of 2^18 / f_CLK
  tpowerdown = std::clamp<uint32_t>(tpowerdown, 2, 255);  // >= 2 for StealthChop tuning

  bool ok = true;
  // A driver that kept its supply across an ESP restart may still be turning the motor.
  if (this->uart_control_ && !this->motion_->is_running())
    ok = this->write_(REG_VACTUAL, 0);
  ok = ok && this->write_(REG_GCONF, this->gconf_value_(false));
  ok = ok && this->write_(REG_CHOPCONF, this->chopconf_value_());
  ok = ok && this->write_(REG_IHOLD_IRUN, this->ihold_irun_value_());
  ok = ok && this->write_(REG_TPOWERDOWN, tpowerdown);
  ok = ok && this->write_(REG_TPWMTHRS, this->tpwmthrs_value_());
#ifdef USE_TMC22XX_STALLGUARD
  if (this->driver_type_ == DRIVER_TMC2209) {
    // StallGuard/DIAG is active while TCOOLTHRS >= TSTEP > TPWMTHRS; enable it for all speeds if DIAG is wired.
    ok = ok && this->write_(REG_TCOOLTHRS, this->diag_pin_ != nullptr ? 0xFFFFFUL : 0UL);
    ok = ok && this->write_(REG_SGTHRS, this->stallguard_threshold_);
  }
#endif
  ok = ok && this->write_(REG_GSTAT, GSTAT_RESET | GSTAT_DRV_ERR | GSTAT_UV_CP);  // write 1 to clear
  this->driver_configured_ = ok;
  if (ok) {
    ESP_LOGI(TAG, "Driver configured (IRUN=%u, IHOLD=%u, vsense=%u)", this->irun_, this->ihold_, this->vsense_);
    if (!this->wiring_checked_)
      this->check_wiring_();
  }
  return ok;
}

void TMC22XX::check_wiring_() {
  // IOIN mirrors the levels the driver sees on its pins, so the STEP/DIR/EN wiring can be verified
  // without moving the motor (the single STEP edge used here is one microstep).
  if (this->uart_control_) {
    this->wiring_checked_ = true;  // no STEP/DIR lines in this mode
    return;
  }
  if (this->motion_->is_running())
    return;
  // On a re-check only the lines that failed before are toggled again.
  const bool first = !this->wiring_checked_;
  uint32_t high = 0, low = 0;
  bool ok = true;
  bool dir_ok = true, step_ok = true;
  if (first || (this->wiring_errors_ & ERR_DIR_WIRING)) {
    this->dir_pin_->digital_write(true);
    ok = this->uart_.read(REG_IOIN, &high);
    this->dir_pin_->digital_write(false);
    ok = ok && this->uart_.read(REG_IOIN, &low);
    dir_ok = ((high ^ low) & IOIN_DIR) != 0;
  }
  if (first || (this->wiring_errors_ & ERR_STEP_WIRING)) {
    this->step_pin_->digital_write(true);
    ok = ok && this->uart_.read(REG_IOIN, &high);
    this->step_pin_->digital_write(false);
    ok = ok && this->uart_.read(REG_IOIN, &low);
    step_ok = ((high ^ low) & IOIN_STEP) != 0;
  }
  if (!ok)
    return;  // retried with the next configuration attempt / poll
  this->wiring_checked_ = true;
  const uint32_t errors = (step_ok ? 0 : ERR_STEP_WIRING) | (dir_ok ? 0 : ERR_DIR_WIRING);
  const bool changed = first || errors != this->wiring_errors_;
  this->wiring_errors_ = errors;
  if (!changed)
    return;

  if (step_ok && dir_ok) {
    ESP_LOGI(TAG, "Wiring check: STEP and DIR reach the driver");
  } else {
    if (!step_ok)
      ESP_LOGE(TAG, "Wiring check: the driver does not see the STEP pin toggling - check the STEP wire / GPIO");
    if (!dir_ok)
      ESP_LOGE(TAG, "Wiring check: the driver does not see the DIR pin toggling - check the DIR wire / GPIO");
  }
  if (low & IOIN_ENN)
    ESP_LOGW(TAG, "Wiring check: ENN is high at the driver, the power stage is disabled by the pin");
  uint32_t drv = 0;
  if (this->uart_.read(REG_DRV_STATUS, &drv)) {
    ESP_LOGD(TAG, "DRV_STATUS=0x%08" PRIX32 " (CS_ACTUAL=%u, stealth=%u, stst=%u)", drv,
             static_cast<unsigned>((drv >> DRV_CS_ACTUAL_SHIFT) & 0x1F), (drv & DRV_STEALTH) ? 1 : 0,
             (drv & DRV_STST) ? 1 : 0);
  }
}

void TMC22XX::spin_test(float speed) {
  if (!this->ready_ || !this->uart_mode_ || !this->driver_configured_) {
    ESP_LOGW(TAG, "spin_test requires a configured driver in mode: uart");
    return;
  }
  if (this->is_moving()) {
    ESP_LOGW(TAG, "spin_test ignored: motor is moving");
    return;
  }
  if (!this->enabled_)
    this->enable();
  // VACTUAL lets the driver generate the steps itself: v [usteps/s] = VACTUAL * f_CLK / 2^24.
  // The STEP input is ignored while VACTUAL != 0 and the position counter is not updated.
  const float usteps = speed * this->steps_per_unit_;
  const int32_t vactual = static_cast<int32_t>(lroundf(usteps * 16777216.0f / static_cast<float>(TMC_FCLK_HZ)));
  ESP_LOGI(TAG, "Spin test: driver-generated motion at %.0f steps/s for 2 s (VACTUAL=%" PRId32 ")", usteps, vactual);
  if (!this->write_(REG_VACTUAL, static_cast<uint32_t>(vactual) & 0x00FFFFFFUL))
    return;
  this->set_timeout("spin_test", 2000, [this]() {
    this->write_(REG_VACTUAL, 0);
    ESP_LOGI(TAG, "Spin test finished");
  });
}

void TMC22XX::comm_failed_() {
  if (this->driver_configured_ || !(this->errors_ & ERR_COMM))
    ESP_LOGW(TAG, "Driver not responding on UART (check wiring, VM supply and address)");
  this->driver_configured_ = false;
  this->update_status_(ERR_COMM, 0);
}

void TMC22XX::poll_driver_() {
  if (!this->driver_configured_) {
    // Retry until the driver shows up, e.g. when the motor supply is switched on later.
    if (!this->configure_driver_()) {
      this->comm_failed_();
      return;
    }
  }
  uint32_t gstat = 0, drv = 0;
  if (!this->uart_.read(REG_GSTAT, &gstat)) {
    this->comm_failed_();
    return;
  }
  if (gstat & GSTAT_RESET) {
    ESP_LOGW(TAG, "Driver reset detected, restoring configuration");
    if (!this->configure_driver_()) {
      this->comm_failed_();
      return;
    }
  } else if (gstat != 0) {
    this->write_(REG_GSTAT, gstat);  // clear latched flags
  }
  if (!this->uart_.read(REG_DRV_STATUS, &drv)) {
    this->comm_failed_();
    return;
  }
  // A failed wiring check stays visible as an error and is repeated until the lines are fixed.
  if (this->wiring_errors_ != 0)
    this->check_wiring_();
  uint32_t errors = (drv & DRV_ERROR_MASK) | this->wiring_errors_;
  if (gstat & GSTAT_UV_CP)
    errors |= ERR_UV_CP;
  this->update_status_(errors, drv);

#ifdef USE_TMC22XX_SENSOR
  if (this->actual_current_sensor_ != nullptr) {
    // Sensors are only published when the value changes, not with every poll.
    const uint8_t cs = (drv >> DRV_CS_ACTUAL_SHIFT) & 0x1F;
    if (cs != this->last_cs_) {
      this->last_cs_ = cs;
      const float vfs = this->vsense_ ? 0.180f : 0.325f;
      this->actual_current_sensor_->publish_state((cs + 1) / 32.0f * vfs / (this->rsense_ + 0.02f) / 1.41421f *
                                                  1000.0f);
    }
  }
#ifdef USE_TMC22XX_STALLGUARD
  if (this->stallguard_result_sensor_ != nullptr && this->driver_type_ == DRIVER_TMC2209) {
    uint32_t sg = 0;
    if (this->uart_.read(REG_SG_RESULT, &sg) && (sg & 0x3FF) != this->last_sg_) {
      this->last_sg_ = sg & 0x3FF;
      this->stallguard_result_sensor_->publish_state(this->last_sg_);
    }
  }
#endif
#endif
}

void TMC22XX::update_status_(uint32_t errors, uint32_t drv_status) {
  // Open load flags are only meaningful while the motor is moving.
  uint32_t status = errors | (drv_status & DRV_OTPW);
  if (!(drv_status & DRV_STST))
    status |= drv_status & (DRV_OLA | DRV_OLB);
  const uint32_t new_errors = errors & ~this->errors_;
  this->errors_ = errors;
  if (status == this->last_status_)
    return;
  this->last_status_ = status;

  static const struct {
    uint32_t mask;
    const char *name;
  } FLAGS[] = {
      {ERR_COMM, "no_uart"}, {ERR_STEP_WIRING, "step_nc"}, {ERR_DIR_WIRING, "dir_nc"}, {DRV_OT, "ot"},       {DRV_OTPW, "otpw"},   {DRV_S2GA, "s2ga"}, {DRV_S2GB, "s2gb"},
      {DRV_S2VSA, "s2vsa"},  {DRV_S2VSB, "s2vsb"}, {ERR_UV_CP, "uv_cp"}, {DRV_OLA, "ola"},   {DRV_OLB, "olb"},
  };
  char text[64];
  size_t pos = 0;
  text[0] = '\0';
  for (const auto &flag : FLAGS) {
    if (!(status & flag.mask))
      continue;
    const size_t len = strlen(flag.name);
    if (pos + len + 2 > sizeof(text))
      break;
    if (pos != 0)
      text[pos++] = ',';
    memcpy(&text[pos], flag.name, len + 1);
    pos += len;
  }
  if (pos == 0)
    strcpy(text, "OK");  // NOLINT

  if (errors != 0) {
    ESP_LOGE(TAG, "Driver error: %s", text);
    this->status_set_warning(LOG_STR("TMC driver error"));
  } else {
    ESP_LOGD(TAG, "Driver status: %s", text);
    this->status_clear_warning();
  }
#ifdef USE_TMC22XX_BINARY_SENSOR
  if (this->driver_error_binary_sensor_ != nullptr)
    this->driver_error_binary_sensor_->publish_state(errors != 0);
  if (this->overtemperature_warning_binary_sensor_ != nullptr)
    this->overtemperature_warning_binary_sensor_->publish_state((status & (DRV_OTPW | DRV_OT)) != 0);
#endif
#ifdef USE_TMC22XX_TEXT_SENSOR
  if (this->driver_status_text_sensor_ != nullptr)
    this->driver_status_text_sensor_->publish_state(text);
#endif
  if (new_errors != 0)
    this->driver_error_cb_.call(errors);
}

void TMC22XX::set_run_current(float milliamps) {
  this->run_current_ma_ = milliamps;
  if (!this->ready_)
    return;
  this->compute_current_scale_();
  if (this->driver_configured_) {
    this->write_(REG_CHOPCONF, this->chopconf_value_());  // vsense may have changed
    this->write_(REG_IHOLD_IRUN, this->ihold_irun_value_());
  }
}

void TMC22XX::set_hold_current(float milliamps) {
  this->hold_current_ma_ = milliamps;
  if (!this->ready_)
    return;
  this->compute_current_scale_();
  if (this->driver_configured_)
    this->write_(REG_IHOLD_IRUN, this->ihold_irun_value_());
}

void TMC22XX::set_stealthchop_threshold(float speed) {
  this->stealthchop_threshold_ = speed;
  if (this->ready_ && this->driver_configured_)
    this->write_(REG_TPWMTHRS, this->tpwmthrs_value_());
}

#else  // !USE_TMC22XX_UART

void TMC22XX::set_run_current(float milliamps) { ESP_LOGW(TAG, "set_current requires mode: uart"); }
void TMC22XX::set_hold_current(float milliamps) { ESP_LOGW(TAG, "set_current requires mode: uart"); }
void TMC22XX::set_stealthchop_threshold(float speed) {}
void TMC22XX::spin_test(float speed) { ESP_LOGW(TAG, "spin_test requires mode: uart"); }

#endif  // USE_TMC22XX_UART

// ---------------------------------------------------------------------------
// StallGuard (TMC2209 only)
// ---------------------------------------------------------------------------
#ifdef USE_TMC22XX_STALLGUARD

void TMC22XX::set_stallguard_threshold(uint8_t threshold) {
  this->stallguard_threshold_ = threshold;
  if (this->ready_ && this->driver_configured_ && this->driver_type_ == DRIVER_TMC2209)
    this->write_(REG_SGTHRS, threshold);
}

void TMC22XX::stall_prepare_() {
  // Called before every user initiated motion: clear the previous stall and re-arm the DIAG trigger.
  this->stall_reported_ = false;
#ifdef USE_TMC22XX_BINARY_SENSOR
  if (this->stall_binary_sensor_ != nullptr)
    this->stall_binary_sensor_->publish_state(false);
#endif
  if (this->stop_on_stall_ && this->diag_pin_ != nullptr)
    this->motion_->arm_trigger(this->diag_pin_, 0, true);
}

void TMC22XX::stall_event_() {
  this->stall_reported_ = true;
  ESP_LOGW(TAG, "Stall detected at %.3f", this->get_position());
#ifdef USE_TMC22XX_BINARY_SENSOR
  if (this->stall_binary_sensor_ != nullptr)
    this->stall_binary_sensor_->publish_state(true);
#endif
  this->stall_cb_.call();
}

#else  // !USE_TMC22XX_STALLGUARD

void TMC22XX::set_stallguard_threshold(uint8_t threshold) {
  ESP_LOGW(TAG, "StallGuard requires a TMC2209 in mode: uart");
}

#endif  // USE_TMC22XX_STALLGUARD

// ---------------------------------------------------------------------------
// Homing
// ---------------------------------------------------------------------------
#ifdef USE_TMC22XX_HOMING

void TMC22XX::home() {
  if (!this->ready_ || !this->has_homing_) {
    ESP_LOGW(TAG, "Homing is not configured");
    return;
  }
  if (this->is_homing())
    return;
  this->motion_->emergency_stop();
  this->motion_->disarm_trigger();
  this->homed_ = false;
  this->homing_started_ms_ = millis();

#if defined(USE_TMC22XX_STALLGUARD) && defined(USE_TMC22XX_UART)
  if (this->homing_source_ == HOMING_SOURCE_STALLGUARD) {
    // StallGuard4 only works in StealthChop: force it for the homing run and enable DIAG for all speeds.
    bool ok = this->driver_configured_;
    ok = ok && this->write_(REG_GCONF, this->gconf_value_(true));
    ok = ok && this->write_(REG_TPWMTHRS, 0);
    ok = ok && this->write_(REG_TCOOLTHRS, 0xFFFFFUL);
    ok = ok && this->write_(REG_SGTHRS, this->stallguard_threshold_);
    if (!ok) {
      this->homing_state_ = HOMING_SEARCH;
      this->homing_fail_(LOG_STR("driver not reachable, StallGuard unavailable"));
      return;
    }
  }
#endif

  ESP_LOGI(TAG, "Homing started");
  if (this->homing_source_ == HOMING_SOURCE_ENDSTOP && this->endstop_pin_->digital_read()) {
    // Already on the switch: release it first.
    if (this->homing_backoff_ <= 0.0f) {
      this->homing_state_ = HOMING_SEARCH;
      this->homing_fail_(LOG_STR("end stop active at start and no backoff configured"));
      return;
    }
    this->homing_start_phase_(HOMING_BACKOFF);
  } else {
    this->homing_start_phase_(HOMING_SEARCH);
  }
  this->publish_flags_();
}

void TMC22XX::homing_start_phase_(HomingState state) {
  this->homing_state_ = state;
  InternalGPIOPin *trigger_pin = this->endstop_pin_;
  uint32_t debounce_us = this->homing_debounce_us_;
  bool cruise_only = false;
#ifdef USE_TMC22XX_STALLGUARD
  if (this->homing_source_ == HOMING_SOURCE_STALLGUARD) {
    // DIAG is only evaluated at constant speed; StallGuard is not reliable during acceleration.
    trigger_pin = this->diag_pin_;
    debounce_us = 0;
    cruise_only = true;
  }
#endif
  const int32_t sign = this->homing_positive_ ? 1 : -1;
  const int32_t position = this->motion_->get_position();
  const int32_t backoff = abs(this->to_steps_(this->homing_backoff_));
  float speed = this->homing_speed_;
  int32_t target = position;

  switch (state) {
    case HOMING_SEARCH:
      this->motion_->arm_trigger(trigger_pin, debounce_us, cruise_only);
      target = position + sign * abs(this->to_steps_(this->homing_max_travel_));
      break;
    case HOMING_SLOW:
      this->motion_->arm_trigger(trigger_pin, debounce_us, cruise_only);
      speed = this->homing_slow_speed_;
      target = position + sign * 2 * backoff;
      break;
    case HOMING_BACKOFF:
    case HOMING_RETRACT:
      this->motion_->disarm_trigger();
      target = position - sign * backoff;
      break;
    case HOMING_PARK:
      this->motion_->disarm_trigger();
      speed = this->max_speed_;
      target = this->to_steps_(this->homing_park_position_);
      break;
    default:
      return;
  }
  ESP_LOGD(TAG, "Homing phase %u: %" PRId32 " -> %" PRId32 " steps at %.2f units/s", state, position, target, speed);
  this->motion_->set_profile(speed * this->steps_per_unit_, this->acceleration_ * this->steps_per_unit_,
                         this->deceleration_ * this->steps_per_unit_);
  this->start_steps_(target);
}

void TMC22XX::homing_on_motion_end_() {
  const bool fired = this->motion_->trigger_fired();
  const bool endstop = this->homing_source_ == HOMING_SOURCE_ENDSTOP;
  switch (this->homing_state_) {
    case HOMING_SEARCH:
      if (!fired) {
        this->homing_fail_(LOG_STR("no trigger within max_travel"));
      } else if (this->homing_slow_speed_ > 0.0f) {
        this->homing_start_phase_(HOMING_BACKOFF);
      } else {
        this->homing_after_reference_();
      }
      break;
    case HOMING_BACKOFF:
      if (endstop && this->endstop_pin_->digital_read()) {
        this->homing_fail_(LOG_STR("end stop still active after backoff"));
      } else {
        this->homing_start_phase_(this->homing_slow_speed_ > 0.0f ? HOMING_SLOW : HOMING_SEARCH);
      }
      break;
    case HOMING_SLOW:
      if (!fired) {
        this->homing_fail_(LOG_STR("no trigger during slow approach"));
      } else {
        this->homing_after_reference_();
      }
      break;
    case HOMING_RETRACT:
      if (endstop && this->endstop_pin_->digital_read()) {
        this->homing_fail_(LOG_STR("end stop still active after retract"));
      } else if (this->has_park_position_) {
        this->homing_start_phase_(HOMING_PARK);
      } else {
        this->homing_finish_();
      }
      break;
    case HOMING_PARK:
      this->homing_finish_();
      break;
    default:
      break;
  }
}

void TMC22XX::homing_after_reference_() {
  // The trigger point becomes the reference position.
  // A backend that stops with some delay reports how far it ran past the trigger point.
  const int32_t overshoot = this->motion_->trigger_overshoot();
  this->motion_->disarm_trigger();
  this->motion_->set_position(this->to_steps_(this->homing_position_) + overshoot);
  if (this->homing_backoff_ > 0.0f) {
    this->homing_start_phase_(HOMING_RETRACT);
  } else if (this->has_park_position_) {
    this->homing_start_phase_(HOMING_PARK);
  } else {
    this->homing_finish_();
  }
}

void TMC22XX::homing_restore_() {
  this->motion_->disarm_trigger();
  this->homing_state_ = HOMING_IDLE;
  this->update_profile_();
#if defined(USE_TMC22XX_STALLGUARD) && defined(USE_TMC22XX_UART)
  if (this->homing_source_ == HOMING_SOURCE_STALLGUARD && this->driver_configured_) {
    this->write_(REG_GCONF, this->gconf_value_(false));
    this->write_(REG_TPWMTHRS, this->tpwmthrs_value_());
  }
#endif
}

void TMC22XX::homing_finish_() {
  this->homing_restore_();
  this->homed_ = true;
  this->publish_position_();
  this->publish_flags_();
#ifdef USE_TMC22XX_NUMBER
  if (this->target_number_ != nullptr)
    this->target_number_->publish_state(this->get_position());
#endif
  ESP_LOGI(TAG, "Homing complete, position %.3f", this->get_position());
  this->homing_complete_cb_.call();
}

void TMC22XX::homing_fail_(const LogString *reason) {
  this->motion_->emergency_stop();
  this->moving_ = false;
  this->last_stop_ms_ = millis();
  this->idle_pending_ = true;
  this->homing_restore_();
  this->homed_ = false;
  this->publish_position_();
  this->publish_flags_();
  ESP_LOGE(TAG, "Homing failed: %s", LOG_STR_ARG(reason));
  this->homing_failed_cb_.call();
}

#else  // !USE_TMC22XX_HOMING

void TMC22XX::home() { ESP_LOGW(TAG, "Homing is not configured"); }

#endif  // USE_TMC22XX_HOMING

}  // namespace esphome::tmc22xx

#endif  // USE_ESP32
