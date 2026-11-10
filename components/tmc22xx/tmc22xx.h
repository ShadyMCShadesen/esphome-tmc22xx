#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP32

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"

#include "tmc22xx_stepgen.h"
#ifdef USE_TMC22XX_UART
#include "tmc22xx_uart.h"
#endif
#ifdef USE_TMC22XX_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_TMC22XX_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_TMC22XX_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif
#ifdef USE_TMC22XX_NUMBER
#include "esphome/components/number/number.h"
#endif

namespace esphome::tmc22xx {

enum DriverType : uint8_t { DRIVER_TMC2208 = 0, DRIVER_TMC2209 = 1 };
enum HomingSource : uint8_t { HOMING_SOURCE_ENDSTOP = 0, HOMING_SOURCE_STALLGUARD = 1 };
enum HomingState : uint8_t {
  HOMING_IDLE = 0,
  HOMING_SEARCH,   // fast approach, trigger armed
  HOMING_BACKOFF,  // release the switch before the slow approach
  HOMING_SLOW,     // slow second approach, trigger armed
  HOMING_RETRACT,  // final retract from the reference point
  HOMING_PARK,     // move to the park position
};

class TMC22XX : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // ---- configuration (codegen) ----
  void set_driver_type(DriverType type) { this->driver_type_ = type; }
  void set_step_pin(InternalGPIOPin *pin) { this->step_pin_ = pin; }
  void set_dir_pin(InternalGPIOPin *pin) { this->dir_pin_ = pin; }
  void set_enable_pin(GPIOPin *pin) { this->enable_pin_ = pin; }
  void set_ms1_pin(GPIOPin *pin) { this->ms1_pin_ = pin; }
  void set_ms2_pin(GPIOPin *pin) { this->ms2_pin_ = pin; }
  void set_steps_per_unit(float steps_per_unit) { this->steps_per_unit_ = steps_per_unit; }
  void set_min_position(float position) {
    this->min_position_ = position;
    this->has_min_position_ = true;
  }
  void set_max_position(float position) {
    this->max_position_ = position;
    this->has_max_position_ = true;
  }
  void set_invert_direction(bool invert) { this->invert_direction_ = invert; }
  void set_interpolation(bool interpolation) { this->interpolation_ = interpolation; }
  void set_enable_on_boot(bool enable) { this->enable_on_boot_ = enable; }
  void set_idle_disable_delay(uint32_t ms) { this->idle_disable_ms_ = ms; }
  void set_report_interval(uint32_t ms) { this->report_interval_ms_ = ms; }
#ifdef USE_TMC22XX_UART
  void set_uart(uart::UARTComponent *parent, uint8_t address) {
    this->uart_.set_parent(parent);
    this->uart_.set_address(address);
    this->uart_mode_ = true;
  }
  void set_rsense(float ohm) { this->rsense_ = ohm; }
  void set_standstill_delay(uint32_t ms) { this->standstill_delay_ms_ = ms; }
  void set_spreadcycle(bool spreadcycle) { this->spreadcycle_ = spreadcycle; }
  void set_poll_interval(uint32_t ms) { this->poll_interval_ms_ = ms; }
#endif
#ifdef USE_TMC22XX_STALLGUARD
  void set_diag_pin(InternalGPIOPin *pin) { this->diag_pin_ = pin; }
  void set_stop_on_stall(bool stop) { this->stop_on_stall_ = stop; }
#endif
#ifdef USE_TMC22XX_HOMING
  void set_homing_source(HomingSource source) {
    this->homing_source_ = source;
    this->has_homing_ = true;
  }
  void set_homing_endstop_pin(InternalGPIOPin *pin) { this->endstop_pin_ = pin; }
  void set_homing_positive(bool positive) { this->homing_positive_ = positive; }
  void set_homing_speed(float speed) { this->homing_speed_ = speed; }
  void set_homing_slow_speed(float speed) { this->homing_slow_speed_ = speed; }
  void set_homing_backoff(float distance) { this->homing_backoff_ = distance; }
  void set_homing_max_travel(float distance) { this->homing_max_travel_ = distance; }
  void set_homing_timeout(uint32_t ms) { this->homing_timeout_ms_ = ms; }
  void set_homing_debounce(uint32_t us) { this->homing_debounce_us_ = us; }
  void set_homing_position(float position) { this->homing_position_ = position; }
  void set_homing_park_position(float position) {
    this->homing_park_position_ = position;
    this->has_park_position_ = true;
  }
  void set_home_on_boot(bool home) { this->home_on_boot_ = home; }
#endif

  // ---- configuration + runtime (codegen and actions) ----
  /// Speeds in units/s, accelerations in units/s^2 (unit = step unless steps_per_unit is set).
  void set_max_speed(float speed);
  void set_acceleration(float acceleration);
  void set_deceleration(float deceleration);
  void set_microsteps(uint16_t microsteps);
  // The following need UART register access; without it they only log a warning.
  void set_run_current(float milliamps);
  void set_hold_current(float milliamps);
  /// StealthChop below, SpreadCycle above this speed (units/s). 0 disables the switch-over.
  void set_stealthchop_threshold(float speed);
  void set_stallguard_threshold(uint8_t threshold);

  // ---- motion API ----
  void move_to(float position);
  void move_by(float distance);
  void run(bool forward);
  void stop();
  void emergency_stop();
  void set_position(float position);
  void enable();
  void disable();
  void home();

  float get_position() { return static_cast<float>(this->gen_.get_position()) / this->steps_per_unit_; }
  float get_target() { return static_cast<float>(this->gen_.get_target()) / this->steps_per_unit_; }
  int32_t get_position_steps() { return this->gen_.get_position(); }
  bool is_moving() const { return this->moving_ || this->gen_.is_running(); }
  bool is_homed() const { return this->homed_; }
  bool is_homing() const { return this->homing_state_ != HOMING_IDLE; }
  bool is_enabled() const { return this->enabled_; }
  uint32_t get_driver_errors() const { return this->errors_; }

  // ---- automation hooks ----
  template<typename F> void add_on_move_complete_callback(F &&cb) { this->move_complete_cb_.add(std::forward<F>(cb)); }
  template<typename F> void add_on_homing_complete_callback(F &&cb) {
    this->homing_complete_cb_.add(std::forward<F>(cb));
  }
  template<typename F> void add_on_homing_failed_callback(F &&cb) { this->homing_failed_cb_.add(std::forward<F>(cb)); }
  template<typename F> void add_on_stall_callback(F &&cb) { this->stall_cb_.add(std::forward<F>(cb)); }
  template<typename F> void add_on_driver_error_callback(F &&cb) { this->driver_error_cb_.add(std::forward<F>(cb)); }

  // ---- optional entities ----
#ifdef USE_TMC22XX_SENSOR
  void set_position_sensor(sensor::Sensor *sensor) { this->position_sensor_ = sensor; }
  void set_stallguard_result_sensor(sensor::Sensor *sensor) { this->stallguard_result_sensor_ = sensor; }
  void set_actual_current_sensor(sensor::Sensor *sensor) { this->actual_current_sensor_ = sensor; }
#endif
#ifdef USE_TMC22XX_BINARY_SENSOR
  void set_moving_binary_sensor(binary_sensor::BinarySensor *sensor) { this->moving_binary_sensor_ = sensor; }
  void set_homed_binary_sensor(binary_sensor::BinarySensor *sensor) { this->homed_binary_sensor_ = sensor; }
  void set_homing_binary_sensor(binary_sensor::BinarySensor *sensor) { this->homing_binary_sensor_ = sensor; }
  void set_driver_error_binary_sensor(binary_sensor::BinarySensor *sensor) {
    this->driver_error_binary_sensor_ = sensor;
  }
  void set_overtemperature_warning_binary_sensor(binary_sensor::BinarySensor *sensor) {
    this->overtemperature_warning_binary_sensor_ = sensor;
  }
  void set_stall_binary_sensor(binary_sensor::BinarySensor *sensor) { this->stall_binary_sensor_ = sensor; }
#endif
#ifdef USE_TMC22XX_TEXT_SENSOR
  void set_driver_status_text_sensor(text_sensor::TextSensor *sensor) { this->driver_status_text_sensor_ = sensor; }
#endif
#ifdef USE_TMC22XX_NUMBER
  void set_target_number(number::Number *number) { this->target_number_ = number; }
#endif

 protected:
  int32_t to_steps_(float units) const;
  void update_profile_();
  void start_steps_(int32_t target);
  void on_motion_end_();
  void publish_position_();
  void publish_flags_();
  void apply_ms_pins_();

#ifdef USE_TMC22XX_UART
  bool configure_driver_();
  void poll_driver_();
  void comm_failed_();
  void update_status_(uint32_t errors, uint32_t drv_status);
  void compute_current_scale_();
  uint32_t gconf_value_(bool force_stealthchop) const;
  uint32_t chopconf_value_() const;
  uint32_t ihold_irun_value_() const;
  uint32_t tpwmthrs_value_() const;
  bool write_(uint8_t reg, uint32_t value);
#endif
#ifdef USE_TMC22XX_STALLGUARD
  void stall_prepare_();
  void stall_event_();
#endif
#ifdef USE_TMC22XX_HOMING
  void homing_start_phase_(HomingState state);
  void homing_on_motion_end_();
  void homing_after_reference_();
  void homing_finish_();
  void homing_fail_(const LogString *reason);
  void homing_restore_();
#endif

  StepGenerator gen_;
  InternalGPIOPin *step_pin_{nullptr};
  InternalGPIOPin *dir_pin_{nullptr};
  GPIOPin *enable_pin_{nullptr};
  GPIOPin *ms1_pin_{nullptr};
  GPIOPin *ms2_pin_{nullptr};

  float steps_per_unit_{1.0f};
  float max_speed_{1000.0f};
  float acceleration_{1000.0f};
  float deceleration_{1000.0f};
  float min_position_{0.0f};
  float max_position_{0.0f};
  uint32_t idle_disable_ms_{0};
  uint32_t report_interval_ms_{500};
  uint32_t last_report_ms_{0};
  uint32_t last_stop_ms_{0};
  uint32_t errors_{0};
  uint16_t microsteps_{8};
  DriverType driver_type_{DRIVER_TMC2208};
  HomingState homing_state_{HOMING_IDLE};
  bool has_min_position_{false};
  bool has_max_position_{false};
  bool invert_direction_{false};
  bool interpolation_{true};
  bool enable_on_boot_{true};
  bool uart_mode_{false};
  bool ready_{false};
  bool enabled_{false};
  bool moving_{false};
  bool idle_pending_{false};
  bool homed_{false};
  bool last_moving_published_{false};

#ifdef USE_TMC22XX_UART
  TMCUart uart_;
  float rsense_{0.11f};
  float run_current_ma_{500.0f};
  float hold_current_ma_{250.0f};
  float stealthchop_threshold_{0.0f};
  uint32_t standstill_delay_ms_{2000};
  uint32_t poll_interval_ms_{2000};
  uint32_t last_poll_ms_{0};
  uint32_t last_status_{0xFFFFFFFF};
  uint8_t irun_{16};
  uint8_t ihold_{8};
  bool vsense_{false};
  bool spreadcycle_{false};
  bool driver_configured_{false};
#endif
#ifdef USE_TMC22XX_STALLGUARD
  InternalGPIOPin *diag_pin_{nullptr};
  uint8_t stallguard_threshold_{0};
  bool stop_on_stall_{false};
  bool stall_reported_{false};
#endif
#ifdef USE_TMC22XX_HOMING
  InternalGPIOPin *endstop_pin_{nullptr};
  float homing_speed_{200.0f};
  float homing_slow_speed_{0.0f};
  float homing_backoff_{0.0f};
  float homing_max_travel_{0.0f};
  float homing_position_{0.0f};
  float homing_park_position_{0.0f};
  uint32_t homing_timeout_ms_{60000};
  uint32_t homing_debounce_us_{1000};
  uint32_t homing_started_ms_{0};
  HomingSource homing_source_{HOMING_SOURCE_ENDSTOP};
  bool has_homing_{false};
  bool has_park_position_{false};
  bool homing_positive_{false};
  bool home_on_boot_{false};
#endif

  LazyCallbackManager<void()> move_complete_cb_;
  LazyCallbackManager<void()> homing_complete_cb_;
  LazyCallbackManager<void()> homing_failed_cb_;
  LazyCallbackManager<void()> stall_cb_;
  LazyCallbackManager<void(uint32_t)> driver_error_cb_;

#ifdef USE_TMC22XX_SENSOR
  sensor::Sensor *position_sensor_{nullptr};
  sensor::Sensor *stallguard_result_sensor_{nullptr};
  sensor::Sensor *actual_current_sensor_{nullptr};
#endif
#ifdef USE_TMC22XX_BINARY_SENSOR
  binary_sensor::BinarySensor *moving_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *homed_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *homing_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *driver_error_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *overtemperature_warning_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *stall_binary_sensor_{nullptr};
#endif
#ifdef USE_TMC22XX_TEXT_SENSOR
  text_sensor::TextSensor *driver_status_text_sensor_{nullptr};
#endif
#ifdef USE_TMC22XX_NUMBER
  number::Number *target_number_{nullptr};
#endif
};

}  // namespace esphome::tmc22xx

#endif  // USE_ESP32
