#pragma once

#include <cstdint>
#include "esphome/core/gpio.h"

namespace esphome::tmc22xx {

/// Common interface of the two ways to move the motor:
///  - StepGenerator: STEP/DIR pulses from a hardware timer ISR (exact positioning)
///  - UartMotion:    velocity commands over UART (VACTUAL), position fed back from the INDEX pin
/// Positions and speeds are in (micro)steps.
class MotionBackend {
 public:
  virtual void set_profile(float max_speed, float acceleration, float deceleration) = 0;
  virtual void move_to(int32_t target) = 0;
  virtual void run(bool forward) = 0;
  /// Decelerate to standstill using the configured ramp.
  virtual void stop() = 0;
  /// Stop immediately (no ramp).
  virtual void emergency_stop() = 0;
  virtual void set_position(int32_t position) = 0;
  virtual int32_t get_position() = 0;
  virtual int32_t get_target() = 0;
  virtual bool is_running() const = 0;
  virtual bool is_cruising() const = 0;

  /// Arms an input (end stop, DIAG) that ends the motion once it reads active for debounce_us.
  virtual void arm_trigger(InternalGPIOPin *pin, uint32_t debounce_us, bool cruise_only) = 0;
  virtual void disarm_trigger() = 0;
  virtual bool trigger_fired() const = 0;
  /// Signed distance travelled past the trigger point before the motor stood still.
  virtual int32_t trigger_overshoot() const { return 0; }

  /// Called from the component loop.
  virtual void loop() {}
  /// True if the backend could not stop the motor (driver unreachable).
  virtual bool stop_failed() const { return false; }
};

}  // namespace esphome::tmc22xx
