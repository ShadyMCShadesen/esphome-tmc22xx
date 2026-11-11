#pragma once
#include <cstdint>
namespace esphome {
namespace gpio {
enum InterruptType { INTERRUPT_ANY_EDGE };
}
class ISRInternalGPIOPin {
 public:
  bool digital_read();
};
class InternalGPIOPin {
 public:
  template<typename T> void attach_interrupt(void (*func)(T *), T *arg, gpio::InterruptType type) {
    this->func_ = reinterpret_cast<void (*)(void *)>(func);
    this->arg_ = arg;
  }
  ISRInternalGPIOPin to_isr() { return {}; }
  void fire() { this->func_(this->arg_); }

 protected:
  void (*func_)(void *){nullptr};
  void *arg_{nullptr};
};
}  // namespace esphome
