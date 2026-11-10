#pragma once

#include "esphome/core/defines.h"

#ifdef USE_TMC22XX_BUTTON

#include "esphome/components/button/button.h"
#include "esphome/core/helpers.h"
#include "tmc22xx.h"

namespace esphome::tmc22xx {

enum ButtonFunction : uint8_t { BUTTON_HOME = 0, BUTTON_STOP = 1, BUTTON_EMERGENCY_STOP = 2 };

class TMC22XXButton : public button::Button, public Parented<TMC22XX> {
 public:
  void set_function(ButtonFunction function) { this->function_ = function; }

 protected:
  void press_action() override {
    switch (this->function_) {
      case BUTTON_HOME:
        this->parent_->home();
        break;
      case BUTTON_STOP:
        this->parent_->stop();
        break;
      case BUTTON_EMERGENCY_STOP:
        this->parent_->emergency_stop();
        break;
    }
  }

  ButtonFunction function_{BUTTON_STOP};
};

}  // namespace esphome::tmc22xx

#endif  // USE_TMC22XX_BUTTON
