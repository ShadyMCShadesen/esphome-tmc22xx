#pragma once

#include "esphome/core/defines.h"

#ifdef USE_TMC22XX_NUMBER

#include "esphome/components/number/number.h"
#include "esphome/core/helpers.h"
#include "tmc22xx.h"

namespace esphome::tmc22xx {

/// Target position in user units; setting the number starts a move.
class TMC22XXTargetNumber : public number::Number, public Parented<TMC22XX> {
 protected:
  void control(float value) override { this->parent_->move_to(value); }
};

}  // namespace esphome::tmc22xx

#endif  // USE_TMC22XX_NUMBER
