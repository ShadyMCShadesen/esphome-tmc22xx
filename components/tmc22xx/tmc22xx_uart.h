#pragma once

#include "esphome/core/defines.h"

#ifdef USE_TMC22XX_UART

#include <cstddef>
#include <cstdint>
#include "esphome/components/uart/uart_component.h"

namespace esphome::tmc22xx {

enum class UartError : uint8_t { NONE = 0, TIMEOUT, CRC, NOT_ACKNOWLEDGED };

/// Minimal single-wire UART master for TMC22xx drivers.
/// All calls are synchronous but strictly bounded (a failed read costs a few ms).
class TMCUart {
 public:
  void set_parent(uart::UARTComponent *parent) { this->parent_ = parent; }
  void set_address(uint8_t address) { this->address_ = address; }
  uint8_t get_address() const { return this->address_; }

  /// Read a register. Retries on timeout / CRC error.
  bool read(uint8_t reg, uint32_t *value);
  /// Write a register and verify via IFCNT that the driver accepted it. Retries on failure.
  bool write(uint8_t reg, uint32_t value);
  /// Send a write datagram without reading IFCNT back (about 1 ms instead of 3 ms).
  /// Only for values that are refreshed continuously, e.g. VACTUAL during a ramp.
  void write_unverified(uint8_t reg, uint32_t value);

  UartError last_error() const { return this->last_error_; }
  uint16_t crc_error_count() const { return this->crc_errors_; }
  uint16_t timeout_count() const { return this->timeouts_; }
  /// Forget the cached IFCNT value (e.g. after a driver reset).
  void invalidate() { this->ifcnt_valid_ = false; }

  static uint8_t crc8(const uint8_t *data, size_t len);

 protected:
  bool read_once_(uint8_t reg, uint32_t *value);
  void send_(const uint8_t *data, size_t len);
  void drain_rx_();
  uint32_t bit_time_us_() const;

  uart::UARTComponent *parent_{nullptr};
  uint32_t last_tx_us_{0};
  uint16_t crc_errors_{0};
  uint16_t timeouts_{0};
  uint8_t address_{0};
  uint8_t ifcnt_{0};
  bool ifcnt_valid_{false};
  UartError last_error_{UartError::NONE};
};

}  // namespace esphome::tmc22xx

#endif  // USE_TMC22XX_UART
