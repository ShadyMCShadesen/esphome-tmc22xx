#include "tmc22xx_uart.h"

#ifdef USE_TMC22XX_UART

#include "esphome/core/hal.h"
#include "tmc22xx_registers.h"

namespace esphome::tmc22xx {

static const uint8_t MAX_ATTEMPTS = 3;
static const uint8_t BUS_IDLE_BITS = 16;  // idle time enforced between two datagrams

// CRC8-ATM, polynomial x^8 + x^2 + x + 1 (0x07), init 0, each byte processed LSB first.
uint8_t TMCUart::crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    uint8_t byte = data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (((crc >> 7) ^ (byte & 0x01)) != 0) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc <<= 1;
      }
      byte >>= 1;
    }
  }
  return crc;
}

uint32_t TMCUart::bit_time_us_() const {
  const uint32_t baud = this->parent_->get_baud_rate();
  return baud == 0 ? 9 : (1000000UL / baud) + 1;
}

void TMCUart::drain_rx_() {
  // Discards stale bytes, including the echo of our own transmissions on the shared wire.
  uint8_t dummy;
  for (uint8_t guard = 0; guard < 64 && this->parent_->available() > 0; guard++) {
    this->parent_->read_byte(&dummy);
  }
}

void TMCUart::send_(const uint8_t *data, size_t len) {
  const uint32_t idle = BUS_IDLE_BITS * this->bit_time_us_();
  const uint32_t since = micros() - this->last_tx_us_;
  if (since < idle)
    delayMicroseconds(idle - since);
  this->drain_rx_();
  this->parent_->write_array(data, len);
  this->parent_->flush();
  this->last_tx_us_ = micros();
}

bool TMCUart::read_once_(uint8_t reg, uint32_t *value) {
  uint8_t request[4] = {TMC_SYNC, this->address_, static_cast<uint8_t>(reg & 0x7F), 0};
  request[3] = crc8(request, 3);
  this->send_(request, sizeof(request));

  // Worst case in the RX buffer: echo of a preceding write (8) + echo of this request (4) + reply (8).
  uint8_t buf[24];
  size_t n = 0;
  bool header_seen = false;
  // Reply starts SENDDELAY (default 8 bit times) after the request; add slack for the RX FIFO timeout.
  const uint32_t timeout_us = 2000 + 300 * this->bit_time_us_();
  const uint32_t start = micros();
  while (micros() - start < timeout_us) {
    size_t avail = this->parent_->available();
    while (avail > 0 && n < sizeof(buf)) {
      this->parent_->read_byte(&buf[n++]);
      avail--;
    }
    // The reply is located by its header, so wiring with or without TX echo both work.
    for (size_t i = 0; i + 8 <= n; i++) {
      if (buf[i] != TMC_SYNC || buf[i + 1] != TMC_MASTER_ADDR || buf[i + 2] != request[2])
        continue;
      header_seen = true;
      if (crc8(&buf[i], 7) != buf[i + 7])
        continue;
      *value = (static_cast<uint32_t>(buf[i + 3]) << 24) | (static_cast<uint32_t>(buf[i + 4]) << 16) |
               (static_cast<uint32_t>(buf[i + 5]) << 8) | static_cast<uint32_t>(buf[i + 6]);
      this->last_tx_us_ = micros();
      this->last_error_ = UartError::NONE;
      return true;
    }
    if (n == sizeof(buf))
      break;
    delayMicroseconds(100);
  }
  this->last_tx_us_ = micros();
  if (header_seen) {
    this->last_error_ = UartError::CRC;
    this->crc_errors_++;
  } else {
    this->last_error_ = UartError::TIMEOUT;
    this->timeouts_++;
  }
  return false;
}

bool TMCUart::read(uint8_t reg, uint32_t *value) {
  for (uint8_t attempt = 0; attempt < MAX_ATTEMPTS; attempt++) {
    if (this->read_once_(reg, value))
      return true;
  }
  return false;
}

bool TMCUart::write(uint8_t reg, uint32_t value) {
  uint8_t datagram[8] = {TMC_SYNC,
                         this->address_,
                         static_cast<uint8_t>(reg | TMC_WRITE_FLAG),
                         static_cast<uint8_t>(value >> 24),
                         static_cast<uint8_t>(value >> 16),
                         static_cast<uint8_t>(value >> 8),
                         static_cast<uint8_t>(value),
                         0};
  datagram[7] = crc8(datagram, 7);

  for (uint8_t attempt = 0; attempt < MAX_ATTEMPTS; attempt++) {
    uint32_t counter;
    if (!this->ifcnt_valid_) {
      if (!this->read_once_(REG_IFCNT, &counter))
        continue;
      this->ifcnt_ = static_cast<uint8_t>(counter);
      this->ifcnt_valid_ = true;
    }
    this->send_(datagram, sizeof(datagram));
    if (!this->read_once_(REG_IFCNT, &counter)) {
      this->ifcnt_valid_ = false;
      continue;
    }
    // IFCNT is an 8 bit counter incremented by the driver for every write with a valid CRC.
    const bool accepted = static_cast<uint8_t>(counter) == static_cast<uint8_t>(this->ifcnt_ + 1);
    this->ifcnt_ = static_cast<uint8_t>(counter);
    if (accepted) {
      this->last_error_ = UartError::NONE;
      return true;
    }
    this->last_error_ = UartError::NOT_ACKNOWLEDGED;
  }
  return false;
}

}  // namespace esphome::tmc22xx

#endif  // USE_TMC22XX_UART
