// --------------------------------------------------------------------------------------
// Minimal Arduino "TwoWire"/Wire API compatibility shim, backed by pico-sdk's
// hardware_i2c driver. Implements only the subset of the API this project uses:
// begin(), setClock(), beginTransmission(), write(), endTransmission(),
// requestFrom(), available(), read().
//
// endTransmission() return codes intentionally mirror the Arduino convention
// (0 = success, 2 = NACK on address, 3 = NACK on data, 4 = other error), since
// Peripheral::write_to_register()/read_from_register() branch on them.
// --------------------------------------------------------------------------------------
#pragma once

#include <cstdint>
#include <cstddef>

#include "hardware/i2c.h"

class TwoWire {
  public:
    TwoWire(i2c_inst_t* instance, uint sda_pin, uint scl_pin);

    void setClock(uint32_t freq_hz);
    void begin();

    void beginTransmission(uint8_t address);
    size_t write(uint8_t data);
    uint8_t endTransmission(bool stop_bit = true);

    uint8_t requestFrom(uint8_t address, uint8_t quantity, bool stop_bit = true);
    int available() const;
    int read();

  private:
    i2c_inst_t* i2c_;
    uint sda_pin_;
    uint scl_pin_;
    uint32_t clock_hz_ = 100000;
    bool began_ = false;

    static constexpr size_t kBufferSize = 32;

    uint8_t tx_address_ = 0;
    uint8_t tx_buffer_[kBufferSize];
    size_t tx_len_ = 0;

    uint8_t rx_buffer_[kBufferSize];
    size_t rx_len_ = 0;
    size_t rx_pos_ = 0;
};
