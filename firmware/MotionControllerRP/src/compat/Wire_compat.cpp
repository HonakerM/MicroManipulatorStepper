#include "Wire_compat.h"

#include <algorithm>

#include "hardware/gpio.h"

TwoWire::TwoWire(i2c_inst_t* instance, uint sda_pin, uint scl_pin)
    : i2c_(instance), sda_pin_(sda_pin), scl_pin_(scl_pin) {}

void TwoWire::setClock(uint32_t freq_hz) {
  clock_hz_ = freq_hz;
  if (began_) {
    i2c_set_baudrate(i2c_, clock_hz_);
  }
}

void TwoWire::begin() {
  if (began_) return;
  i2c_init(i2c_, clock_hz_);
  gpio_set_function(sda_pin_, GPIO_FUNC_I2C);
  gpio_set_function(scl_pin_, GPIO_FUNC_I2C);
  gpio_pull_up(sda_pin_);
  gpio_pull_up(scl_pin_);
  began_ = true;
}

void TwoWire::beginTransmission(uint8_t address) {
  tx_address_ = address;
  tx_len_ = 0;
}

size_t TwoWire::write(uint8_t data) {
  if (tx_len_ >= kBufferSize) return 0;
  tx_buffer_[tx_len_++] = data;
  return 1;
}

uint8_t TwoWire::endTransmission(bool stop_bit) {
  size_t requested_len = tx_len_;
  int ret = i2c_write_blocking(i2c_, tx_address_, tx_buffer_, tx_len_, !stop_bit);
  tx_len_ = 0;
  if (ret == PICO_ERROR_GENERIC) return 2;  // no device responded (NACK on address)
  if (ret == PICO_ERROR_TIMEOUT) return 5;  // timeout
  if (ret >= 0 && (size_t)ret != requested_len) return 3;  // NACK partway through data
  return 0;  // success
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, bool stop_bit) {
  quantity = (uint8_t)std::min<size_t>(quantity, kBufferSize);
  int ret = i2c_read_blocking(i2c_, address, rx_buffer_, quantity, !stop_bit);
  rx_pos_ = 0;
  rx_len_ = (ret < 0) ? 0 : (size_t)ret;
  return (uint8_t)rx_len_;
}

int TwoWire::available() const {
  return (int)(rx_len_ - rx_pos_);
}

int TwoWire::read() {
  if (rx_pos_ >= rx_len_) return -1;
  return rx_buffer_[rx_pos_++];
}
