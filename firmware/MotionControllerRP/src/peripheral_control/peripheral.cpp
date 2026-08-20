#include <hw_config.h>
#include "peripheral.h"
#include <utilities/logging.h>
#include "compat/Serial_compat.h"
#include "compat/Arduino_compat.h"

namespace {
const char* i2c_status_to_string(uint8_t status) {
  switch (status) {
    case 0: return "success";
    case 1: return "data_too_long";
    case 2: return "nack_on_address";
    case 3: return "nack_on_data";
    case 4: return "other_error";
    case 5: return "timeout";
    default: return "unknown";
  }
}
}

Peripheral::Peripheral(TwoWire* wire_ptr) {
  // initialize the two wire system
  wire = wire_ptr;
  // ensure the clock is low enough
  wire->setClock(PERIPHERAL_I2C_CLOCK);
}

void Peripheral::begin(bool wait) {
  wire->begin();

  // if wait is enable then wait for the status register to return any non 0 value
  while (wait){
    if(is_ready()){
      break;
    }
    delay(100);
  }
}

bool Peripheral::is_ready() {
  uint8_t buffer = 0;
  read_from_register(STATUS_REG, 1, &buffer);
  Serial.print("Status reg: ");
  Serial.println(buffer, BIN);
  return buffer!=0;
}


void Peripheral::set_rot(RotDegree deg) {
  FourByteArray trans_val = deg.get_trans_value();
  write_to_register(ROT_TARGET_00_REG, 1, &trans_val.bytes[0]);
  write_to_register(ROT_TARGET_01_REG, 1, &trans_val.bytes[1]);
  write_to_register(ROT_TARGET_02_REG, 1, &trans_val.bytes[2]);
  write_to_register(ROT_TARGET_03_REG, 1, &trans_val.bytes[3]);
  execute(true, false, false);
}

RotDegree Peripheral::get_desired_rot() {
  FourByteArray trans_val = FourByteArray{0};
  read_from_register(ROT_TARGET_00_REG, 1, &trans_val.bytes[0]);
  read_from_register(ROT_TARGET_01_REG, 1, &trans_val.bytes[1]);
  read_from_register(ROT_TARGET_02_REG, 1, &trans_val.bytes[2]);
  read_from_register(ROT_TARGET_03_REG, 1, &trans_val.bytes[3]);

  return RotDegree(trans_val.integer);
}

RotDegree Peripheral::get_cur_rot() {
  FourByteArray trans_val = FourByteArray{0};
  read_from_register(ROT_POS_00_REG, 1, &trans_val.bytes[0]);
  read_from_register(ROT_POS_01_REG, 1, &trans_val.bytes[1]);
  read_from_register(ROT_POS_02_REG, 1, &trans_val.bytes[2]);
  read_from_register(ROT_POS_03_REG, 1, &trans_val.bytes[3]);

  return RotDegree(trans_val.integer);
}

void Peripheral::set_temp(TempDegree deg) {
  FourByteArray trans_val = deg.get_trans_value();
  write_to_register(TEMP_TARGET_00_REG, 1, &trans_val.bytes[0]);
  write_to_register(TEMP_TARGET_01_REG, 1, &trans_val.bytes[1]);
  write_to_register(TEMP_TARGET_02_REG, 1, &trans_val.bytes[2]);
  write_to_register(TEMP_TARGET_03_REG, 1, &trans_val.bytes[3]);
  execute(false, true, false);
}

TempDegree Peripheral::get_desired_temp() {
  FourByteArray trans_val = FourByteArray{0};
  read_from_register(TEMP_TARGET_00_REG, 1, &trans_val.bytes[0]);
  read_from_register(TEMP_TARGET_01_REG, 1, &trans_val.bytes[1]);
  read_from_register(TEMP_TARGET_02_REG, 1, &trans_val.bytes[2]);
  read_from_register(TEMP_TARGET_03_REG, 1, &trans_val.bytes[3]);

  return TempDegree(trans_val.integer);
}

TempDegree Peripheral::get_cur_temp() {
  FourByteArray trans_val = FourByteArray{0};
  read_from_register(TEMP_POS_00_REG, 1, &trans_val.bytes[0]);
  read_from_register(TEMP_POS_01_REG, 1, &trans_val.bytes[1]);
  read_from_register(TEMP_POS_02_REG, 1, &trans_val.bytes[2]);
  read_from_register(TEMP_POS_03_REG, 1, &trans_val.bytes[3]);

  return TempDegree(trans_val.integer);
}


void Peripheral::set_vac(bool status) {
  uint8_t val = status ? 100 : 0;
  write_to_register(VAC_REG, 1, &val);
  execute(false, false, true);
}

bool Peripheral::get_vac() {
  uint8_t status;
  read_from_register(VAC_REG, 1,&status);
  if (status > 0) {
    return true;
  }
  return false;
}

bool Peripheral::home() {
  write_to_register(HOME_REG, 0, NULL);
  return true;
}
void Peripheral::estop() {
  write_to_register(ESTOP_REG, 0, NULL);
}

void Peripheral::execute(bool rot, bool temp, bool vac) {
  uint8_t base = 0x00;
  if(rot){
    base|=(1<<0);
  }
  if(temp){
    base|=(1<<1);
  }
  if(vac){
    base|=(1<<2);
  }
  write_to_register(EXEC_REG, 1, &base);
}


// TwoWire Helpers
void Peripheral::write_to_register(uint8_t reg, size_t size, uint8_t* buffer){
    const uint8_t max_attempts = 3;
    uint8_t last_tx_status = 0;
    for (uint8_t attempt = 0; attempt < max_attempts; attempt++) {
      wire->beginTransmission(PERIPHERAL_I2C_ADDRESS);
      wire->write(reg);
      for(size_t i=0;i<size;i++){
        wire->write(buffer[i]);
      }

      uint8_t tx_status = wire->endTransmission(true);
      last_tx_status = tx_status;
      if (tx_status == 0) {
        return;
      }

      delay(2);
    }

    LOG_DEBUG("I2C write failed. Reg: 0x%02X, tx_status: %u (%s)\n",
              (unsigned int)reg,
              (unsigned int)last_tx_status,
              i2c_status_to_string(last_tx_status));
}

void Peripheral::read_from_register(uint8_t reg, size_t size, uint8_t* buffer){
    const uint8_t max_attempts = 3;
    uint8_t last_tx_status = 0;
    uint8_t expected_size = (uint8_t)size;
    uint8_t last_received_size = 0;
    const char* last_failure_stage = "none";
    for (uint8_t attempt = 0; attempt < max_attempts; attempt++) {
      wire->beginTransmission(PERIPHERAL_I2C_ADDRESS);
      wire->write(reg);

      uint8_t tx_status = wire->endTransmission(false);
      last_tx_status = tx_status;
      if (tx_status != 0) {
        last_failure_stage = "register_write";
        delay(2);
        continue;
      }

      uint8_t received_size = wire->requestFrom(PERIPHERAL_I2C_ADDRESS, expected_size);
      last_received_size = received_size;
      if (received_size != expected_size) {
        last_failure_stage = "size_mismatch";
        delay(2);
        continue;
      }

      bool short_read = false;
      for(size_t i=0;i<size;i++){
        if (wire->available()) {
          buffer[i] = wire->read();
        } else {
          short_read = true;
          last_failure_stage = "rx_underflow";
          break;
        }
      }

      if (short_read) {
        delay(2);
        continue;
      }

      return;
    }

    for(size_t i=0;i<size;i++){
      buffer[i] = 0;
    }

    LOG_DEBUG("I2C read failed. Reg: 0x%02X, stage: %s, tx_status: %u (%s), expected: %u, received: %u\n",
              (unsigned int)reg,
              last_failure_stage,
              (unsigned int)last_tx_status,
              i2c_status_to_string(last_tx_status),
              (unsigned int)expected_size,
              (unsigned int)last_received_size);
  }
