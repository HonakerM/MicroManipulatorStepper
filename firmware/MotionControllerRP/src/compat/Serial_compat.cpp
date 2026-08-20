#include "Serial_compat.h"

#include <cstdio>
#include <cstring>
#include <cstdarg>

#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

SerialClass Serial;

namespace {
// getchar_timeout_us(0) consumes the character if one is available, so a
// single-character pushback buffer is needed to support the Arduino pattern
// of calling available() and read() as two separate calls.
int g_pushback = -1;
}  // namespace

void SerialClass::begin(unsigned long /*baudrate*/) {
  // Intentionally empty. See header comment.
}

SerialClass::operator bool() const {
  return stdio_usb_connected();
}

int SerialClass::available() {
  if (g_pushback != -1) return 1;
  int c = getchar_timeout_us(0);
  if (c == PICO_ERROR_TIMEOUT) return 0;
  g_pushback = c;
  return 1;
}

int SerialClass::read() {
  if (g_pushback != -1) {
    int c = g_pushback;
    g_pushback = -1;
    return c;
  }
  int c = getchar_timeout_us(0);
  return (c == PICO_ERROR_TIMEOUT) ? -1 : c;
}

size_t SerialClass::write(uint8_t c) {
  putchar_raw((int)c);
  return 1;
}

size_t SerialClass::write(const char* str) {
  if (!str) return 0;
  size_t len = strlen(str);
  return write((const uint8_t*)str, len);
}

size_t SerialClass::write(const uint8_t* buf, size_t size) {
  for (size_t i = 0; i < size; i++) putchar_raw(buf[i]);
  return size;
}

namespace {
void print_uint8_with_base(uint8_t value, int base) {
  if (base == BIN) {
    char buf[9];
    for (int i = 7; i >= 0; i--) {
      buf[7 - i] = ((value >> i) & 0x01) ? '1' : '0';
    }
    buf[8] = '\0';
    fputs(buf, stdout);
  } else if (base == HEX) {
    printf("%02X", (unsigned)value);
  } else {
    // DEC and anything else
    printf("%u", (unsigned)value);
  }
}
}  // namespace

void SerialClass::print(const char* str) {
  fputs(str, stdout);
}

void SerialClass::print(uint8_t value, int base) {
  print_uint8_with_base(value, base);
}

void SerialClass::println(const char* str) {
  fputs(str, stdout);
  fputc('\n', stdout);
}

void SerialClass::println(uint8_t value, int base) {
  print_uint8_with_base(value, base);
  fputc('\n', stdout);
}

int SerialClass::printf(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  int ret = vprintf(fmt, args);
  va_end(args);
  return ret;
}
