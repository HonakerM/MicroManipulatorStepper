// --------------------------------------------------------------------------------------
// Minimal Arduino "Serial" API compatibility shim, backed by pico-sdk stdio (USB CDC).
//
// This exists so the application code (originally written against arduino-pico's
// Serial object) did not need to be rewritten line-by-line when porting from the
// Arduino framework to plain pico-sdk. It implements only the subset of the API that
// this project actually uses.
//
// stdio itself is brought up exactly once, in main(), via stdio_init_all(). begin()
// here is a no-op kept only for source compatibility (baud rate is meaningless over
// USB CDC).
// --------------------------------------------------------------------------------------
#pragma once

#include <cstdint>
#include <cstddef>

// Arduino print()/println() base constants used by this project.
#ifndef BIN
#define BIN 2
#endif
#ifndef DEC
#define DEC 10
#endif
#ifndef HEX
#define HEX 16
#endif

class SerialClass {
  public:
    // No-op: stdio is brought up once in main() via stdio_init_all().
    // Kept for source compatibility with the previous Arduino Serial API.
    void begin(unsigned long baudrate = 115200);

    // True once a USB CDC terminal is connected.
    explicit operator bool() const;

    // Returns 1 if a byte is available to read, 0 otherwise. Non-blocking.
    int available();

    // Returns the next byte, or -1 if none is available. Non-blocking.
    int read();

    size_t write(uint8_t c);
    size_t write(const char* str);
    size_t write(const uint8_t* buf, size_t size);

    void print(const char* str);
    void print(uint8_t value, int base);

    void println(const char* str = "");
    void println(uint8_t value, int base);

    int printf(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
};

extern SerialClass Serial;
