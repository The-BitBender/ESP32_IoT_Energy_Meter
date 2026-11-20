#pragma once
#include <Arduino.h>
#include <stddef.h>
#include "driver/spi_master.h"
#include "M90E26_regs.h"

/**
 * M90E26 SPI transport driver for ESP32 (Arduino framework).
 *
 * - Uses the ESP-IDF spi_master driver (available inside the Arduino core),
 *   NOT the Arduino SPIClass, so transfers are queued and interrupt-driven.
 *   Do not call SPI.begin() on the same host.
 * - SPI mode 3 (CPOL=1, CPHA=1), MSB first. Datasheet timing limits a 50 % duty
 *   clock to about 136 kHz (tCLH >= 3.67 us), so the default is 100 kHz.
 * - Frame: [R/W|addr(7)] [data hi] [data lo], CS low for the whole 24 bits.
 *     R/W = 1 -> read, 0 -> write.
 *
 * Non-blocking usage:
 *     drv.startRead(reg);
 *     ... later, every loop():
 *     uint16_t v;
 *     if (drv.poll(&v) == M90E26::Result::Done) { use v }
 */
class M90E26 {
 public:
  struct Pins {
    int8_t sck;
    int8_t miso;
    int8_t mosi;
    int8_t cs;
  };

  enum class Result : uint8_t { Idle, Busy, Done, Error };

  bool begin(const Pins& pins, spi_host_device_t host = SPI2_HOST,
             uint32_t clock_hz = 100000);
  void end();

  // ---- Non-blocking API -----------------------------------------------------
  bool startRead(uint8_t addr) { return start(Op::Read, addr, 0); }
  bool startWrite(uint8_t addr, uint16_t value) { return start(Op::Write, addr, value); }
  Result poll(uint16_t* value = nullptr);
  bool busy() const { return _op != Op::Idle; }

  // ---- Blocking helpers (setup / calibration tools only) ---------------------
  bool readBlocking(uint8_t addr, uint16_t& value, uint32_t timeout_ms = 10);
  bool writeBlocking(uint8_t addr, uint16_t value, uint32_t timeout_ms = 10);

  // ---- Checksum used by CS1 / CS2 ---------------------------------------------
  // High byte = XOR of all bytes, low byte = 8-bit sum of all bytes.
  static uint16_t checksum(const uint16_t* regs, size_t count);

 private:
  enum class Op : uint8_t { Idle, Read, Write };

  bool start(Op op, uint8_t addr, uint16_t value);

  spi_device_handle_t _dev = nullptr;
  spi_host_device_t _host = SPI2_HOST;
  spi_transaction_t _t;
  Op _op = Op::Idle;
};