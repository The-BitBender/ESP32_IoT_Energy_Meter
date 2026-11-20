#include "M90E26.h"
#include <string.h>

bool M90E26::begin(const Pins& p, spi_host_device_t host, uint32_t clock_hz) {
  if (_dev) return true;

  spi_bus_config_t bus = {};
  bus.mosi_io_num = p.mosi;
  bus.miso_io_num = p.miso;
  bus.sclk_io_num = p.sck;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = 8;
  if (spi_bus_initialize(host, &bus, SPI_DMA_CH_AUTO) != ESP_OK) return false;

  spi_device_interface_config_t dev = {};
  dev.mode = 3;                    // CPOL=1, CPHA=1
  dev.clock_speed_hz = (int)clock_hz;
  dev.spics_io_num = p.cs;
  dev.queue_size = 1;
  if (spi_bus_add_device(host, &dev, &_dev) != ESP_OK) {
    spi_bus_free(host);
    _dev = nullptr;
    return false;
  }
  _host = host;
  _op = Op::Idle;
  return true;
}

void M90E26::end() {
  if (!_dev) return;
  spi_bus_remove_device(_dev);
  spi_bus_free(_host);
  _dev = nullptr;
  _op = Op::Idle;
}

bool M90E26::start(Op op, uint8_t addr, uint16_t value) {
  if (!_dev || _op != Op::Idle) return false;

  memset(&_t, 0, sizeof(_t));
  _t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  _t.length = 24;  // bits; rxlength = 0 -> same as length (full duplex)
  _t.tx_data[0] = (uint8_t)((op == Op::Read ? 0x80 : 0x00) | (addr & 0x7F));
  _t.tx_data[1] = (op == Op::Write) ? (uint8_t)(value >> 8) : 0;
  _t.tx_data[2] = (op == Op::Write) ? (uint8_t)(value & 0xFF) : 0;

  if (spi_device_queue_trans(_dev, &_t, 0) != ESP_OK) return false;
  _op = op;
  return true;
}

M90E26::Result M90E26::poll(uint16_t* value) {
  if (_op == Op::Idle) return Result::Idle;

  spi_transaction_t* done = nullptr;
  esp_err_t e = spi_device_get_trans_result(_dev, &done, 0);  // 0 ticks: never blocks
  if (e == ESP_ERR_TIMEOUT) return Result::Busy;

  Op finished = _op;
  _op = Op::Idle;
  if (e != ESP_OK || done == nullptr) return Result::Error;

  if (finished == Op::Read && value) {
    *value = ((uint16_t)done->rx_data[1] << 8) | done->rx_data[2];
  }
  return Result::Done;
}

bool M90E26::readBlocking(uint8_t addr, uint16_t& value, uint32_t timeout_ms) {
  if (!startRead(addr)) return false;
  uint32_t t0 = millis();
  for (;;) {
    Result r = poll(&value);
    if (r == Result::Done) return true;
    if (r != Result::Busy) return false;
    if (millis() - t0 > timeout_ms) return false;
  }
}

bool M90E26::writeBlocking(uint8_t addr, uint16_t value, uint32_t timeout_ms) {
  if (!startWrite(addr, value)) return false;
  uint32_t t0 = millis();
  for (;;) {
    Result r = poll();
    if (r == Result::Done) return true;
    if (r != Result::Busy) return false;
    if (millis() - t0 > timeout_ms) return false;
  }
}

uint16_t M90E26::checksum(const uint16_t* regs, size_t count) {
  uint8_t sum = 0, x = 0;
  for (size_t i = 0; i < count; i++) {
    uint8_t hi = (uint8_t)(regs[i] >> 8);
    uint8_t lo = (uint8_t)(regs[i] & 0xFF);
    sum = (uint8_t)(sum + hi + lo);
    x ^= hi ^ lo;
  }
  return ((uint16_t)x << 8) | sum;
}