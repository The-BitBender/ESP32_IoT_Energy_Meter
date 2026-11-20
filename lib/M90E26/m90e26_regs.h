#pragma once
#include <stdint.h>

// M90E26 register map (Atmel-46002-SE-M90E26 datasheet)
namespace M90E26Reg {

// ---- Special / status registers -------------------------------------------
constexpr uint8_t SoftReset = 0x00;  // write 0x789A
constexpr uint8_t SysStatus = 0x01;
constexpr uint8_t FuncEn    = 0x02;
constexpr uint8_t SagTh     = 0x03;
constexpr uint8_t SmallPMod = 0x04;
constexpr uint8_t LastData  = 0x06;
constexpr uint8_t LSB       = 0x08;  // RMS/power LSB extension (unused here)

// ---- Metering calibration start-up registers (CalStart = 0x5678 to write) --
constexpr uint8_t CalStart  = 0x20;
constexpr uint8_t PLconstH  = 0x21;
constexpr uint8_t PLconstL  = 0x22;
constexpr uint8_t Lgain     = 0x23;
constexpr uint8_t Lphi      = 0x24;
constexpr uint8_t Ngain     = 0x25;
constexpr uint8_t Nphi      = 0x26;
constexpr uint8_t PStartTh  = 0x27;
constexpr uint8_t PNolTh    = 0x28;
constexpr uint8_t QStartTh  = 0x29;
constexpr uint8_t QNolTh    = 0x2A;
constexpr uint8_t MMode     = 0x2B;
constexpr uint8_t CS1       = 0x2C;

// ---- Measurement calibration registers (AdjStart = 0x5678 to write) --------
constexpr uint8_t AdjStart  = 0x30;
constexpr uint8_t Ugain     = 0x31;
constexpr uint8_t IgainL    = 0x32;
constexpr uint8_t IgainN    = 0x33;
constexpr uint8_t Uoffset   = 0x34;
constexpr uint8_t IoffsetL  = 0x35;
constexpr uint8_t IoffsetN  = 0x36;
constexpr uint8_t PoffsetL  = 0x37;
constexpr uint8_t QoffsetL  = 0x38;
constexpr uint8_t PoffsetN  = 0x39;
constexpr uint8_t QoffsetN  = 0x3A;
constexpr uint8_t CS2       = 0x3B;

// ---- Energy registers (read-and-clear, 0.1 pulse / LSB) --------------------
constexpr uint8_t APenergy  = 0x40;  // forward active
constexpr uint8_t ANenergy  = 0x41;  // reverse active
constexpr uint8_t ATenergy  = 0x42;  // absolute active
constexpr uint8_t RPenergy  = 0x43;  // forward reactive
constexpr uint8_t RNenergy  = 0x44;  // reverse reactive
constexpr uint8_t RTenergy  = 0x45;  // absolute reactive
constexpr uint8_t EnStatus  = 0x46;

// ---- Measurement registers, L line ----------------------------------------
constexpr uint8_t Irms      = 0x48;  // 1 LSB = 1 mA
constexpr uint8_t Urms      = 0x49;  // 1 LSB = 0.01 V
constexpr uint8_t Pmean     = 0x4A;  // signed, 1 LSB = 1 W
constexpr uint8_t Qmean     = 0x4B;  // signed, 1 LSB = 1 var
constexpr uint8_t Freq      = 0x4C;  // 1 LSB = 0.01 Hz
constexpr uint8_t PowerF    = 0x4D;  // signed, 1 LSB = 0.001
constexpr uint8_t Pangle    = 0x4E;  // signed, 1 LSB = 0.1 deg
constexpr uint8_t Smean     = 0x4F;  // 1 LSB = 1 VA

// ---- Measurement registers, N line -----------------------------------------
constexpr uint8_t IrmsN     = 0x68;
constexpr uint8_t PmeanN    = 0x6A;
constexpr uint8_t QmeanN    = 0x6B;
constexpr uint8_t PowerFN   = 0x6D;
constexpr uint8_t PangleN   = 0x6E;
constexpr uint8_t SmeanN    = 0x6F;

// ---- Magic values ----------------------------------------------------------
constexpr uint16_t SOFT_RESET_KEY = 0x789A;
constexpr uint16_t CAL_ENTER      = 0x5678;  // CalStart/AdjStart: allow writes
constexpr uint16_t CAL_EXIT       = 0x8765;  // CalStart/AdjStart: normal mode

}  // namespace M90E26Reg