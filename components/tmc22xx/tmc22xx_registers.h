#pragma once

#include <cstdint>

// Register map of the TMC2208 / TMC2209 (UART variants of the TMC22xx family).
// Addresses and bit positions follow the Trinamic/ADI datasheets
// (TMC220x/TMC222x rev. 1.14 and TMC2209 rev. 1.09).

namespace esphome::tmc22xx {

// --- UART datagram constants -------------------------------------------------
static const uint8_t TMC_SYNC = 0x05;         // sync nibble 0b0101 (+ 4 reserved bits)
static const uint8_t TMC_MASTER_ADDR = 0xFF;  // address used by the driver in replies
static const uint8_t TMC_WRITE_FLAG = 0x80;   // MSB of the register address byte

// --- Register addresses ------------------------------------------------------
static const uint8_t REG_GCONF = 0x00;       // RW
static const uint8_t REG_GSTAT = 0x01;       // R + write-1-to-clear
static const uint8_t REG_IFCNT = 0x02;       // R, counts successful UART writes
static const uint8_t REG_SLAVECONF = 0x03;   // W
static const uint8_t REG_IOIN = 0x06;        // R
static const uint8_t REG_IHOLD_IRUN = 0x10;  // W
static const uint8_t REG_TPOWERDOWN = 0x11;  // W
static const uint8_t REG_TSTEP = 0x12;       // R
static const uint8_t REG_TPWMTHRS = 0x13;    // W
static const uint8_t REG_TCOOLTHRS = 0x14;   // W, TMC2209 only
static const uint8_t REG_VACTUAL = 0x22;     // W
static const uint8_t REG_SGTHRS = 0x40;      // W, TMC2209 only
static const uint8_t REG_SG_RESULT = 0x41;   // R, TMC2209 only
static const uint8_t REG_COOLCONF = 0x42;    // W, TMC2209 only
static const uint8_t REG_CHOPCONF = 0x6C;    // RW
static const uint8_t REG_DRV_STATUS = 0x6F;  // R
static const uint8_t REG_PWMCONF = 0x70;     // RW

// --- GCONF -------------------------------------------------------------------
static const uint32_t GCONF_I_SCALE_ANALOG = 1UL << 0;    // 1: VREF pin scales current
static const uint32_t GCONF_INTERNAL_RSENSE = 1UL << 1;   // keep 0 with external sense resistors
static const uint32_t GCONF_EN_SPREADCYCLE = 1UL << 2;    // 0: StealthChop, 1: SpreadCycle
static const uint32_t GCONF_SHAFT = 1UL << 3;             // inverse motor direction
static const uint32_t GCONF_PDN_DISABLE = 1UL << 6;       // 1: PDN_UART pin is UART only
static const uint32_t GCONF_MSTEP_REG_SELECT = 1UL << 7;  // 1: microsteps from MRES, not MS1/MS2
static const uint32_t GCONF_MULTISTEP_FILT = 1UL << 8;    // step pulse filtering

// --- GSTAT -------------------------------------------------------------------
static const uint32_t GSTAT_RESET = 1UL << 0;
static const uint32_t GSTAT_DRV_ERR = 1UL << 1;
static const uint32_t GSTAT_UV_CP = 1UL << 2;

// --- IOIN --------------------------------------------------------------------
static const uint32_t IOIN_ENN = 1UL << 0;
static const uint32_t IOIN_MS1 = 1UL << 2;
static const uint32_t IOIN_MS2 = 1UL << 3;
static const uint32_t IOIN_DIAG = 1UL << 4;
static const uint32_t IOIN_PDN_UART = 1UL << 6;
static const uint32_t IOIN_STEP = 1UL << 7;
static const uint32_t IOIN_DIR = 1UL << 9;
static const uint8_t IOIN_VERSION_SHIFT = 24;
static const uint8_t IOIN_VERSION_TMC2208 = 0x20;
static const uint8_t IOIN_VERSION_TMC2209 = 0x21;

// --- IHOLD_IRUN --------------------------------------------------------------
static const uint8_t IHOLD_SHIFT = 0;        // 5 bit
static const uint8_t IRUN_SHIFT = 8;         // 5 bit
static const uint8_t IHOLDDELAY_SHIFT = 16;  // 4 bit

// --- CHOPCONF ----------------------------------------------------------------
static const uint32_t CHOPCONF_TOFF_MASK = 0x0FUL;  // bits 3..0, 0 = driver off
static const uint8_t CHOPCONF_HSTRT_SHIFT = 4;      // bits 6..4
static const uint8_t CHOPCONF_HEND_SHIFT = 7;       // bits 10..7
static const uint8_t CHOPCONF_TBL_SHIFT = 15;       // bits 16..15
static const uint32_t CHOPCONF_VSENSE = 1UL << 17;  // 1: high sensitivity (V_FS = 180 mV)
static const uint8_t CHOPCONF_MRES_SHIFT = 24;      // bits 27..24, 0 = 256 ... 8 = full step
static const uint32_t CHOPCONF_MRES_MASK = 0x0FUL << CHOPCONF_MRES_SHIFT;
static const uint32_t CHOPCONF_INTPOL = 1UL << 28;  // interpolation to 256 microsteps

// --- DRV_STATUS --------------------------------------------------------------
static const uint32_t DRV_OTPW = 1UL << 0;   // overtemperature pre-warning
static const uint32_t DRV_OT = 1UL << 1;     // overtemperature shutdown
static const uint32_t DRV_S2GA = 1UL << 2;   // short to GND phase A
static const uint32_t DRV_S2GB = 1UL << 3;
static const uint32_t DRV_S2VSA = 1UL << 4;  // low side short phase A
static const uint32_t DRV_S2VSB = 1UL << 5;
static const uint32_t DRV_OLA = 1UL << 6;    // open load phase A
static const uint32_t DRV_OLB = 1UL << 7;
static const uint32_t DRV_T120 = 1UL << 8;
static const uint32_t DRV_T143 = 1UL << 9;
static const uint32_t DRV_T150 = 1UL << 10;
static const uint32_t DRV_T157 = 1UL << 11;
static const uint8_t DRV_CS_ACTUAL_SHIFT = 16;  // 5 bit
static const uint32_t DRV_STEALTH = 1UL << 30;
static const uint32_t DRV_STST = 1UL << 31;  // standstill indicator

// Flags that switch the power stage off or indicate a hard fault.
static const uint32_t DRV_ERROR_MASK = DRV_OT | DRV_S2GA | DRV_S2GB | DRV_S2VSA | DRV_S2VSB;

// Component level error bits (reported to on_driver_error / driver_error entities).
// Bits 0..11 mirror DRV_STATUS, the bits below are synthesized by the component.
static const uint32_t ERR_UV_CP = 1UL << 24;  // GSTAT.uv_cp, charge pump undervoltage
static const uint32_t ERR_COMM = 1UL << 25;   // driver does not answer on UART

// Nominal internal clock, used for TSTEP based thresholds.
static const uint32_t TMC_FCLK_HZ = 12000000UL;

}  // namespace esphome::tmc22xx
