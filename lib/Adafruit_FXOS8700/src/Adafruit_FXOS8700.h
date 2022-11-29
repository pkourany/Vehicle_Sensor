/*!
 * @file Adafruit_FXOS8700.h
 *
 * This is part of Adafruit's FXOS8700 driver for the Arduino platform.  It is
 * designed specifically to work with the Adafruit FXOS8700 breakout:
 * https://www.adafruit.com/products/3463
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
/** \file Adafruit_FXOS8700.h */
#ifndef __FXOS8700_H__
#define __FXOS8700_H__

#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>

/*=========================================================================
    I2C ADDRESS/BITS AND SETTINGS
    -----------------------------------------------------------------------*/
/** 7-bit I2C address for this sensor
    SA1 SA0 Slave address
    0   0       0x1E    // Default for Mikroe breakout
    0   1       0x1D
    1   0       0x1C
    1   1       0x1F
*/
#define FXOS8700_ADDRESS (0x1E) // SA0 = 0, SA1 = 0
/** Device ID for this sensor (used as sanity check during init) */
#define FXOS8700_ID (0xC7) // 1100 0111

/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
/*!
    Raw register addresses used to communicate with the sensor.
*/
typedef enum {
  FXOS8700_REGISTER_STATUS =    0x00,    /**< 0x00 */
  FXOS8700_REGISTER_OUT_X_MSB = 0x01, /**< 0x01 */
  FXOS8700_REGISTER_OUT_X_LSB = 0x02, /**< 0x02 */
  FXOS8700_REGISTER_OUT_Y_MSB = 0x03, /**< 0x03 */
  FXOS8700_REGISTER_OUT_Y_LSB = 0x04, /**< 0x04 */
  FXOS8700_REGISTER_OUT_Z_MSB = 0x05, /**< 0x05 */
  FXOS8700_REGISTER_OUT_Z_LSB = 0x06, /**< 0x06 */
  FXOS8600_REGISTER_SYSMOD =    0x0B,    /**< 0x0B */
  FXOS8600_REGISTER_INT_SOURCE = 0x0C,    /**< 0x0C */
  FXOS8700_REGISTER_WHO_AM_I =  0x0D, /**< 0x0D (default value = 0b11000111, read only) */
  FXOS8700_REGISTER_XYZ_DATA_CFG = 0x0E, /**< 0x0E */
  FXOS8700_REGISTER_CTRL_REG1 = 0x2A, /**< 0x2A (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_CTRL_REG2 = 0x2B, /**< 0x2B (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_CTRL_REG3 = 0x2C, /**< 0x2C (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_CTRL_REG4 = 0x2D, /**< 0x2D (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_CTRL_REG5 = 0x2E, /**< 0x2E (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_MSTATUS =   0x32,    /**< 0x32 */
  FXOS8700_REGISTER_MOUT_X_MSB = 0x33, /**< 0x33 */
  FXOS8700_REGISTER_MOUT_X_LSB = 0x34, /**< 0x34 */
  FXOS8700_REGISTER_MOUT_Y_MSB = 0x35, /**< 0x35 */
  FXOS8700_REGISTER_MOUT_Y_LSB = 0x36, /**< 0x36 */
  FXOS8700_REGISTER_MOUT_Z_MSB = 0x37, /**< 0x37 */
  FXOS8700_REGISTER_MOUT_Z_LSB = 0x38, /**< 0x38 */
  FXOS8700_REGISTER_MCTRL_REG1 = 0x5B, /**< 0x5B (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_MCTRL_REG2 = 0x5C, /**< 0x5C (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_MCTRL_REG3 = 0x5D, /**< 0x5D (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_M_INT_SRC  = 0x5E, /**< 0x5E (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_M_VECM_CFG = 0x69, /**< 0x69 (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_M_VECM_THS_MSB = 0x6A, /**< 0x6A (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_M_VECM_THS_LSB = 0x6B, /**< 0x6B (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_M_VECM_CNT = 0x6C, /**< 0x6C (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_M_VECM_INITX_MSB = 0x6D, /**< 0x6D (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_M_VECM_INITX_LSB = 0x6E, /**< 0x6E (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_M_VECM_INITY_MSB = 0x6F, /**< 0x6F (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_M_VECM_INITY_LSB = 0x70, /**< 0x70 (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_M_VECM_INITZ_MSB = 0x71, /**< 0x71 (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_M_VECM_INITZ_LSB = 0x72, /**< 0x72 (default value = 0b00000000, read/write) */
} fxos8700Registers_t;
/*=========================================================================*/

/*=========================================================================
    OPTIONAL SENSOR MODE SETTINGS
    -----------------------------------------------------------------------*/
/*!
    System status for the overall FXOS8700 sensor. Gets the current mode as
    the 2bit sysmod[1:0] setting to ensure proper mode configuration
*/
typedef enum {
  STANDBY = 0x00, /**< sysmod[1:0]. Standby status */
  WAKE = 0x01,    /**< sysmod[1:0]. Wake status */
  SLEEP = 0x02    /**< sysmod[1:0]. Sleep status */
} fxos8700SystemStatus_t;

/*!
    Sensor mode settings for the overall FXOS8700 sensor. Sets the sensor in
    accelerometer-only, magnerometer-only, or hybrid modes. Sent to
    FXOS8700_REGISTER_MCTRL_REG1
*/
typedef enum {
  ACCEL_ONLY_MODE = 0b00, /**< m_hms[1:0] = 0b00. Accel-only mode */
  MAG_ONLY_MODE = 0b01,   /**< m_hms[1:0] = 0b01. Mag-only mode */
  HYBRID_MODE = 0b11      /**< m_hms[1:0] = 0b11. Hybrid mode */
} fxos8700SensorMode_t;
/*=========================================================================*/

/*!
    Sensor mode settings for FXS8700 ccelerometer Wake-mode OSR mode.
    Sets the OSR wake mode to normal, low noise/low power or low power modes.
    Sent to FXOS8700_REGISTER_CTRL_REG2
*/
typedef enum {
  NORMAL = 0b00,              /**< (s)mods[1:0] = 0b00. Accel-only mode */
  LOW_NOISE_LOW_POWER = 0b01, /**< m_hms[1:0] = 0b01. Mag-only mode */
  HIGH_RESOLUTION = 0b10,      /**< m_hms[1:0] = 0b11. Hybrid mode */
  LOW_POWER = 0b11,            /**< m_hms[1:0] = 0b11. Hybrid mode */
} fxos8700AccelWakeOSR_t;
/*=========================================================================*/

/*=========================================================================
    OPTIONAL SPEED SETTINGS
    -----------------------------------------------------------------------*/
/*!
    Output Data Rate (ODR) key for the overall FXOS8700 sensor. Called by
    user for convenient variable name matching
*/
typedef enum {
  ODR_800HZ,    /**< 800Hz, only available in accel/mag-only modes */
  ODR_400HZ,    /**< 400Hz, available in all modes */
  ODR_200HZ,    /**< 200Hz, available in all modes */
  ODR_100HZ,    /**< 100Hz, available in all modes */
  ODR_50HZ,     /**< 50Hz, available in all modes */
  ODR_25HZ,     /**< 25Hz, only available in hybrid mode */
  ODR_12_5HZ,   /**< 12.5Hz, only available in accel/mag-only modes */
  ODR_6_25HZ,   /**< 6.25Hz, available in all modes */
  ODR_3_125HZ,  /**< 3.125Hz, only available in hybrid mode */
  ODR_1_5625HZ, /**< 3.125Hz, only available in accel/mag-only modes */
  ODR_0_7813HZ, /**< 0.7813Hz, only available in hybrid mode */
} fxos8700ODR_t;

/*!
    Output Data Rates (ODRs) available for accel/mag-only modes, type array
    of user set fxos8700ODR_t
*/
const fxos8700ODR_t ACCEL_MAG_ONLY_AVAILABLE_ODRs[] = {
    ODR_800HZ,    /**< 800Hz, only available in accel/mag-only modes */
    ODR_400HZ,    /**< 400Hz, available in all modes */
    ODR_200HZ,    /**< 200Hz, available in all modes */
    ODR_100HZ,    /**< 100Hz, available in all modes */
    ODR_50HZ,     /**< 50Hz, available in all modes */
    ODR_12_5HZ,   /**< 12.5Hz, only available in accel/mag-only modes */
    ODR_6_25HZ,   /**< 6.25Hz, available in all modes */
    ODR_1_5625HZ, /**< 3.125Hz, only available in accel/mag-only modes */
};

/*!
    Output Data Rates (ODRs) available for hybrid mode, type array of user
    set fxos8700ODR_t
*/
const fxos8700ODR_t HYBRID_AVAILABLE_ODRs[] = {
    ODR_400HZ,    /**< 400Hz, available in all modes */
    ODR_200HZ,    /**< 200Hz, available in all modes */
    ODR_100HZ,    /**< 100Hz, available in all modes */
    ODR_50HZ,     /**< 50Hz, available in all modes */
    ODR_25HZ,     /**< 25Hz, only available in hybrid mode */
    ODR_6_25HZ,   /**< 6.25Hz, available in all modes */
    ODR_3_125HZ,  /**< 3.125Hz, only available in hybrid mode */
    ODR_0_7813HZ, /**< 0.7813Hz, only available in hybrid mode */
};

/*!
    Output Data Rate (ODR) settings to write to the dr[2:0] bits in
    CTRL_REG_1, array type to pass the index from available accel/mag-only
    and hyrbrid modes
*/
const uint16_t ODR_drBits[] = {
    0x00, /**< dr=0b000. 800Hz accel/mag-only modes, 400Hz hyrbid mode */
    0x08, /**< dr=0b001. 400Hz accel/mag-only modes, 200Hz hyrbid mode */
    0x10, /**< dr=0b010. 200Hz accel/mag-only modes, 100Hz hyrbid mode */
    0x18, /**< dr=0b011. 100Hz accel/mag-only modes, 50Hz hyrbid mode */
    0x20, /**< dr=0b100. 50Hz accel/mag-only modes, 25Hz hyrbid mode */
    0x28, /**< dr=0b101. 12.5Hz accel/mag-only modes, 6.25Hz hyrbid mode */
    0x30, /**< dr=0b110. 6.25Hz accel/mag-only modes, 3.125Hz hyrbid mode */
    0x38, /**< dr=0b111. 1.5625Hz accel/mag-only modes, 0.7813Hz hyrbid mode */
};
/*=========================================================================*/

/*=========================================================================
    OPTIONAL RANGE SETTINGS
    -----------------------------------------------------------------------*/
/*!
    Range settings for the accelerometer sensor.
*/
typedef enum {
  ACCEL_RANGE_2G = 0x00, /**< +/- 2g range */
  ACCEL_RANGE_4G = 0x01, /**< +/- 4g range */
  ACCEL_RANGE_8G = 0x02  /**< +/- 8g range */
} fxos8700AccelRange_t;
/*=========================================================================*/

/*=========================================================================
    OPTIONAL MAGNETOMETER OVERSAMPLING SETTINGS
    -----------------------------------------------------------------------*/
/*!
    Range settings for the accelerometer sensor.
*/
typedef enum {
  MAG_OSR_0, /**< Mag oversampling ratio = 0 */
  MAG_OSR_1, /**< Mag oversampling ratio = 1 */
  MAG_OSR_2, /**< Mag oversampling ratio = 2 */
  MAG_OSR_3, /**< Mag oversampling ratio = 3 */
  MAG_OSR_4, /**< Mag oversampling ratio = 4 */
  MAG_OSR_5, /**< Mag oversampling ratio = 5 */
  MAG_OSR_6, /**< Mag oversampling ratio = 6 */
  MAG_OSR_7, /**< Mag oversampling ratio = 7 */
} fxos8700MagOSR_t;
/*=========================================================================*/

/*=========================================================================
    RAW 3DOF SENSOR DATA TYPE
    -----------------------------------------------------------------------*/
/*!
    @brief  Raw (integer) values from a 3dof sensor.
*/
typedef struct {
  int16_t x; /**< Raw int16_t value from the x axis */
  int16_t y; /**< Raw int16_t value from the y axis */
  int16_t z; /**< Raw int16_t value from the z axis */
} fxos8700RawData_t;


// bit 0 = pp_od, bit 1 = ipol
typedef enum {
  INT_ACTIVE_LOW_PP = 0b00,   // ipol = 0 (INT1/INT2 Active LOW), pp_od = 0 (Push-Pull) - default
  INT_ACTIVE_HIGH_PP = 0b10,  // ipol = 1 (INT1/INT2 Active LOW), pp_od = 0 (Push-Pull)
  INT_ACTIVE_LOW_OD = 0b01,   // ipol = 0 (INT1/INT2 Active LOW), pp_od = 1 (Open Drain) - default
  INT_ACTIVE_HIGH_OD = 0b11   // ipol = 1 (INT1/INT2 Active LOW), pp_od = 1 (Open Drain)
} fxos8700IntOutModes_t;


/*=========================================================================
    ACCELEROMETER INTERRUPT CONTROL, ENABLE AND MAP REGISTER BIT VALUES
    CRTL_REG3, CRTL_REG4, CRTL_REG5
    -----------------------------------------------------------------------*/
/*!
    @brief  Raw (bit) values for Accelerometer Interrupt registers
*/
// CRTL_REG3
#define  INT_CNTL_PP_OD           0x01
#define  INT_CNTL_IPOL            0x02
#define  INT_CNTL_WAKE_A_VECM     0x04
#define  INT_CNTL_WAKE_FFMT       0x08
#define  INT_CNTL_WAKE_PULSE      0x10
#define  INT_CNTL_WAKE_LNDPRT     0x20
#define  INT_CNTL_WAKE_TRANS      0x40
#define  INT_CNTL_WAKE_FIFO_GATE  0x80

// CRTL_REG4
#define  INT_EN_DRDY				      0x01
#define  INT_EN_A_VECM            0x02
#define  INT_EN_FFMT              0x04
#define  INT_EN_PULSE             0x08
#define  INT_EN_LNDPRT            0x10
#define  INT_EN_TRANS             0x20
#define  INT_EN_FIFO              0x40
#define  INT_EN_ASLP              0x80

// CRTL_REG5
#define  INT_CFG_DRDY				      0x01
#define  INT_CFG_A_VECM           0x02
#define  INT_CFG_FFMT             0x04
#define  INT_CFG_PULSE            0x08
#define  INT_CFG_LNDPRT           0x10
#define  INT_CFG_TRANS            0x20
#define  INT_CFG_FIFO             0x40
#define  INT_CFG_ASLP             0x80

// INT_SOURCE (register 0x0C)
#define INT_SRC_DRDY          0x01
#define INT_SRC_A_VECM        0x02
#define INT_SRC_FFMT          0x04
#define INT_SRC_PULSE         0x08
#define INT_SRC_LNDPRT        0x10
#define INT_SRC_TRANS         0x20
#define INT_SRC_FIFO          0x40
#define INT_SRC_ASLP          0x80


/*=========================================================================
    MAGNETOMETER INTERRUPT SOURCE AND CONFIG REGISTER BIT VALUES
    CRTL_REG3, CRTL_REG4, CRTL_REG5
    -----------------------------------------------------------------------*/
/*!
    @brief  Raw (bit) values for Accelerometer Interrupt registers
*/
// M_INT_SOURCE (register 0x5E)
#define  MAG_INT_SRC_M_DRDY   0x01
#define  MAG_INT_SRC_M_VECM   0x02
#define  MAG_INT_SRC_M_THS    0x04

// M_VECM_CFG (register 0x69())
#define  MAG_M_VECM_INT_CFG   0x01
#define  MAG_M_VECM_INT_EN    0x02
#define  MAG_M_VECM_WAKE_EN   0x04
#define  MAG_M_VECM_EN        0x08
#define  MAG_M_VECM_UPDM      0x10
#define  MAG_M_VECM_INITM     0x20
#define  MAG__VECM_ELE        0x40


#define LAST_REG_ADDR		0x78

static const char *register_name[] = {
    "DR_STATUS       ",
    "OUT_X_MSB       ",
    "OUT_X_LSB       ",
    "OUT_Y_MSB       ",
    "OUT_Y_LSB       ",
    "OUT_Z_MSB       ",
    "OUT_Z_LSB       ",
    "RESERVED1       ",
    "RESERVED2       ",
    "F_SETUP         ",
    "TRIG_CFG        ",
    "SYSMOD          ",
    "INT_SOURCE      ",
    "WHOAMI          ",
    "XYZ_DATA_CFG    ",
    "HP_FILTER_CUTOFF",
    "PL_STATUS       ",
    "PL_CFG          ",
    "PL_COUNT        ",
    "PL_BF_ZCOMP     ",
    "P_L_THS_REG     ",
    "A_FFMT_CFG      ",
    "A_FFMT_SRC      ",
    "A_FFMT_THS      ",
    "A_FFMT_COUNT    ",
    "RESERVED3       ",
    "RESERVED4       ",
    "RESERVED5       ",
    "RESERVED6       ",
    "TRANSIENT_CFG   ",
    "TRANSIENT_SRC   ",
    "TRANSIENT_THS   ",
    "TRANSIENT_COUNT ",
    "PULSE_CFG       ",
    "PULSE_SRC       ",
    "PULSE_THSX      ",
    "PULSE_THSY      ",
    "PULSE_THSZ      ",
    "PULSE_TMLT      ",
    "PULSE_LTCY      ",
    "PULSE_WIND      ",
    "ASLP_COUNT      ",
    "CTRL_REG1       ",
    "CTRL_REG2       ",
    "CTRL_REG3       ",
    "CTRL_REG4       ",
    "CTRL_REG5       ",
    "OFF_X           ",
    "OFF_Y           ",
    "OFF_Z           ",
    "M_DR_STATUS     ",
    "M_OUT_X_MSB     ",
    "M_OUT_X_LSB     ",
    "M_OUT_Y_MSB     ",
    "M_OUT_Y_LSB     ",
    "M_OUT_Z_MSB     ",
    "M_OUT_Z_LSB     ",
    "CMP_OUT_X_MSB   ",
    "CMP_OUT_X_LSB   ",
    "CMP_OUT_Y_MSB   ",
    "CMP_OUT_Y_LSB   ",
    "CMP_OUT_Z_MSB   ",
    "CMP_OUT_Z_LSB   ",
    "M_OFF_X_MSB     ",
    "M_OFF_X_LSB     ",
    "M_OFF_Y_MSB     ",
    "M_OFF_Y_LSB     ",
    "M_OFF_Z_MSB     ",
    "M_OFF_Z_LSB     ",
    "MAX_X_MSB       ",
    "MAX_X_LSB       ",
    "MAX_Y_MSB       ",
    "MAX_Y_LSB       ",
    "MAX_Z_MSB       ",
    "MAX_Z_LSB       ",
    "MIN_X_MSB       ",
    "MIN_X_LSB       ",
    "MIN_Y_MSB       ",
    "MIN_Y_LSB       ",
    "MIN_Z_MSB       ",
    "MIN_Z_LSB       ",
    "TEMP            ",
    "M_THS_CFG       ",
    "M_THS_SRC       ",
    "M_THS_X_MSB     ",
    "M_THS_X_LSB     ",
    "M_THS_Y_MSB     ",
    "M_THS_Y_LSB     ",
    "M_THS_Z_MSB     ",
    "M_THS_Z_LSB     ",
    "M_THS_COUNT     ",
    "M_CTRL_REG1     ",
    "M_CTRL_REG2     ",
    "M_CTRL_REG3     ",
    "M_INT_SOURCE    ",
    "A_VECM_CFG      ",
    "A_VECM_THS_MSB  ",
    "A_VECM_THS_LSB  ",
    "A_VECM_CNT      ",
    "A_VECM_INITX_MSB",
    "A_VECM_INITX_LSB",
    "A_VECM_INITY_MSB",
    "A_VECM_INITY_LSB",
    "A_VECM_INITZ_MSB",
    "A_VECM_INITZ_LSB",
    "M_VECM_CFG      ",
    "M_VECM_THS_MSB  ",
    "M_VECM_THS_LSB  ",
    "M_VECM_CNT      ",
    "M_VECM_INITX_MSB",
    "M_VECM_INITX_LSB",
    "M_VECM_INITY_MSB",
    "M_VECM_INITY_LSB",
    "M_VECM_INITZ_MSB",
    "M_VECM_INITZ_LSB",
    "A_FFMT_THS_X_MSB",
    "A_FFMT_THS_X_LSB",
    "A_FFMT_THS_Y_MSB",
    "A_FFMT_THS_Y_LSB",
    "A_FFMT_THS_Z_MSB",
    "A_FFMT_THS_Z_LSB"
};

static const void *field_labels[LAST_REG_ADDR + 1][8] = {
    { "" }, // FX_DR_STATUS
    { "-" }, // FX_OUT_X_MSB
    { " " }, // FX_OUT_X_LSB
    { "-" }, // FX_OUT_Y_MSB
    { "" }, // FX_OUT_Y_LSB
    { "-" }, // FX_OUT_Z_MSB
    { "" }, // FX_OUT_Z_LSB
    { "" }, // FX_RESERVED1
    { "" }, // FX_RESERVED2
    { "" }, // FX_F_SETUP
    { "" }, // FX_TRIG_CFG
    { "" }, // FX_SYSMOD
    { "SRC_DRDY", "SRC_A_VECM", "SRC_FFMT", "SRC_PULSE", "SRC_LNDPRT", "SRC_TRANS", "SRC_FIFO", "SRC_ASLP" }, // FX_INT_SOURCE
    { "h" }, // FX_WHOAMI
    { "" }, // FX_XYZ_DATA_CFG
    { "" }, // FX_HP_FILTER_CUTOFF
    { "" }, // FX_PL_STATUS
    { "" }, // FX_PL_CFG
    { "d" }, // FX_PL_COUNT
    { "" }, // FX_PL_BF_ZCOMP
    { "" }, // FX_P_L_THS_REG
    { "" }, // FX_A_FFMT_CFG
    { "" }, // FX_A_FFMT_SRC
    { "" }, // FX_A_FFMT_THS
    { "d" }, // FX_A_FFMT_COUNT
    { "" }, // FX_RESERVED3
    { "" }, // FX_RESERVED4
    { "" }, // FX_RESERVED5
    { "" }, // FX_RESERVED6
    { "A_TRAN_HYP_BYP", "A_TRAN_XEFE", "A_TRAN_YEFE", "A_TRAN_ZEFE", "A_TRAN_ELE" }, // FX_TRANSIENT_CFG
    { "" }, // FX_TRANSIENT_SRC
    { "" }, // FX_TRANSIENT_THS
    { "d" }, // FX_TRANSIENT_COUNT
    { "" }, // FX_PULSE_CFG
    { "" }, // FX_PULSE_SRC
    { "" }, // FX_PULSE_THSX
    { "" }, // FX_PULSE_THSY
    { "" }, // FX_PULSE_THSZ
    { "" }, // FX_PULSE_TMLT
    { "" }, // FX_PULSE_LTCY
    { "d" }, // FX_PULSE_WIND
    { "d" }, // FX_ASLP_COUNT
    { "ACTIVE", "F_READ", "LNOISE", "F", (const void *) 3, "", "F", (const void *) 2 }, // FX_CTRL_REG1
    { "F", (const void *) 2, "SLPE", "F", (const void *) 2, "RST", "ST" },  // FX_CTRL_REG2
    { "PP/OD", "IPOL", "WAKE_A_VECM", "WAKE_FFMT", "WAKE_PULSE", "WAKE_LNDPRT", "WAKE_TRANS", "WAKE_FIFO_GATE" }, // FX_CTRL_REG3
    { "INT_EN_DRDY", "INT_EN_A_VECM", "INT_EN_FFMT", "INT_EN_PULSE", "INT_LNDPRT", "INT_EN_TRANS", "INT_EN_FIFO", "INT_EN_ASLP" }, // FX_CTRL_REG4
    { "INT_CFG_DRDY", "INT_CFG_A_VECM", "INT_CFG_FFMT", "INT_CFG_PULSE", "INT_CFG_LNDPRT", "INT_CFG_TRANS", "INT_CFG_FIFO", "INT_CFG_ASLP" }, // FX_CTRL_REG5
    { "" }, // FX_OFF_X
    { "" }, // FX_OFF_Y
    { "" }, // FX_OFF_Z
    { "" }, // FX_M_DR_STATUS
    { "_" }, // FX_M_OUT_X_MSB
    { "" }, // FX_M_OUT_X_LSB
    { "_" }, // FX_M_OUT_Y_MSB
    { "" }, // FX_M_OUT_Y_LSB
    { "_" }, // FX_M_OUT_Z_MSB
    { "" }, // FX_M_OUT_Z_LSB
    { "_" }, // FX_CMP_OUT_X_MSB
    { "" }, // FX_CMP_OUT_X_LSB
    { "_" }, // FX_CMP_OUT_Y_MSB
    { "" }, // FX_CMP_OUT_Y_LSB
    { "_" }, // FX_CMP_OUT_Z_MSB
    { "" }, // FX_CMP_OUT_Z_LSB
    { "_" }, // FX_M_OFF_X_MSB
    { "" }, // FX_M_OFF_X_LSB
    { "_" }, // FX_M_OFF_Y_MSB
    { "" }, // FX_M_OFF_Y_LSB
    { "_" }, // FX_M_OFF_Z_MSB
    { "" }, // FX_M_OFF_Z_LSB
    { "_" }, // FX_MAX_X_MSB
    { "" }, // FX_MAX_X_LSB
    { "_" }, // FX_MAX_Y_MSB
    { "" }, // FX_MAX_Y_LSB
    { "_" }, // FX_MAX_Z_MSB
    { "" }, // FX_MAX_Z_LSB
    { "_" }, // FX_MIN_X_MSB
    { "" }, // FX_MIN_X_LSB
    { "_" }, // FX_MIN_Y_MSB
    { "" }, // FX_MIN_Y_LSB
    { "_" }, // FX_MIN_Z_MSB
    { "" }, // FX_MIN_Z_LSB
    { "t" }, // FX_TEMP
    { "THS_INT_CFG", "THS_INT_EN", "THS_WAKE_EN", "THS_XEFE", "THS_YEFE", "THS_ZEFE", "THS_OAE", "THS_ELE" }, // FX_M_THS_CFG
    { "" }, // FX_M_THS_SRC
    { "_" }, // FX_M_THS_X_MSB
    { "" }, // FX_M_THS_X_LSB
    { "_" }, // FX_M_THS_Y_MSB
    { "" }, // FX_M_THS_Y_LSB
    { "_" }, // FX_M_THS_Z_MSB
    { "" }, // FX_M_THS_Z_LSB
    { "d" }, // FX_M_THS_COUNT
    { "" }, // FX_M_CTRL_REG1
    { "" }, // FX_M_CTRL_REG2
    { "" }, // FX_M_CTRL_REG3
    { "SRC_M_DRDY", "SRC_M_VECM", "SRC_M_THS" }, // FX_M_INT_SOURCE
    { "A_VECM_EN", "A_VECM_UPDM", "A_VECM_INITM", "A_VECM_ELE" }, // FX_A_VECM_CFG
    { "_" }, // FX_A_VECM_THS_MSB
    { "" }, // FX_A_VECM_THS_LSB
    { "d" }, // FX_A_VECM_CNT
    { "_" }, // FX_A_VECM_INITX_MSB
    { "" }, // FX_A_VECM_INITX_LSB
    { "_" }, // FX_A_VECM_INITY_MSB
    { "" }, // FX_A_VECM_INITY_LSB
    { "_" }, // FX_A_VECM_INITZ_MSB
    { "" }, // FX_A_VECM_INITZ_LSB
    { "" }, // FX_M_VECM_CFG
    { "_" }, // FX_M_VECM_THS_MSB
    { "" }, // FX_M_VECM_THS_LSB
    { "d" }, // FX_M_VECM_CNT
    { "_" }, // FX_M_VECM_INITX_MSB
    { "" }, // FX_M_VECM_INITX_LSB
    { "_" }, // FX_M_VECM_INITY_MSB
    { "" }, // FX_M_VECM_INITY_LSB
    { "_" }, // FX_M_VECM_INITZ_MSB
    { "" }, // FX_M_VECM_INITZ_LSB
    { "_" }, // FX_A_FFMT_THS_X_MSB
    { "" }, // FX_A_FFMT_THS_X_LSB
    { "_" }, // FX_A_FFMT_THS_Y_MSB
    { "" }, // FX_A_FFMT_THS_Y_LSB
    { "_" }, // FX_A_FFMT_THS_Z_MSB
    { "" }  // FX_A_FFMT_THS_Z_LSB
};

static const unsigned int MAX_FIELD_SETS = 2;

static const struct field_label_data_struct {
    byte reg_addr;
    const char *field0[8];
    const char *field1[8];
} field_label_data[MAX_FIELD_SETS] = { { 0x2A, {"DR_800", "DR_400", "DR_200", "DR_100", "DR_50", "DR_13", "DR_6", "DR_2" }, { "ASLP_50", "ASLP_12", "ASLP_6", "ASLP1" } },
    { 0x2B, {"NORMAL", "LOW_NOISE", "HIGH_RES", "LOW_POWER"}, { "NORMAL", "LOW_NOISE", "HIGH_RES", "LOW_POWER" } },
};

/*
 Most recent snapshot of chip registers
 */
static byte regs[LAST_REG_ADDR + 1];



/*=========================================================================*/

class Adafruit_FXOS8700;

/** Adafruit Unified Sensor interface for accelerometer component of FXOS8700 */
class Adafruit_FXOS8700_Accelerometer : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the accelerometer
     sensor
      @param parent A pointer to the FXOS8700 class */
  Adafruit_FXOS8700_Accelerometer(Adafruit_FXOS8700 *parent) {
    _theFXOS8700 = parent;
  }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 8701;
  Adafruit_FXOS8700 *_theFXOS8700 = NULL;
};

/** Adafruit Unified Sensor interface for magnetometer component of FXOS8700 */
class Adafruit_FXOS8700_Magnetometer : public Adafruit_Sensor {

public:
  /** @brief Create an Adafruit_Sensor compatible object for the magnetometer
     sensor
      @param parent A pointer to the FXOS8700 class */
  Adafruit_FXOS8700_Magnetometer(Adafruit_FXOS8700 *parent) {
    _theFXOS8700 = parent;
  }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 8702;
  Adafruit_FXOS8700 *_theFXOS8700 = NULL;
};

/**************************************************************************/
/*!
    @brief  Unified sensor driver for the Adafruit FXOS8700 breakout.
*/
/**************************************************************************/
class Adafruit_FXOS8700 : public Adafruit_Sensor {
public:
  Adafruit_FXOS8700(int32_t accelSensorID = -1, int32_t magSensorID = -1);
  ~Adafruit_FXOS8700();
  bool begin(uint8_t addr = FXOS8700_ADDRESS, TwoWire *wire = &Wire);

  bool initialize(bool complete);

  // Return I2C device instance pointer
  Adafruit_I2CDevice * getAddr();

  bool getEvent(sensors_event_t *accel);
  void getSensor(sensor_t *singleSensorEvent);
  bool getEvent(sensors_event_t *accel, sensors_event_t *mag);
  void getSensor(sensor_t *accel, sensor_t *mag);
  void standby(boolean standby);

  /*! Raw accelerometer values from last sucsessful sensor read */
  fxos8700RawData_t accel_raw;
  /*! Raw magnetometer values from last successful sensor read */
  fxos8700RawData_t mag_raw;

  Adafruit_Sensor *getMagnetometerSensor(void);
  Adafruit_Sensor *getAccelerometerSensor(void);

  Adafruit_FXOS8700_Accelerometer *accel_sensor = NULL; ///< Accelerometer data object
  Adafruit_FXOS8700_Magnetometer *mag_sensor = NULL; ///< Mag data object

  void setSensorMode(fxos8700SensorMode_t mode);
  fxos8700SensorMode_t getSensorMode();

  void setAccelRange(fxos8700AccelRange_t range);
  fxos8700AccelRange_t getAccelRange();

  void setOutputDataRate(fxos8700ODR_t rate);
  fxos8700ODR_t getOutputDataRate();

  void setMagOversamplingRatio(fxos8700MagOSR_t ratio);
  fxos8700MagOSR_t getMagOversamplingRatio();

  void setReducedNoiseMode(bool lnoise);
  void setAccelWakeModeOSR(fxos8700AccelWakeOSR_t mode);
  void setAutoSleepMode(bool autosleep);

  uint8_t readMagInterruptSource();
  uint8_t readInterruptSource();

  void setMagVectorMagnitudeConfig(uint8_t regValue);
  void setMagVectorMagnitudeThreshold(uint16_t threshold);
  void setMagVectorMagnitudeDbounceCount(uint8_t count);
  void setMagVectorMagnitudeReferenceValue(int16_t xref, int16_t yref, int16_t zref);

  void setInterruptMode();
  void setInterruptOutputMode();

  void setInterruptPOL_PP_Config(fxos8700IntOutModes_t mode);
  void enableInterrupts(uint8_t int_bits);
  void disableInterrupts(uint8_t int_bits);
  void setInterruptMapToINT1(uint8_t int_bits);
  void setInterruptMapToINT2(uint8_t int_bits);

  void dumpFXRegisters(const char *msg, bool compare);

protected:
  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface

private:
  fxos8700SensorMode_t _mode = HYBRID_MODE;
  fxos8700AccelRange_t _range = ACCEL_RANGE_2G;
  fxos8700ODR_t _rate = ODR_100HZ;
  fxos8700MagOSR_t _ratio = MAG_OSR_7;
  int32_t _accelSensorID;
  int32_t _magSensorID;

  short bytesToHalfWord(const char labels[], unsigned int index, bool signed14);
  void formatBin(byte value, char *out);
};

#endif
