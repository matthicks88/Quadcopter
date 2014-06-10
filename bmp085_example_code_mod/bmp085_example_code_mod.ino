/* BMP085 Extended Example Code
  by: Jim Lindblom
  SparkFun Electronics
  date: 1/18/11
  updated: 2/26/13
  license: CC BY-SA v3.0 - http://creativecommons.org/licenses/by-sa/3.0/
  
  Get pressure and temperature from the BMP085 and calculate 
  altitude. Serial.print it out at 9600 baud to serial monitor.

  Update (7/19/11): I've heard folks may be encountering issues
  with this code, who're running an Arduino at 8MHz. If you're 
  using an Arduino Pro 3.3V/8MHz, or the like, you may need to 
  increase some of the delays in the bmp085ReadUP and 
  bmp085ReadUT functions.
*/

#include <Wire.h>

#define BMP085_ADDRESS 0x77  // I2C address of BMP085
#define HMC5883_ADDRESS 0x1E
#define MPU6050_AUX_VDDIO          0x01   // R/W
#define MPU6050_SMPLRT_DIV         0x19   // R/W
#define MPU6050_CONFIG             0x1A   // R/W
#define MPU6050_GYRO_CONFIG        0x1B   // R/W
#define MPU6050_ACCEL_CONFIG       0x1C   // R/W
#define MPU6050_FF_THR             0x1D   // R/W
#define MPU6050_FF_DUR             0x1E   // R/W
#define MPU6050_MOT_THR            0x1F   // R/W
#define MPU6050_MOT_DUR            0x20   // R/W
#define MPU6050_ZRMOT_THR          0x21   // R/W
#define MPU6050_ZRMOT_DUR          0x22   // R/W
#define MPU6050_FIFO_EN            0x23   // R/W
#define MPU6050_I2C_MST_CTRL       0x24   // R/W
#define MPU6050_I2C_SLV0_ADDR      0x25   // R/W
#define MPU6050_I2C_SLV0_REG       0x26   // R/W
#define MPU6050_I2C_SLV0_CTRL      0x27   // R/W
#define MPU6050_I2C_SLV1_ADDR      0x28   // R/W
#define MPU6050_I2C_SLV1_REG       0x29   // R/W
#define MPU6050_I2C_SLV1_CTRL      0x2A   // R/W
#define MPU6050_I2C_SLV2_ADDR      0x2B   // R/W
#define MPU6050_I2C_SLV2_REG       0x2C   // R/W
#define MPU6050_I2C_SLV2_CTRL      0x2D   // R/W
#define MPU6050_I2C_SLV3_ADDR      0x2E   // R/W
#define MPU6050_I2C_SLV3_REG       0x2F   // R/W
#define MPU6050_I2C_SLV3_CTRL      0x30   // R/W
#define MPU6050_I2C_SLV4_ADDR      0x31   // R/W
#define MPU6050_I2C_SLV4_REG       0x32   // R/W
#define MPU6050_I2C_SLV4_DO        0x33   // R/W
#define MPU6050_I2C_SLV4_CTRL      0x34   // R/W
#define MPU6050_I2C_SLV4_DI        0x35   // R  
#define MPU6050_I2C_MST_STATUS     0x36   // R
#define MPU6050_INT_PIN_CFG        0x37   // R/W
#define MPU6050_INT_ENABLE         0x38   // R/W
#define MPU6050_INT_STATUS         0x3A   // R  
#define MPU6050_ACCEL_XOUT_H       0x3B   // R  
#define MPU6050_ACCEL_XOUT_L       0x3C   // R  
#define MPU6050_ACCEL_YOUT_H       0x3D   // R  
#define MPU6050_ACCEL_YOUT_L       0x3E   // R  
#define MPU6050_ACCEL_ZOUT_H       0x3F   // R  
#define MPU6050_ACCEL_ZOUT_L       0x40   // R  
#define MPU6050_TEMP_OUT_H         0x41   // R  
#define MPU6050_TEMP_OUT_L         0x42   // R  
#define MPU6050_GYRO_XOUT_H        0x43   // R  
#define MPU6050_GYRO_XOUT_L        0x44   // R  
#define MPU6050_GYRO_YOUT_H        0x45   // R  
#define MPU6050_GYRO_YOUT_L        0x46   // R  
#define MPU6050_GYRO_ZOUT_H        0x47   // R  
#define MPU6050_GYRO_ZOUT_L        0x48   // R  
#define MPU6050_EXT_SENS_DATA_00   0x49   // R  
#define MPU6050_EXT_SENS_DATA_01   0x4A   // R  
#define MPU6050_EXT_SENS_DATA_02   0x4B   // R  
#define MPU6050_EXT_SENS_DATA_03   0x4C   // R  
#define MPU6050_EXT_SENS_DATA_04   0x4D   // R  
#define MPU6050_EXT_SENS_DATA_05   0x4E   // R  
#define MPU6050_EXT_SENS_DATA_06   0x4F   // R  
#define MPU6050_EXT_SENS_DATA_07   0x50   // R  
#define MPU6050_EXT_SENS_DATA_08   0x51   // R  
#define MPU6050_EXT_SENS_DATA_09   0x52   // R  
#define MPU6050_EXT_SENS_DATA_10   0x53   // R  
#define MPU6050_EXT_SENS_DATA_11   0x54   // R  
#define MPU6050_EXT_SENS_DATA_12   0x55   // R  
#define MPU6050_EXT_SENS_DATA_13   0x56   // R  
#define MPU6050_EXT_SENS_DATA_14   0x57   // R  
#define MPU6050_EXT_SENS_DATA_15   0x58   // R  
#define MPU6050_EXT_SENS_DATA_16   0x59   // R  
#define MPU6050_EXT_SENS_DATA_17   0x5A   // R  
#define MPU6050_EXT_SENS_DATA_18   0x5B   // R  
#define MPU6050_EXT_SENS_DATA_19   0x5C   // R  
#define MPU6050_EXT_SENS_DATA_20   0x5D   // R  
#define MPU6050_EXT_SENS_DATA_21   0x5E   // R  
#define MPU6050_EXT_SENS_DATA_22   0x5F   // R  
#define MPU6050_EXT_SENS_DATA_23   0x60   // R  
#define MPU6050_MOT_DETECT_STATUS  0x61   // R  
#define MPU6050_I2C_SLV0_DO        0x63   // R/W
#define MPU6050_I2C_SLV1_DO        0x64   // R/W
#define MPU6050_I2C_SLV2_DO        0x65   // R/W
#define MPU6050_I2C_SLV3_DO        0x66   // R/W
#define MPU6050_I2C_MST_DELAY_CTRL 0x67   // R/W
#define MPU6050_SIGNAL_PATH_RESET  0x68   // R/W
#define MPU6050_MOT_DETECT_CTRL    0x69   // R/W
#define MPU6050_USER_CTRL          0x6A   // R/W
#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_PWR_MGMT_2         0x6C   // R/W
#define MPU6050_FIFO_COUNTH        0x72   // R/W
#define MPU6050_FIFO_COUNTL        0x73   // R/W
#define MPU6050_FIFO_R_W           0x74   // R/W
#define MPU6050_WHO_AM_I           0x75   // R



// Defines for the bits, to be able to change 
// between bit number and binary definition.
// By using the bit number, programming the sensor 
// is like programming the AVR microcontroller.
// But instead of using "(1<<X)", or "_BV(X)", 
// the Arduino "bit(X)" is used.
#define MPU6050_D0 0
#define MPU6050_D1 1
#define MPU6050_D2 2
#define MPU6050_D3 3
#define MPU6050_D4 4
#define MPU6050_D5 5
#define MPU6050_D6 6
#define MPU6050_D7 7

// AUX_VDDIO Register
#define MPU6050_AUX_VDDIO MPU6050_D7  // I2C high: 1=VDD, 0=VLOGIC

// CONFIG Register
// DLPF is Digital Low Pass Filter for both gyro and accelerometers.
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_DLPF_CFG0     MPU6050_D0
#define MPU6050_DLPF_CFG1     MPU6050_D1
#define MPU6050_DLPF_CFG2     MPU6050_D2
#define MPU6050_EXT_SYNC_SET0 MPU6050_D3
#define MPU6050_EXT_SYNC_SET1 MPU6050_D4
#define MPU6050_EXT_SYNC_SET2 MPU6050_D5

// Combined definitions for the EXT_SYNC_SET values
#define MPU6050_EXT_SYNC_SET_0 (0)
#define MPU6050_EXT_SYNC_SET_1 (bit(MPU6050_EXT_SYNC_SET0))
#define MPU6050_EXT_SYNC_SET_2 (bit(MPU6050_EXT_SYNC_SET1))
#define MPU6050_EXT_SYNC_SET_3 (bit(MPU6050_EXT_SYNC_SET1)|bit(MPU6050_EXT_SYNC_SET0))
#define MPU6050_EXT_SYNC_SET_4 (bit(MPU6050_EXT_SYNC_SET2))
#define MPU6050_EXT_SYNC_SET_5 (bit(MPU6050_EXT_SYNC_SET2)|bit(MPU6050_EXT_SYNC_SET0))
#define MPU6050_EXT_SYNC_SET_6 (bit(MPU6050_EXT_SYNC_SET2)|bit(MPU6050_EXT_SYNC_SET1))
#define MPU6050_EXT_SYNC_SET_7 (bit(MPU6050_EXT_SYNC_SET2)|bit(MPU6050_EXT_SYNC_SET1)|bit(MPU6050_EXT_SYNC_SET0))

// Alternative names for the combined definitions.
#define MPU6050_EXT_SYNC_DISABLED     MPU6050_EXT_SYNC_SET_0
#define MPU6050_EXT_SYNC_TEMP_OUT_L   MPU6050_EXT_SYNC_SET_1
#define MPU6050_EXT_SYNC_GYRO_XOUT_L  MPU6050_EXT_SYNC_SET_2
#define MPU6050_EXT_SYNC_GYRO_YOUT_L  MPU6050_EXT_SYNC_SET_3
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L  MPU6050_EXT_SYNC_SET_4
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L MPU6050_EXT_SYNC_SET_5
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L MPU6050_EXT_SYNC_SET_6
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L MPU6050_EXT_SYNC_SET_7

// Combined definitions for the DLPF_CFG values
#define MPU6050_DLPF_CFG_0 (0)
#define MPU6050_DLPF_CFG_1 (bit(MPU6050_DLPF_CFG0))
#define MPU6050_DLPF_CFG_2 (bit(MPU6050_DLPF_CFG1))
#define MPU6050_DLPF_CFG_3 (bit(MPU6050_DLPF_CFG1)|bit(MPU6050_DLPF_CFG0))
#define MPU6050_DLPF_CFG_4 (bit(MPU6050_DLPF_CFG2))
#define MPU6050_DLPF_CFG_5 (bit(MPU6050_DLPF_CFG2)|bit(MPU6050_DLPF_CFG0))
#define MPU6050_DLPF_CFG_6 (bit(MPU6050_DLPF_CFG2)|bit(MPU6050_DLPF_CFG1))
#define MPU6050_DLPF_CFG_7 (bit(MPU6050_DLPF_CFG2)|bit(MPU6050_DLPF_CFG1)|bit(MPU6050_DLPF_CFG0))

// Alternative names for the combined definitions
// This name uses the bandwidth (Hz) for the accelometer,
// for the gyro the bandwidth is almost the same.
#define MPU6050_DLPF_260HZ    MPU6050_DLPF_CFG_0
#define MPU6050_DLPF_184HZ    MPU6050_DLPF_CFG_1
#define MPU6050_DLPF_94HZ     MPU6050_DLPF_CFG_2
#define MPU6050_DLPF_44HZ     MPU6050_DLPF_CFG_3
#define MPU6050_DLPF_21HZ     MPU6050_DLPF_CFG_4
#define MPU6050_DLPF_10HZ     MPU6050_DLPF_CFG_5
#define MPU6050_DLPF_5HZ      MPU6050_DLPF_CFG_6
#define MPU6050_DLPF_RESERVED MPU6050_DLPF_CFG_7

// GYRO_CONFIG Register
// The XG_ST, YG_ST, ZG_ST are bits for selftest.
// The FS_SEL sets the range for the gyro.
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_FS_SEL0 MPU6050_D3
#define MPU6050_FS_SEL1 MPU6050_D4
#define MPU6050_ZG_ST   MPU6050_D5
#define MPU6050_YG_ST   MPU6050_D6
#define MPU6050_XG_ST   MPU6050_D7

// Combined definitions for the FS_SEL values
#define MPU6050_FS_SEL_0 (0)
#define MPU6050_FS_SEL_1 (bit(MPU6050_FS_SEL0))
#define MPU6050_FS_SEL_2 (bit(MPU6050_FS_SEL1))
#define MPU6050_FS_SEL_3 (bit(MPU6050_FS_SEL1)|bit(MPU6050_FS_SEL0))

// Alternative names for the combined definitions
// The name uses the range in degrees per second.
#define MPU6050_FS_SEL_250  MPU6050_FS_SEL_0
#define MPU6050_FS_SEL_500  MPU6050_FS_SEL_1
#define MPU6050_FS_SEL_1000 MPU6050_FS_SEL_2
#define MPU6050_FS_SEL_2000 MPU6050_FS_SEL_3

// ACCEL_CONFIG Register
// The XA_ST, YA_ST, ZA_ST are bits for selftest.
// The AFS_SEL sets the range for the accelerometer.
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_ACCEL_HPF0 MPU6050_D0
#define MPU6050_ACCEL_HPF1 MPU6050_D1
#define MPU6050_ACCEL_HPF2 MPU6050_D2
#define MPU6050_AFS_SEL0   MPU6050_D3
#define MPU6050_AFS_SEL1   MPU6050_D4
#define MPU6050_ZA_ST      MPU6050_D5
#define MPU6050_YA_ST      MPU6050_D6
#define MPU6050_XA_ST      MPU6050_D7

// Combined definitions for the ACCEL_HPF values
#define MPU6050_ACCEL_HPF_0 (0)
#define MPU6050_ACCEL_HPF_1 (bit(MPU6050_ACCEL_HPF0))
#define MPU6050_ACCEL_HPF_2 (bit(MPU6050_ACCEL_HPF1))
#define MPU6050_ACCEL_HPF_3 (bit(MPU6050_ACCEL_HPF1)|bit(MPU6050_ACCEL_HPF0))
#define MPU6050_ACCEL_HPF_4 (bit(MPU6050_ACCEL_HPF2))
#define MPU6050_ACCEL_HPF_7 (bit(MPU6050_ACCEL_HPF2)|bit(MPU6050_ACCEL_HPF1)|bit(MPU6050_ACCEL_HPF0))

// Alternative names for the combined definitions
// The name uses the Cut-off frequency.
#define MPU6050_ACCEL_HPF_RESET  MPU6050_ACCEL_HPF_0
#define MPU6050_ACCEL_HPF_5HZ    MPU6050_ACCEL_HPF_1
#define MPU6050_ACCEL_HPF_2_5HZ  MPU6050_ACCEL_HPF_2
#define MPU6050_ACCEL_HPF_1_25HZ MPU6050_ACCEL_HPF_3
#define MPU6050_ACCEL_HPF_0_63HZ MPU6050_ACCEL_HPF_4
#define MPU6050_ACCEL_HPF_HOLD   MPU6050_ACCEL_HPF_7

// Combined definitions for the AFS_SEL values
#define MPU6050_AFS_SEL_0 (0)
#define MPU6050_AFS_SEL_1 (bit(MPU6050_AFS_SEL0))
#define MPU6050_AFS_SEL_2 (bit(MPU6050_AFS_SEL1))
#define MPU6050_AFS_SEL_3 (bit(MPU6050_AFS_SEL1)|bit(MPU6050_AFS_SEL0))

// Alternative names for the combined definitions
// The name uses the full scale range for the accelerometer.
#define MPU6050_AFS_SEL_2G  MPU6050_AFS_SEL_0
#define MPU6050_AFS_SEL_4G  MPU6050_AFS_SEL_1
#define MPU6050_AFS_SEL_8G  MPU6050_AFS_SEL_2
#define MPU6050_AFS_SEL_16G MPU6050_AFS_SEL_3

// FIFO_EN Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_SLV0_FIFO_EN  MPU6050_D0
#define MPU6050_SLV1_FIFO_EN  MPU6050_D1
#define MPU6050_SLV2_FIFO_EN  MPU6050_D2
#define MPU6050_ACCEL_FIFO_EN MPU6050_D3
#define MPU6050_ZG_FIFO_EN    MPU6050_D4
#define MPU6050_YG_FIFO_EN    MPU6050_D5
#define MPU6050_XG_FIFO_EN    MPU6050_D6
#define MPU6050_TEMP_FIFO_EN  MPU6050_D7

// I2C_MST_CTRL Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_MST_CLK0  MPU6050_D0
#define MPU6050_I2C_MST_CLK1  MPU6050_D1
#define MPU6050_I2C_MST_CLK2  MPU6050_D2
#define MPU6050_I2C_MST_CLK3  MPU6050_D3
#define MPU6050_I2C_MST_P_NSR MPU6050_D4
#define MPU6050_SLV_3_FIFO_EN MPU6050_D5
#define MPU6050_WAIT_FOR_ES   MPU6050_D6
#define MPU6050_MULT_MST_EN   MPU6050_D7

// Combined definitions for the I2C_MST_CLK
#define MPU6050_I2C_MST_CLK_0 (0)
#define MPU6050_I2C_MST_CLK_1  (bit(MPU6050_I2C_MST_CLK0))
#define MPU6050_I2C_MST_CLK_2  (bit(MPU6050_I2C_MST_CLK1))
#define MPU6050_I2C_MST_CLK_3  (bit(MPU6050_I2C_MST_CLK1)|bit(MPU6050_I2C_MST_CLK0))
#define MPU6050_I2C_MST_CLK_4  (bit(MPU6050_I2C_MST_CLK2))
#define MPU6050_I2C_MST_CLK_5  (bit(MPU6050_I2C_MST_CLK2)|bit(MPU6050_I2C_MST_CLK0))
#define MPU6050_I2C_MST_CLK_6  (bit(MPU6050_I2C_MST_CLK2)|bit(MPU6050_I2C_MST_CLK1))
#define MPU6050_I2C_MST_CLK_7  (bit(MPU6050_I2C_MST_CLK2)|bit(MPU6050_I2C_MST_CLK1)|bit(MPU6050_I2C_MST_CLK0))
#define MPU6050_I2C_MST_CLK_8  (bit(MPU6050_I2C_MST_CLK3))
#define MPU6050_I2C_MST_CLK_9  (bit(MPU6050_I2C_MST_CLK3)|bit(MPU6050_I2C_MST_CLK0))
#define MPU6050_I2C_MST_CLK_10 (bit(MPU6050_I2C_MST_CLK3)|bit(MPU6050_I2C_MST_CLK1))
#define MPU6050_I2C_MST_CLK_11 (bit(MPU6050_I2C_MST_CLK3)|bit(MPU6050_I2C_MST_CLK1)|bit(MPU6050_I2C_MST_CLK0))
#define MPU6050_I2C_MST_CLK_12 (bit(MPU6050_I2C_MST_CLK3)|bit(MPU6050_I2C_MST_CLK2))
#define MPU6050_I2C_MST_CLK_13 (bit(MPU6050_I2C_MST_CLK3)|bit(MPU6050_I2C_MST_CLK2)|bit(MPU6050_I2C_MST_CLK0))
#define MPU6050_I2C_MST_CLK_14 (bit(MPU6050_I2C_MST_CLK3)|bit(MPU6050_I2C_MST_CLK2)|bit(MPU6050_I2C_MST_CLK1))
#define MPU6050_I2C_MST_CLK_15 (bit(MPU6050_I2C_MST_CLK3)|bit(MPU6050_I2C_MST_CLK2)|bit(MPU6050_I2C_MST_CLK1)|bit(MPU6050_I2C_MST_CLK0))

// Alternative names for the combined definitions
// The names uses I2C Master Clock Speed in kHz.
#define MPU6050_I2C_MST_CLK_348KHZ MPU6050_I2C_MST_CLK_0
#define MPU6050_I2C_MST_CLK_333KHZ MPU6050_I2C_MST_CLK_1
#define MPU6050_I2C_MST_CLK_320KHZ MPU6050_I2C_MST_CLK_2
#define MPU6050_I2C_MST_CLK_308KHZ MPU6050_I2C_MST_CLK_3
#define MPU6050_I2C_MST_CLK_296KHZ MPU6050_I2C_MST_CLK_4
#define MPU6050_I2C_MST_CLK_286KHZ MPU6050_I2C_MST_CLK_5
#define MPU6050_I2C_MST_CLK_276KHZ MPU6050_I2C_MST_CLK_6
#define MPU6050_I2C_MST_CLK_267KHZ MPU6050_I2C_MST_CLK_7
#define MPU6050_I2C_MST_CLK_258KHZ MPU6050_I2C_MST_CLK_8
#define MPU6050_I2C_MST_CLK_500KHZ MPU6050_I2C_MST_CLK_9
#define MPU6050_I2C_MST_CLK_471KHZ MPU6050_I2C_MST_CLK_10
#define MPU6050_I2C_MST_CLK_444KHZ MPU6050_I2C_MST_CLK_11
#define MPU6050_I2C_MST_CLK_421KHZ MPU6050_I2C_MST_CLK_12
#define MPU6050_I2C_MST_CLK_400KHZ MPU6050_I2C_MST_CLK_13
#define MPU6050_I2C_MST_CLK_381KHZ MPU6050_I2C_MST_CLK_14
#define MPU6050_I2C_MST_CLK_364KHZ MPU6050_I2C_MST_CLK_15

// I2C_SLV0_ADDR Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_SLV0_RW MPU6050_D7

// I2C_SLV0_CTRL Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_SLV0_LEN0    MPU6050_D0
#define MPU6050_I2C_SLV0_LEN1    MPU6050_D1
#define MPU6050_I2C_SLV0_LEN2    MPU6050_D2
#define MPU6050_I2C_SLV0_LEN3    MPU6050_D3
#define MPU6050_I2C_SLV0_GRP     MPU6050_D4
#define MPU6050_I2C_SLV0_REG_DIS MPU6050_D5
#define MPU6050_I2C_SLV0_BYTE_SW MPU6050_D6
#define MPU6050_I2C_SLV0_EN      MPU6050_D7

// A mask for the length
#define MPU6050_I2C_SLV0_LEN_MASK 0x0F

// I2C_SLV1_ADDR Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_SLV1_RW MPU6050_D7

// I2C_SLV1_CTRL Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_SLV1_LEN0    MPU6050_D0
#define MPU6050_I2C_SLV1_LEN1    MPU6050_D1
#define MPU6050_I2C_SLV1_LEN2    MPU6050_D2
#define MPU6050_I2C_SLV1_LEN3    MPU6050_D3
#define MPU6050_I2C_SLV1_GRP     MPU6050_D4
#define MPU6050_I2C_SLV1_REG_DIS MPU6050_D5
#define MPU6050_I2C_SLV1_BYTE_SW MPU6050_D6
#define MPU6050_I2C_SLV1_EN      MPU6050_D7

// A mask for the length
#define MPU6050_I2C_SLV1_LEN_MASK 0x0F

// I2C_SLV2_ADDR Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_SLV2_RW MPU6050_D7

// I2C_SLV2_CTRL Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_SLV2_LEN0    MPU6050_D0
#define MPU6050_I2C_SLV2_LEN1    MPU6050_D1
#define MPU6050_I2C_SLV2_LEN2    MPU6050_D2
#define MPU6050_I2C_SLV2_LEN3    MPU6050_D3
#define MPU6050_I2C_SLV2_GRP     MPU6050_D4
#define MPU6050_I2C_SLV2_REG_DIS MPU6050_D5
#define MPU6050_I2C_SLV2_BYTE_SW MPU6050_D6
#define MPU6050_I2C_SLV2_EN      MPU6050_D7

// A mask for the length
#define MPU6050_I2C_SLV2_LEN_MASK 0x0F

// I2C_SLV3_ADDR Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_SLV3_RW MPU6050_D7

// I2C_SLV3_CTRL Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_SLV3_LEN0    MPU6050_D0
#define MPU6050_I2C_SLV3_LEN1    MPU6050_D1
#define MPU6050_I2C_SLV3_LEN2    MPU6050_D2
#define MPU6050_I2C_SLV3_LEN3    MPU6050_D3
#define MPU6050_I2C_SLV3_GRP     MPU6050_D4
#define MPU6050_I2C_SLV3_REG_DIS MPU6050_D5
#define MPU6050_I2C_SLV3_BYTE_SW MPU6050_D6
#define MPU6050_I2C_SLV3_EN      MPU6050_D7

// A mask for the length
#define MPU6050_I2C_SLV3_LEN_MASK 0x0F

// I2C_SLV4_ADDR Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_SLV4_RW MPU6050_D7

// I2C_SLV4_CTRL Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_MST_DLY0     MPU6050_D0
#define MPU6050_I2C_MST_DLY1     MPU6050_D1
#define MPU6050_I2C_MST_DLY2     MPU6050_D2
#define MPU6050_I2C_MST_DLY3     MPU6050_D3
#define MPU6050_I2C_MST_DLY4     MPU6050_D4
#define MPU6050_I2C_SLV4_REG_DIS MPU6050_D5
#define MPU6050_I2C_SLV4_INT_EN  MPU6050_D6
#define MPU6050_I2C_SLV4_EN      MPU6050_D7

// A mask for the delay
#define MPU6050_I2C_MST_DLY_MASK 0x1F

// I2C_MST_STATUS Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_SLV0_NACK MPU6050_D0
#define MPU6050_I2C_SLV1_NACK MPU6050_D1
#define MPU6050_I2C_SLV2_NACK MPU6050_D2
#define MPU6050_I2C_SLV3_NACK MPU6050_D3
#define MPU6050_I2C_SLV4_NACK MPU6050_D4
#define MPU6050_I2C_LOST_ARB  MPU6050_D5
#define MPU6050_I2C_SLV4_DONE MPU6050_D6
#define MPU6050_PASS_THROUGH  MPU6050_D7

// I2C_PIN_CFG Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_CLKOUT_EN       MPU6050_D0
#define MPU6050_I2C_BYPASS_EN   MPU6050_D1
#define MPU6050_FSYNC_INT_EN    MPU6050_D2
#define MPU6050_FSYNC_INT_LEVEL MPU6050_D3
#define MPU6050_INT_RD_CLEAR    MPU6050_D4
#define MPU6050_LATCH_INT_EN    MPU6050_D5
#define MPU6050_INT_OPEN        MPU6050_D6
#define MPU6050_INT_LEVEL       MPU6050_D7

// INT_ENABLE Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_DATA_RDY_EN    MPU6050_D0
#define MPU6050_I2C_MST_INT_EN MPU6050_D3
#define MPU6050_FIFO_OFLOW_EN  MPU6050_D4
#define MPU6050_ZMOT_EN        MPU6050_D5
#define MPU6050_MOT_EN         MPU6050_D6
#define MPU6050_FF_EN          MPU6050_D7

// INT_STATUS Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_DATA_RDY_INT   MPU6050_D0
#define MPU6050_I2C_MST_INT    MPU6050_D3
#define MPU6050_FIFO_OFLOW_INT MPU6050_D4
#define MPU6050_ZMOT_INT       MPU6050_D5
#define MPU6050_MOT_INT        MPU6050_D6
#define MPU6050_FF_INT         MPU6050_D7

// MOT_DETECT_STATUS Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_MOT_ZRMOT MPU6050_D0
#define MPU6050_MOT_ZPOS  MPU6050_D2
#define MPU6050_MOT_ZNEG  MPU6050_D3
#define MPU6050_MOT_YPOS  MPU6050_D4
#define MPU6050_MOT_YNEG  MPU6050_D5
#define MPU6050_MOT_XPOS  MPU6050_D6
#define MPU6050_MOT_XNEG  MPU6050_D7

// IC2_MST_DELAY_CTRL Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_SLV0_DLY_EN MPU6050_D0
#define MPU6050_I2C_SLV1_DLY_EN MPU6050_D1
#define MPU6050_I2C_SLV2_DLY_EN MPU6050_D2
#define MPU6050_I2C_SLV3_DLY_EN MPU6050_D3
#define MPU6050_I2C_SLV4_DLY_EN MPU6050_D4
#define MPU6050_DELAY_ES_SHADOW MPU6050_D7

// SIGNAL_PATH_RESET Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_TEMP_RESET  MPU6050_D0
#define MPU6050_ACCEL_RESET MPU6050_D1
#define MPU6050_GYRO_RESET  MPU6050_D2

// MOT_DETECT_CTRL Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_MOT_COUNT0      MPU6050_D0
#define MPU6050_MOT_COUNT1      MPU6050_D1
#define MPU6050_FF_COUNT0       MPU6050_D2
#define MPU6050_FF_COUNT1       MPU6050_D3
#define MPU6050_ACCEL_ON_DELAY0 MPU6050_D4
#define MPU6050_ACCEL_ON_DELAY1 MPU6050_D5

// Combined definitions for the MOT_COUNT
#define MPU6050_MOT_COUNT_0 (0)
#define MPU6050_MOT_COUNT_1 (bit(MPU6050_MOT_COUNT0))
#define MPU6050_MOT_COUNT_2 (bit(MPU6050_MOT_COUNT1))
#define MPU6050_MOT_COUNT_3 (bit(MPU6050_MOT_COUNT1)|bit(MPU6050_MOT_COUNT0))

// Alternative names for the combined definitions
#define MPU6050_MOT_COUNT_RESET MPU6050_MOT_COUNT_0

// Combined definitions for the FF_COUNT
#define MPU6050_FF_COUNT_0 (0)
#define MPU6050_FF_COUNT_1 (bit(MPU6050_FF_COUNT0))
#define MPU6050_FF_COUNT_2 (bit(MPU6050_FF_COUNT1))
#define MPU6050_FF_COUNT_3 (bit(MPU6050_FF_COUNT1)|bit(MPU6050_FF_COUNT0))

// Alternative names for the combined definitions
#define MPU6050_FF_COUNT_RESET MPU6050_FF_COUNT_0

// Combined definitions for the ACCEL_ON_DELAY
#define MPU6050_ACCEL_ON_DELAY_0 (0)
#define MPU6050_ACCEL_ON_DELAY_1 (bit(MPU6050_ACCEL_ON_DELAY0))
#define MPU6050_ACCEL_ON_DELAY_2 (bit(MPU6050_ACCEL_ON_DELAY1))
#define MPU6050_ACCEL_ON_DELAY_3 (bit(MPU6050_ACCEL_ON_DELAY1)|bit(MPU6050_ACCEL_ON_DELAY0))

// Alternative names for the ACCEL_ON_DELAY
#define MPU6050_ACCEL_ON_DELAY_0MS MPU6050_ACCEL_ON_DELAY_0
#define MPU6050_ACCEL_ON_DELAY_1MS MPU6050_ACCEL_ON_DELAY_1
#define MPU6050_ACCEL_ON_DELAY_2MS MPU6050_ACCEL_ON_DELAY_2
#define MPU6050_ACCEL_ON_DELAY_3MS MPU6050_ACCEL_ON_DELAY_3

// USER_CTRL Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_SIG_COND_RESET MPU6050_D0
#define MPU6050_I2C_MST_RESET  MPU6050_D1
#define MPU6050_FIFO_RESET     MPU6050_D2
#define MPU6050_I2C_IF_DIS     MPU6050_D4   // must be 0 for MPU-6050
#define MPU6050_I2C_MST_EN     MPU6050_D5
#define MPU6050_FIFO_EN        MPU6050_D6

// PWR_MGMT_1 Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_CLKSEL0      MPU6050_D0
#define MPU6050_CLKSEL1      MPU6050_D1
#define MPU6050_CLKSEL2      MPU6050_D2
#define MPU6050_TEMP_DIS     MPU6050_D3    // 1: disable temperature sensor
#define MPU6050_CYCLE        MPU6050_D5    // 1: sample and sleep
#define MPU6050_SLEEP        MPU6050_D6    // 1: sleep mode
#define MPU6050_DEVICE_RESET MPU6050_D7    // 1: reset to default values

// Combined definitions for the CLKSEL
#define MPU6050_CLKSEL_0 (0)
#define MPU6050_CLKSEL_1 (bit(MPU6050_CLKSEL0))
#define MPU6050_CLKSEL_2 (bit(MPU6050_CLKSEL1))
#define MPU6050_CLKSEL_3 (bit(MPU6050_CLKSEL1)|bit(MPU6050_CLKSEL0))
#define MPU6050_CLKSEL_4 (bit(MPU6050_CLKSEL2))
#define MPU6050_CLKSEL_5 (bit(MPU6050_CLKSEL2)|bit(MPU6050_CLKSEL0))
#define MPU6050_CLKSEL_6 (bit(MPU6050_CLKSEL2)|bit(MPU6050_CLKSEL1))
#define MPU6050_CLKSEL_7 (bit(MPU6050_CLKSEL2)|bit(MPU6050_CLKSEL1)|bit(MPU6050_CLKSEL0))

// Alternative names for the combined definitions
#define MPU6050_CLKSEL_INTERNAL    MPU6050_CLKSEL_0
#define MPU6050_CLKSEL_X           MPU6050_CLKSEL_1
#define MPU6050_CLKSEL_Y           MPU6050_CLKSEL_2
#define MPU6050_CLKSEL_Z           MPU6050_CLKSEL_3
#define MPU6050_CLKSEL_EXT_32KHZ   MPU6050_CLKSEL_4
#define MPU6050_CLKSEL_EXT_19_2MHZ MPU6050_CLKSEL_5
#define MPU6050_CLKSEL_RESERVED    MPU6050_CLKSEL_6
#define MPU6050_CLKSEL_STOP        MPU6050_CLKSEL_7

// PWR_MGMT_2 Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_STBY_ZG       MPU6050_D0
#define MPU6050_STBY_YG       MPU6050_D1
#define MPU6050_STBY_XG       MPU6050_D2
#define MPU6050_STBY_ZA       MPU6050_D3
#define MPU6050_STBY_YA       MPU6050_D4
#define MPU6050_STBY_XA       MPU6050_D5
#define MPU6050_LP_WAKE_CTRL0 MPU6050_D6
#define MPU6050_LP_WAKE_CTRL1 MPU6050_D7

// Combined definitions for the LP_WAKE_CTRL
#define MPU6050_LP_WAKE_CTRL_0 (0)
#define MPU6050_LP_WAKE_CTRL_1 (bit(MPU6050_LP_WAKE_CTRL0))
#define MPU6050_LP_WAKE_CTRL_2 (bit(MPU6050_LP_WAKE_CTRL1))
#define MPU6050_LP_WAKE_CTRL_3 (bit(MPU6050_LP_WAKE_CTRL1)|bit(MPU6050_LP_WAKE_CTRL0))

// Alternative names for the combined definitions
// The names uses the Wake-up Frequency.
#define MPU6050_LP_WAKE_1_25HZ MPU6050_LP_WAKE_CTRL_0
#define MPU6050_LP_WAKE_2_5HZ  MPU6050_LP_WAKE_CTRL_1
#define MPU6050_LP_WAKE_5HZ    MPU6050_LP_WAKE_CTRL_2
#define MPU6050_LP_WAKE_10HZ   MPU6050_LP_WAKE_CTRL_3


// Default I2C address for the MPU-6050 is 0x68.
// But only if the AD0 pin is low.
// Some sensor boards have AD0 high, and the
// I2C address thus becomes 0x69.
#define MPU6050_I2C_ADDRESS 0x68


float calcaccelx;
float calcaccely;
float calcaccelz;
long accfactor = 16384;
float initrotx;
float initroty;
float initrotz;

float sumrotx =0;
float sumroty =0;
float sumrotz =0;

float rotx;
float roty;
float rotz;
float initcalc = 0;
float loopcount = 0;
float rotvalx = 0;
// Declaring an union for the registers and the axis values.
// The byte order does not match the byte order of 
// the compiler and AVR chip.
// The AVR chip (on the Arduino board) has the Low Byte 
// at the lower address.
// But the MPU-6050 has a different order: High Byte at
// lower address, so that has to be corrected.
// The register part "reg" is only used internally, 
// and are swapped in code.
typedef union accel_t_gyro_union
{
  struct
  {
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
    uint8_t t_h;
    uint8_t t_l;
    uint8_t x_gyro_h;
    uint8_t x_gyro_l;
    uint8_t y_gyro_h;
    uint8_t y_gyro_l;
    uint8_t z_gyro_h;
    uint8_t z_gyro_l;
  } reg;
  struct 
  {
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int16_t temperature;
    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;
  } value;
};


const unsigned char OSS = 3;  // Oversampling Setting

// Calibration values
int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;

// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 

short temperature;
long pressure;

// Use these for altitude conversions
const float p0 = 101325;     // Pressure at sea level (Pa)
float altitude;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  bmp085Calibration();
    Serial.begin(9600);
  Wire.begin();
  
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(HMC5883_ADDRESS); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
   int error;
  uint8_t c;


  Serial.begin(9600);
  Serial.println(F("InvenSense MPU-6050"));
  Serial.println(F("June 2012"));

  // Initialize the 'Wire' class for the I2C-bus.
  Wire.begin();


  // default at power-up:
  //    Gyro at 250 degrees second
  //    Acceleration at 2g
  //    Clock source at internal 8MHz
  //    The device is in sleep mode.
  //

  error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
  Serial.print(F("WHO_AM_I : "));
  Serial.print(c,HEX);
  Serial.print(F(", error = "));
  Serial.println(error,DEC);

  // According to the datasheet, the 'sleep' bit
  // should read a '1'.
  // That bit has to be cleared, since the sensor
  // is in sleep mode at power-up. 
  error = MPU6050_read (MPU6050_PWR_MGMT_1, &c, 1);
  Serial.print(F("PWR_MGMT_1 : "));
  Serial.print(c,HEX);
  Serial.print(F(", error = "));
  Serial.println(error,DEC);


  // Clear the 'sleep' bit to start the sensor.
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);

}

void loop()
{
  temperature = bmp085GetTemperature(bmp085ReadUT());
  pressure = bmp085GetPressure(bmp085ReadUP());
  altitude = (float)44330 * (1 - pow(((float) pressure/p0), 0.190295));

  //Serial.print("Temperature: ");
  //Serial.print(temperature, DEC);
  //Serial.println(" *0.1 deg C");
  //Serial.print("Pressure: ");
  //Serial.print(pressure, DEC);
  //Serial.println(" Pa");
  //Serial.print("Altitude: ");
  //Serial.print(altitude, 2);
  //Serial.println(" m");
  //Serial.println();
  
  int x,y,z; //triple axis data

  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(HMC5883_ADDRESS);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
 
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(HMC5883_ADDRESS, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }
  
  //Print out values of each axis
  //Serial.print("x: ");
  //Serial.print(x);
  //Serial.print("  y: ");
  //Serial.print(y);
  //Serial.print("  z: ");
  //Serial.println(z);

int error;
  double dT;
  accel_t_gyro_union accel_t_gyro;


  //Serial.println(F(""));
  //Serial.println(F("MPU-6050"));

  // Read the raw values.
  // Read 14 bytes at once, 
  // containing acceleration, temperature and gyro.
  // With the default settings of the MPU-6050,
  // there is no filter enabled, and the values
  // are not very stable.
  error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
  //Serial.print(F("Read accel, temp and gyro, error = "));
  //Serial.println(error,DEC);


  // Swap all high and low bytes.
  // After this, the registers values are swapped, 
  // so the structure name like x_accel_l does no 
  // longer contain the lower byte.
  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap

  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);


  // Print the raw acceleration values
  calcaccelx = accel_t_gyro.value.x_accel;
  calcaccelx=calcaccelx/16384;
  calcaccely = accel_t_gyro.value.y_accel;
  calcaccely=calcaccely/16384;
  calcaccelz = accel_t_gyro.value.z_accel;
  calcaccelz=calcaccelz/16384;
 //Serial.print(F("accel x,y,z: "));
 //Serial.print(calcaccelx);
  //Serial.print(F(", "));
  //Serial.print(calcaccely);
  //Serial.print(F(", "));
  //Serial.print(calcaccelz);
  //Serial.println(F(", "));


  // The temperature sensor is -40 to +85 degrees Celsius.
  // It is a signed integer.
  // According to the datasheet: 
  //   340 per degrees Celsius, -512 at 35 degrees.
  // At 0 degrees: -512 - (340 * 35) = -12412

  //Serial.print(F("temperature: "));
  dT = ( (double) accel_t_gyro.value.temperature + 12412.0) / 340.0;
  //Serial.print(dT, 3);
  //Serial.print(F(" degrees Celsius"));
  //Serial.println(F(""));


  // Print the raw gyro values.

  //Serial.print(F("gyro x,y,z : "));
  //Serial.print(accel_t_gyro.value.x_gyro, DEC);
  //Serial.print(F(", "));
  //Serial.print(accel_t_gyro.value.y_gyro, DEC);
  //Serial.print(F(", "));
  //Serial.print(accel_t_gyro.value.z_gyro, DEC);
  //Serial.print(F(", "));
  //Serial.println(F(""));
  
if (initcalc==0) initrotx=accel_t_gyro.value.x_gyro, initcalc=1;
rotvalx=accel_t_gyro.value.x_gyro-initrotx;  
if (abs(rotvalx) < 200) rotvalx=0;

sumrotx=sumrotx+rotvalx;
loopcount = loopcount +1;
if (loopcount==100) Serial.println(sumrotx),loopcount = 0;

 
}

// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
short bmp085GetTemperature(unsigned int ut)
{
  long x1, x2;
  
  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8)>>4);  
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up)
{
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;
  
  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;
  
  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
  
  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
    
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;
  
  return p;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;
    
  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  
  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT()
{
  unsigned int ut;
  
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  
  // Wait at least 4.5ms
  delay(5);
  
  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;
  
  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();
  
  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));
  
  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);
  
  // Wait for data to become available
  while(Wire.available() < 3)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();
  
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);
  
  return up;
}

int MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}


// --------------------------------------------------------
// MPU6050_write
//
// This is a common function to write multiple bytes to an I2C device.
//
// If only a single register is written,
// use the function MPU_6050_write_reg().
//
// Parameters:
//   start : Start address, use a define for the register
//   pData : A pointer to the data to write.
//   size  : The number of bytes to write.
//
// If only a single register is written, a pointer
// to the data has to be used, and the size is
// a single byte:
//   int data = 0;        // the data to write
//   MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);
//
int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}

// --------------------------------------------------------
// MPU6050_write_reg
//
// An extra function to write a single register.
// It is just a wrapper around the MPU_6050_write()
// function, and it is only a convenient function
// to make it easier to write a single register.
//
int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;

  error = MPU6050_write(reg, &data, 1);

  return (error);
}

