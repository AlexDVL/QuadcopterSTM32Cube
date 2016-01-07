#include "stm32f1xx_hal.h"
#include "MPU6050.h"

uint8_t settingsMPU6050[] = {
    // 8000/1+7 = 1000Hz
    MPU6050_RA_SMPLRT_DIV, 0x07,

    // Full DLPF
    MPU6050_RA_CONFIG, 0x06,

    // 2000 deg/s
    MPU6050_RA_GYRO_CONFIG, 0x18,

    // +-16g.
    MPU6050_RA_ACCEL_CONFIG, 0x18,
    
    //Freefall threshold of |0mg|
    MPU6050_RA_FF_THR, 0x00,

    //Freefall duration limit of 0
    MPU6050_RA_FF_DUR, 0x00,
    
    //Motion threshold of 0mg
    MPU6050_RA_MOT_THR, 0x00,
    
    //Motion duration of 0s
    MPU6050_RA_MOT_DUR, 0x00,
    
    //Zero motion threshold
    MPU6050_RA_ZRMOT_THR, 0x00,
    
    //Zero motion duration threshold
    MPU6050_RA_ZRMOT_DUR, 0x00,

    //Disable sensor output to FIFO buffer
    MPU6050_RA_FIFO_EN, 0x00,
  

    //AUX I2C setup
    //Sets AUX I2C to single master control, plus other config
    MPU6050_RA_I2C_MST_CTRL, 0x00,
    
    //Setup AUX I2C slaves
    MPU6050_RA_I2C_SLV0_ADDR, 0x00,
    MPU6050_RA_I2C_SLV0_REG, 0x00,
    MPU6050_RA_I2C_SLV0_CTRL, 0x00,
    MPU6050_RA_I2C_SLV1_ADDR, 0x00,
    MPU6050_RA_I2C_SLV1_REG, 0x00,
    MPU6050_RA_I2C_SLV1_CTRL, 0x00,
    MPU6050_RA_I2C_SLV2_ADDR, 0x00,
    MPU6050_RA_I2C_SLV2_REG, 0x00,
    MPU6050_RA_I2C_SLV2_CTRL, 0x00,
    MPU6050_RA_I2C_SLV3_ADDR, 0x00,
    MPU6050_RA_I2C_SLV3_REG, 0x00,
    MPU6050_RA_I2C_SLV3_CTRL, 0x00,
    MPU6050_RA_I2C_SLV4_ADDR, 0x00,
    MPU6050_RA_I2C_SLV4_REG, 0x00,
    MPU6050_RA_I2C_SLV4_DO, 0x00,
    MPU6050_RA_I2C_SLV4_CTRL, 0x00,
    MPU6050_RA_I2C_SLV4_DI, 0x00,
    //MPU6050_RA_I2C_MST_STATUS //Read-only
    
    //Setup INT pin and AUX I2C pass through
    MPU6050_RA_INT_PIN_CFG, 0x00,

    //Enable data ready interrupt
    MPU6050_RA_INT_ENABLE, 0x00,
  

    //Slave out, dont care
    MPU6050_RA_I2C_SLV0_DO, 0x00,
    MPU6050_RA_I2C_SLV1_DO, 0x00,
    MPU6050_RA_I2C_SLV2_DO, 0x00,
    MPU6050_RA_I2C_SLV3_DO, 0x00,
    
    //More slave config
    MPU6050_RA_I2C_MST_DELAY_CTRL, 0x00,
    
    //Reset sensor signal paths
    MPU6050_RA_SIGNAL_PATH_RESET, 0x00,
    
    //Motion detection control
    MPU6050_RA_MOT_DETECT_CTRL, 0x00,

    //Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0
    MPU6050_RA_USER_CTRL, 0x00,

    //Sets clock source to gyro reference w/ PLL
    MPU6050_RA_PWR_MGMT_1, 0x02,

    //Controls frequency of wakeups in accel low power mode plus the sensor standby modes
    MPU6050_RA_PWR_MGMT_2, 0x00
};

void InitMPU6050(I2C_HandleTypeDef *hi2c)
{
    uint8_t size = sizeof(settingsMPU6050) / sizeof(*settingsMPU6050);
    for (uint8_t i = 0; i < size; i += 2)
    {
        HAL_I2C_Master_Transmit(hi2c, MPU6050_W, &settingsMPU6050[i], 2, 1);
    }
}