#include "icm20948_driver.h"
#include <math.h>
#include <string.h>

// Static variables for global device instance (for test function)
static icm20948_spi_dev_t g_icm_dev;

// Forward declarations
static ICM_20948_Status_e icm20948_write_spi(uint8_t reg, uint8_t *data, uint32_t len, void *user);
static ICM_20948_Status_e icm20948_read_spi(uint8_t reg, uint8_t *data, uint32_t len, void *user);

// SPI write function wrapper
static ICM_20948_Status_e icm20948_write_spi(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
    if (user == NULL) {
        return ICM_20948_Stat_ParamErr;
    }
    
    icm20948_spi_dev_t *dev = (icm20948_spi_dev_t *)user;
    if (dev->spi_interface.spi_write == NULL) {
        return ICM_20948_Stat_NotImpl;
    }
    
    dev->spi_interface.spi_write(reg, data, len);
    return ICM_20948_Stat_Ok;
}

// SPI read function wrapper
static ICM_20948_Status_e icm20948_read_spi(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
    if (user == NULL) {
        return ICM_20948_Stat_ParamErr;
    }
    
    icm20948_spi_dev_t *dev = (icm20948_spi_dev_t *)user;
    if (dev->spi_interface.spi_read == NULL) {
        return ICM_20948_Stat_NotImpl;
    }
    
    dev->spi_interface.spi_read(reg, data, len);
    return ICM_20948_Stat_Ok;
}

// Initialize ICM20948 SPI device
ICM_20948_Status_e icm20948_spi_init(icm20948_spi_dev_t *dev, icm20948_spi_interface_t *spi_interface)
{
    if (dev == NULL || spi_interface == NULL) {
        return ICM_20948_Stat_ParamErr;
    }
    
    // Initialize device structure
    ICM_20948_Status_e status = ICM_20948_init_struct(&dev->device);
    if (status != ICM_20948_Stat_Ok) {
        return status;
    }
    
    // Copy SPI interface
    dev->spi_interface = *spi_interface;
    
    // Setup serif
    dev->serif.write = icm20948_write_spi;
    dev->serif.read = icm20948_read_spi;
    dev->serif.user = (void *)dev;
    
    // Link serif to device
    status = ICM_20948_link_serif(&dev->device, &dev->serif);
    if (status != ICM_20948_Stat_Ok) {
        return status;
    }
    
#if defined(ICM_20948_USE_DMP)
    dev->device._dmp_firmware_available = true;
#else
    dev->device._dmp_firmware_available = false;
#endif

    dev->device._firmware_loaded = false;
    dev->device._last_bank = 255;
    dev->device._last_mems_bank = 255;
    dev->device._gyroSF = 0;
    dev->device._gyroSFpll = 0;
    dev->device._enabled_Android_0 = 0;
    dev->device._enabled_Android_1 = 0;
    dev->device._enabled_Android_intr_0 = 0;
    dev->device._enabled_Android_intr_1 = 0;
    
    return ICM_20948_Stat_Ok;
}

// Check device ID
ICM_20948_Status_e icm20948_check_id(icm20948_spi_dev_t *dev)
{
    if (dev == NULL) {
        return ICM_20948_Stat_ParamErr;
    }
    
    return ICM_20948_check_id(&dev->device);
}

// Software reset
ICM_20948_Status_e icm20948_sw_reset(icm20948_spi_dev_t *dev)
{
    if (dev == NULL) {
        return ICM_20948_Stat_ParamErr;
    }
    
    return ICM_20948_sw_reset(&dev->device);
}

// Magnetometer Who Am I check
ICM_20948_Status_e icm20948_mag_who_am_i(icm20948_spi_dev_t *dev)
{
    if (dev == NULL) {
        return ICM_20948_Stat_ParamErr;
    }
    
    ICM_20948_Status_e status = ICM_20948_Stat_Ok;
    uint8_t whoiam1, whoiam2;
    
    // Read WIA1
    status = ICM_20948_i2c_master_single_r(&dev->device, MAG_AK09916_I2C_ADDR, AK09916_REG_WIA1, &whoiam1);
    if (status != ICM_20948_Stat_Ok) {
        return status;
    }
    
    // Read WIA2
    status = ICM_20948_i2c_master_single_r(&dev->device, MAG_AK09916_I2C_ADDR, AK09916_REG_WIA2, &whoiam2);
    if (status != ICM_20948_Stat_Ok) {
        return status;
    }
    
    // Check if values match expected
    if ((whoiam1 == (MAG_AK09916_WHO_AM_I >> 8)) && (whoiam2 == (MAG_AK09916_WHO_AM_I & 0xFF))) {
        return ICM_20948_Stat_Ok;
    }
    
    return ICM_20948_Stat_WrongID;
}

// Startup magnetometer
ICM_20948_Status_e icm20948_startup_magnetometer(icm20948_spi_dev_t *dev, bool minimal)
{
    if (dev == NULL) {
        return ICM_20948_Stat_ParamErr;
    }
    
    ICM_20948_Status_e status = ICM_20948_Stat_Ok;
    
    // Disable I2C master passthrough
    status = ICM_20948_i2c_master_passthrough(&dev->device, false);
    if (status != ICM_20948_Stat_Ok) {
        return status;
    }
    
    // Enable I2C master
    status = ICM_20948_i2c_master_enable(&dev->device, true);
    if (status != ICM_20948_Stat_Ok) {
        return status;
    }
    
    // Reset magnetometer
    uint8_t srst = 1;
    status = ICM_20948_i2c_master_single_w(&dev->device, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL3, &srst);
    if (status != ICM_20948_Stat_Ok) {
        return status;
    }
    
    // Try to communicate with magnetometer (with retries)
    uint8_t tries = 0;
    const uint8_t MAX_MAG_TRIES = 10;
    
    while (tries < MAX_MAG_TRIES) {
        tries++;
        
        status = icm20948_mag_who_am_i(dev);
        if (status == ICM_20948_Stat_Ok) {
            break;
        }
        
        // Reset I2C master if failed
        ICM_20948_i2c_master_reset(&dev->device);
        
        if (dev->spi_interface.delay_ms) {
            dev->spi_interface.delay_ms(10);
        }
    }
    
    if (tries == MAX_MAG_TRIES) {
        return ICM_20948_Stat_WrongID;
    }
    
    // Return early if minimal startup
    if (minimal) {
        return ICM_20948_Stat_Ok;
    }
    
    // Configure magnetometer for continuous mode at 100Hz
    uint8_t mag_mode = AK09916_mode_cont_100hz;
    status = ICM_20948_i2c_master_single_w(&dev->device, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL2, &mag_mode);
    if (status != ICM_20948_Stat_Ok) {
        return status;
    }
    
    // Configure I2C peripheral 0 to read from magnetometer
    status = ICM_20948_i2c_controller_configure_peripheral(&dev->device, 0, MAG_AK09916_I2C_ADDR, 
                                                          AK09916_REG_ST1, 9, true, true, false, false, false, 0);
    if (status != ICM_20948_Stat_Ok) {
        return status;
    }
    
    return ICM_20948_Stat_Ok;
}

// Default startup sequence
ICM_20948_Status_e icm20948_startup_default(icm20948_spi_dev_t *dev, bool minimal)
{
    if (dev == NULL) {
        return ICM_20948_Stat_ParamErr;
    }
    
    ICM_20948_Status_e status = ICM_20948_Stat_Ok;
    
    // Check device ID
    status = icm20948_check_id(dev);
    if (status != ICM_20948_Stat_Ok) {
        return status;
    }
    
    // Software reset
    status = icm20948_sw_reset(dev);
    if (status != ICM_20948_Stat_Ok) {
        return status;
    }
    
    if (dev->spi_interface.delay_ms) {
        dev->spi_interface.delay_ms(50);
    }
    
    // Wake up device
    status = ICM_20948_sleep(&dev->device, false);
    if (status != ICM_20948_Stat_Ok) {
        return status;
    }
    
    // Disable low power mode
    status = ICM_20948_low_power(&dev->device, false);
    if (status != ICM_20948_Stat_Ok) {
        return status;
    }
    
    // Setup magnetometer
    status = icm20948_startup_magnetometer(dev, minimal);
    if (status != ICM_20948_Stat_Ok) {
        return status;
    }
    
    if (minimal) {
        return ICM_20948_Stat_Ok;
    }
    
    // Configure sample mode
    status = ICM_20948_set_sample_mode(&dev->device, 
                                      (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), 
                                      ICM_20948_Sample_Mode_Continuous);
    if (status != ICM_20948_Stat_Ok) {
        return status;
    }
    
    // Set full scale ranges
    ICM_20948_fss_t fss;
    fss.a = gpm2;   // ±2g for accelerometer
    fss.g = dps250; // ±250dps for gyroscope
    
    status = ICM_20948_set_full_scale(&dev->device, 
                                     (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), 
                                     fss);
    if (status != ICM_20948_Stat_Ok) {
        return status;
    }
    
    // Configure DLPF
    ICM_20948_dlpcfg_t dlpcfg;
    dlpcfg.a = acc_d473bw_n499bw;
    dlpcfg.g = gyr_d361bw4_n376bw5;
    
    status = ICM_20948_set_dlpf_cfg(&dev->device,
                                   (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
                                   dlpcfg);
    if (status != ICM_20948_Stat_Ok) {
        return status;
    }
    
    // Disable DLPF for both accelerometer and gyroscope
    status = ICM_20948_enable_dlpf(&dev->device, ICM_20948_Internal_Acc, false);
    if (status != ICM_20948_Stat_Ok) {
        return status;
    }
    
    status = ICM_20948_enable_dlpf(&dev->device, ICM_20948_Internal_Gyr, false);
    if (status != ICM_20948_Stat_Ok) {
        return status;
    }
    
    return ICM_20948_Stat_Ok;
}

// Initialize DMP
ICM_20948_Status_e icm20948_initialize_dmp(icm20948_spi_dev_t *dev)
{
    if (dev == NULL) {
        return ICM_20948_Stat_ParamErr;
    }
    
    // Check if DMP is available
    if (dev->device._dmp_firmware_available != true) {
        return ICM_20948_Stat_DMPNotSupported;
    }
    
    ICM_20948_Status_e status = ICM_20948_Stat_Ok;
    ICM_20948_Status_e worst_result = ICM_20948_Stat_Ok;
    
#if defined(ICM_20948_USE_DMP)
    
    // Configure I2C_SLV0 to read 10 bytes from magnetometer starting at RSV2
    status = ICM_20948_i2c_controller_configure_peripheral(&dev->device, 0, MAG_AK09916_I2C_ADDR, 
                                                          AK09916_REG_RSV2, 10, true, true, false, true, true, 0);
    if (status > worst_result) worst_result = status;
    
    // Configure I2C_SLV1 to trigger single measurement
    status = ICM_20948_i2c_controller_configure_peripheral(&dev->device, 1, MAG_AK09916_I2C_ADDR,
                                                          AK09916_REG_CNTL2, 1, false, true, false, false, false, AK09916_mode_single);
    if (status > worst_result) worst_result = status;
    
    // Set I2C Master ODR configuration
    status = ICM_20948_set_bank(&dev->device, 3);
    if (status > worst_result) worst_result = status;
    
    uint8_t mst_odr_config = 0x04; // 68.75Hz
    status = ICM_20948_execute_w(&dev->device, AGB3_REG_I2C_MST_ODR_CONFIG, &mst_odr_config, 1);
    if (status > worst_result) worst_result = status;
    
    // Configure clock source
    status = ICM_20948_set_clock_source(&dev->device, ICM_20948_Clock_Auto);
    if (status > worst_result) worst_result = status;
    
    // Enable accel and gyro sensors
    status = ICM_20948_set_bank(&dev->device, 0);
    if (status > worst_result) worst_result = status;
    
    uint8_t pwr_mgmt2 = 0x40; // Reserved bit 6
    status = ICM_20948_execute_w(&dev->device, AGB0_REG_PWR_MGMT_2, &pwr_mgmt2, 1);
    if (status > worst_result) worst_result = status;
    
    // Set I2C Master to cycled mode
    status = ICM_20948_set_sample_mode(&dev->device, ICM_20948_Internal_Mst, ICM_20948_Sample_Mode_Cycled);
    if (status > worst_result) worst_result = status;
    
    // Disable FIFO
    status = ICM_20948_enable_FIFO(&dev->device, false);
    if (status > worst_result) worst_result = status;
    
    // Disable DMP
    status = ICM_20948_enable_DMP(&dev->device, false);
    if (status > worst_result) worst_result = status;
    
    // Set full scale ranges
    ICM_20948_fss_t fss;
    fss.a = gpm4;   // ±4g
    fss.g = dps2000; // ±2000dps
    
    status = ICM_20948_set_full_scale(&dev->device, 
                                     (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), 
                                     fss);
    if (status > worst_result) worst_result = status;
    
    // Enable gyro DLPF
    status = ICM_20948_enable_dlpf(&dev->device, ICM_20948_Internal_Gyr, true);
    if (status > worst_result) worst_result = status;
    
    // Turn off FIFO
    status = ICM_20948_set_bank(&dev->device, 0);
    if (status > worst_result) worst_result = status;
    
    uint8_t zero = 0;
    status = ICM_20948_execute_w(&dev->device, AGB0_REG_FIFO_EN_1, &zero, 1);
    if (status > worst_result) worst_result = status;
    
    status = ICM_20948_execute_w(&dev->device, AGB0_REG_FIFO_EN_2, &zero, 1);
    if (status > worst_result) worst_result = status;
    
    // Reset FIFO
    status = ICM_20948_reset_FIFO(&dev->device);
    if (status > worst_result) worst_result = status;
    
    // Set sample rate
    ICM_20948_smplrt_t smplrt;
    smplrt.g = 19; // 55Hz
    smplrt.a = 19; // 56.25Hz
    
    status = ICM_20948_set_sample_rate(&dev->device, 
                                      (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), 
                                      smplrt);
    if (status > worst_result) worst_result = status;
    
    // Setup DMP start address
    status = ICM_20948_set_dmp_start_address(&dev->device, DMP_START_ADDRESS);
    if (status > worst_result) worst_result = status;
    
    // Load DMP firmware
    status = ICM_20948_firmware_load(&dev->device);
    if (status > worst_result) worst_result = status;
    
    // Set DMP start address again
    status = ICM_20948_set_dmp_start_address(&dev->device, DMP_START_ADDRESS);
    if (status > worst_result) worst_result = status;
    
    // Configure hardware fix disable register
    status = ICM_20948_set_bank(&dev->device, 0);
    if (status > worst_result) worst_result = status;
    
    uint8_t hw_fix = 0x48;
    status = ICM_20948_execute_w(&dev->device, AGB0_REG_HW_FIX_DISABLE, &hw_fix, 1);
    if (status > worst_result) worst_result = status;
    
    // Set FIFO priority select
    uint8_t fifo_prio = 0xE4;
    status = ICM_20948_execute_w(&dev->device, AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifo_prio, 1);
    if (status > worst_result) worst_result = status;
    
    // Configure accel scaling for DMP
    const unsigned char acc_scale[4] = {0x04, 0x00, 0x00, 0x00};
    status = inv_icm20948_write_mems(&dev->device, ACC_SCALE, 4, &acc_scale[0]);
    if (status > worst_result) worst_result = status;
    
    const unsigned char acc_scale2[4] = {0x00, 0x04, 0x00, 0x00};
    status = inv_icm20948_write_mems(&dev->device, ACC_SCALE2, 4, &acc_scale2[0]);
    if (status > worst_result) worst_result = status;
    
    // Configure compass mount matrix
    const unsigned char mount_zero[4] = {0x00, 0x00, 0x00, 0x00};
    const unsigned char mount_plus[4] = {0x09, 0x99, 0x99, 0x99};
    const unsigned char mount_minus[4] = {0xF6, 0x66, 0x66, 0x67};
    
    status = inv_icm20948_write_mems(&dev->device, CPASS_MTX_00, 4, &mount_plus[0]);
    if (status > worst_result) worst_result = status;
    status = inv_icm20948_write_mems(&dev->device, CPASS_MTX_01, 4, &mount_zero[0]);
    if (status > worst_result) worst_result = status;
    status = inv_icm20948_write_mems(&dev->device, CPASS_MTX_02, 4, &mount_zero[0]);
    if (status > worst_result) worst_result = status;
    status = inv_icm20948_write_mems(&dev->device, CPASS_MTX_10, 4, &mount_zero[0]);
    if (status > worst_result) worst_result = status;
    status = inv_icm20948_write_mems(&dev->device, CPASS_MTX_11, 4, &mount_minus[0]);
    if (status > worst_result) worst_result = status;
    status = inv_icm20948_write_mems(&dev->device, CPASS_MTX_12, 4, &mount_zero[0]);
    if (status > worst_result) worst_result = status;
    status = inv_icm20948_write_mems(&dev->device, CPASS_MTX_20, 4, &mount_zero[0]);
    if (status > worst_result) worst_result = status;
    status = inv_icm20948_write_mems(&dev->device, CPASS_MTX_21, 4, &mount_zero[0]);
    if (status > worst_result) worst_result = status;
    status = inv_icm20948_write_mems(&dev->device, CPASS_MTX_22, 4, &mount_minus[0]);
    if (status > worst_result) worst_result = status;
    
    // Configure B2S mount matrix
    const unsigned char b2s_zero[4] = {0x00, 0x00, 0x00, 0x00};
    const unsigned char b2s_plus[4] = {0x40, 0x00, 0x00, 0x00};
    
    status = inv_icm20948_write_mems(&dev->device, B2S_MTX_00, 4, &b2s_plus[0]);
    if (status > worst_result) worst_result = status;
    status = inv_icm20948_write_mems(&dev->device, B2S_MTX_01, 4, &b2s_zero[0]);
    if (status > worst_result) worst_result = status;
    status = inv_icm20948_write_mems(&dev->device, B2S_MTX_02, 4, &b2s_zero[0]);
    if (status > worst_result) worst_result = status;
    status = inv_icm20948_write_mems(&dev->device, B2S_MTX_10, 4, &b2s_zero[0]);
    if (status > worst_result) worst_result = status;
    status = inv_icm20948_write_mems(&dev->device, B2S_MTX_11, 4, &b2s_plus[0]);
    if (status > worst_result) worst_result = status;
    status = inv_icm20948_write_mems(&dev->device, B2S_MTX_12, 4, &b2s_zero[0]);
    if (status > worst_result) worst_result = status;
    status = inv_icm20948_write_mems(&dev->device, B2S_MTX_20, 4, &b2s_zero[0]);
    if (status > worst_result) worst_result = status;
    status = inv_icm20948_write_mems(&dev->device, B2S_MTX_21, 4, &b2s_zero[0]);
    if (status > worst_result) worst_result = status;
    status = inv_icm20948_write_mems(&dev->device, B2S_MTX_22, 4, &b2s_plus[0]);
    if (status > worst_result) worst_result = status;
    
    // Configure gyro scaling factor
    status = inv_icm20948_set_gyro_sf(&dev->device, 19, 3); // 55Hz, 2000dps
    if (status > worst_result) worst_result = status;
    
    // Configure gyro full scale
    const unsigned char gyro_full_scale[4] = {0x10, 0x00, 0x00, 0x00}; // 2000dps
    status = inv_icm20948_write_mems(&dev->device, GYRO_FULLSCALE, 4, &gyro_full_scale[0]);
    if (status > worst_result) worst_result = status;
    
    // Configure accel only gain for 56Hz
    const unsigned char accel_only_gain[4] = {0x03, 0xA4, 0x92, 0x49};
    status = inv_icm20948_write_mems(&dev->device, ACCEL_ONLY_GAIN, 4, &accel_only_gain[0]);
    if (status > worst_result) worst_result = status;
    
    // Configure accel alpha var for 56Hz
    const unsigned char accel_alpha_var[4] = {0x34, 0x92, 0x49, 0x25};
    status = inv_icm20948_write_mems(&dev->device, ACCEL_ALPHA_VAR, 4, &accel_alpha_var[0]);
    if (status > worst_result) worst_result = status;
    
    // Configure accel A var for 56Hz
    const unsigned char accel_a_var[4] = {0x0B, 0x6D, 0xB6, 0xDB};
    status = inv_icm20948_write_mems(&dev->device, ACCEL_A_VAR, 4, &accel_a_var[0]);
    if (status > worst_result) worst_result = status;
    
    // Configure accel cal rate
    const unsigned char accel_cal_rate[2] = {0x00, 0x00};
    status = inv_icm20948_write_mems(&dev->device, ACCEL_CAL_RATE, 2, &accel_cal_rate[0]);
    if (status > worst_result) worst_result = status;
    
    // Configure compass time buffer
    const unsigned char compass_rate[2] = {0x00, 0x45}; // 69Hz
    status = inv_icm20948_write_mems(&dev->device, CPASS_TIME_BUFFER, 2, &compass_rate[0]);
    if (status > worst_result) worst_result = status;
    
#endif

    return worst_result;
}

// Get quaternion data from DMP
ICM_20948_Status_e icm20948_get_quat9_data(icm20948_spi_dev_t *dev, double *q0, double *q1, double *q2, double *q3, uint16_t *accuracy)
{
    if (dev == NULL || q0 == NULL || q1 == NULL || q2 == NULL || q3 == NULL || accuracy == NULL) {
        return ICM_20948_Stat_ParamErr;
    }
    
    if (dev->device._dmp_firmware_available != true) {
        return ICM_20948_Stat_DMPNotSupported;
    }
    
    icm_20948_DMP_data_t dmp_data;
    ICM_20948_Status_e status = inv_icm20948_read_dmp_data(&dev->device, &dmp_data);
    
    if (status == ICM_20948_Stat_Ok || status == ICM_20948_Stat_FIFOMoreDataAvail) {
        if ((dmp_data.header & DMP_header_bitmap_Quat9) > 0) {
            // Scale quaternion data from 2^30 to +/- 1
            *q1 = ((double)dmp_data.Quat9.Data.Q1) / 1073741824.0; // 2^30
            *q2 = ((double)dmp_data.Quat9.Data.Q2) / 1073741824.0;
            *q3 = ((double)dmp_data.Quat9.Data.Q3) / 1073741824.0;
            
            // Calculate Q0 from quaternion constraint
            double q_sum = (*q1) * (*q1) + (*q2) * (*q2) + (*q3) * (*q3);
            if (q_sum <= 1.0) {
                *q0 = sqrt(1.0 - q_sum);
            } else {
                *q0 = 0.0;
            }
            
            *accuracy = dmp_data.Quat9.Data.Accuracy;
            return ICM_20948_Stat_Ok;
        }
    }
    
    return status;
}

// Weak SPI interface functions - to be implemented by user
__attribute__((weak)) void user_spi_write(uint8_t reg, uint8_t *data, uint32_t len)
{
    // User should implement this function
    // reg: register address (bit 7 should be 0 for write)
    // data: data to write
    // len: number of bytes to write
}

__attribute__((weak)) void user_spi_read(uint8_t reg, uint8_t *data, uint32_t len)
{
    // User should implement this function
    // reg: register address (bit 7 should be 1 for read)  
    // data: buffer to store read data
    // len: number of bytes to read
}

__attribute__((weak)) void user_delay_ms(uint32_t ms)
{
    // User should implement this function for delay
    // ms: delay in milliseconds
}

