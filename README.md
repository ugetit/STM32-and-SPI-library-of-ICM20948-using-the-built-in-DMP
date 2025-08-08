# ICM20948 DMP Library for STM32

English | [中文](README.md)

A high-precision ICM20948 9-axis sensor DMP (Digital Motion Processor) library for STM32, providing accurate Roll, Pitch, and Yaw angle data via SPI interface.

## ✨ Features

- 🚀 STM32 HAL library support
- 📡 SPI interface communication
- 🧭 Hardware DMP attitude calculation
- 🎯 Direct Roll, Pitch, Yaw angle output
- 📊 Built-in FIFO buffer management
- ⚡ High-precision quaternion to Euler angle conversion
- 🔧 Simple and easy-to-use API

## 📋 Hardware Requirements

- STM32 series microcontroller (HAL library support)
- ICM20948 9-axis sensor module
- SPI interface connection

### SPI Configuration Requirements
```c
hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;    // Clock Polarity: High
hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;         // Clock Phase: Second Edge
```

## 🚀 Quick Start

### 1. Add Files to Project

Add the following files to your STM32 project:
- All `.c` source files
- All `.h` header files

### 2. Compilation Configuration

Define the macro in your project:
```c
#define ICM_20948_USE_DMP
```

### 3. Hardware Connection

| ICM20948 | STM32 |
|----------|-------|
| VCC      | 3.3V  |
| GND      | GND   |
| SCL      | SPI4_SCK |
| SDA      | SPI4_MOSI |
| SDO      | SPI4_MISO |
| CS       | PE7 (configurable) |

### 4. Usage Example

```c
#include "icm20948_driver.h"

#define M_PI 3.141592653579
double roll, pitch, yaw;
uint16_t accuracy;

void advanced_icm20948_example(void)
{
    // Configure SPI interface
    icm20948_spi_interface_t spi_interface = {
        .spi_write = user_spi_write,
        .spi_read = user_spi_read,
        .delay_ms = user_delay_ms
    };
    
    icm20948_spi_dev_t icm_dev;
    
    // Initialize device
    ICM_20948_Status_e status = icm20948_spi_init(&icm_dev, &spi_interface);
    if (status != ICM_20948_Stat_Ok) {
        // Handle error
        return;
    }
    
    // Startup and initialize DMP
    status = icm20948_startup_default(&icm_dev, true);
    if (status != ICM_20948_Stat_Ok) return;
    
    status = icm20948_initialize_dmp(&icm_dev);
    if (status != ICM_20948_Stat_Ok) return;
    
    // Enable orientation sensor
    status = inv_icm20948_enable_dmp_sensor(&icm_dev.device, 
                                            INV_ICM20948_SENSOR_ORIENTATION, 1);
    if (status != ICM_20948_Stat_Ok) return;
    
    // Set output data rate (0 = maximum rate)
    status = inv_icm20948_set_dmp_sensor_period(&icm_dev.device, 
                                                DMP_ODR_Reg_Quat9, 0);
    if (status != ICM_20948_Stat_Ok) return;
    
    // Enable FIFO and DMP
    ICM_20948_enable_FIFO(&icm_dev.device, true);
    ICM_20948_enable_DMP(&icm_dev.device, true);
    
    // Reset DMP and FIFO
    ICM_20948_reset_DMP(&icm_dev.device);
    ICM_20948_reset_FIFO(&icm_dev.device);
    
    // Main reading loop
    while (1) {
        double q0, q1, q2, q3;
        
        status = icm20948_get_quat9_data(&icm_dev, &q0, &q1, &q2, &q3, &accuracy);
        
        if (status == ICM_20948_Stat_Ok || status == ICM_20948_Stat_FIFOMoreDataAvail) {
            // Quaternion to Euler angle conversion
            roll = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
            pitch = asin(2.0 * (q0 * q2 - q3 * q1));
            yaw = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
            
            // Convert to degrees
            roll = roll * 180.0 / M_PI;
            pitch = pitch * 180.0 / M_PI;
            yaw = yaw * 180.0 / M_PI;
            
            // Now you can use roll, pitch, yaw variables
            // printf("Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", roll, pitch, yaw);
        }
        
        // Delay if no more data available
        if (status != ICM_20948_Stat_FIFOMoreDataAvail) {
            user_delay_ms(10);
        }
    }
}
```

## 📡 SPI Interface Functions

You need to implement the following three interface functions:

### SPI Write Function
```c
void user_spi_write(uint8_t reg, uint8_t *data, uint32_t len)
{
    // Pull CS low
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
    
    // Send register address (write mode, bit 7 = 0)
    uint8_t tx_reg = reg & 0x7F;
    HAL_SPI_Transmit(&hspi4, &tx_reg, 1, HAL_MAX_DELAY);
    
    // Send data
    if (len > 0) {
        HAL_SPI_Transmit(&hspi4, data, len, HAL_MAX_DELAY);
    }
    
    // Pull CS high
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
}
```

### SPI Read Function
```c
void user_spi_read(uint8_t reg, uint8_t *data, uint32_t len)
{
    // Set read mode (bit 7 = 1)
    uint8_t tx_reg = reg | 0x80;
    
    // Pull CS low
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
    
    // Send register address
    HAL_SPI_Transmit(&hspi4, &tx_reg, 1, HAL_MAX_DELAY);
    
    // Read data
    if (len > 0) {
        HAL_SPI_Receive(&hspi4, data, len, HAL_MAX_DELAY);
    }
    
    // Pull CS high
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
}
```

### Delay Function
```c
void user_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}
```

## 📊 Output Data Description

- **Roll**: Rotation angle around X-axis, range -180° ~ +180°
- **Pitch**: Rotation angle around Y-axis, range -90° ~ +90°
- **Yaw**: Rotation angle around Z-axis, range -180° ~ +180°
- **Accuracy**: Data accuracy indicator

## ⚙️ Configuration Options

### Data Output Rate Configuration
```c
// Set DMP sensor period (0 = maximum rate, higher value = slower rate)
inv_icm20948_set_dmp_sensor_period(&icm_dev.device, DMP_ODR_Reg_Quat9, 0);
```

### GPIO Configuration
Default CS pin is configured as PE7, modify as needed:
```c
// Modify in user_spi_write and user_spi_read functions
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);  // Change to your pin
```

## 🔧 Troubleshooting

### Common Issues

1. **Initialization Failed**
   - Check SPI configuration (clock polarity and phase)
   - Verify hardware connections
   - Check power supply (3.3V)

2. **Data Reading Anomalies**
   - Confirm CS pin configuration is correct
   - Check SPI clock frequency (recommended not to exceed 7MHz)
   - Verify ICM_20948_USE_DMP macro is defined

3. **Attitude Angle Jumping**
   - Check if sensor installation is secure
   - Ensure no strong magnetic field interference nearby
   - Sensor calibration may be required

## 📝 License

This project is released under an open source license. Contributions and feedback are welcome.

## 🤝 Contributing

Feel free to submit Issues and Pull Requests to improve this library!

## 📞 Support

If you encounter problems during use, please:
1. Check the troubleshooting guide above
2. Submit an Issue on GitHub
3. Provide detailed error information and hardware configuration

---

**Important Notes:**
- Ensure correct SPI clock configuration: `SPI_POLARITY_HIGH` and `SPI_PHASE_2EDGE`
- CS pin can be modified according to actual hardware connection
- Sensor calibration is recommended after initialization for optimal accuracy
