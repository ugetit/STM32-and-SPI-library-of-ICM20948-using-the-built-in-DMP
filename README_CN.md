# ICM20948 DMP Library for STM32

[English](README.md) | 中文

一个用于STM32的ICM20948九轴传感器DMP（Digital Motion Processor）库，支持通过SPI接口获取高精度的姿态角度数据（Roll、Pitch、Yaw）。

## ✨ 特性

- 🚀 支持STM32 HAL库
- 📡 SPI接口通信
- 🧭 DMP硬件姿态解算
- 🎯 直接输出Roll、Pitch、Yaw角度
- 📊 内置FIFO缓冲区管理
- ⚡ 高精度四元数到欧拉角转换
- 🔧 简单易用的API接口

## 📋 硬件要求

- STM32系列微控制器（支持HAL库）
- ICM20948九轴传感器模块
- SPI接口连接

### SPI配置要求
```c
hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;    // 时钟极性：高电平
hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;         // 时钟相位：第二个边沿
```

## 🚀 快速开始

### 1. 添加文件到项目

将以下文件添加到您的STM32项目中：
- 所有 `.c` 源文件
- 所有 `.h` 头文件

### 2. 编译配置

在项目中定义宏：
```c
#define ICM_20948_USE_DMP
```

### 3. 硬件连接

| ICM20948 | STM32 |
|----------|-------|
| VCC      | 3.3V  |
| GND      | GND   |
| SCL      | SPI4_SCK |
| SDA      | SPI4_MOSI |
| SDO      | SPI4_MISO |
| CS       | PE7 (可配置) |

### 4. 使用示例

```c
#include "icm20948_driver.h"

#define M_PI 3.141592653579
double roll, pitch, yaw;
uint16_t accuracy;

void advanced_icm20948_example(void)
{
    // 配置SPI接口
    icm20948_spi_interface_t spi_interface = {
        .spi_write = user_spi_write,
        .spi_read = user_spi_read,
        .delay_ms = user_delay_ms
    };
    
    icm20948_spi_dev_t icm_dev;
    
    // 初始化设备
    ICM_20948_Status_e status = icm20948_spi_init(&icm_dev, &spi_interface);
    if (status != ICM_20948_Stat_Ok) {
        // 处理错误
        return;
    }
    
    // 启动并初始化DMP
    status = icm20948_startup_default(&icm_dev, true);
    if (status != ICM_20948_Stat_Ok) return;
    
    status = icm20948_initialize_dmp(&icm_dev);
    if (status != ICM_20948_Stat_Ok) return;
    
    // 启用姿态传感器
    status = inv_icm20948_enable_dmp_sensor(&icm_dev.device, 
                                            INV_ICM20948_SENSOR_ORIENTATION, 1);
    if (status != ICM_20948_Stat_Ok) return;
    
    // 设置输出数据率（0 = 最大速率）
    status = inv_icm20948_set_dmp_sensor_period(&icm_dev.device, 
                                                DMP_ODR_Reg_Quat9, 0);
    if (status != ICM_20948_Stat_Ok) return;
    
    // 启用FIFO和DMP
    ICM_20948_enable_FIFO(&icm_dev.device, true);
    ICM_20948_enable_DMP(&icm_dev.device, true);
    
    // 重置DMP和FIFO
    ICM_20948_reset_DMP(&icm_dev.device);
    ICM_20948_reset_FIFO(&icm_dev.device);
    
    // 主循环读取数据
    while (1) {
        double q0, q1, q2, q3;
        
        status = icm20948_get_quat9_data(&icm_dev, &q0, &q1, &q2, &q3, &accuracy);
        
        if (status == ICM_20948_Stat_Ok || status == ICM_20948_Stat_FIFOMoreDataAvail) {
            // 四元数转欧拉角
            roll = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
            pitch = asin(2.0 * (q0 * q2 - q3 * q1));
            yaw = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
            
            // 转换为度数
            roll = roll * 180.0 / M_PI;
            pitch = pitch * 180.0 / M_PI;
            yaw = yaw * 180.0 / M_PI;
            
            // 现在可以使用roll、pitch、yaw变量
            // printf("Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", roll, pitch, yaw);
        }
        
        // 如果没有更多数据，延时等待
        if (status != ICM_20948_Stat_FIFOMoreDataAvail) {
            user_delay_ms(10);
        }
    }
}
```

## 📡 SPI接口函数

您需要实现以下三个接口函数：

### SPI写函数
```c
void user_spi_write(uint8_t reg, uint8_t *data, uint32_t len)
{
    // 拉低CS
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
    
    // 发送寄存器地址（写模式，bit 7 = 0）
    uint8_t tx_reg = reg & 0x7F;
    HAL_SPI_Transmit(&hspi4, &tx_reg, 1, HAL_MAX_DELAY);
    
    // 发送数据
    if (len > 0) {
        HAL_SPI_Transmit(&hspi4, data, len, HAL_MAX_DELAY);
    }
    
    // 拉高CS
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
}
```

### SPI读函数
```c
void user_spi_read(uint8_t reg, uint8_t *data, uint32_t len)
{
    // 设置读模式（bit 7 = 1）
    uint8_t tx_reg = reg | 0x80;
    
    // 拉低CS
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
    
    // 发送寄存器地址
    HAL_SPI_Transmit(&hspi4, &tx_reg, 1, HAL_MAX_DELAY);
    
    // 读取数据
    if (len > 0) {
        HAL_SPI_Receive(&hspi4, data, len, HAL_MAX_DELAY);
    }
    
    // 拉高CS
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
}
```

### 延时函数
```c
void user_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}
```

## 📊 输出数据说明

- **Roll（横滚角）**：围绕X轴的旋转角度，范围 -180° ~ +180°
- **Pitch（俯仰角）**：围绕Y轴的旋转角度，范围 -90° ~ +90°
- **Yaw（偏航角）**：围绕Z轴的旋转角度，范围 -180° ~ +180°
- **Accuracy**：数据精度指示器

## ⚙️ 配置选项

### 数据输出率配置
```c
// 设置DMP传感器周期（0 = 最大速率，数值越大速率越慢）
inv_icm20948_set_dmp_sensor_period(&icm_dev.device, DMP_ODR_Reg_Quat9, 0);
```

### GPIO配置
默认CS引脚配置为PE7，可以根据需要修改：
```c
// 在user_spi_write和user_spi_read函数中修改
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);  // 改为你的引脚
```

## 🔧 故障排除

### 常见问题

1. **初始化失败**
   - 检查SPI配置（时钟极性和相位）
   - 确认硬件连接
   - 检查电源供电（3.3V）

2. **数据读取异常**
   - 确认CS引脚配置正确
   - 检查SPI时钟频率（建议不超过7MHz）
   - 验证ICM_20948_USE_DMP宏已定义

3. **姿态角度跳变**
   - 检查传感器安装是否牢固
   - 确认周围无强磁场干扰
   - 可能需要校准传感器

## 📝 许可证

本项目采用开源许可证发布，欢迎贡献代码和反馈问题。

## 🤝 贡献

欢迎提交Issue和Pull Request来改进这个库！

## 📞 支持

如果您在使用过程中遇到问题，请：
1. 检查上述故障排除指南
2. 在GitHub上提交Issue
3. 提供详细的错误信息和硬件配置

---

**注意事项：**
- 确保SPI时钟配置正确：`SPI_POLARITY_HIGH` 和 `SPI_PHASE_2EDGE`
- CS引脚可根据实际硬件连接修改
- 建议在初始化完成后进行传感器校准以获得最佳精度
