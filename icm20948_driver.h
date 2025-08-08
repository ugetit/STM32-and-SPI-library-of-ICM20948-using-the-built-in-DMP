#ifndef _ICM20948_SPI_H_
#define _ICM20948_SPI_H_

#include <stdint.h>
#include <stdbool.h>
#include "icm20948.h"
#include "ak09916_registers.h"

#ifdef __cplusplus
extern "C" {
#endif

// SPI Configuration
typedef struct {
    void (*spi_write)(uint8_t reg, uint8_t *data, uint32_t len);
    void (*spi_read)(uint8_t reg, uint8_t *data, uint32_t len);
    void (*delay_ms)(uint32_t ms);
} icm20948_spi_interface_t;

// ICM20948 SPI Device structure
typedef struct {
    ICM_20948_Device_t device;
    icm20948_spi_interface_t spi_interface;
    ICM_20948_Serif_t serif;
} icm20948_spi_dev_t;

// Function declarations
ICM_20948_Status_e icm20948_spi_init(icm20948_spi_dev_t *dev, icm20948_spi_interface_t *spi_interface);
ICM_20948_Status_e icm20948_startup_default(icm20948_spi_dev_t *dev, bool minimal);
ICM_20948_Status_e icm20948_initialize_dmp(icm20948_spi_dev_t *dev);
ICM_20948_Status_e icm20948_get_quat9_data(icm20948_spi_dev_t *dev, double *q0, double *q1, double *q2, double *q3, uint16_t *accuracy);

// Utility functions
ICM_20948_Status_e icm20948_check_id(icm20948_spi_dev_t *dev);
ICM_20948_Status_e icm20948_sw_reset(icm20948_spi_dev_t *dev);
ICM_20948_Status_e icm20948_startup_magnetometer(icm20948_spi_dev_t *dev, bool minimal);
ICM_20948_Status_e icm20948_mag_who_am_i(icm20948_spi_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* _ICM20948_SPI_H_ */
