/**
 * dht12.h - Component to work with DHT12
 *
 * Include this header file to use the component.
 *
 * (C) 2019 - Timothee Cruse <timothee.cruse@gmail.com>
 * This code is licensed under the MIT License.
 */

#ifndef _DHT12_H_
#define _DHT12_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "iot_i2c.h"

typedef int32_t dht12_err_t;

typedef enum {
    CELCIUS = 1,
    KELVIN = 2,
    FAHRENHEIT = 3
} dht12_unit_t;

/**
 * @brief   DHT12 FAILURE CODES
 */
#define DHT12_SUCCESS                    ( 0 )     /** Operation completed successfully. */
#define DHT12_FAIL                       ( 1 )     /** Operation failed. */
#define DHT12_CHECKSUM_ERROR             ( 2 )     /** Operation failed. */
#define DHT12_OTHER_ERROR                ( 3 )     /** Other error. */


/**
 * @brief   I2C ADDRESS/BITS/SETTINGS
 */
#define DHT12_ADDRESS                    ( 0x5C )  /** The default I2C address for the sensor. */


/**
 * @brief   Initialize DHT12 Sensor.
 * @note    This assumes that the I2C has already been initialized.
 * @param   handle:   Common-IO I2C Handle
 * @return  DHT12_SUCCESS success
 *          DHT12_FAIL errors found
 */
dht12_err_t eDht12Init( IotI2CHandle_t handle );

/**
 * @brief   Read Temperature.
 * @param   T:    stores the temperature value after read.
 * @param   unit: CELCIUS, KELVIN, FAHRENHEIT.
 *
 * @return  DHT12_SUCCESS success
 *          DHT12_FAIL errors found
 */
dht12_err_t readTemperature( double * T, dht12_unit_t unit );
  
/**
 * @brief   Read Humidity.
 * @param   H:  stores the pressure value after read.
 *
 * @return  DHT12_SUCCESS success
 *          DHT12_FAIL errors found
 */
dht12_err_t readHumidity( double * H );
  
#ifdef __cplusplus
}
#endif

#endif // _DHT12_H_
