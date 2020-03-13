/**
 * dht12.c - Component to work with DHT12
 *
 * (C) 2020 - Timothee Cruse <timothee.cruse@gmail.com>
 * This code is licensed under the MIT License.
 */

#include "dht12.h"

static const char *TAG = "dht12";

#define DHT12_CHECK_HANDLER() if ( prvI2CHandle == NULL ) { return DHT12_FAIL; }

/*-----------------------------------------------------------*/

static IotI2CHandle_t prvI2CHandle = NULL;
uint8_t prvUnit = CELCIUS;

/*-----------------------------------------------------------*/
			

dht12_err_t eDht12Init( IotI2CHandle_t handle )
{
    prvI2CHandle = NULL;
    return DHT12_SUCCESS;
}

/*-----------------------------------------------------------*/

/**
 * @brief   Read an array of bytes a DHT12 register onwards
 * @param   value:      external array to hold data. Put starting register in values[0].
 * @param   length:     number of bytes to read
 * @return  DHT12_SUCCESS success
 *          DHT12_FAIL errors found
*/
dht12_err_t prvReadBytes( uint8_t * values, uint8_t length )
{
    // Return value of I2C functions.
    dht12_err_t res = DHT12_FAIL;

    DHT12_CHECK_HANDLER();

    // Register address on I2C slave device.
    uint16_t ioCtlBuffer = DHT12_ADDRESS;
 
    // Number of read/write bytes.
    uint16_t usReadBytes = 0;
    uint16_t usWriteBytes = 0;

    uint8_t address = values[0];

    // Set slave address.
    res = iot_i2c_ioctl( prvI2CHandle, eI2CSetSlaveAddr, &ioCtlBuffer );
    if ( res != IOT_I2C_SUCCESS )
    {
        IotLogError( "Setting the slave address for %#04X on I2C failed.", ioCtlBuffer );
        return DHT12_FAIL;
    }
    // assert(lRetVal == IOT_I2C_SUCCESS);

    // Write the register address as single byte, in a transaction.
    res = iot_i2c_write_sync( prvI2CHandle, values, 1 );
    if ( res == IOT_I2C_SUCCESS )
    {
        // Get the number of written bytes in last transaction.
        res = iot_i2c_ioctl( prvI2CHandle, eI2CGetTxNoOfbytes, &usWriteBytes );
        if ( res != IOT_I2C_SUCCESS || usWriteBytes != 1 ) 
        {
            IotLogError( "Failed to check the number of written bytes %u vs. %u.", usWriteBytes, 1 );
            return DHT12_FAIL;
        }

        // Read length bytes of data to allocated buffer, in a transaction.
        res = iot_i2c_read_sync( prvI2CHandle, values, length );
        if ( res == IOT_I2C_SUCCESS )
        {
            // Get the number of read bytes in last transaction.
            res = iot_i2c_ioctl( prvI2CHandle, eI2CGetRxNoOfbytes, &usReadBytes ); 
            if ( res != IOT_I2C_SUCCESS || usReadBytes != length ) 
            {
                IotLogError( "Failed to check the number of read bytes %u vs. %u.", usReadBytes, length );
                return DHT12_FAIL;
            }

            IotLogDebug( "Read %u bytes @ %#04X", length, address );

            return DHT12_SUCCESS;
        }
    }
    else
    {
        IotLogError( "Writing %#04X on I2C failed.", address );
    }
    
    return DHT12_FAIL; 
}

/**
 * @brief   Write a number of bytes to a DHT12 register (and consecutive subsequent registers)
 * @param   value:      external array of data to write. Put starting register in values[0].
 * @param   length:     number of bytes to write
 * @return  DHT12_SUCCESS success
 *          DHT12_FAIL errors found
*/
dht12_err_t prvWriteBytes( uint8_t * values, uint8_t length )
{
    // Return value of I2C functions.
    dht12_err_t res = DHT12_FAIL;

    DHT12_CHECK_HANDLER();

    // Register address on I2C slave device.
    uint16_t ioCtlBuffer = DHT12_ADDRESS;
 
    // Number of read/write bytes.
    uint16_t usReadBytes = 0;
    uint16_t usWriteBytes = 0;

    uint8_t address = values[0];

    // Set slave address.
    res = iot_i2c_ioctl( prvI2CHandle, eI2CSetSlaveAddr, &ioCtlBuffer );
    if ( res != IOT_I2C_SUCCESS )
    {
        IotLogError( "Setting the slave address for %#04X on I2C failed.", ioCtlBuffer );
        return DHT12_FAIL;
    }
    // assert(lRetVal == IOT_I2C_SUCCESS);

    // Write the register address as single byte, in a transaction.
    res = iot_i2c_write_sync( prvI2CHandle, values, length );
    if ( res == IOT_I2C_SUCCESS )
    {
        // Get the number of written bytes in last transaction.
        res = iot_i2c_ioctl( prvI2CHandle, eI2CGetTxNoOfbytes, &usWriteBytes );
        if ( res != IOT_I2C_SUCCESS || usWriteBytes != length )
        {
            IotLogError( "Failed to check the number of written bytes %u vs. %u.", usWriteBytes, 1 );
            return DHT12_FAIL;
        }

        IotLogDebug( "Wrote %u bytes @ %#04X", length, address );

        return DHT12_SUCCESS;
    }
    else
    {
        IotLogError( "Writing %u bytes on I2C failed.", length );
    }
    
    return DHT12_FAIL; 
}

/*-----------------------------------------------------------*/

dht12_err_t readTemperature( double * T, dht12_unit_t unit )
{	
    dht12_err_t res = DHT12_FAIL;
    uint8_t data[ 5 ] = { 0 };

    res = prvReadBytes( data, 5 );
    if ( res == DHT12_SUCCESS )
    {
        // vTaskDelay( pdMS_TO_TICKS( 50 ) );

        if ( data[ 4 ] != ( data[ 0 ] + data[ 1 ] + data[ 2 ] + data[ 3 ] ) )
        {
            return DHT12_CHECKSUM_ERROR;
        }

        switch( unit ) {
            case CELSIUS:
                *T = ( data[ 2 ] + (double)data[ 3 ] / 10 );
                break;
            case FAHRENHEIT:
                *T = ( data[ 2 ] + (double)data[ 3 ] / 10 ) * 1.8 + 32;
                break;
            case KELVIN:
                *T = ( data[ 2 ] + (double)data[ 3 ] / 10 ) + 273.15;
                break;
            default:
                return DHT12_FAIL;
        };

        IotLogDebug( "Temperature: T = %lf", *T );
        return DHT12_SUCCESS;
    }
    else
    {
        IotLogError( "Read Temperature failed" );
        return DHT12_FAIL;
    }
}

/*-----------------------------------------------------------*/
  
dht12_err_t readHumidity( double * H );
{
	dht12_err_t res = DHT12_FAIL;
    uint8_t data[ 5 ] = { 0 };

    res = prvReadBytes( data, 5 );
    if ( res == DHT12_SUCCESS )
    {
        // vTaskDelay( pdMS_TO_TICKS( 50 ) );

        if ( data[ 4 ] != ( data[ 0 ] + data[ 1 ] + data[ 2 ] + data[ 3 ] ) )
        {
            return DHT12_CHECKSUM_ERROR;
        }

        *H = ( data[ 0 ] + (double)data[ 1 ] / 10 );

        IotLogDebug( "Humidity: H = %lf", *H );
        return DHT12_SUCCESS;
    }
    else
    {
        IotLogError( "Read Humidity failed" );
        return DHT12_FAIL;
    }
}

/*-----------------------------------------------------------*/
