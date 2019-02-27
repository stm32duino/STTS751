/**
 ******************************************************************************
 * @file    STTS751Sensor.cpp
 * @author  SRA
 * @version V1.0.0
 * @date    February 2019
 * @brief   Implementation of a STTS751 pressure sensor. 
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/

#include "STTS751Sensor.h"


/* Class Implementation ------------------------------------------------------*/

/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 */
STTS751Sensor::STTS751Sensor(TwoWire *i2c, uint8_t address) : dev_i2c(i2c), address(address)
{
  reg_ctx.write_reg = STTS751_io_write;
  reg_ctx.read_reg = STTS751_io_read;
  reg_ctx.handle = (void *)this;
  
  /* Disable EVENT pin of SMBus. */
  if (stts751_pin_event_route_set(&reg_ctx,  PROPERTY_ENABLE) != STTS751_OK)
  {
    return;
  }
  /* Set default ODR */
  temp_odr = 1.0f;
  /* Set the resolution to the maximum allowed value */
  if (stts751_resolution_set(&reg_ctx, STTS751_12bit) != STTS751_OK)
  {
    return;
  }
  /* Put the component in standby mode. */
  if (stts751_temp_data_rate_set(&reg_ctx, STTS751_TEMP_ODR_OFF) != STTS751_OK)
  {
    return;
  }
  
  temp_is_enabled = 0;
  
  return;
}

/**
 * @brief  Get WHO_AM_I value
 * @param  Id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
STTS751StatusTypeDef STTS751Sensor::ReadID(uint8_t *Id)
{
  stts751_id_t buf;

  if (stts751_device_id_get(&reg_ctx, &buf) != STTS751_OK)
  {
    return STTS751_ERROR;
  }

  *Id = buf.manufacturer_id;

  return STTS751_OK;
}

/**
 * @brief  Enable the STTS751 temperature sensor
 * @retval 0 in case of success, an error code otherwise
 */
STTS751StatusTypeDef STTS751Sensor::Enable()
{
  /* Check if the component is already enabled */
  if (temp_is_enabled == 1U)
  {
    return STTS751_OK;
  }

  /* Power on the component and set the odr. */
  if (SetOutputDataRate(temp_odr) != STTS751_OK)
  {
    return STTS751_ERROR;
  }

  temp_is_enabled = 1;

  return STTS751_OK;
}

/**
 * @brief  Disable the STTS751 temperature sensor
 * @retval 0 in case of success, an error code otherwise
 */
STTS751StatusTypeDef STTS751Sensor::Disable()
{
  /* Check if the component is already disabled */
  if (temp_is_enabled == 0U)
  {
    return STTS751_OK;
  }

  /* Save the current odr. */
  if (GetOutputDataRate(&temp_odr) != STTS751_OK)
  {
    return STTS751_ERROR;
  }
  
  /* Put the component in standby mode. */
  if (stts751_temp_data_rate_set(&reg_ctx, STTS751_TEMP_ODR_OFF) != STTS751_OK)
  {
    return STTS751_ERROR;
  }

  temp_is_enabled = 0;

  return STTS751_OK;
}

/**
 * @brief  Get the STTS751 temperature sensor output data rate
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
STTS751StatusTypeDef STTS751Sensor::GetOutputDataRate(float *Odr)
{
  STTS751StatusTypeDef ret = STTS751_OK;
  stts751_odr_t odr_low_level;

  if (stts751_temp_data_rate_get(&reg_ctx, &odr_low_level) != STTS751_OK)
  {
    return STTS751_ERROR;
  }

  switch (odr_low_level)
  {
    case STTS751_TEMP_ODR_OFF:
	case STTS751_TEMP_ODR_ONE_SHOT:
      *Odr = 0.0f;
      break;

    case STTS751_TEMP_ODR_62mHz5:
      *Odr = 0.0625f;
      break;

    case STTS751_TEMP_ODR_125mHz:
      *Odr = 0.125f;
      break;

    case STTS751_TEMP_ODR_250mHz:
      *Odr = 0.250f;
      break;

    case STTS751_TEMP_ODR_500mHz:
      *Odr = 0.500f;
      break;

    case STTS751_TEMP_ODR_1Hz:
      *Odr = 1.0f;
      break;

    case STTS751_TEMP_ODR_2Hz:
      *Odr = 2.0f;
      break;

    case STTS751_TEMP_ODR_4Hz:
      *Odr = 4.0f;
      break;

    case STTS751_TEMP_ODR_8Hz:
      *Odr = 8.0f;
      break;

    case STTS751_TEMP_ODR_16Hz:
      *Odr = 16.0f;
      break;

    case STTS751_TEMP_ODR_32Hz:
      *Odr = 32.0f;
      break;

    default:
      ret = STTS751_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the STTS751 temperature sensor output data rate
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
STTS751StatusTypeDef STTS751Sensor::SetOutputDataRate(float Odr)
{
  stts751_odr_t new_odr;
  stts751_tres_t res;

  /* Get the current resolution */
  if (stts751_resolution_get(&reg_ctx, &res) != STTS751_OK)
  {
    return STTS751_ERROR;
  }

  /* If the requested odr is 16Hz we cannot use the 12 bits resolution */
  if(Odr == 16.0f && res == STTS751_12bit)
  {
    /* We force resolution to the maximum allowed value */
    if (stts751_resolution_set(&reg_ctx, STTS751_11bit) != STTS751_OK)
    {
      return STTS751_ERROR;
    }
  }

  /* If the requested odr is 32Hz we cannot use the 12 bits and 11 bits resolutions */
  if(Odr == 32.0f && (res == STTS751_12bit || res == STTS751_11bit))
  {
    /* We force resolution to the maximum allowed value */
    if (stts751_resolution_set(&reg_ctx, STTS751_10bit) != STTS751_OK)
    {
      return STTS751_ERROR;
    }
  }

  new_odr = (Odr <= 0.0625f) ? STTS751_TEMP_ODR_62mHz5
          : (Odr <= 0.125f ) ? STTS751_TEMP_ODR_125mHz
          : (Odr <= 0.25f  ) ? STTS751_TEMP_ODR_250mHz
          : (Odr <= 0.5f   ) ? STTS751_TEMP_ODR_500mHz
          : (Odr <= 1.0f   ) ? STTS751_TEMP_ODR_1Hz
          : (Odr <= 2.0f   ) ? STTS751_TEMP_ODR_2Hz
          : (Odr <= 4.0f   ) ? STTS751_TEMP_ODR_4Hz
          : (Odr <= 8.0f   ) ? STTS751_TEMP_ODR_8Hz
          : (Odr <= 16.0f  ) ? STTS751_TEMP_ODR_16Hz
          :                    STTS751_TEMP_ODR_32Hz;

  if (stts751_temp_data_rate_set(&reg_ctx, new_odr) != STTS751_OK)
  {
    return STTS751_ERROR;
  }

  return STTS751_OK;
}

/**
 * @brief  Get the STTS751 temperature value
 * @param  Value pointer where the temperature value is written
 * @retval 0 in case of success, an error code otherwise
 */
STTS751StatusTypeDef STTS751Sensor::GetTemperature(float *Value)
{
  int16_t raw_value = 0;

  /* Get the temperature */
  if (stts751_temperature_raw_get(&reg_ctx, &raw_value) != STTS751_OK)
  {
    return STTS751_ERROR;
  }

  *Value = stts751_from_lsb_to_celsius(raw_value);

  return STTS751_OK;
}

/**
 * @brief  Get the STTS751 temperature data ready bit value
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
STTS751StatusTypeDef STTS751Sensor::Get_DRDY_Status(uint8_t *Status)
{
  uint8_t val;

  if (stts751_flag_busy_get(&reg_ctx, &val) != STTS751_OK)
  {
    return STTS751_ERROR;
  }

  if(val)
  {
    *Status = 0;
  } else
  {
    *Status = 1;
  }

  return STTS751_OK;
}

/**
 * @brief  Set the STTS751 low temperature threshold value
 * @param  Value the high temperature threshold to be set
 * @retval 0 in case of success, an error code otherwise
 */
STTS751StatusTypeDef STTS751Sensor::SetLowTemperatureThreshold(float Value)
{
  int16_t raw_value;

  raw_value = stts751_from_celsius_to_lsb(Value);

  /* Set the temperature threshold */
  if (stts751_low_temperature_threshold_set(&reg_ctx, raw_value) != STTS751_OK)
  {
    return STTS751_ERROR;
  }

  return STTS751_OK;
}

/**
 * @brief  Set the STTS751 high temperature threshold value
 * @param  Value the high temperature threshold to be set
 * @retval 0 in case of success, an error code otherwise
 */
STTS751StatusTypeDef STTS751Sensor::SetHighTemperatureThreshold(float Value)
{
  int16_t raw_value;

  raw_value = stts751_from_celsius_to_lsb(Value);

  /* Set the temperature threshold */
  if (stts751_high_temperature_threshold_set(&reg_ctx, raw_value) != STTS751_OK)
  {
    return STTS751_ERROR;
  }

  return STTS751_OK;
}

/**
 * @brief  Get the STTS751 temperature limits status
 * @param  HighLimit indicates that high temperature limit has been exceeded
 * @param  LowhLimit indicates that low temperature limit has been exceeded
 * @param  ThermLimit indicates that therm temperature limit has been exceeded
 * @retval 0 in case of success, an error code otherwise
 */
STTS751StatusTypeDef STTS751Sensor::GetTemperatureLimitStatus(uint8_t *HighLimit, uint8_t *LowLimit, uint8_t *ThermLimit)
{
  stts751_status_t status;

  /* Read status register */
  if (stts751_status_reg_get(&reg_ctx, &status) != STTS751_OK)
  {
    return STTS751_ERROR;
  }

  *HighLimit = status.t_high;
  *LowLimit = status.t_low;
  *ThermLimit = status.thrm;

  return STTS751_OK;
}

/**
 * @brief  Enable or disable interrupt on EVENT pin
 * @param  Enable 0 disable the EVENT pin, 1 enable EVENT pin
 * @retval 0 in case of success, an error code otherwise
 */
STTS751StatusTypeDef STTS751Sensor::SetEventPin(uint8_t Enable)
{
  uint8_t state;

  /* The MASK1 bit in configuration register has inverted logic */
  if (Enable == 0) state = PROPERTY_ENABLE; else state = PROPERTY_DISABLE;

  if (stts751_pin_event_route_set(&reg_ctx,  state) != STTS751_OK)
  {
    return STTS751_ERROR;
  }

  return STTS751_OK;
}

/**
 * @brief  Get the STTS751 register value
 * @param  Reg address to be read
 * @param  Data pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
STTS751StatusTypeDef STTS751Sensor::Read_Reg(uint8_t Reg, uint8_t *Data)
{
  if (stts751_read_reg(&reg_ctx, Reg, Data, 1) != STTS751_OK)
  {
    return STTS751_ERROR;
  }

  return STTS751_OK;
}

/**
 * @brief  Set the STTS751 register value
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
STTS751StatusTypeDef STTS751Sensor::Write_Reg(uint8_t Reg, uint8_t Data)
{
  if (stts751_write_reg(&reg_ctx, Reg, &Data, 1) != STTS751_OK)
  {
    return STTS751_ERROR;
  }

  return STTS751_OK;
}

/**
 * @brief  Set the STTS751 One Shot Mode
 * @retval 0 in case of success, an error code otherwise
 */
STTS751StatusTypeDef STTS751Sensor::Set_One_Shot()
{
  /* Start One Shot Measurement */
  if(stts751_temp_data_rate_set(&reg_ctx, STTS751_TEMP_ODR_ONE_SHOT) != STTS751_OK)
  {
    return STTS751_ERROR;
  }

  return STTS751_OK;
}

/**
 * @brief  Get the STTS751 One Shot Status
 * @param  Status pointer to the one shot status (1 means measurements available, 0 means measurements not available yet)
 * @retval 0 in case of success, an error code otherwise
 */
STTS751StatusTypeDef STTS751Sensor::Get_One_Shot_Status(uint8_t *Status)
{
  uint8_t busy;

  /* Get Busy flag */
  if(stts751_flag_busy_get(&reg_ctx, &busy) != STTS751_OK)
  {
    return STTS751_ERROR;
  }

  if(busy)
  {
    *Status = 0;
  }
  else
  {
    *Status = 1;
  }

  return STTS751_OK;
}


int32_t STTS751_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
  return ((STTS751Sensor *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}

int32_t STTS751_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
  return ((STTS751Sensor *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}
