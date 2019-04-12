/**
 ******************************************************************************
 * @file    STTS751Sensor.h
 * @author  SRA
 * @version V1.0.0
 * @date    February 2018
 * @brief   Abstract Class of a STTS751 temperature sensor.
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


/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __STTS751Sensor_H__
#define __STTS751Sensor_H__


/* Includes ------------------------------------------------------------------*/

#include "Wire.h"
#include "stts751_reg.h"

/* Defines -------------------------------------------------------------------*/



/* Typedefs ------------------------------------------------------------------*/

typedef enum
{
  STTS751_OK = 0,
  STTS751_ERROR =-1
} STTS751StatusTypeDef;


/* Class Declaration ---------------------------------------------------------*/
   
/**
 * Abstract class of a LPS22HH pressure sensor.
 */
class STTS751Sensor
{
  public:
    STTS751Sensor(TwoWire *i2c, uint8_t address=STTS751_1xxxx_ADD_7K5);
    STTS751StatusTypeDef ReadID(uint8_t *Id);
    STTS751StatusTypeDef Enable();
    STTS751StatusTypeDef Disable();
    STTS751StatusTypeDef GetOutputDataRate(float *Odr);
    STTS751StatusTypeDef SetOutputDataRate(float Odr);
    STTS751StatusTypeDef GetTemperature(float *Value);
    STTS751StatusTypeDef Get_DRDY_Status(uint8_t *Status);
    STTS751StatusTypeDef SetLowTemperatureThreshold(float Value);
    STTS751StatusTypeDef SetHighTemperatureThreshold(float Value);
    STTS751StatusTypeDef GetTemperatureLimitStatus(uint8_t *HighLimit, uint8_t *LowLimit, uint8_t *ThermLimit);
    STTS751StatusTypeDef SetEventPin(uint8_t Enable);
    STTS751StatusTypeDef Read_Reg(uint8_t Reg, uint8_t *Data);
    STTS751StatusTypeDef Write_Reg(uint8_t Reg, uint8_t Data);
    STTS751StatusTypeDef Set_One_Shot();
    STTS751StatusTypeDef Get_One_Shot_Status(uint8_t *Status);

    
    /**
     * @brief Utility function to read data.
     * @param  pBuffer: pointer to data to be read.
     * @param  RegisterAddr: specifies internal address register to be read.
     * @param  NumByteToRead: number of bytes to be read.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Read(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
    {        
		
      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));
        dev_i2c->write(RegisterAddr);
        dev_i2c->endTransmission(false);

        dev_i2c->requestFrom(((uint8_t)(((address) >> 1) & 0x7F)), (uint8_t) NumByteToRead);

        int i=0;
        while (dev_i2c->available()) {
          pBuffer[i] = dev_i2c->read();
          i++;
        }

        return 0;
      }

      return 1;
    }
    
    /**
     * @brief Utility function to write data.
     * @param  pBuffer: pointer to data to be written.
     * @param  RegisterAddr: specifies internal address register to be written.
     * @param  NumByteToWrite: number of bytes to write.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Write(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
    {  
      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));

        dev_i2c->write(RegisterAddr);
        for (uint16_t i = 0 ; i < NumByteToWrite ; i++) {
          dev_i2c->write(pBuffer[i]);
        }

        dev_i2c->endTransmission(true);

        return 0;
      }

      return 1;
    }

  private:

    /* Helper classes. */
    TwoWire *dev_i2c;
    
    /* Configuration */
    uint8_t address;
    
    float temp_odr;
    uint8_t temp_is_enabled;
    
    stts751_ctx_t reg_ctx;
    
};

#ifdef __cplusplus
 extern "C" {
#endif
int32_t STTS751_io_write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
int32_t STTS751_io_read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
#ifdef __cplusplus
  }
#endif

#endif
