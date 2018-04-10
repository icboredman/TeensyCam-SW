// =============================================================================
// This code is .......
//
// Copyright (c) 2017 boredman <http://BoredomProjects.net>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// =============================================================================


#pragma once

#include <i2c_t3.h>
#include "mt9v034_registermap.h"



class mt9v034
{

public:

  mt9v034(i2c_t3* wr, uint8_t addr)
  {
    wire = wr;
    device_addr = addr;
  }
  
  uint16_t getVersion(void)
  {
    uint16_t ver = 0;
    readReg16(MTV_CHIP_VERSION_REG, &ver);
    return ver;
  }

  uint8_t initialize(void)
  {
    // slave mode
    writeReg16( MTV_CHIP_CONTROL_REG,  MTV_CHIP_CONTROL_SLAVE_MODE
                                     | MTV_CHIP_CONTROL_DOUT_ENABLE
                                     | MTV_CHIP_CONTROL_SEQUENTIAL );
    // column flip
    writeReg16( MTV_READ_MODE_REG_A, MTV_READ_MODE_COL_FLIP );
  }

  void extra(void)
  {
    //setDigitalGain(8);
  }
  
  void setCompanding(bool comp)
  {
    writeReg16( MTV_ADC_RES_CTRL_REG, (comp ? MTV_ADC_RES_COMP_A : MTV_ADC_RES_LINR_A) );
  }

  void setAGC(bool agc_on)
  {
    writeReg16( MTV_AEC_AGC_ENABLE_REG, (agc_on ? MTV_AGC_ENABLE_A : MTV_AGC_DISABLE_A) 
                                       | MTV_AEC_DISABLE_A );
  }
  
  // valid rage: 16 - 64 dec
  void setAnalogGain(uint8_t ag)
  {
    uint16_t val = ag;
    if( ag < 16 )
      val = 16 | MTV_ANALOG_GAIN_FORCE_0_75;
    else if( ag > 64 )
      val = 64;
    writeReg16( MTV_ANALOG_GAIN_CTRL_REG_A, val );
  }

  void setDigitalGain(uint16_t dg)
  {
    if( dg > 15 )
      dg = 15;
    for(uint16_t i=0; i<=24; i++)
      writeReg16( MTV_TILED_DIGITAL_GAIN_REG + i, dg | (0xF << 4) );
  }

protected:

  i2c_t3* wire;
  uint8_t device_addr;

  uint8_t writeReg16(uint8_t reg, uint16_t val)
  {
    wire->beginTransmission(device_addr);
    wire->write(reg);
    wire->write(val>>8);
    wire->write(val&0xFF);
    return wire->endTransmission();
  }
  
  uint8_t readReg16(uint8_t reg, uint16_t *val)
  {
    wire->beginTransmission(device_addr);
    wire->write(reg);
    wire->endTransmission(I2C_NOSTOP);
    wire->requestFrom(device_addr, 2);
    uint8_t res[2];
    if( wire->read(res, 2) != 2 )
      return -1;
    *val = res[0] << 8 | res[1];
    return 0;
  }
  


};

