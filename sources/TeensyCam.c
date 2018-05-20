/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    TeensyCam.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK66F18.h"
/* TODO: insert other include files here. */
status_t I2C_Write16(uint8_t dev, uint8_t reg, uint16_t val);
status_t I2C_Read16(uint8_t dev, uint8_t reg, uint16_t* val);
#include "mt9v034.h"
#include "usb.h"

/* TODO: insert other definitions and declarations here. */
extern void APPInit(void);
extern void APPTask(void);
extern usb_status_t USB_Send(uint8_t* buf, size_t len);

#define ARM_DEMCR				(*(volatile uint32_t *)0xE000EDFC) // Debug Exception and Monitor Control
#define ARM_DEMCR_TRCENA		(1 << 24)        // Enable debugging & monitoring blocks
#define ARM_DWT_CTRL			(*(volatile uint32_t *)0xE0001000) // DWT control register
#define ARM_DWT_CTRL_CYCCNTENA	(1 << 0)                // Enable cycle count
#define ARM_DWT_CYCCNT			(*(volatile uint32_t *)0xE0001004) // Cycle count register

#define CLK_CNTR_RESET()	{ ARM_DEMCR |= ARM_DEMCR_TRCENA;	\
							  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA; \
							  ARM_DWT_CYCCNT = 0; }
#define CLK_CNTR_VALUE		ARM_DWT_CYCCNT

static const uint8_t mt1addr = 0x90;
static const uint8_t mt2addr = 0xB0;

/*
 * @brief   Application entry point.
 */
int main(void) {
  	/* Init board hardware. */
	BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

	//GPIO_PinWrite(BOARD_INITPINS_STDBY_GPIO, 1u << BOARD_INITPINS_STDBY_PIN, 0);

	APPInit();

	uint16_t ver;
	status_t st = MT9GetVersion(mt1addr, &ver);
	char str[50];
	sprintf(str, "ver = %d (%d)\r\n", ver, st);

    const uint32_t cycles = 5 * 240000000;

    while(1)
    {
    	CLK_CNTR_RESET();

    	while (CLK_CNTR_VALUE < cycles)
    	{
    		APPTask();
    	}

    	//GPIO_PortToggle(BOARD_INITPINS_LED_GPIO, 1u << BOARD_INITPINS_LED_PIN);

    	st = MT9GetVersion(mt1addr, &ver);
    	sprintf(str, "ver = %d (%d)\r\n", ver, st);
    	USB_Send((uint8_t*)str, strlen(str));

    }
    return 0 ;
}


status_t I2C_Write16(uint8_t dev, uint8_t reg, uint16_t val)
{
    i2c_master_transfer_t masterXfer;
    uint8_t g_master_txBuff[2];

    *(uint16_t*)g_master_txBuff = val;

    /* subAddress = 0x01, data = g_master_txBuff - write to slave.
      start + slaveaddress(w) + subAddress + length of data buffer + data buffer + stop*/
//    uint8_t deviceAddress = 0x01U;
    masterXfer.slaveAddress = dev >> 1;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = (uint32_t)reg;
    masterXfer.subaddressSize = 1;
    masterXfer.data = g_master_txBuff;
    masterXfer.dataSize = 2;
    masterXfer.flags = kI2C_TransferDefaultFlag;

	return I2C_MasterTransferBlocking(I2C_PERIPHERAL, &masterXfer);
}

status_t I2C_Read16(uint8_t dev, uint8_t reg, uint16_t* val)
{
    i2c_master_transfer_t masterXfer;
    //uint8_t g_master_rxBuff[2];

    /* subAddress = 0x01, data = g_master_rxBuff - read from slave.
      start + slaveaddress(w) + subAddress + repeated start + slaveaddress(r) + rx data buffer + stop */
    masterXfer.slaveAddress = dev >> 1;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = (uint32_t)reg;
    masterXfer.subaddressSize = 1;
    masterXfer.data = (uint8_t*)val;
    masterXfer.dataSize = 2;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    return I2C_MasterTransferBlocking(I2C_PERIPHERAL, &masterXfer);
}
