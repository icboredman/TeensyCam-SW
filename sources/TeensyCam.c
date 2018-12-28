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
#include "fsl_pit.h"

/* TODO: insert other definitions and declarations here. */
extern void APPInit(void);
extern void APPTask(void);
extern usb_status_t USB_Send(uint8_t* buf, size_t len);
extern usb_status_t USB_CheckBusy(void);
extern bool USB_Recv(uint8_t *buf, uint32_t len);

void __attribute__ ((noinline)) TakeSnapshot(void);
static void PulsePins(uint32_t pin, uint32_t cnt);
static __attribute__ ((noinline)) bool ReadCameraLine(uint16_t *a, uint16_t *b, uint16_t px);

static void SendLine(uint16_t line, uint16_t *data1, uint16_t *data2, uint16_t dataSize);
static void Pack8bits(uint8_t *dstByte, uint16_t *srcWord, size_t srcLength);
static void Pack10bits(uint8_t *dstByte, uint16_t *srcWord, size_t srcLength);

#define FPS_TICK_HANDLER PIT0_IRQHandler
#define PIT_IRQ_ID PIT0_IRQn
bool take_snapshot = false;

#define ARM_DEMCR				(*(volatile uint32_t *)0xE000EDFC) // Debug Exception and Monitor Control
#define ARM_DEMCR_TRCENA		(1 << 24)        // Enable debugging & monitoring blocks
#define ARM_DWT_CTRL			(*(volatile uint32_t *)0xE0001000) // DWT control register
#define ARM_DWT_CTRL_CYCCNTENA	(1 << 0)                // Enable cycle count
#define ARM_DWT_CYCCNT			(*(volatile uint32_t *)0xE0001004) // Cycle count register

#define CLK_CNTR_RESET()		{ ARM_DEMCR |= ARM_DEMCR_TRCENA;	\
							  	  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA; \
							  	  ARM_DWT_CYCCNT = 0; }
#define CLK_CNTR_VALUE		  	ARM_DWT_CYCCNT
#define CLK_CNTR_DELAY_US(us)	{ CLK_CNTR_RESET();		\
								  while(CLK_CNTR_VALUE < us*(BOARD_BOOTCLOCKHSRUN_CORE_CLOCK/1000000)){} }

#define PORT_CAM1	(GPIOC->PDIR)
#define PORT_CAM2	(GPIOD->PDIR)

static const uint8_t mt1addr = 0x90;
static const uint8_t mt2addr = 0xB0;

#define LIGHT_FREQ	50U			// indoor lighting flicker rate in Hz

// 4-byte alignment
typedef struct {
	uint32_t exposure_us;
	uint32_t analogGain;
	uint32_t digitalGain;
	uint32_t n_lines;
	uint32_t cpf;
	uint32_t aec_enable;
	uint32_t agc_enable;
	uint32_t cmp_enable;
} cam_config_t;

cam_config_t camconfig;

bool adjust_exposure = false;
uint16_t camRegs[6];

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

    pit_config_t pitConfig;    // Structure of initialize PIT
    PIT_GetDefaultConfig(&pitConfig);
    PIT_Init(PIT, &pitConfig);    // Init pit module
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, CLOCK_GetFreq(kCLOCK_BusClk) / LIGHT_FREQ);    // Set timer period for channel 0
    PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);    // Enable timer interrupts for channel 0
    EnableIRQ(PIT_IRQ_ID);    // Enable at the NVIC
    PIT_StartTimer(PIT, kPIT_Chnl_0);	// Start channel 0

	GPIO_PinWrite(OUTPUTS_GPIO, BOARD_INITPINS_STANDBY_PIN, 0);

	APPInit();

	CLK_CNTR_DELAY_US(1000000);

    camconfig.exposure_us = 4000;	// up to 16383 us
    camconfig.analogGain = 16;		// 16 - 64
    camconfig.digitalGain = 4;		// 0 - 15
    camconfig.n_lines = 100;		// MAX_IMAGE_HEIGHT;
    camconfig.cpf = 25;				// 2 Hz
    camconfig.aec_enable = true;
    camconfig.agc_enable = false;
    camconfig.cmp_enable = true;

    // configure camera slave mode
    MT9Initialize(mt1addr);
    MT9Initialize(mt2addr);
    MT9SetAGC_AEC(mt1addr, camconfig.agc_enable, false);
    MT9SetAGC_AEC(mt2addr, camconfig.agc_enable, false);
    MT9SetCompanding(mt1addr, camconfig.cmp_enable);
    MT9SetCompanding(mt2addr, camconfig.cmp_enable);

    while(1)
    {
    	if (take_snapshot)
    	{
			take_snapshot = false;
    		TakeSnapshot();
    	}

        if (USB_Recv((uint8_t*)&camconfig, sizeof(camconfig)))
        {
            MT9SetAGC_AEC(mt1addr, camconfig.agc_enable, false);
            MT9SetAGC_AEC(mt2addr, camconfig.agc_enable, false);
            MT9SetCompanding(mt1addr, camconfig.cmp_enable);
            MT9SetCompanding(mt2addr, camconfig.cmp_enable);
			Mt9SetAnalogGain(mt1addr, camconfig.analogGain);
			Mt9SetAnalogGain(mt2addr, camconfig.analogGain);
			MT9SetDigitalGain(mt1addr, camconfig.digitalGain);
			MT9SetDigitalGain(mt2addr, camconfig.digitalGain);
        }

        if (adjust_exposure)
        {
        	adjust_exposure = false;

        	I2C_Read16(mt1addr, MTV_AGC_AEC_CURRENT_BIN_REG, &camRegs[0]);
        	I2C_Read16(mt2addr, MTV_AGC_AEC_CURRENT_BIN_REG, &camRegs[1]);

        	// proportional control:
        	if (camconfig.aec_enable)
        	{
				int diff = (int)camRegs[0] - 0x002C;
				int amount;
				if (diff > 5)
					amount = -200;
				else if (diff < -5)
					amount = 200;
				else
					amount = diff * (-2);

				camconfig.exposure_us += amount;

				if (camconfig.exposure_us > 16383)
					camconfig.exposure_us = 16383;
        	}
        	camRegs[2] = camconfig.exposure_us;
        }

        //CLK_CNTR_DELAY_US(1000);
    }

    return 0 ;
}


void __attribute__ ((noinline)) TakeSnapshot()
{
	uint16_t ln_cnt = 0;
	uint16_t nv_cnt = 0;
	uint16_t zr_cnt = 0;

	uint16_t a[MAX_IMAGE_WIDTH], b[MAX_IMAGE_WIDTH];

	PulsePins(BOARD_INITPINS_EXPOSURE_PIN, 50);
	CLK_CNTR_DELAY_US(camconfig.exposure_us);
	PulsePins(BOARD_INITPINS_STFRM_OUT_PIN, 50);

	for (int ln=0; ln<530; ln++)   // min 525 lines, including blanking
	{
		//PulsePins(BOARD_INITPINS_STLN_OUT1_PIN, 40);

		if( ReadCameraLine(a, b, MAX_IMAGE_WIDTH) )
		{	// make sure last data point contains valid line and frame markers
			if( (a[0]>>10) != 0x3 || (b[0]>>10) != 0xC )
				zr_cnt++;
			else
				if( (ln_cnt >= MAX_IMAGE_HEIGHT/2 - camconfig.n_lines/2) &&
					(ln_cnt <  MAX_IMAGE_HEIGHT/2 + camconfig.n_lines/2) )
				{
					SendLine(ln_cnt, a, b, MAX_IMAGE_WIDTH);
					//CLK_CNTR_DELAY_US(10);
				}

			ln_cnt++;
		}
		else
			nv_cnt++;
	}

	camRegs[3] = ln_cnt;
	camRegs[4] = nv_cnt;
	camRegs[5] = zr_cnt;
}


void FPS_TICK_HANDLER(void)
{
	static unsigned frmCounter = 0;
	static unsigned aecCounter = 0;

    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);

	if (++frmCounter >= camconfig.cpf)
	{
		frmCounter = 0;
		take_snapshot = true;
	}

	if (++aecCounter >= (LIGHT_FREQ/2))		// 0.5 sec
	{
		aecCounter = 0;
		adjust_exposure = true;
	}
}


/***************************************************
 * Utility Functions
 ***************************************************/

static void PulsePins(uint32_t pin, uint32_t cnt)
{
	//__disable_irq();

	GPIO_PinWrite(OUTPUTS_GPIO, pin, 1);

    while (cnt != 0)
    {
        if (GPIO_PinRead(INPUTS_GPIO, BOARD_INITPINS_PIXCLK1_PIN))
            cnt--;
    }

    GPIO_PinWrite(OUTPUTS_GPIO, pin, 0);

    //__enable_irq();
}


static __attribute__ ((noinline)) bool ReadCameraLine(uint16_t *a, uint16_t *b, uint16_t px)
{
	//  int px = 750;  // max (752-1)
	px -= 3;

	__disable_irq();

	PulsePins(BOARD_INITPINS_STLN_OUT1_PIN, 50);

	// PIXCLKs between STLN_OUT1 pulses
	// need about 60 extra pulses before LINE_VALID, so min is 752 + 60 = 812
	int cnt = 820;
	// if no LINE_VALID, simply count PIXCLKs until it's time for next STLN_OUT pulse
	while ( ! GPIO_PinRead(INPUTS_GPIO, BOARD_INITPINS_LNVAL1_PIN) )
	{	// count PIXCLKs
		if ( GPIO_PinRead(INPUTS_GPIO, BOARD_INITPINS_PIXCLK1_PIN) )
		{
			if (--cnt == 0)
			{
				__enable_irq();
				return false;
			}
		}
	}

	// capture one line of data
    while (1)
    {	// sample data on low level of PIXCLK (first pixel may be lost)
        if ( ! GPIO_PinRead(INPUTS_GPIO, BOARD_INITPINS_PIXCLK1_PIN) )
        {
            register uint16_t d1 = PORT_CAM1;
            register uint16_t d2 = PORT_CAM2;
            a[px] = d1;
            b[px] = d2;
            if( px-- == 0 )
                break;
        }
    }

	__enable_irq();
	return true;
}


static usb_status_t USB_SendRetry(uint8_t* buf, size_t len, int count)
{
	usb_status_t st;

	// try sending 'count' times with small pauses in between:
	while (kStatus_USB_Success != (st=USB_Send(buf, len)))
	{
		if (0 == count--)
			break;
		CLK_CNTR_DELAY_US(10);
	}

	return st;
}


USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) uint8_t s_imgSendBuf[2048];


static void SendLine(uint16_t line, uint16_t *data1, uint16_t *data2, uint16_t dataSize)
{
//	if (false == send_picture_data)
//		return;

	s_imgSendBuf[0] = (0x00) | (line >> 8);
	s_imgSendBuf[1] = line;

	for(int i=0; i<12; i++)
		s_imgSendBuf[2+i] = ((uint8_t*)camRegs)[i];

	Pack8bits(s_imgSendBuf+2+12, data1, dataSize);
	Pack8bits(s_imgSendBuf+2+12+dataSize, data2, dataSize);

	USB_Send(s_imgSendBuf, dataSize*2+2+12);
}


//
// dstLength = srcLength
//
static void Pack8bits(uint8_t *dstByte, uint16_t *srcWord, size_t srcLength)
{
	for (size_t i = 0; i < srcLength; i++)
	{
		dstByte[i] = srcWord[i] >> 2;
	}
}


//
// dstLength = srcLength / 4 * 5
//
static void Pack10bits(uint8_t *dstByte, uint16_t *srcWord, size_t srcLength)
{
	for (size_t i = 0; i < srcLength; i+=4)
	{
		dstByte[0] = srcWord[0];
		dstByte[1] = ((srcWord[0] & 0x3FF) >> 8) | (srcWord[1] << 2);
		dstByte[2] = ((srcWord[1] & 0x3FF) >> 6) | (srcWord[2] << 4);
		dstByte[3] = ((srcWord[2] & 0x3FF) >> 4) | (srcWord[3] << 6);
		dstByte[4] = ((srcWord[3] & 0x3FF) >> 2);
		dstByte += 5;
		srcWord += 4;
	}
}



/***************************************************
 * I2C Functions
 ***************************************************/

status_t I2C_Write16(uint8_t dev, uint8_t reg, uint16_t val)
{
    i2c_master_transfer_t masterXfer;
    uint8_t g_master_txBuff[2];

    g_master_txBuff[0] = val >> 8;
    g_master_txBuff[1] = val & 0xFF;

    /* subAddress = 0x01, data = g_master_txBuff - write to slave.
      start + slaveaddress(w) + subAddress + length of data buffer + data buffer + stop*/
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
    uint8_t g_master_rxBuff[2];

    /* subAddress = 0x01, data = g_master_rxBuff - read from slave.
      start + slaveaddress(w) + subAddress + repeated start + slaveaddress(r) + rx data buffer + stop */
    masterXfer.slaveAddress = dev >> 1;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = (uint32_t)reg;
    masterXfer.subaddressSize = 1;
    masterXfer.data = g_master_rxBuff;
    masterXfer.dataSize = 2;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    status_t st = I2C_MasterTransferBlocking(I2C_PERIPHERAL, &masterXfer);

    if (kStatus_Success == st)
    	*val = (g_master_rxBuff[0] << 8) | g_master_rxBuff[1];

    return st;
}
