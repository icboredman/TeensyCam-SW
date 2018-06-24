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
extern size_t USB_Recv(uint8_t** buf);
extern usb_status_t USB_RecvReady(void);

void __attribute__ ((noinline)) TakeSnapshot(void);
static void PulsePins(uint32_t pin, uint32_t cnt);
static bool ReadCameraLine(uint16_t *a, uint16_t *b, uint16_t px);

static void SendLine(uint16_t line, uint16_t *data1, uint16_t *data2, uint16_t dataSize);
static void Pack8bits(uint8_t *dstByte, uint16_t *srcWord, size_t srcLength);
static void Pack10bits(uint8_t *dstByte, uint16_t *srcWord, size_t srcLength);

void InitSysTick(void);
void __attribute__ ((noinline)) DelayLoop(unsigned long ulDelay_us);

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
//#define CLK_CNTR_DELAY_US  DelayLoop

#define PORT_CAM1	(GPIOC->PDIR)
#define PORT_CAM2	(GPIOD->PDIR)

static const uint8_t mt1addr = 0x90;
static const uint8_t mt2addr = 0xB0;

uint16_t exposure_us = 4000;	// up to 16383 us
uint8_t analogGain = 16;		// 16 - 64
uint8_t digitalGain = 4;		// 0 - 15
uint16_t n_lines = 100;			// MAX_IMAGE_HEIGHT;
//bool send_picture_data = false;
#define LIGHT_FREQ	50U			// indoor lighting flicker rate in Hz
uint8_t _fps = 25;				// 2 Hz


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

//	InitSysTick();
//	DelayLoop(1000000);
	CLK_CNTR_DELAY_US(1000000);

    // configure camera slave mode
    status_t st = MT9Initialize(mt1addr);
    st += MT9Initialize(mt2addr);
    st += MT9SetAGC(mt1addr, false);
    st += MT9SetAGC(mt2addr, false);
    st += MT9SetCompanding(mt1addr, true);
    st += MT9SetCompanding(mt2addr, true);

    char str[100];
    sprintf(str, "init status=%d\r\n", st);
    USB_Send((uint8_t*)str, strlen(str));

    size_t rcv_sz;
    uint8_t *rcv_dta;

    while(1)
    {
    	if (take_snapshot)
    	{
			take_snapshot = false;
    		TakeSnapshot();
    	}

        while (0 != (rcv_sz=USB_Recv(&rcv_dta)))
        {
        	while (3 <= rcv_sz)
        	{
        		switch (rcv_dta[0])
        		{
        		case 'E' :	exposure_us = rcv_dta[1] << 8;
        					exposure_us |= rcv_dta[2];
        					break;
        		case 'A' :	analogGain = rcv_dta[2];
        					Mt9SetAnalogGain(mt1addr,analogGain);
        					Mt9SetAnalogGain(mt2addr,analogGain);
        					break;
        		case 'D' :	digitalGain = rcv_dta[2];
        					MT9SetDigitalGain(mt1addr,digitalGain);
        					MT9SetDigitalGain(mt2addr,digitalGain);
        					break;
        		case 'N' :	n_lines = rcv_dta[1] << 8;
        					n_lines |= rcv_dta[2];
        					break;
//        		case 'P' :	send_picture_data = rcv_dta[2] & 0x01;
//        					break;
        		case 'F' :	_fps = rcv_dta[2];
        					if (_fps > 50)
        						_fps = 50;
//        					if (_fps < 2)
//        						_fps = 2;
        					break;
        		default  :	break;
        		}

        		rcv_sz -= 3;
        		rcv_dta += 3;
        	}

        	USB_RecvReady();
        }


//		CLK_CNTR_DELAY_US(1000);
//        DelayLoop(50000);
//        APPTask();
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
	CLK_CNTR_DELAY_US(exposure_us);
	PulsePins(BOARD_INITPINS_STFRM_OUT_PIN, 50);

	for (int ln=0; ln<530; ln++)   // min 525 lines, including blanking
	{
		PulsePins(BOARD_INITPINS_STLN_OUT1_PIN, 50);

		if( ReadCameraLine(a, b, MAX_IMAGE_WIDTH) )
		{	// make sure last data point contains valid line and frame markers
			if( (a[0]>>10) != 0x3 || (b[0]>>10) != 0xC )
				zr_cnt++;
			else
				if( (ln_cnt >= MAX_IMAGE_HEIGHT/2 - n_lines/2) &&
					(ln_cnt <  MAX_IMAGE_HEIGHT/2 + n_lines/2) )
				{
					SendLine(ln_cnt, a, b, MAX_IMAGE_WIDTH);
//							CLK_CNTR_DELAY_US(10);
				}

			ln_cnt++;
		}
		else
			nv_cnt++;
	}

}


void FPS_TICK_HANDLER(void)
{
	static int counter = 0;

    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);

//	if ((LIGHT_FREQ/FPS) == counter++)
	if (++counter >= _fps)
	{
		counter = 0;
		take_snapshot = true;
	}
}


/***************************************************
 * Utility Functions
 ***************************************************/

static __attribute__ ((noinline)) void PulsePins(uint32_t pin, uint32_t cnt)
{
	__disable_irq();

	GPIO_PinWrite(OUTPUTS_GPIO, pin, 1);

    while( cnt != 0 )
    {
        if (GPIO_PinRead(INPUTS_GPIO, BOARD_INITPINS_PIXCLK1_PIN))
            cnt--;
    }

    GPIO_PinWrite(OUTPUTS_GPIO, pin, 0);

    __enable_irq();
}


static bool ReadCameraLine(uint16_t *a, uint16_t *b, uint16_t px)
{
	//  int px = 750;  // max (752-1)
	px -= 2;

	__disable_irq();

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

//	__ASM volatile ("mov r0, r0");

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
	Pack8bits(s_imgSendBuf+2, data1, dataSize);
	Pack8bits(s_imgSendBuf+2+dataSize, data2, dataSize);

	USB_Send(s_imgSendBuf, dataSize*2+2);
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



void InitSysTick(void)
{
	SysTick->LOAD = SysTick_LOAD_RELOAD_Msk;                                 // temporarily set maximum reload value
	SysTick->VAL = SysTick_LOAD_RELOAD_Msk;                                // write to the current value to cause the counter value to be reset to 0 and the reload value be set
    (void)SysTick->CTRL;                                                   // ensure that the SYSTICK_COUNTFLAG flag is cleared
    SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk);                 // allow SYSTICK to run so that loop delays can already use it
}

void __attribute__ ((noinline)) DelayLoop(unsigned long ulDelay_us)
{
	#define CORE_US (BOARD_BOOTCLOCKHSRUN_CORE_CLOCK/1000000)                                 // the number of core clocks in a us
	register unsigned long ulPresentSystick;
	register unsigned long ulMatch;
	register unsigned long _ulDelay_us = ulDelay_us;                     // ensure that the compiler puts the variable in a register rather than work with it on the stack
	if (_ulDelay_us == 0) {                                              // minimum delay is 1us
		_ulDelay_us = 1;
	}
	(void)SysTick->CTRL;                                                 // clear the SysTick reload flag
	ulMatch = (SysTick->VAL - CORE_US);	                                 // next 1us match value (SysTick counts down)
	do {
		while ((ulPresentSystick = SysTick->VAL) > ulMatch) {            // wait until a us period has expired
			if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {            // if we missed a reload
				(void)SysTick->CTRL;
				break;                                                   // assume a us period expired
			}
		}
		ulMatch = (ulPresentSystick - CORE_US);
	} while (--_ulDelay_us);

//	register unsigned long _ulDelay_us = ulDelay_us;                     // ensure that the compiler puts the variable in a register rather than work with it on the stack
//    register unsigned long ul_us;
//    while (_ulDelay_us--) {                                              // for each us required
//        ul_us = (240000000/6000000);//(BOARD_BOOTCLOCKHSRUN_CORE_CLOCK/6000000);               // tuned but may be slightly compiler dependent - interrupt processing may increase delay
//        while (ul_us--) {}                                               // simple loop tuned to perform us timing
//    }
}
