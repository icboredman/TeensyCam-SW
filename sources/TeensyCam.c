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

/* TODO: insert other definitions and declarations here. */
extern void APPInit(void);
extern void APPTask(void);

#define ARM_DEMCR				(*(volatile uint32_t *)0xE000EDFC) // Debug Exception and Monitor Control
#define ARM_DEMCR_TRCENA		(1 << 24)        // Enable debugging & monitoring blocks
#define ARM_DWT_CTRL			(*(volatile uint32_t *)0xE0001000) // DWT control register
#define ARM_DWT_CTRL_CYCCNTENA	(1 << 0)                // Enable cycle count
#define ARM_DWT_CYCCNT			(*(volatile uint32_t *)0xE0001004) // Cycle count register

#define CLK_CNTR_RESET()	{ ARM_DEMCR |= ARM_DEMCR_TRCENA;	\
							  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA; \
							  ARM_DWT_CYCCNT = 0; }
#define CLK_CNTR_VALUE		ARM_DWT_CYCCNT


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

	APPInit();

    //printf("Hello World\n");

    const uint32_t cycles = 90000000;

    while(1)
    {
    	CLK_CNTR_RESET();

    	while (CLK_CNTR_VALUE < cycles)
    	{
    		APPTask();
    	}

    	GPIO_PortToggle(BOARD_INITPINS_LED_GPIO, 1u << BOARD_INITPINS_LED_PIN);

    }
    return 0 ;
}
