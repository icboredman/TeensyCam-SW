/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v4.0
processor: MK66FX1M0xxx18
package_id: MK66FX1M0VLQ18
mcu_data: ksdk2_0
processor_version: 3.0.1
pin_labels:
- {pin_num: '55', pin_signal: PTA5/USB0_CLKIN/FTM0_CH2/RMII0_RXER/MII0_RXER/CMP2_OUT/I2S0_TX_BCLK/JTAG_TRST_b, label: LNVAL1, identifier: LNVAL1}
- {pin_num: '64', pin_signal: CMP2_IN0/PTA12/CAN0_TX/FTM1_CH0/RMII0_RXD1/MII0_RXD1/I2C2_SCL/I2S0_TXD0/FTM1_QD_PHA/TPM1_CH0, label: SYSCLK, identifier: SYSCLK}
- {pin_num: '66', pin_signal: PTA14/SPI0_PCS0/UART0_TX/RMII0_CRS_DV/MII0_RXDV/I2C2_SCL/I2S0_RX_BCLK/I2S0_TXD1, label: FRVAL1, identifier: FRVAL1}
- {pin_num: '67', pin_signal: CMP3_IN1/PTA15/SPI0_SCK/UART0_RX/RMII0_TXEN/MII0_TXEN/I2S0_RXD0, label: PIXCLK1, identifier: PIXCLK1}
- {pin_num: '77', pin_signal: PTA26/MII0_TXD3/FB_A27, label: PIXCLK2, identifier: PIXCLK2}
- {pin_num: '79', pin_signal: PTA28/MII0_TXER/FB_A25, label: LNVAL2, identifier: LNVAL2}
- {pin_num: '80', pin_signal: PTA29/MII0_COL/FB_A24, label: FRVAL2, identifier: FRVAL2}
- {pin_num: '83', pin_signal: ADC0_SE12/TSI0_CH7/PTB2/I2C0_SCL/UART0_RTS_b/ENET0_1588_TMR0/SDRAM_WE/FTM0_FLT3, label: SCL, identifier: SCL}
- {pin_num: '84', pin_signal: ADC0_SE13/TSI0_CH8/PTB3/I2C0_SDA/UART0_CTS_b/UART0_COL_b/ENET0_1588_TMR1/SDRAM_CS0_b/FTM0_FLT0, label: SDA, identifier: SDA}
- {pin_num: '91', pin_signal: ADC1_SE14/PTB10/SPI1_PCS0/UART3_RX/FB_AD19/SDRAM_D19/FTM0_FLT1, label: STLN_OUT1, identifier: STLN_OUT1}
- {pin_num: '92', pin_signal: ADC1_SE15/PTB11/SPI1_SCK/UART3_TX/FB_AD18/SDRAM_D18/FTM0_FLT2, label: STLN_OUT2, identifier: STLN_OUT2}
- {pin_num: '95', pin_signal: TSI0_CH9/PTB16/SPI1_SOUT/UART0_RX/FTM_CLKIN0/FB_AD17/SDRAM_D17/EWM_IN/TPM_CLKIN0, label: STANDBY, identifier: STANDBY}
- {pin_num: '97', pin_signal: TSI0_CH11/PTB18/CAN0_TX/FTM2_CH0/I2S0_TX_BCLK/FB_AD15/SDRAM_A23/FTM2_QD_PHA/TPM2_CH0, label: EXPOSURE, identifier: EXPOSURE}
- {pin_num: '98', pin_signal: TSI0_CH12/PTB19/CAN0_RX/FTM2_CH1/I2S0_TX_FS/FB_OE_b/FTM2_QD_PHB/TPM2_CH1, label: STFRM_OUT, identifier: STFRM_OUT}
- {pin_num: '103', pin_signal: ADC0_SE14/TSI0_CH13/PTC0/SPI0_PCS4/PDB0_EXTRG/USB0_SOF_OUT/FB_AD14/SDRAM_A22/I2S0_TXD1, label: PTC0, identifier: PTC0}
- {pin_num: '104', pin_signal: ADC0_SE15/TSI0_CH14/PTC1/LLWU_P6/SPI0_PCS3/UART1_RTS_b/FTM0_CH0/FB_AD13/SDRAM_A21/I2S0_TXD0, label: PTC1, identifier: PTC1}
- {pin_num: '105', pin_signal: ADC0_SE4b/CMP1_IN0/TSI0_CH15/PTC2/SPI0_PCS2/UART1_CTS_b/FTM0_CH1/FB_AD12/SDRAM_A20/I2S0_TX_FS, label: PTC2, identifier: PTC2}
- {pin_num: '106', pin_signal: CMP1_IN1/PTC3/LLWU_P7/SPI0_PCS1/UART1_RX/FTM0_CH2/CLKOUT/I2S0_TX_BCLK, label: PTC3, identifier: PTC3}
- {pin_num: '109', pin_signal: PTC4/LLWU_P8/SPI0_PCS0/UART1_TX/FTM0_CH3/FB_AD11/SDRAM_A19/CMP1_OUT, label: PTC4, identifier: PTC4}
- {pin_num: '110', pin_signal: PTC5/LLWU_P9/SPI0_SCK/LPTMR0_ALT2/I2S0_RXD0/FB_AD10/SDRAM_A18/CMP0_OUT/FTM0_CH2, label: PTC5, identifier: PTC5}
- {pin_num: '111', pin_signal: CMP0_IN0/PTC6/LLWU_P10/SPI0_SOUT/PDB0_EXTRG/I2S0_RX_BCLK/FB_AD9/SDRAM_A17/I2S0_MCLK, label: PTC6, identifier: PTC6}
- {pin_num: '112', pin_signal: CMP0_IN1/PTC7/SPI0_SIN/USB0_SOF_OUT/I2S0_RX_FS/FB_AD8/SDRAM_A16, label: PTC7, identifier: PTC7}
- {pin_num: '113', pin_signal: ADC1_SE4b/CMP0_IN2/PTC8/FTM3_CH4/I2S0_MCLK/FB_AD7/SDRAM_A15, label: PTC8, identifier: PTC8}
- {pin_num: '114', pin_signal: ADC1_SE5b/CMP0_IN3/PTC9/FTM3_CH5/I2S0_RX_BCLK/FB_AD6/SDRAM_A14/FTM2_FLT0, label: PTC9, identifier: PTC9}
- {pin_num: '115', pin_signal: ADC1_SE6b/PTC10/I2C1_SCL/FTM3_CH6/I2S0_RX_FS/FB_AD5/SDRAM_A13, label: PTC10, identifier: PTC10}
- {pin_num: '116', pin_signal: ADC1_SE7b/PTC11/LLWU_P11/I2C1_SDA/FTM3_CH7/I2S0_RXD1/FB_RW_b, label: PTC11, identifier: PTC11}
- {pin_num: '127', pin_signal: PTD0/LLWU_P12/SPI0_PCS0/UART2_RTS_b/FTM3_CH0/FB_ALE/FB_CS1_b/FB_TS_b, label: PTD0, identifier: PTD0}
- {pin_num: '128', pin_signal: ADC0_SE5b/PTD1/SPI0_SCK/UART2_CTS_b/FTM3_CH1/FB_CS0_b, label: PTD1, identifier: PTD1}
- {pin_num: '129', pin_signal: PTD2/LLWU_P13/SPI0_SOUT/UART2_RX/FTM3_CH2/FB_AD4/SDRAM_A12/I2C0_SCL, label: PTD2, identifier: PTD2}
- {pin_num: '130', pin_signal: PTD3/SPI0_SIN/UART2_TX/FTM3_CH3/FB_AD3/SDRAM_A11/I2C0_SDA, label: PTD3, identifier: PTD3}
- {pin_num: '131', pin_signal: PTD4/LLWU_P14/SPI0_PCS1/UART0_RTS_b/FTM0_CH4/FB_AD2/SDRAM_A10/EWM_IN/SPI1_PCS0, label: PTD4, identifier: PTD4}
- {pin_num: '132', pin_signal: ADC0_SE6b/PTD5/SPI0_PCS2/UART0_CTS_b/UART0_COL_b/FTM0_CH5/FB_AD1/SDRAM_A9/EWM_OUT_b/SPI1_SCK, label: PTD5, identifier: PTD5}
- {pin_num: '133', pin_signal: ADC0_SE7b/PTD6/LLWU_P15/SPI0_PCS3/UART0_RX/FTM0_CH6/FB_AD0/FTM0_FLT0/SPI1_SOUT, label: PTD6, identifier: PTD6}
- {pin_num: '136', pin_signal: PTD7/CMT_IRO/UART0_TX/FTM0_CH7/SDRAM_CKE/FTM0_FLT1/SPI1_SIN, label: PTD7, identifier: PTD7}
- {pin_num: '137', pin_signal: PTD8/LLWU_P24/I2C0_SCL/LPUART0_RX/FB_A16, label: PTD8, identifier: PTD8}
- {pin_num: '138', pin_signal: PTD9/I2C0_SDA/LPUART0_TX/FB_A17, label: PTD9, identifier: PTD9}
- {pin_num: '141', pin_signal: PTD12/SPI2_SCK/FTM3_FLT0/SDHC0_D4/FB_A20, label: PTD12, identifier: PTD12}
- {pin_num: '142', pin_signal: PTD13/SPI2_SOUT/SDHC0_D5/FB_A21, label: PTD13, identifier: PTD13}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_port.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '64', peripheral: FTM1, signal: 'CH, 0', pin_signal: CMP2_IN0/PTA12/CAN0_TX/FTM1_CH0/RMII0_RXD1/MII0_RXD1/I2C2_SCL/I2S0_TXD0/FTM1_QD_PHA/TPM1_CH0, direction: OUTPUT}
  - {pin_num: '83', peripheral: I2C0, signal: SCL, pin_signal: ADC0_SE12/TSI0_CH7/PTB2/I2C0_SCL/UART0_RTS_b/ENET0_1588_TMR0/SDRAM_WE/FTM0_FLT3, open_drain: enable,
    pull_select: no_init, pull_enable: no_init}
  - {pin_num: '84', peripheral: I2C0, signal: SDA, pin_signal: ADC0_SE13/TSI0_CH8/PTB3/I2C0_SDA/UART0_CTS_b/UART0_COL_b/ENET0_1588_TMR1/SDRAM_CS0_b/FTM0_FLT0, open_drain: enable,
    pull_select: no_init, pull_enable: no_init}
  - {pin_num: '95', peripheral: GPIOB, signal: 'GPIO, 16', pin_signal: TSI0_CH9/PTB16/SPI1_SOUT/UART0_RX/FTM_CLKIN0/FB_AD17/SDRAM_D17/EWM_IN/TPM_CLKIN0, direction: OUTPUT}
  - {pin_num: '103', peripheral: GPIOC, signal: 'GPIO, 0', pin_signal: ADC0_SE14/TSI0_CH13/PTC0/SPI0_PCS4/PDB0_EXTRG/USB0_SOF_OUT/FB_AD14/SDRAM_A22/I2S0_TXD1, direction: INPUT}
  - {pin_num: '104', peripheral: GPIOC, signal: 'GPIO, 1', pin_signal: ADC0_SE15/TSI0_CH14/PTC1/LLWU_P6/SPI0_PCS3/UART1_RTS_b/FTM0_CH0/FB_AD13/SDRAM_A21/I2S0_TXD0,
    direction: INPUT}
  - {pin_num: '105', peripheral: GPIOC, signal: 'GPIO, 2', pin_signal: ADC0_SE4b/CMP1_IN0/TSI0_CH15/PTC2/SPI0_PCS2/UART1_CTS_b/FTM0_CH1/FB_AD12/SDRAM_A20/I2S0_TX_FS,
    direction: INPUT}
  - {pin_num: '106', peripheral: GPIOC, signal: 'GPIO, 3', pin_signal: CMP1_IN1/PTC3/LLWU_P7/SPI0_PCS1/UART1_RX/FTM0_CH2/CLKOUT/I2S0_TX_BCLK, direction: INPUT}
  - {pin_num: '109', peripheral: GPIOC, signal: 'GPIO, 4', pin_signal: PTC4/LLWU_P8/SPI0_PCS0/UART1_TX/FTM0_CH3/FB_AD11/SDRAM_A19/CMP1_OUT, direction: INPUT}
  - {pin_num: '110', peripheral: GPIOC, signal: 'GPIO, 5', pin_signal: PTC5/LLWU_P9/SPI0_SCK/LPTMR0_ALT2/I2S0_RXD0/FB_AD10/SDRAM_A18/CMP0_OUT/FTM0_CH2, direction: INPUT}
  - {pin_num: '111', peripheral: GPIOC, signal: 'GPIO, 6', pin_signal: CMP0_IN0/PTC6/LLWU_P10/SPI0_SOUT/PDB0_EXTRG/I2S0_RX_BCLK/FB_AD9/SDRAM_A17/I2S0_MCLK, direction: INPUT}
  - {pin_num: '112', peripheral: GPIOC, signal: 'GPIO, 7', pin_signal: CMP0_IN1/PTC7/SPI0_SIN/USB0_SOF_OUT/I2S0_RX_FS/FB_AD8/SDRAM_A16, direction: INPUT}
  - {pin_num: '113', peripheral: GPIOC, signal: 'GPIO, 8', pin_signal: ADC1_SE4b/CMP0_IN2/PTC8/FTM3_CH4/I2S0_MCLK/FB_AD7/SDRAM_A15, direction: INPUT}
  - {pin_num: '114', peripheral: GPIOC, signal: 'GPIO, 9', pin_signal: ADC1_SE5b/CMP0_IN3/PTC9/FTM3_CH5/I2S0_RX_BCLK/FB_AD6/SDRAM_A14/FTM2_FLT0, direction: INPUT}
  - {pin_num: '115', peripheral: GPIOC, signal: 'GPIO, 10', pin_signal: ADC1_SE6b/PTC10/I2C1_SCL/FTM3_CH6/I2S0_RX_FS/FB_AD5/SDRAM_A13, direction: INPUT, pull_select: up,
    pull_enable: enable}
  - {pin_num: '116', peripheral: GPIOC, signal: 'GPIO, 11', pin_signal: ADC1_SE7b/PTC11/LLWU_P11/I2C1_SDA/FTM3_CH7/I2S0_RXD1/FB_RW_b, direction: INPUT, pull_select: up,
    pull_enable: enable}
  - {pin_num: '127', peripheral: GPIOD, signal: 'GPIO, 0', pin_signal: PTD0/LLWU_P12/SPI0_PCS0/UART2_RTS_b/FTM3_CH0/FB_ALE/FB_CS1_b/FB_TS_b, direction: INPUT}
  - {pin_num: '128', peripheral: GPIOD, signal: 'GPIO, 1', pin_signal: ADC0_SE5b/PTD1/SPI0_SCK/UART2_CTS_b/FTM3_CH1/FB_CS0_b, direction: INPUT}
  - {pin_num: '129', peripheral: GPIOD, signal: 'GPIO, 2', pin_signal: PTD2/LLWU_P13/SPI0_SOUT/UART2_RX/FTM3_CH2/FB_AD4/SDRAM_A12/I2C0_SCL, direction: INPUT}
  - {pin_num: '130', peripheral: GPIOD, signal: 'GPIO, 3', pin_signal: PTD3/SPI0_SIN/UART2_TX/FTM3_CH3/FB_AD3/SDRAM_A11/I2C0_SDA, direction: INPUT}
  - {pin_num: '131', peripheral: GPIOD, signal: 'GPIO, 4', pin_signal: PTD4/LLWU_P14/SPI0_PCS1/UART0_RTS_b/FTM0_CH4/FB_AD2/SDRAM_A10/EWM_IN/SPI1_PCS0, direction: INPUT}
  - {pin_num: '132', peripheral: GPIOD, signal: 'GPIO, 5', pin_signal: ADC0_SE6b/PTD5/SPI0_PCS2/UART0_CTS_b/UART0_COL_b/FTM0_CH5/FB_AD1/SDRAM_A9/EWM_OUT_b/SPI1_SCK,
    direction: INPUT}
  - {pin_num: '133', peripheral: GPIOD, signal: 'GPIO, 6', pin_signal: ADC0_SE7b/PTD6/LLWU_P15/SPI0_PCS3/UART0_RX/FTM0_CH6/FB_AD0/FTM0_FLT0/SPI1_SOUT, direction: INPUT}
  - {pin_num: '136', peripheral: GPIOD, signal: 'GPIO, 7', pin_signal: PTD7/CMT_IRO/UART0_TX/FTM0_CH7/SDRAM_CKE/FTM0_FLT1/SPI1_SIN, direction: INPUT}
  - {pin_num: '137', peripheral: GPIOD, signal: 'GPIO, 8', pin_signal: PTD8/LLWU_P24/I2C0_SCL/LPUART0_RX/FB_A16, direction: INPUT}
  - {pin_num: '138', peripheral: GPIOD, signal: 'GPIO, 9', pin_signal: PTD9/I2C0_SDA/LPUART0_TX/FB_A17, direction: INPUT}
  - {pin_num: '141', peripheral: GPIOD, signal: 'GPIO, 12', pin_signal: PTD12/SPI2_SCK/FTM3_FLT0/SDHC0_D4/FB_A20, direction: INPUT, pull_enable: disable}
  - {pin_num: '142', peripheral: GPIOD, signal: 'GPIO, 13', pin_signal: PTD13/SPI2_SOUT/SDHC0_D5/FB_A21, direction: INPUT, pull_enable: disable}
  - {pin_num: '97', peripheral: GPIOB, signal: 'GPIO, 18', pin_signal: TSI0_CH11/PTB18/CAN0_TX/FTM2_CH0/I2S0_TX_BCLK/FB_AD15/SDRAM_A23/FTM2_QD_PHA/TPM2_CH0, direction: OUTPUT}
  - {pin_num: '98', peripheral: GPIOB, signal: 'GPIO, 19', pin_signal: TSI0_CH12/PTB19/CAN0_RX/FTM2_CH1/I2S0_TX_FS/FB_OE_b/FTM2_QD_PHB/TPM2_CH1, direction: OUTPUT}
  - {pin_num: '91', peripheral: GPIOB, signal: 'GPIO, 10', pin_signal: ADC1_SE14/PTB10/SPI1_PCS0/UART3_RX/FB_AD19/SDRAM_D19/FTM0_FLT1, direction: OUTPUT}
  - {pin_num: '92', peripheral: GPIOB, signal: 'GPIO, 11', pin_signal: ADC1_SE15/PTB11/SPI1_SCK/UART3_TX/FB_AD18/SDRAM_D18/FTM0_FLT2, direction: OUTPUT}
  - {pin_num: '55', peripheral: GPIOA, signal: 'GPIO, 5', pin_signal: PTA5/USB0_CLKIN/FTM0_CH2/RMII0_RXER/MII0_RXER/CMP2_OUT/I2S0_TX_BCLK/JTAG_TRST_b, direction: INPUT,
    pull_select: up, pull_enable: enable}
  - {pin_num: '66', peripheral: GPIOA, signal: 'GPIO, 14', pin_signal: PTA14/SPI0_PCS0/UART0_TX/RMII0_CRS_DV/MII0_RXDV/I2C2_SCL/I2S0_RX_BCLK/I2S0_TXD1, direction: INPUT,
    pull_select: up, pull_enable: enable}
  - {pin_num: '67', peripheral: GPIOA, signal: 'GPIO, 15', pin_signal: CMP3_IN1/PTA15/SPI0_SCK/UART0_RX/RMII0_TXEN/MII0_TXEN/I2S0_RXD0, direction: INPUT, pull_select: no_init,
    pull_enable: disable}
  - {pin_num: '79', peripheral: GPIOA, signal: 'GPIO, 28', pin_signal: PTA28/MII0_TXER/FB_A25, direction: INPUT, pull_select: no_init, pull_enable: disable}
  - {pin_num: '80', peripheral: GPIOA, signal: 'GPIO, 29', pin_signal: PTA29/MII0_COL/FB_A24, direction: INPUT, pull_select: no_init, pull_enable: disable}
  - {pin_num: '77', peripheral: GPIOA, signal: 'GPIO, 26', pin_signal: PTA26/MII0_TXD3/FB_A27, direction: INPUT, pull_select: no_init, pull_enable: disable}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void)
{
    /* Port A Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortA);
    /* Port B Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortB);
    /* Port C Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortC);
    /* Port D Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortD);

    /* PORTA12 (pin 64) is configured as FTM1_CH0 */
    PORT_SetPinMux(BOARD_INITPINS_SYSCLK_PORT, BOARD_INITPINS_SYSCLK_PIN, kPORT_MuxAlt3);

    /* PORTA14 (pin 66) is configured as PTA14 */
    PORT_SetPinMux(BOARD_INITPINS_FRVAL1_PORT, BOARD_INITPINS_FRVAL1_PIN, kPORT_MuxAsGpio);

    PORTA->PCR[14] = ((PORTA->PCR[14] &
                       /* Mask bits to zero which are setting */
                       (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                      /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the
                       * corresponding PE field is set. */
                      | (uint32_t)(kPORT_PullUp));

    /* PORTA15 (pin 67) is configured as PTA15 */
    PORT_SetPinMux(BOARD_INITPINS_PIXCLK1_PORT, BOARD_INITPINS_PIXCLK1_PIN, kPORT_MuxAsGpio);

    PORTA->PCR[15] = ((PORTA->PCR[15] &
                       /* Mask bits to zero which are setting */
                       (~(PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                      /* Pull Enable: Internal pullup or pulldown resistor is not enabled on the corresponding pin. */
                      | PORT_PCR_PE(kPORT_PullDisable));

    /* PORTA26 (pin 77) is configured as PTA26 */
    PORT_SetPinMux(BOARD_INITPINS_PIXCLK2_PORT, BOARD_INITPINS_PIXCLK2_PIN, kPORT_MuxAsGpio);

    PORTA->PCR[26] = ((PORTA->PCR[26] &
                       /* Mask bits to zero which are setting */
                       (~(PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                      /* Pull Enable: Internal pullup or pulldown resistor is not enabled on the corresponding pin. */
                      | PORT_PCR_PE(kPORT_PullDisable));

    /* PORTA28 (pin 79) is configured as PTA28 */
    PORT_SetPinMux(BOARD_INITPINS_LNVAL2_PORT, BOARD_INITPINS_LNVAL2_PIN, kPORT_MuxAsGpio);

    PORTA->PCR[28] = ((PORTA->PCR[28] &
                       /* Mask bits to zero which are setting */
                       (~(PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                      /* Pull Enable: Internal pullup or pulldown resistor is not enabled on the corresponding pin. */
                      | PORT_PCR_PE(kPORT_PullDisable));

    /* PORTA29 (pin 80) is configured as PTA29 */
    PORT_SetPinMux(BOARD_INITPINS_FRVAL2_PORT, BOARD_INITPINS_FRVAL2_PIN, kPORT_MuxAsGpio);

    PORTA->PCR[29] = ((PORTA->PCR[29] &
                       /* Mask bits to zero which are setting */
                       (~(PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                      /* Pull Enable: Internal pullup or pulldown resistor is not enabled on the corresponding pin. */
                      | PORT_PCR_PE(kPORT_PullDisable));

    /* PORTA5 (pin 55) is configured as PTA5 */
    PORT_SetPinMux(BOARD_INITPINS_LNVAL1_PORT, BOARD_INITPINS_LNVAL1_PIN, kPORT_MuxAsGpio);

    PORTA->PCR[5] = ((PORTA->PCR[5] &
                      /* Mask bits to zero which are setting */
                      (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                     /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the
                      * corresponding PE field is set. */
                     | (uint32_t)(kPORT_PullUp));

    /* PORTB10 (pin 91) is configured as PTB10 */
    PORT_SetPinMux(BOARD_INITPINS_STLN_OUT1_PORT, BOARD_INITPINS_STLN_OUT1_PIN, kPORT_MuxAsGpio);

    /* PORTB11 (pin 92) is configured as PTB11 */
    PORT_SetPinMux(BOARD_INITPINS_STLN_OUT2_PORT, BOARD_INITPINS_STLN_OUT2_PIN, kPORT_MuxAsGpio);

    /* PORTB16 (pin 95) is configured as PTB16 */
    PORT_SetPinMux(BOARD_INITPINS_STANDBY_PORT, BOARD_INITPINS_STANDBY_PIN, kPORT_MuxAsGpio);

    /* PORTB18 (pin 97) is configured as PTB18 */
    PORT_SetPinMux(BOARD_INITPINS_EXPOSURE_PORT, BOARD_INITPINS_EXPOSURE_PIN, kPORT_MuxAsGpio);

    /* PORTB19 (pin 98) is configured as PTB19 */
    PORT_SetPinMux(BOARD_INITPINS_STFRM_OUT_PORT, BOARD_INITPINS_STFRM_OUT_PIN, kPORT_MuxAsGpio);

    /* PORTB2 (pin 83) is configured as I2C0_SCL */
    PORT_SetPinMux(BOARD_INITPINS_SCL_PORT, BOARD_INITPINS_SCL_PIN, kPORT_MuxAlt2);

    PORTB->PCR[2] = ((PORTB->PCR[2] &
                      /* Mask bits to zero which are setting */
                      (~(PORT_PCR_ODE_MASK | PORT_PCR_ISF_MASK)))

                     /* Open Drain Enable: Open drain output is enabled on the corresponding pin, if the pin is
                      * configured as a digital output. */
                     | PORT_PCR_ODE(kPORT_OpenDrainEnable));

    /* PORTB3 (pin 84) is configured as I2C0_SDA */
    PORT_SetPinMux(BOARD_INITPINS_SDA_PORT, BOARD_INITPINS_SDA_PIN, kPORT_MuxAlt2);

    PORTB->PCR[3] = ((PORTB->PCR[3] &
                      /* Mask bits to zero which are setting */
                      (~(PORT_PCR_ODE_MASK | PORT_PCR_ISF_MASK)))

                     /* Open Drain Enable: Open drain output is enabled on the corresponding pin, if the pin is
                      * configured as a digital output. */
                     | PORT_PCR_ODE(kPORT_OpenDrainEnable));

    /* PORTC0 (pin 103) is configured as PTC0 */
    PORT_SetPinMux(BOARD_INITPINS_PTC0_PORT, BOARD_INITPINS_PTC0_PIN, kPORT_MuxAsGpio);

    /* PORTC1 (pin 104) is configured as PTC1 */
    PORT_SetPinMux(BOARD_INITPINS_PTC1_PORT, BOARD_INITPINS_PTC1_PIN, kPORT_MuxAsGpio);

    /* PORTC10 (pin 115) is configured as PTC10 */
    PORT_SetPinMux(BOARD_INITPINS_PTC10_PORT, BOARD_INITPINS_PTC10_PIN, kPORT_MuxAsGpio);

    PORTC->PCR[10] = ((PORTC->PCR[10] &
                       /* Mask bits to zero which are setting */
                       (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                      /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the
                       * corresponding PE field is set. */
                      | (uint32_t)(kPORT_PullUp));

    /* PORTC11 (pin 116) is configured as PTC11 */
    PORT_SetPinMux(BOARD_INITPINS_PTC11_PORT, BOARD_INITPINS_PTC11_PIN, kPORT_MuxAsGpio);

    PORTC->PCR[11] = ((PORTC->PCR[11] &
                       /* Mask bits to zero which are setting */
                       (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                      /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the
                       * corresponding PE field is set. */
                      | (uint32_t)(kPORT_PullUp));

    /* PORTC2 (pin 105) is configured as PTC2 */
    PORT_SetPinMux(BOARD_INITPINS_PTC2_PORT, BOARD_INITPINS_PTC2_PIN, kPORT_MuxAsGpio);

    /* PORTC3 (pin 106) is configured as PTC3 */
    PORT_SetPinMux(BOARD_INITPINS_PTC3_PORT, BOARD_INITPINS_PTC3_PIN, kPORT_MuxAsGpio);

    /* PORTC4 (pin 109) is configured as PTC4 */
    PORT_SetPinMux(BOARD_INITPINS_PTC4_PORT, BOARD_INITPINS_PTC4_PIN, kPORT_MuxAsGpio);

    /* PORTC5 (pin 110) is configured as PTC5 */
    PORT_SetPinMux(BOARD_INITPINS_PTC5_PORT, BOARD_INITPINS_PTC5_PIN, kPORT_MuxAsGpio);

    /* PORTC6 (pin 111) is configured as PTC6 */
    PORT_SetPinMux(BOARD_INITPINS_PTC6_PORT, BOARD_INITPINS_PTC6_PIN, kPORT_MuxAsGpio);

    /* PORTC7 (pin 112) is configured as PTC7 */
    PORT_SetPinMux(BOARD_INITPINS_PTC7_PORT, BOARD_INITPINS_PTC7_PIN, kPORT_MuxAsGpio);

    /* PORTC8 (pin 113) is configured as PTC8 */
    PORT_SetPinMux(BOARD_INITPINS_PTC8_PORT, BOARD_INITPINS_PTC8_PIN, kPORT_MuxAsGpio);

    /* PORTC9 (pin 114) is configured as PTC9 */
    PORT_SetPinMux(BOARD_INITPINS_PTC9_PORT, BOARD_INITPINS_PTC9_PIN, kPORT_MuxAsGpio);

    /* PORTD0 (pin 127) is configured as PTD0 */
    PORT_SetPinMux(BOARD_INITPINS_PTD0_PORT, BOARD_INITPINS_PTD0_PIN, kPORT_MuxAsGpio);

    /* PORTD1 (pin 128) is configured as PTD1 */
    PORT_SetPinMux(BOARD_INITPINS_PTD1_PORT, BOARD_INITPINS_PTD1_PIN, kPORT_MuxAsGpio);

    /* PORTD12 (pin 141) is configured as PTD12 */
    PORT_SetPinMux(BOARD_INITPINS_PTD12_PORT, BOARD_INITPINS_PTD12_PIN, kPORT_MuxAsGpio);

    PORTD->PCR[12] = ((PORTD->PCR[12] &
                       /* Mask bits to zero which are setting */
                       (~(PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                      /* Pull Enable: Internal pullup or pulldown resistor is not enabled on the corresponding pin. */
                      | PORT_PCR_PE(kPORT_PullDisable));

    /* PORTD13 (pin 142) is configured as PTD13 */
    PORT_SetPinMux(BOARD_INITPINS_PTD13_PORT, BOARD_INITPINS_PTD13_PIN, kPORT_MuxAsGpio);

    PORTD->PCR[13] = ((PORTD->PCR[13] &
                       /* Mask bits to zero which are setting */
                       (~(PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                      /* Pull Enable: Internal pullup or pulldown resistor is not enabled on the corresponding pin. */
                      | PORT_PCR_PE(kPORT_PullDisable));

    /* PORTD2 (pin 129) is configured as PTD2 */
    PORT_SetPinMux(BOARD_INITPINS_PTD2_PORT, BOARD_INITPINS_PTD2_PIN, kPORT_MuxAsGpio);

    /* PORTD3 (pin 130) is configured as PTD3 */
    PORT_SetPinMux(BOARD_INITPINS_PTD3_PORT, BOARD_INITPINS_PTD3_PIN, kPORT_MuxAsGpio);

    /* PORTD4 (pin 131) is configured as PTD4 */
    PORT_SetPinMux(BOARD_INITPINS_PTD4_PORT, BOARD_INITPINS_PTD4_PIN, kPORT_MuxAsGpio);

    /* PORTD5 (pin 132) is configured as PTD5 */
    PORT_SetPinMux(BOARD_INITPINS_PTD5_PORT, BOARD_INITPINS_PTD5_PIN, kPORT_MuxAsGpio);

    /* PORTD6 (pin 133) is configured as PTD6 */
    PORT_SetPinMux(BOARD_INITPINS_PTD6_PORT, BOARD_INITPINS_PTD6_PIN, kPORT_MuxAsGpio);

    /* PORTD7 (pin 136) is configured as PTD7 */
    PORT_SetPinMux(BOARD_INITPINS_PTD7_PORT, BOARD_INITPINS_PTD7_PIN, kPORT_MuxAsGpio);

    /* PORTD8 (pin 137) is configured as PTD8 */
    PORT_SetPinMux(BOARD_INITPINS_PTD8_PORT, BOARD_INITPINS_PTD8_PIN, kPORT_MuxAsGpio);

    /* PORTD9 (pin 138) is configured as PTD9 */
    PORT_SetPinMux(BOARD_INITPINS_PTD9_PORT, BOARD_INITPINS_PTD9_PIN, kPORT_MuxAsGpio);

    SIM->SOPT4 = ((SIM->SOPT4 &
                   /* Mask bits to zero which are setting */
                   (~(SIM_SOPT4_FTM1CH0SRC_MASK)))

                  /* FTM1 channel 0 input capture source select: FTM1_CH0 signal. */
                  | SIM_SOPT4_FTM1CH0SRC(SOPT4_FTM1CH0SRC_FTM));
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/