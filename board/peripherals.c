/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v1.0
processor: MK66FX1M0xxx18
package_id: MK66FX1M0VLQ18
mcu_data: ksdk2_0
processor_version: 3.0.1
functionalGroups:
- name: BOARD_InitPeripherals
  called_from_default_init: true
  selectedCore: core0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'system'
- type_id: 'system_54b53072540eeeb8f8e9343e71f28176'
- global_system_definitions: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "peripherals.h"

/***********************************************************************************************************************
 * BOARD_InitPeripherals functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * I2C initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'I2C'
- type: 'i2c'
- mode: 'I2C_Polling'
- type_id: 'i2c_2566d7363e7e9aaedabb432110e372d7'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'I2C0'
- config_sets:
  - fsl_i2c:
    - i2c_mode: 'kI2C_Master'
    - i2c_master_config:
      - enableMaster: 'true'
      - enableStopHold: 'false'
      - baudRate_Bps: '100000'
      - glitchFilterWidth: '0'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const i2c_master_config_t I2C_config = {
  .enableMaster = true,
  .enableStopHold = false,
  .baudRate_Bps = 100000,
  .glitchFilterWidth = 0
};

void I2C_init(void) {
  /* Initialization function */
  I2C_MasterInit(I2C_PERIPHERAL, &I2C_config, CLOCK_GetFreq(I2C_CLKSRC));
}

/***********************************************************************************************************************
 * BUS_C initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'BUS_C'
- type: 'gpio'
- mode: 'GPIO'
- type_id: 'gpio_f970a92e447fa4793838db25a2947ed7'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'GPIOC'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'false'
    - port_interrupt:
      - IRQn: 'PORTC_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
    - gpio_config:
      - 0:
        - signal_number: 'GPIO.0'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 1:
        - signal_number: 'GPIO.1'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 2:
        - signal_number: 'GPIO.2'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 3:
        - signal_number: 'GPIO.3'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 4:
        - signal_number: 'GPIO.4'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 5:
        - signal_number: 'GPIO.5'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 6:
        - signal_number: 'GPIO.6'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 7:
        - signal_number: 'GPIO.7'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 8:
        - signal_number: 'GPIO.8'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 9:
        - signal_number: 'GPIO.9'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 10:
        - signal_number: 'GPIO.10'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 11:
        - signal_number: 'GPIO.11'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
gpio_pin_config_t BUS_C_config[12] = {
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  }
};

void BUS_C_init(void) {
  /* Make sure, the clock gate for port C is enabled (e. g. in pin_mux.c) */
  /* Initialize GPIO functionality on pin PTC0 */
  GPIO_PinInit(BUS_C_GPIO, 0U, &BUS_C_config[0]);
  /* Initialize GPIO functionality on pin PTC1 */
  GPIO_PinInit(BUS_C_GPIO, 1U, &BUS_C_config[1]);
  /* Initialize GPIO functionality on pin PTC2 */
  GPIO_PinInit(BUS_C_GPIO, 2U, &BUS_C_config[2]);
  /* Initialize GPIO functionality on pin PTC3 */
  GPIO_PinInit(BUS_C_GPIO, 3U, &BUS_C_config[3]);
  /* Initialize GPIO functionality on pin PTC4 */
  GPIO_PinInit(BUS_C_GPIO, 4U, &BUS_C_config[4]);
  /* Initialize GPIO functionality on pin PTC5 */
  GPIO_PinInit(BUS_C_GPIO, 5U, &BUS_C_config[5]);
  /* Initialize GPIO functionality on pin PTC6 */
  GPIO_PinInit(BUS_C_GPIO, 6U, &BUS_C_config[6]);
  /* Initialize GPIO functionality on pin PTC7 */
  GPIO_PinInit(BUS_C_GPIO, 7U, &BUS_C_config[7]);
  /* Initialize GPIO functionality on pin PTC8 */
  GPIO_PinInit(BUS_C_GPIO, 8U, &BUS_C_config[8]);
  /* Initialize GPIO functionality on pin PTC9 */
  GPIO_PinInit(BUS_C_GPIO, 9U, &BUS_C_config[9]);
  /* Initialize GPIO functionality on pin PTC10 */
  GPIO_PinInit(BUS_C_GPIO, 10U, &BUS_C_config[10]);
  /* Initialize GPIO functionality on pin PTC11 */
  GPIO_PinInit(BUS_C_GPIO, 11U, &BUS_C_config[11]);
}

/***********************************************************************************************************************
 * BUS_D initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'BUS_D'
- type: 'gpio'
- mode: 'GPIO'
- type_id: 'gpio_f970a92e447fa4793838db25a2947ed7'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'GPIOD'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'false'
    - port_interrupt:
      - IRQn: 'PORTD_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
    - gpio_config:
      - 0:
        - signal_number: 'GPIO.0'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 1:
        - signal_number: 'GPIO.1'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 2:
        - signal_number: 'GPIO.2'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 3:
        - signal_number: 'GPIO.3'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 4:
        - signal_number: 'GPIO.4'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 5:
        - signal_number: 'GPIO.5'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 6:
        - signal_number: 'GPIO.6'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 7:
        - signal_number: 'GPIO.7'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 8:
        - signal_number: 'GPIO.8'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 9:
        - signal_number: 'GPIO.9'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 10:
        - signal_number: 'GPIO.12'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 11:
        - signal_number: 'GPIO.13'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
gpio_pin_config_t BUS_D_config[12] = {
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  }
};

void BUS_D_init(void) {
  /* Make sure, the clock gate for port D is enabled (e. g. in pin_mux.c) */
  /* Initialize GPIO functionality on pin PTD0 */
  GPIO_PinInit(BUS_D_GPIO, 0U, &BUS_D_config[0]);
  /* Initialize GPIO functionality on pin PTD1 */
  GPIO_PinInit(BUS_D_GPIO, 1U, &BUS_D_config[1]);
  /* Initialize GPIO functionality on pin PTD2 */
  GPIO_PinInit(BUS_D_GPIO, 2U, &BUS_D_config[2]);
  /* Initialize GPIO functionality on pin PTD3 */
  GPIO_PinInit(BUS_D_GPIO, 3U, &BUS_D_config[3]);
  /* Initialize GPIO functionality on pin PTD4 */
  GPIO_PinInit(BUS_D_GPIO, 4U, &BUS_D_config[4]);
  /* Initialize GPIO functionality on pin PTD5 */
  GPIO_PinInit(BUS_D_GPIO, 5U, &BUS_D_config[5]);
  /* Initialize GPIO functionality on pin PTD6 */
  GPIO_PinInit(BUS_D_GPIO, 6U, &BUS_D_config[6]);
  /* Initialize GPIO functionality on pin PTD7 */
  GPIO_PinInit(BUS_D_GPIO, 7U, &BUS_D_config[7]);
  /* Initialize GPIO functionality on pin PTD8 */
  GPIO_PinInit(BUS_D_GPIO, 8U, &BUS_D_config[8]);
  /* Initialize GPIO functionality on pin PTD9 */
  GPIO_PinInit(BUS_D_GPIO, 9U, &BUS_D_config[9]);
  /* Initialize GPIO functionality on pin PTD12 */
  GPIO_PinInit(BUS_D_GPIO, 12U, &BUS_D_config[10]);
  /* Initialize GPIO functionality on pin PTD13 */
  GPIO_PinInit(BUS_D_GPIO, 13U, &BUS_D_config[11]);
}

/***********************************************************************************************************************
 * OUTPUTS initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'OUTPUTS'
- type: 'gpio'
- mode: 'GPIO'
- type_id: 'gpio_f970a92e447fa4793838db25a2947ed7'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'GPIOB'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'false'
    - port_interrupt:
      - IRQn: 'PORTB_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
    - gpio_config:
      - 0:
        - signal_number: 'GPIO.10'
        - pinDirection: 'kGPIO_DigitalOutput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 1:
        - signal_number: 'GPIO.11'
        - pinDirection: 'kGPIO_DigitalOutput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 2:
        - signal_number: 'GPIO.16'
        - pinDirection: 'kGPIO_DigitalOutput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 3:
        - signal_number: 'GPIO.18'
        - pinDirection: 'kGPIO_DigitalOutput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 4:
        - signal_number: 'GPIO.19'
        - pinDirection: 'kGPIO_DigitalOutput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
gpio_pin_config_t OUTPUTS_config[5] = {
  {
    .pinDirection = kGPIO_DigitalOutput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalOutput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalOutput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalOutput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalOutput,
    .outputLogic = 0U
  }
};

void OUTPUTS_init(void) {
  /* Make sure, the clock gate for port B is enabled (e. g. in pin_mux.c) */
  /* Initialize GPIO functionality on pin PTB10 */
  GPIO_PinInit(OUTPUTS_GPIO, 10U, &OUTPUTS_config[0]);
  /* Initialize GPIO functionality on pin PTB11 */
  GPIO_PinInit(OUTPUTS_GPIO, 11U, &OUTPUTS_config[1]);
  /* Initialize GPIO functionality on pin PTB16 */
  GPIO_PinInit(OUTPUTS_GPIO, 16U, &OUTPUTS_config[2]);
  /* Initialize GPIO functionality on pin PTB18 */
  GPIO_PinInit(OUTPUTS_GPIO, 18U, &OUTPUTS_config[3]);
  /* Initialize GPIO functionality on pin PTB19 */
  GPIO_PinInit(OUTPUTS_GPIO, 19U, &OUTPUTS_config[4]);
}

/***********************************************************************************************************************
 * INPUTS initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'INPUTS'
- type: 'gpio'
- mode: 'GPIO'
- type_id: 'gpio_f970a92e447fa4793838db25a2947ed7'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'GPIOA'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'false'
    - port_interrupt:
      - IRQn: 'PORTA_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
    - gpio_config:
      - 0:
        - signal_number: 'GPIO.5'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 1:
        - signal_number: 'GPIO.14'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 2:
        - signal_number: 'GPIO.15'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 3:
        - signal_number: 'GPIO.28'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 4:
        - signal_number: 'GPIO.29'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
      - 5:
        - signal_number: 'GPIO.26'
        - pinDirection: 'kGPIO_DigitalInput'
        - interrupt_configuration: 'kPORT_InterruptOrDMADisabled'
        - outputLogic: '0U'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
gpio_pin_config_t INPUTS_config[6] = {
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  },
  {
    .pinDirection = kGPIO_DigitalInput,
    .outputLogic = 0U
  }
};

void INPUTS_init(void) {
  /* Make sure, the clock gate for port A is enabled (e. g. in pin_mux.c) */
  /* Initialize GPIO functionality on pin PTA5 */
  GPIO_PinInit(INPUTS_GPIO, 5U, &INPUTS_config[0]);
  /* Initialize GPIO functionality on pin PTA14 */
  GPIO_PinInit(INPUTS_GPIO, 14U, &INPUTS_config[1]);
  /* Initialize GPIO functionality on pin PTA15 */
  GPIO_PinInit(INPUTS_GPIO, 15U, &INPUTS_config[2]);
  /* Initialize GPIO functionality on pin PTA28 */
  GPIO_PinInit(INPUTS_GPIO, 28U, &INPUTS_config[3]);
  /* Initialize GPIO functionality on pin PTA29 */
  GPIO_PinInit(INPUTS_GPIO, 29U, &INPUTS_config[4]);
  /* Initialize GPIO functionality on pin PTA26 */
  GPIO_PinInit(INPUTS_GPIO, 26U, &INPUTS_config[5]);
}

/***********************************************************************************************************************
 * SYSCLK initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'SYSCLK'
- type: 'ftm'
- mode: 'EdgeAligned'
- type_id: 'ftm_5e037045c21cf6f361184c371dbbbab2'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'FTM1'
- config_sets:
  - ftm_main_config:
    - ftm_config:
      - clockSource: 'kFTM_SystemClock'
      - prescale: 'kFTM_Prescale_Divide_1'
      - timerFrequency: '10000000'
      - bdmMode: 'kFTM_BdmMode_0'
      - pwmSyncMode: 'kFTM_SoftwareTrigger'
      - reloadPoints: ''
      - faultMode: 'kFTM_Fault_Disable'
      - faultFilterValue: '0'
      - deadTimePrescale: 'kFTM_Deadtime_Prescale_1'
      - deadTimeValue: '0'
      - extTriggers: ''
      - chnlInitState: ''
      - chnlPolarity: ''
      - useGlobalTimeBase: 'false'
    - timer_interrupts: ''
    - enable_irq: 'false'
    - ftm_interrupt:
      - IRQn: 'FTM1_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
    - EnableTimerInInit: 'true'
  - ftm_edge_aligned_mode:
    - ftm_edge_aligned_channels_config:
      - 0:
        - edge_aligned_mode: 'kFTM_EdgeAlignedPwm'
        - edge_aligned_pwm:
          - chnlNumber: 'kFTM_Chnl_0'
          - level: 'kFTM_LowTrue'
          - dutyCyclePercent: '50'
          - enable_chan_irq: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const ftm_config_t SYSCLK_config = {
  .prescale = kFTM_Prescale_Divide_1,
  .bdmMode = kFTM_BdmMode_0,
  .pwmSyncMode = kFTM_SoftwareTrigger,
  .reloadPoints = 0,
  .faultMode = kFTM_Fault_Disable,
  .faultFilterValue = 0,
  .deadTimePrescale = kFTM_Deadtime_Prescale_1,
  .deadTimeValue = 0,
  .extTriggers = 0,
  .chnlInitState = 0,
  .chnlPolarity = 0,
  .useGlobalTimeBase = false
};

const ftm_chnl_pwm_signal_param_t SYSCLK_pwmSignalParams[] = { 
  {
    .chnlNumber = kFTM_Chnl_0,
    .level = kFTM_LowTrue,
    .dutyCyclePercent = 50
  }
};

void SYSCLK_init(void) {
  FTM_Init(SYSCLK_PERIPHERAL, &SYSCLK_config);
  FTM_SetupPwm(SYSCLK_PERIPHERAL, SYSCLK_pwmSignalParams, sizeof(SYSCLK_pwmSignalParams) / sizeof(ftm_chnl_pwm_signal_param_t), kFTM_EdgeAlignedPwm, 10000000U, SYSCLK_CLOCK_SOURCE);
  FTM_StartTimer(SYSCLK_PERIPHERAL, kFTM_SystemClock);
}

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void)
{
  /* Initialize components */
  I2C_init();
  BUS_C_init();
  BUS_D_init();
  OUTPUTS_init();
  INPUTS_init();
  SYSCLK_init();
}

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
  BOARD_InitPeripherals();
}