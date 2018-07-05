/*
 * mt9v034.h
 *
 *  Created on: 18 Mar 2018
 *      Author: boredman
 */


/* =================================================================== */
/*                          local definitions                          */
/* =================================================================== */

#define BINNING_ROW_A                   4
#define BINNING_COLUMN_A                4
#define BINNING_ROW_B                   2
#define BINNING_COLUMN_B                2
#define MINIMUM_HORIZONTAL_BLANKING     91 // see datasheet
#define MAX_IMAGE_HEIGHT                480
#define MAX_IMAGE_WIDTH                 752
#define MINIMUM_COLUMN_START            1
#define MINIMUM_ROW_START               4

/* Camera I2C registers */
#define MTV_DEVICE_WRITE_ADDRESS        0x90
#define MTV_DEVICE_READ_ADDRESS         (MTV_DEVICE_WRITE_ADDRESS | 0x01)

/* Context A */
#define MTV_COLUMN_START_REG_A          0x01
#define MTV_ROW_START_REG_A             0x02
#define MTV_WINDOW_HEIGHT_REG_A         0x03
#define MTV_WINDOW_WIDTH_REG_A          0x04
#define MTV_HOR_BLANKING_REG_A          0x05
#define MTV_VER_BLANKING_REG_A          0x06
#define MTV_COARSE_SW_1_REG_A           0x08
#define MTV_COARSE_SW_2_REG_A           0x09
#define MTV_COARSE_SW_CTRL_REG_A        0x0A
#define MTV_COARSE_SW_TOTAL_REG_A       0x0B
#define MTV_FINE_SW_1_REG_A             0xD3
#define MTV_FINE_SW_2_REG_A             0xD4
#define MTV_FINE_SW_TOTAL_REG_A         0xD5
#define MTV_READ_MODE_REG_A             0x0D
#define MTV_V1_CTRL_REG_A               0x31
#define MTV_V2_CTRL_REG_A               0x32
#define MTV_V3_CTRL_REG_A               0x33
#define MTV_V4_CTRL_REG_A               0x34
#define MTV_ANALOG_GAIN_CTRL_REG_A      0x35
#define MTV_ANALOG_GAIN_FORCE_0_75      (1 << 15)

/* Context B */
#define MTV_COLUMN_START_REG_B          0xC9
#define MTV_ROW_START_REG_B             0xCA
#define MTV_WINDOW_HEIGHT_REG_B         0xCB
#define MTV_WINDOW_WIDTH_REG_B          0xCC
#define MTV_HOR_BLANKING_REG_B          0xCD
#define MTV_VER_BLANKING_REG_B          0xCE
#define MTV_COARSE_SW_1_REG_B           0xCF
#define MTV_COARSE_SW_2_REG_B           0xD0
#define MTV_COARSE_SW_CTRL_REG_B        0xD1
#define MTV_COARSE_SW_TOTAL_REG_B       0xD2
#define MTV_FINE_SW_1_REG_B             0xD6
#define MTV_FINE_SW_2_REG_B             0xD7
#define MTV_FINE_SW_TOTAL_REG_B         0xD8
#define MTV_READ_MODE_REG_B             0x0E
#define MTV_V1_CTRL_REG_B               0x39
#define MTV_V2_CTRL_REG_B               0x3A
#define MTV_V3_CTRL_REG_B               0x3B
#define MTV_V4_CTRL_REG_B               0x3C
#define MTV_ANALOG_GAIN_CTRL_REG_B      0x36

/* General Registers */
#define MTV_CHIP_VERSION_REG            0x00

#define MTV_CHIP_CONTROL_REG            0x07
#define MTV_CHIP_CONTROL_SLAVE_MODE     (0 << 3)
#define MTV_CHIP_CONTROL_MASTER_MODE    (1 << 3)
#define MTV_CHIP_CONTROL_SNAPSHOT_MODE  (3 << 3)
#define MTV_CHIP_CONTROL_DOUT_ENABLE    (1 << 7)
#define MTV_CHIP_CONTROL_SEQUENTIAL     (0 << 8)
#define MTV_CHIP_CONTROL_SIMULTANEOUS   (1 << 8)

#define MTV_SOFT_RESET_REG              0x0C

#define MTV_HDR_ENABLE_REG              0x0F
#define MTV_ADC_RES_CTRL_REG            0x1C
#define MTV_ADC_RES_LINR_A              (2)
#define MTV_ADC_RES_COMP_A              (3)
#define MTV_ADC_RES_LINR_B              (2 << 8)
#define MTV_ADC_RES_COMP_B              (3 << 8)
#define MTV_ROW_NOISE_CORR_CTRL_REG     0x70
#define MTV_TEST_PATTERN_REG            0x7F
#define MTV_TILED_DIGITAL_GAIN_REG      0x80
#define MTV_AGC_AEC_DESIRED_BIN_REG     0xA5
#define MTV_AEC_EXP_LPF_REG             0xA8
#define MTV_MAX_GAIN_REG                0xAB
#define MTV_MIN_EXPOSURE_REG            0xAC  // datasheet min coarse shutter width
#define MTV_MAX_EXPOSURE_REG            0xAD  // datasheet max coarse shutter width
#define MTV_AEC_AGC_ENABLE_REG          0xAF
#define MTV_AEC_DISABLE_A               (0)
#define MTV_AEC_ENABLE_A                (1)
#define MTV_AGC_DISABLE_A               (0)
#define MTV_AGC_ENABLE_A                (1 << 1)
#define MTV_AGC_AEC_PIXEL_COUNT_REG     0xB0
#define MTV_AGC_OUTPUT_REG              0xBA
#define MTV_AEC_OUTPUT_REG              0xBB
#define MTV_AGC_AEC_CURRENT_BIN_REG     0xBC
#define MTV_AEC_UPDATE_REG              0xA6
#define MTV_AEC_LOWPASS_REG             0xA8
#define MTV_AGC_UPDATE_REG              0xA9
#define MTV_AGC_LOWPASS_REG             0xAA
#define MTV_DIGITAL_TEST_REG            0x7F

/*
 * Resolution:
 * ROW_SIZE * BINNING_ROW <= MAX_IMAGE_WIDTH
 * COLUMN_SIZE * BINNING_COLUMN <= MAX_IMAGE_HEIGHT
 */

static status_t MT9Initialize(uint8_t dev)
{
    // slave mode
	status_t st = I2C_Write16(dev, MTV_CHIP_CONTROL_REG,  MTV_CHIP_CONTROL_SLAVE_MODE
                                          | MTV_CHIP_CONTROL_DOUT_ENABLE
                                          | MTV_CHIP_CONTROL_SEQUENTIAL );
	// recommended reserved register settings
	I2C_Write16(dev, 0x20, 0x03C7);
	I2C_Write16(dev, 0x24, 0x001B);
	I2C_Write16(dev, 0x2B, 0x0003);
	I2C_Write16(dev, 0x2F, 0x0003);

	return st;
}

static status_t MT9GetVersion(uint8_t dev, uint16_t* uiVer)
{
	return I2C_Read16(dev, MTV_CHIP_VERSION_REG, uiVer);
}

static status_t MT9SetCompanding(uint8_t dev, bool on)
{
	return I2C_Write16(dev, MTV_ADC_RES_CTRL_REG, (on ? MTV_ADC_RES_COMP_A : MTV_ADC_RES_LINR_A) );
}

static status_t MT9SetAGC_AEC(uint8_t dev, bool agc_on, bool aec_on)
{
	return I2C_Write16(dev, MTV_AEC_AGC_ENABLE_REG, (agc_on ? MTV_AGC_ENABLE_A : MTV_AGC_DISABLE_A)
                                                  | (aec_on ? MTV_AEC_ENABLE_A : MTV_AEC_DISABLE_A) );
}

// valid rage: 16 - 64 dec
static status_t Mt9SetAnalogGain(uint8_t dev, uint8_t ag)
{
	uint16_t val = ag;
    if( ag < 16 )
        val = 16 | MTV_ANALOG_GAIN_FORCE_0_75;
    else if( ag > 64 )
        val = 64;
    return I2C_Write16(dev, MTV_ANALOG_GAIN_CTRL_REG_A, val);
}

static status_t MT9SetDigitalGain(uint8_t dev, uint16_t dg)
{
	status_t st;
    if( dg > 15 )
        dg = 15;
    for(uint8_t i=0; i<=24; i++)
    	st = I2C_Write16(dev, MTV_TILED_DIGITAL_GAIN_REG + i, dg | (0xF << 4));
    return st;
}

static status_t MT9GetAGC_AECvalues(uint8_t dev, uint16_t* uiVer)
{
	return I2C_Read16(dev, MTV_AGC_OUTPUT_REG, uiVer);
}
