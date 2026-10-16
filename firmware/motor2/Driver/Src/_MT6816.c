#include "../Inc/_MT6816.h"
#include "../Inc/_LED.h"
typedef struct
{
	uint16_t rawData;
	uint16_t rawAngle;
	bool noMagFlag;
	bool checksumFlag;
} SpiRawData_t;

SpiRawData_t spiRawData;

void MT6816_Calibration(void)
{
	// update later
}

uint16_t UpdateAngle(void)
{
	uint16_t rawData = 0;
	uint16_t rawAngle;
	bool noMagnaticFlag = false;
	bool parityOk = false;
	bool sampleValid = false;

	for(uint8_t i=0; i<3; i++)
	{
		rawData = SendData16bit(MT6816_READ_ANGLE_CMD);
		parityOk = CheckSum(rawData) != 0U;
		noMagnaticFlag = (rawData & 0x0002U) != 0U;

		if(parityOk && !noMagnaticFlag) {
			sampleValid = true;
			break;
		}
	}

	/* Never inject a corrupt SPI sample into the velocity controller. Keep the
	   last valid angle for this tick and expose the invalid state separately. */
	if (!sampleValid)
	{
		led_flag = true;
		spiRawData.noMagFlag = noMagnaticFlag;
		spiRawData.checksumFlag = false;
		return spiRawData.rawData;
	}

	rawAngle = rawData >> 2;

	spiRawData.rawData = rawData;
	spiRawData.rawAngle = rawAngle;
	spiRawData.noMagFlag = false;
	spiRawData.checksumFlag = true;

	return rawData;
}

uint8_t MT6816_IsValid(void)
{
	return (spiRawData.checksumFlag && !spiRawData.noMagFlag) ? 1U : 0U;
}

uint8_t CheckSum(uint16_t rawData)
{
	uint8_t hCount = 0;

	for(uint8_t i=0; i<16; i++)
	{
		if(rawData & (0x0001 << i))
			hCount++;
	}
	if((hCount & 0x01) == 0)
		return 1;
	else
		return 0;
}

uint16_t SendData16bit(uint16_t tx_data)
{
    uint16_t tx_buf[2];
    uint16_t rx_buf[2];

    tx_buf[0] = tx_data & 0xFF00;
    tx_buf[1] = (tx_data << 8) & 0xFF00;

    CHIP_SELECT(MT6816);
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &tx_buf[0],
                            (uint8_t*) &rx_buf[0], 1, HAL_MAX_DELAY);
    CHIP_DESELECT(MT6816);

    CHIP_SELECT(MT6816);
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &tx_buf[1],
                            (uint8_t*) &rx_buf[1], 1, HAL_MAX_DELAY);
    CHIP_DESELECT(MT6816);

    return ((rx_buf[0] & 0x00FF) << 8) | (rx_buf[1] & 0x00FF);
}
