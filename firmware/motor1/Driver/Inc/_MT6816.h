
#ifndef INC__MT6816_H_
#define INC__MT6816_H_

#include <spi.h>
#include <stdbool.h>

// 0x81 0b1000_0001 : Read, Status Reg
// 0x82 0b1000_0010 : Read, Configuration Reg
// 0x83 0b1000_0011 : Read, Angle High Byte
// 0x84 0b1000_0100 : Read, Angle Low Byte

#define MT6816_READ_ANGLE_CMD   		0x8384

#define STATUS_MAGNETIC_NO 				(1U << 0)
#define STATUS_MAGNETIC_WEAK 			(1U << 1)
#define STATUS_MAGNETIC_STRONG 			(1U << 2)
#define STATUS_CORDIC_ERR 				(1U << 3)
#define STATUS_DATA_INVALID 			(1U << 4)

#define CONFIG_RESOLUTION 				(1U << 0) // HIGH : 14bit, LOW : 12bit -> HIGH
#define CONFIG_DIRECTION 				(1U << 1) // HIGH : CCW, LOW : CW -> HIGH/LOW
#define CONFIG_FILTER 					(1U << 3) // HIGH : ON, LOW : OFF -> HIGH
#define CONFIG_ZERO_ALIGN 				(1U << 4) // HIGH : Current Angle Zero Offset Setting -> HIGH/LOW
#define CONFIG_OTP 						(1U << 5) // HIGH : Flash EN, LOW : RAM EN -> LOW


#define CHIP_SELECT(MT6816) (GPIOA->BRR  = GPIO_PIN_15)
#define CHIP_DESELECT(MT6816) (GPIOA->BSRR = GPIO_PIN_15)


uint16_t SendData16bit(uint16_t tx_data);
uint8_t CheckSum(uint16_t rawData);

void MT6816_Calibration(void);
uint16_t UpdateAngle(void);
uint8_t MT6816_IsValid(void);

#endif /* INC__MT6816_H_ */
