#!/usr/bin/env python
'''
these tables are generated from the STM32 datasheets for the
STM32F303RE
'''

# additional build information for ChibiOS
build = {
    "CHIBIOS_STARTUP_MK"  : "os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f3xx.mk",
    "CHIBIOS_PLATFORM_MK" : "os/hal/ports/STM32/STM32F3xx/platform.mk"
    }

# MCU parameters
mcu = {
    # location of MCU serial number
    'UDID_START' : 0x1FFFF7AC,

    # ram map, as list of (address, size-kb, flags)
    # flags of 1 means DMA-capable
    # flags of 2 means faster memory for CPU intensive work
    'RAM_MAP' : [
        (0x20000000,  40, 1), # main memory, DMA safe
        (0x10000000,   16, 2), # CCM memory, faster, but not DMA safe
    ],

    'EXPECTED_CLOCK' : 72000000
}

AltFunction_map = {
	# format is PIN:FUNCTION : AFNUM
	# extracted from tabula-AF-STM32F303cc.csv
	"PA0:COMP1_OUT"     	:	8,
	"PA0:EVENTOUT"      	:	15,
	"PA0:TIM2_CH1_ETR"  	:	1,
	"PA0:TIM8_BKIN"     	:	9,
	"PA0:TIM8_ETR"      	:	10,
	"PA0:TSC_G1_IO1"    	:	3,
	"PA0:USART2_CTS"    	:	7,
	"PA1:EVENTOUT"      	:	15,
	"PA1:RTC_REFIN"     	:	0,
	"PA1:TIM15_CH1N"    	:	9,
	"PA1:TIM2_CH2"      	:	1,
	"PA1:TSC_G1_IO2"    	:	3,
	"PA1:USART2_RTS_DE" 	:	7,
	"PA2:COMP2_OUT"     	:	8,
	"PA2:EVENTOUT"      	:	15,
	"PA2:TIM15_CH1"     	:	9,
	"PA2:TIM2_CH3"      	:	1,
	"PA2:TSC_G1_IO3"    	:	3,
	"PA2:USART2_TX"     	:	7,
	"PA3:EVENTOUT"      	:	15,
	"PA3:TIM15_CH2"     	:	9,
	"PA3:TIM2_CH4"      	:	1,
	"PA3:TSC_G1_IO4"    	:	3,
	"PA3:USART2_RX"     	:	7,
	"PA4:EVENTOUT"      	:	15,
	"PA4:SPI1_NSS"      	:	5,
	"PA4:SPI3_NSS,I2S3_WS"	:	6,
	"PA4:TIM3_CH2"      	:	2,
	"PA4:TSC_G2_IO1"    	:	3,
	"PA4:USART2_CK"     	:	7,
	"PA5:EVENTOUT"      	:	15,
	"PA5:SPI1_SCK"      	:	5,
	"PA5:TIM2_CH1_ETR"  	:	1,
	"PA5:TSC_G2_IO2"    	:	3,
	"PA6:COMP1_OUT"     	:	8,
	"PA6:EVENTOUT"      	:	15,
	"PA6:SPI1_MISO"     	:	5,
	"PA6:TIM16_CH1"     	:	1,
	"PA6:TIM1_BKIN"     	:	6,
	"PA6:TIM3_CH1"      	:	2,
	"PA6:TIM8_BKIN"     	:	4,
	"PA6:TSC_G2_IO3"    	:	3,
	#"PA7:COMP2_OUT"     	:	8,
	"PA7:EVENTOUT"      	:	15,
	"PA7:SPI1_MOSI"     	:	5,
	"PA7:TIM17_CH1"     	:	1,
	"PA7:TIM1_CH1N"     	:	6,
	"PA7:TIM3_CH2"      	:	2,
	"PA7:TIM8_CH1N"     	:	4,
	"PA7:TSC_G2_IO4"    	:	3,
	"PA8:COMP3_OUT"     	:	8,
	"PA8:EVENTOUT"      	:	15,
	
	"PA8:I2C2_SMBA"     	:	4,
	
	"PA8:I2S2_MCK"      	:	5,
	"PA8:MCO"           	:	0,
	"PA8:TIM1_CH1"      	:	6,
	"PA8:TIM4_ETR"      	:	10,
	"PA8:USART1_CK"     	:	7,
	"PA8:I2C3_SCL"          :   3,
	
	"PA9:COMP5_OUT"     	:	8,
	"PA9:EVENTOUT"      	:	15,
	"PA9:I2C2_SCL"      	:	4,
	"PA9:I2S3_MCK"      	:	5,
	"PA9:TIM15_BKIN"    	:	9,
	"PA9:TIM1_CH2"      	:	6,
	"PA9:TIM2_CH3"      	:	10,
	"PA9:TSC_G4_IO1"    	:	3,
	"PA9:USART1_TX"     	:	7,

	"PA9:I2C2_SMBA"     	:	2,

	"PA10:COMP6_OUT"    	:	8,
	"PA10:EVENTOUT"     	:	15,
	"PA10:I2C2_SDA"     	:	4,
	"PA10:TIM17_BKIN"   	:	1,
	"PA10:TIM1_CH3"     	:	6,
	"PA10:TIM2_CH4"     	:	10,
	"PA10:TIM8_BKIN"    	:	11,
	"PA10:TSC_G4_IO2"   	:	3,
	"PA10:USART1_RX"    	:	7,
	"PA10:SPI2_MISO,I2S2EXT_SD"	:	5,

	"PA11:CAN_RX"       	:	9,
	"PA11:COMP1_OUT"    	:	8,
	"PA11:EVENTOUT"     	:	15,
	"PA11:TIM1_BKIN2"   	:	12,
	"PA11:TIM1_CH1N"    	:	6,
	"PA11:TIM1_CH4"     	:	11,
	"PA11:TIM4_CH1"     	:	10,
	"PA11:USART1_CTS"   	:	7,
#	"PA11:USB_DM"       	:	14,
	"PA11:SPI2_MOSI,I2S2_SD"	:	5,

	"PA12:CAN_TX"       	:	9,
	"PA12:COMP2_OUT"    	:	8,
	"PA12:EVENTOUT"     	:	15,
	"PA12:TIM16_CH1"    	:	1,
	"PA12:TIM1_CH2N"    	:	6,
	"PA12:TIM1_ETR"     	:	11,
	"PA12:TIM4_CH2"     	:	10,

	"PA12:USART1_RTS_DE"	:	7,

#	"PA12:USB_DP"       	:	14,
	"PA12:I2S_CKIN"      	:	5,

	"PA13:EVENTOUT"     	:	15,
	"PA13:IR_OUT"       	:	5,
	"PA13:JTMS-SWDIO"   	:	0,
	"PA13:TIM16_CH1N"   	:	1,
	"PA13:TIM4_CH3"     	:	10,
	"PA13:TSC_G4_IO3"   	:	3,
	"PA13:USART3_CTS"   	:	7,

	"PA14:EVENTOUT"     	:	15,
	"PA14:I2C1_SDA"     	:	4,
	"PA14:JTCK-SWCLK"   	:	0,
	"PA14:TIM1_BKIN"    	:	6,
	"PA14:TIM8_CH2"     	:	5,
	"PA14:TSC_G4_IO4"   	:	3,
	"PA14:USART2_TX"    	:	7,

	"PA15:EVENTOUT"     	:	15,
	"PA15:I2C1_SCL"     	:	4,
	"PA15:JTDI"         	:	0,
	"PA15:SPI1_NSS"     	:	5,
	"PA15:SPI3_NSS,I2S3_WS"	:	6,
	"PA15:TIM1_BKIN"    	:	9,

	"PA15:TIM2_CH1_ETR" 	:	1,

	"PA15:TIM8_CH1"     	:	2,
	"PA15:USART2_RX"    	:	7,
	"PA15:TSC_SYNC"      	:	3,

	"PB0:EVENTOUT"      	:	15,
	"PB0:TIM1_CH2N"     	:	6,
	"PB0:TIM3_CH3"      	:	2,
	"PB0:TIM8_CH2N"     	:	4,
	"PB0:TSC_G3_IO2"    	:	3,

	"PB1:COMP4_OUT"     	:	8,
	"PB1:EVENTOUT"      	:	15, 
	"PB1:TIM1_CH3N"     	:	6,
	"PB1:TIM3_CH4"      	:	2,
	"PB1:TIM8_CH3N"     	:	4,
	"PB1:TSC_G3_IO3"    	:	3,

	"PB2:EVENTOUT"      	:	15,
	"PB2:TSC_G3_IO4"    	:	3,

	"PB3:EVENTOUT"      	:	15,
	"PB3:JTDO-TRACESWO" 	:	0,
	"PB3:SPI1_SCK"      	:	5,
	"PB3:SPI3_SCK,I2S3_CK"	:	6,
	"PB3:TIM2_CH2"      	:	1,
	"PB3:TIM3_ETR"      	:	10,
	"PB3:TIM4_ETR"      	:	2,
	"PB3:TIM8_CH1N"     	:	4,
	"PB3:TSC_G5_IO1"    	:	3,
	"PB3:USART2_TX"     	:	7,

	"PB4:EVENTOUT"      	:	15,
	"PB4:NJTRST"        	:	0,
	"PB4:SPI1_MISO"     	:	5,
	"PB4:SPI3_MISO,I2S3EXT_SD"	:	6,
	"PB4:TIM16_CH1"     	:	1,
	"PB4:TIM17_BKIN"    	:	10,
	"PB4:TIM3_CH1"      	:	2,
	"PB4:TIM8_CH2N"     	:	4,
	"PB4:TSC_G5_IO2"    	:	3,
	"PB4:USART2_RX"     	:	7,

	"PB5:EVENTOUT"      	:	15,
	"PB5:I2C1_SMBA"     	:	4,
	"PB5:SPI1_MOSI"     	:	5,
	"PB5:SPI3_MOSI,I2S3_SD"	:	6,
	"PB5:TIM16_BKIN"    	:	1,
	"PB5:TIM17_CH1"     	:	10,
	"PB5:TIM3_CH2"      	:	2,
	"PB5:TIM8_CH3N"     	:	3,
	"PB5:USART2_CK"     	:	7,
	"PB5:I2C3_SDA"     	    :	8,

	"PB6:EVENTOUT"      	:	15,
	"PB6:I2C1_SCL"      	:	4,
	"PB6:TIM16_CH1N"    	:	1,
	"PB6:TIM4_CH1"      	:	2,
	"PB6:TIM8_BKIN2"    	:	10,
	"PB6:TIM8_CH1"      	:	5,
	"PB6:TIM8_ETR"      	:	6,
	"PB6:TSC_G5_IO3"    	:	3,
	"PB6:USART1_TX"     	:	7,

	"PB7:EVENTOUT"      	:	15,
	"PB7:I2C1_SDA"      	:	4,
	"PB7:TIM17_CH1N"    	:	1,
	"PB7:TIM3_CH4"      	:	10,
	"PB7:TIM4_CH2"      	:	2,
	"PB7:TIM8_BKIN"     	:	5,
	"PB7:TSC_G5_IO4"    	:	3,
	"PB7:USART1_RX"     	:	7,

	"PB7:FMC_NADV"      	:	12,

	"PB8:CAN_RX"        	:	9,
	"PB8:COMP1_OUT"     	:	8,
	"PB8:EVENTOUT"      	:	15,
	"PB8:I2C1_SCL"      	:	4,
	"PB8:TIM16_CH1"     	:	1,
	"PB8:TIM1_BKIN"     	:	12,
	"PB8:TIM4_CH3"      	:	2,
	"PB8:TIM8_CH2"      	:	10,
	"PB8:TSC_SYNC"      	:	3,
	"PB8:USART3_RX"      	:	7,
	
	"PB9:CAN_TX"        	:	9,
	"PB9:COMP2_OUT"     	:	8,
	"PB9:EVENTOUT"      	:	15,
	"PB9:I2C1_SDA"      	:	4,
	"PB9:IR_OUT"        	:	6,
	"PB9:TIM17_CH1"     	:	1,
	"PB9:TIM4_CH4"      	:	2,
	"PB9:TIM8_CH3"      	:	10,
	"PB9:USART3_TX"      	:	7,

	"PB10:EVENTOUT"     	:	15,
	"PB10:TIM2_CH3"     	:	1,
	"PB10:TSC_SYNC"     	:	3,
	"PB10:USART3_TX"    	:	7,

	"PB11:EVENTOUT"     	:	15,
	"PB11:TIM2_CH4"     	:	1,
	"PB11:TSC_G6_IO1"   	:	3,
	"PB11:USART3_RX"    	:	7,

	"PB12:EVENTOUT"     	:	15,
	"PB12:I2C2_SMBA"    	:	4,
	"PB12:SPI2_NSS,I2S2_WS"	:	5,
	"PB12:TIM1_BKIN"    	:	6,
	"PB12:TSC_G6_IO2"   	:	3,
	"PB12:USART3_CK"    	:	7,

	"PB13:EVENTOUT"     	:	15,
	"PB13:SPI2_SCK,I2S2_CK"	:	5,
	"PB13:TIM1_CH1N"    	:	6,
	"PB13:TSC_G6_IO3"   	:	3,
	"PB13:USART3_CTS"   	:	7,

	"PB14:EVENTOUT"     	:	15,
	"PB14:SPI2_MISO,I2S2EXT_SD"	:	5,
	"PB14:TIM15_CH1"    	:	1,
	"PB14:TIM1_CH2N"    	:	6,
	"PB14:TSC_G6_IO4"   	:	3,
	"PB14:USART3_RTS_DE"	:	7,

	"PB15:EVENTOUT"     	:	15,
	"PB15:RTC_REFIN"    	:	0,
	"PB15:SPI2_MOSI,I2S2_SD"	:	5,
	"PB15:TIM15_CH1N"   	:	2,
	"PB15:TIM15_CH2"    	:	1,
	"PB15:TIM1_CH3N"    	:	4,

	"PC0:EVENTOUT"      	:	1,
	"PC0:TIM1_CH1"      	:	2,
	
	"PC1:EVENTOUT"      	:	1,
	"PC1:TIM1_CH2"      	:	2,
	
	"PC2:COMP7_OUT"     	:	3,
	"PC2:EVENTOUT"      	:	1,
	"PC2:TIM1_CH3"      	:	2,

	"PC3:EVENTOUT"      	:	1,
	"PC3:TIM1_CH4"      	:	2,
	"PC3:TIM1_BKIN2"    	:	6,

	"PC4:EVENTOUT"      	:	1,
	"PC4:TIM1_ETR"      	:	2,
	"PC4:USART1_TX"     	:	7,

	"PC5:EVENTOUT"      	:	1,
	"PC5:TSC_G3_IO1"    	:	3,
	"PC5:TIM15_BKIN"     	:	2,
	"PC5:USART1_RX"     	:	7,

	"PC6:COMP6_OUT"     	:	7,
	"PC6:EVENTOUT"      	:	1,
	"PC6:I2S2_MCK"      	:	6,
	"PC6:TIM3_CH1"      	:	2,
	"PC6:TIM8_CH1"      	:	4,

	"PC7:COMP5_OUT"     	:	7,
	"PC7:EVENTOUT"      	:	1,
	"PC7:I2S3_MCK"      	:	6,
	"PC7:TIM3_CH2"      	:	2,
	"PC7:TIM8_CH2"      	:	4,

	"PC8:COMP3_OUT"     	:	7,
	"PC8:EVENTOUT"      	:	1,
	"PC8:TIM3_CH3"      	:	2,
	"PC8:TIM8_CH3"      	:	4,

	"PC9:EVENTOUT"      	:	1,
	"PC9:I2S_CKIN"      	:	5,
	"PC9:TIM3_CH4"      	:	2,
	"PC9:TIM8_BKIN2"    	:	6,
	"PC9:TIM8_CH4"      	:	4,
	"PC9:I2C3_SDA"      	:	3,

	"PC10:EVENTOUT"     	:	1,
	"PC10:SPI3_SCK,I2S3_CK"	:	6,
	"PC10:TIM8_CH1N"    	:	4,
	"PC10:UART4_TX"     	:	5,
	"PC10:USART3_TX"    	:	7,

	"PC11:EVENTOUT"     	:	1,
	"PC11:SPI3_MISO,I2S3EXT_SD"	:	6,
	"PC11:TIM8_CH2N"    	:	4,
	"PC11:UART4_RX"     	:	5,
	"PC11:USART3_RX"    	:	7,

	"PC12:EVENTOUT"     	:	1,
	"PC12:SPI3_MOSI,I2S3_SD"	:	6,
	"PC12:TIM8_CH3N"    	:	4,
	"PC12:UART5_TX"     	:	5,
	"PC12:USART3_CK"    	:	7,
	
	"PC13:TIM1_CH1N"    	:	4,
	"PC13:EVENTOUT"     	:	1,

	"PC14:EVENTOUT"     	:	1,

	"PC15:EVENTOUT"     	:	1,
	
	"PD0:CAN_RX"        	:	7,
	"PD0:EVENTOUT"      	:	1,
	"PD0:FMC_D2"      	:	12,
	

	"PD1:CAN_TX"        	:	7,
	"PD1:EVENTOUT"      	:	1,
	"PD1:TIM8_BKIN2"    	:	6,
	"PD1:TIM8_CH4"      	:	4,
	"PD1:FMC_D3"      	:	12,

	"PD2:EVENTOUT"      	:	1,
	"PD2:TIM3_ETR"      	:	2,
	"PD2:TIM8_BKIN"     	:	4,
	"PD2:UART5_RX"      	:	5,
	
	"PD3:EVENTOUT"      	:	1,
	"PD3:TIM2_CH1_ETR"  	:	2,
	"PD3:USART2_CTS"    	:	7,
	"PD3:FMC_CLK"      	:	12,

	"PD4:EVENTOUT"      	:	1,
	"PD4:TIM2_CH2"      	:	2,
	"PD4:USART2_RTS_DE" 	:	7,
	"PD4:FMC_NOE"      	:	12,

	"PD5:EVENTOUT"      	:	1,
	"PD5:USART2_TX"     	:	7,
	"PD5:FMC_NWE"      	:	12,

	"PD6:EVENTOUT"      	:	1,
	"PD6:TIM2_CH4"      	:	2,
	"PD6:USART2_RX"     	:	7,
	"PD6:FMC_NWAIT"      	:	12,

	"PD7:EVENTOUT"      	:	1,
	"PD7:TIM2_CH3"      	:	2,
	"PD7:USART2_CK"     	:	7,
	"PD7:FMC_NE1,FMC_NCE2"      	:	12,

	"PD8:EVENTOUT"      	:	1,
	"PD8:USART3_TX"     	:	7,
	"PD8:FMC_D13"      	:	12,

	"PD9:EVENTOUT"      	:	1,
	"PD9:USART3_RX"     	:	6,
	"PD9:FMC_D14"      	:	12,

	"PD10:EVENTOUT"     	:	1,
	"PD10:USART3_CK"    	:	7,
	"PD10:FMC_D15"      	:	12,
	
	"PD11:EVENTOUT"     	:	1,
	"PD11:USART3_CTS"   	:	7,
	"PD11:FMC_A16"      	:	12,

	"PD12:EVENTOUT"     	:	1,
	"PD12:TIM4_CH1"     	:	2,
	"PD12:TSC_G8_IO1"   	:	3,
	"PD12:USART3_RTS_DE"	:	7,
	"PD12:FMC_A17"      	:	12,

	"PD13:EVENTOUT"     	:	1,
	"PD13:TIM4_CH2"     	:	2,
	"PD13:TSC_G8_IO2"   	:	3,
	"PD13:FMC_A18"      	:	12,

	"PD14:EVENTOUT"     	:	1,
	"PD14:TIM4_CH3"     	:	2,
	"PD14:TSC_G8_IO3"   	:	3,
	"PD14:FMC_D0"      	:	12,

	"PD15:EVENTOUT"     	:	1,
	"PD15:SPI2_NSS"     	:	6,
	"PD15:TIM4_CH4"     	:	2,
	"PD15:TSC_G8_IO4"   	:	3,
	"PD15:FMC_D1"      	:	12,

	"PE0:EVENTOUT"      	:	1,
	"PE0:TIM16_CH1"     	:	4,
	"PE0:TIM4_ETR"      	:	2,
	"PE0:USART1_TX"     	:	7,
	"PE0:TIM20_CH4"     	:	6,
	"PE0:FMC_NBL0"      	:	12,


	"PE1:EVENTOUT"      	:	1,
	"PE1:TIM17_CH1"     	:	4,
	"PE1:USART1_RX"     	:	7,
	"PE1:TIM20_CH4"     	:	6,
	"PE1:FMC_NBL1"      	:	12,

	"PE2:EVENTOUT"      	:	1,
	"PE2:TIM3_CH1"      	:	2,
	"PE2:TRACECK"       	:	0,
	"PE2:TSC_G7_IO1"    	:	3,
	"PE2:SPI4_SCK"      	:	5,
	"PE2:TIM20_CH1"      	:	6,
	"PE2:FMC_A23"      	:	12,

	"PE3:EVENTOUT"      	:	1,
	"PE3:TIM3_CH2"      	:	2,
	"PE3:TRACED0"       	:	0,
	"PE3:TSC_G7_IO2"    	:	3,
	"PE3:SPI4_NSS"       	:	5,
	"PE3:TIM20_CH2"       	:	6,
	"PE3:FMC_A19"      	:	12,

	"PE4:EVENTOUT"      	:	1,
	"PE4:TIM3_CH3"      	:	2,
	"PE4:TRACED1"       	:	0,
	"PE4:TSC_G7_IO3"    	:	3,
	"PE4:SPI4_NSS"      	:	5,
	"PE4:TIM20_CH1N"    	:	6,
	"PE4:FMC_A20"      	:	12,

	"PE5:EVENTOUT"      	:	1,
	"PE5:TIM3_CH4"      	:	2,
	"PE5:TRACED2"       	:	0,
	"PE5:TSC_G7_IO4"    	:	3,
	"PE5:SPI4_MISO"     	:	5,
	"PE5:SPI4_MOSI"     	:	6,
	"PE5:FMC_A21"      	:	12,

	"PE6:EVENTOUT"      	:	1,
	"PE6:TRACED3"       	:	0,
	"PE6:SPI4_MOSI"       	:	5,
	"PE6:TIM20_CH3N"       	:	6,
	"PE6:FMC_A22"      	:	12,

	"PE7:EVENTOUT"      	:	1,
	"PE7:TIM1_ETR"      	:	2,
	"PE7:FMC_D4"      	:	12,

	"PE8:EVENTOUT"      	:	1,
	"PE8:TIM1_CH1N"     	:	2,
	"PE8:FMC_D5"      	:	12,

	"PE9:EVENTOUT"      	:	1,
	"PE9:TIM1_CH1"      	:	2,
	"PE9:FMC_D6"      	:	12,

	"PE10:EVENTOUT"     	:	1,
	"PE10:TIM1_CH2N"    	:	2,
	"PE10:FMC_D7"      	:	12,

	"PE11:EVENTOUT"     	:	1,
	"PE11:TIM1_CH2"     	:	2,
	"PE11:SPI4_NSS"     	:	5,
	"PE11:FMC_D8"      	:	12,

	"PE12:EVENTOUT"     	:	1,
	"PE12:TIM1_CH3N"    	:	2,
	"PE12:SPI4_SCK"     	:	5,
	"PE12:FMC_D9"      	:	12,


	"PE13:EVENTOUT"     	:	1,
	"PE13:TIM1_CH3"     	:	2,
	"PE13:SPI4_MISO"     	:	5,
	"PE13:FMC_D10"      	:	12,
	
	"PE14:EVENTOUT"     	:	1,
	"PE14:TIM1_BKIN2"   	:	6,
	"PE14:TIM1_CH4"     	:	2,
	"PE14:SPI4_MISO"     	:	5,
	"PE14:FMC_D11"      	:	12,

	"PE15:EVENTOUT"     	:	1,
	"PE15:TIM1_BKIN"    	:	2,
	"PE15:USART3_RX"    	:	7,
	"PE14:FMC_D12"      	:	12,

	"PF0:EVENTOUT"      	:	1,
	"PF0:I2C2_SDA"      	:	4,
	"PF0:TIM1_CH3N"     	:	6,
	"PF0:SPI2_NSS,I2S2_WS"	:	5,

	"PF1:EVENTOUT"      	:	1,
	"PF1:I2C2_SCL"      	:	4,
	"PF1:SPI2_SCK,I2S2_CK"	:	5,

	"PF2:EVENTOUT"      	:	1,
	"PF2:TIM20_CH3"      	:	2,
	"PF2:FMC_A2"      	:	12,

	"PF3:EVENTOUT"      	:	1,
	"PF3:TIM20_CH4"      	:	2,
	"PF3:FMC_A3"      	:	12,

	"PF4:COMP1_OUT"     	:	2,
	"PF4:EVENTOUT"      	:	1,
	"PF4:TIM20_CH1N"      	:	3,
	"PF4:FMC_A4"      	:	12,

	"PF5:EVENTOUT"      	:	1,
	"PF5:TIM20_CH2N"      	:	2,
	"PF5:FMC_A5"      	:	12,

	"PF6:EVENTOUT"      	:	1,
	"PF6:I2C2_SCL"      	:	4,
	"PF6:TIM4_CH4"      	:	2,
	"PF6:USART3_RTS_DE" 	:	7,
	"PF6:FMC_NIORD"      	:	12,

	"PF7:EVENTOUT"      	:	1,
	"PF7:TIM20_BKIN"      	:	2,
	"PF7:FMC_NREG"      	:	12,

	"PF8:EVENTOUT"      	:	1,
	"PF8:TIM20_BKIN2"      	:	2,
	"PF8:FMC_NIOWR"      	:	12,

	"PF9:EVENTOUT"      	:	1,
	"PF9:TIM20_BKIN"      	:	2,
	"PF9:SPI2_SCK"      	:	5,
	"PF9:TIM15_CH1"     	:	3,
	"PF9:FMC_CD"      	:	12,

	"PF10:EVENTOUT"     	:	1,
	"PF10:TIM20_BKIN2"      	:	2,
	"PF10:SPI2_SCK"     	:	4,
	"PF10:TIM15_CH2"    	:	3,
	"PF10:SPI2_SCK"     	:	5,
	"PF10:FMC_INTR"      	:	12,

	"PF11:EVENTOUT"      	:	1,
	"PF11:TIM20_ETR"      	:	2,

	"PF12:EVENTOUT"      	:	1,
	"PF12:TIM20_CH1"      	:	2,
	"PF12:FMC_A6"      	:	12,

	"PF13:EVENTOUT"      	:	1,
	"PF13:TIM20_CH2"      	:	2,
	"PF13:FMC_A7"      	:	12,

	"PF14:EVENTOUT"      	:	1,
	"PF14:TIM20_CH3"      	:	2,
	"PF14:FMC_A8"      	:	12,

	"PF15:EVENTOUT"      	:	1,
	"PF15:TIM20_CH4"      	:	2,
	"PF15:FMC_A9"      	:	12,
	
	"PG0:EVENTOUT"      	:	1,
	"PG0:TIM20_CH1N"      	:	2,
	"PG0:FMC_A10"      	:	12,
		
	"PG1:EVENTOUT"      	:	1,
	"PG1:TIM20_CH2N"      	:	2,
	"PG1:FMC_A11"      	:	12,
	
	"PG2:EVENTOUT"      	:	1,
	"PG2:TIM20_CH3N"      	:	2,
	"PG2:FMC_A12"      	:	12,
	
	"PG3:EVENTOUT"      	:	1,
	"PG3:TIM20_BKIN"      	:	2,
	"PG3:FMC_A13"      	:	12,
	
	"PG4:EVENTOUT"      	:	1,
	"PG4:TIM20_BKIN2"      	:	2,
	"PG4:FMC_A14"      	:	12,
	
	"PG5:EVENTOUT"      	:	1,
	"PG5:TIM20_ETR"      	:	2,
	"PG5:FMC_A15"      	:	12,

	"PG6:EVENTOUT"      	:	1,
	"PG6:FMC_INT2"      	:	12,

	"PG7:EVENTOUT"      	:	1,
	"PG7:FMC_INT3"      	:	12,

	"PG8:EVENTOUT"      	:	1,

	"PG9:EVENTOUT"      	:	1,
	"PG9:FMC_NE2,FMC_NCE3"      	:	12,

	"PG10:EVENTOUT"      	:	1,
	"PG10:FMC_NCE4_1,FMC_NE3"      	:	12,

	"PG11:EVENTOUT"      	:	1,
	"PG11:FMC_NCE4_2"      	:	12,

	"PG12:EVENTOUT"      	:	1,
	"PG12:FMC_NE4"      	:	12,

	"PG13:EVENTOUT"      	:	1,
	"PG13:FMC_A24"      	:	12,

	"PG14:EVENTOUT"      	:	1,
	"PG14:FMC_A25"      	:	12,

	"PG15:EVENTOUT"      	:	1,
	
	"PH0:EVENTOUT"      	:	1,
	"PH0:TIM20_CH1"      	:	2,
	"PH0:FMC_A0"      	:	12,


	"PH1:EVENTOUT"      	:	1,
	"PH1:TIM20_CH2"      	:	2,
	"PH1:FMC_A1"      	:	12,

	"PH2:EVENTOUT"      	:	1,
	
}
    
ADC1_map = {
	# format is PIN : ADC1_CHAN
	# extracted from tabula-addfunc-F303.csv
    "PA0"	:	1,
    "PA1"	:	2,
    "PA2"	:	3,
    "PA3"	:	4,
    "PF4"	:	5,
    "PC0"	:	6,
    "PC1"	:	7,
    "PC2"	:	8,
    "PC3"	:	9,
    "PF2"	:	10,
}
DMA_Map = {
	# format is (DMA_TABLE, StreamNum, Channel)
    # extracted from tabula-STM32F303-DMA.csv
    "ADC1"      :   [(1,1,0)],
    "SPI1_RX"   :   [(1,2,0)],
    "SPI1_TX"   :   [(1,3,0)],
    "SPI2_RX"   :   [(1,4,0)],
    "SPI2_TX"   :   [(1,5,0)],
    "USART3_TX" :   [(1,2,0)],
    "USART3_RX" :   [(1,3,0)],
    "USART1_TX" :   [(1,4,0)],
    "USART1_RX" :   [(1,5,0)],
    "USART2_RX" :   [(1,6,0)],
    "USART2_TX" :   [(1,7,0)],
    "I2C2_TX"   :   [(1,4,0)],
    "I2C2_RX"   :   [(1,5,0)],
    "I2C1_TX"   :   [(1,6,0)],
    "I2C1_RX"   :   [(1,7,0)],
    "TIM1_CH1"  :   [(1,2,0)],
    "TIM1_CH2"  :   [(1,3,0)],
    "TIM1_CH4"  :   [(1,4,0)],
    "TIM1_UP"   :   [(1,5,0)],
    "TIM1_CH3"  :   [(1,6,0)],
    "TIM2_CH3"  :   [(1,1,0)],
    "TIM2_UP"   :   [(1,2,0)],
    "TIM2_CH1"  :   [(1,5,0)],
    "TIM2_CH2"  :   [(1,7,0)],
    "TIM2_CH4"  :   [(1,7,0)],
    "TIM3_CH3"  :   [(1,2,0)],
    "TIM3_CH4"  :   [(1,3,0)],
    "TIM3_UP"   :   [(1,3,0)],
    "TIM3_CH1"  :   [(1,6,0)],
    "TIM15_CH1" :   [(1,5,0)],
    "TIM15_UP"  :   [(1,5,0)],
    "TIM16_CH1" :   [(1,3,0)],
    "TIM16_UP"  :   [(1,3,0)],
    "TIM17_CH1" :   [(1,1,0)],
    "TIM17_UP"  :   [(1,1,0)],
}
