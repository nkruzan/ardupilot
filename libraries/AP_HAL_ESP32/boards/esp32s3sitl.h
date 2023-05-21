/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#define TRUE 1
#define FALSE 0

#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_ESP32_S3SITL
#define HAL_ESP32_BOARD_NAME "esp32s3sitl"

//SERIAL DEFAULTS
//#define DEFAULT_SERIAL0_PROTOCOL				SerialProtocol_MAVLink2			//A	UART0: Always: Console, MAVLink2
//#define DEFAULT_SERIAL0_BAUD				AP_SERIALMANAGER_CONSOLE_BAUD/1000	//115200

//#define DEFAULT_SERIAL1_PROTOCOL				SerialProtocol_MAVLink2			//C	WiFi:  TCP, UDP, or disable (depends on HAL_ESP32_WIFI)
//#define DEFAULT_SERIAL1_BAUD				AP_SERIALMANAGER_MAVLINK_BAUD/1000	//57600

//Inertial sensors
#define HAL_INS_DEFAULT				HAL_INS_NONE
#define AP_INERTIALSENSOR_ENABLED 1
#define AP_SIM_INS_ENABLED 1
//I2C Buses
#define HAL_ESP32_I2C_BUSES				{}

//SPI Buses
#define HAL_ESP32_SPI_BUSES				{}

//SPI Devices
#define HAL_ESP32_SPI_DEVICES			{}

//RCIN
#define HAL_ESP32_RCIN 					GPIO_NUM_14

//RCOUT
#define HAL_ESP32_RCOUT 				{ GPIO_NUM_11,GPIO_NUM_10, GPIO_NUM_9, GPIO_NUM_8, GPIO_NUM_7, GPIO_NUM_6 }

//BAROMETER
#define HAL_BARO_ALLOW_INIT_NO_BARO		1

//COMPASS
#define ALLOW_ARM_NO_COMPASS 			1

//WIFI
#define HAL_ESP32_WIFI					1	//1-TCP, 2-UDP, comment this line = without wifi
#define WIFI_SSID						"ardupilot123"
#define WIFI_PWD						"ardupilot123"

//UARTs
#define HAL_ESP32_UART_DEVICES \
  {.port=UART_NUM_0, .rx=GPIO_NUM_44, .tx=GPIO_NUM_43 },{.port=UART_NUM_1, .rx=GPIO_NUM_17, .tx=GPIO_NUM_18 }

//ADC
#define HAL_DISABLE_ADC_DRIVER 			TRUE
#define HAL_USE_ADC 					FALSE

//LED
#define DEFAULT_NTF_LED_TYPES			Notify_LED_None

//RMT pin number
#define HAL_ESP32_RMT_RX_PIN_NUMBER 	GPIO_NUM_14


#define HAL_ESP32_ADC_PINS HAL_ESP32_ADC_PINS_OPTION1

// see boards.py
#ifndef ENABLE_HEAP
#define ENABLE_HEAP 1
#endif
#define HAVE_FILESYSTEM_SUPPORT 1

#define HAL_ESP32_SDMMC 1

#define HAL_ESP32_SDCARD 1
#define LOGGER_MAVLINK_SUPPORT 1
#define HAL_BOARD_LOG_DIRECTORY "/SDCARD/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "/SDCARD/APM/TERRAIN"
#define HAL_BOARD_STORAGE_DIRECTORY "/SDCARD/APM/STORAGE"
#define HAL_OS_POSIX_IO 1

#define HAL_LOGGING_BACKENDS_DEFAULT 1

//- these are missing from esp-idf......will not be needed later
#define RTC_WDT_STG_SEL_OFF             0
#define RTC_WDT_STG_SEL_INT             1
#define RTC_WDT_STG_SEL_RESET_CPU       2
#define RTC_WDT_STG_SEL_RESET_SYSTEM    3
#define RTC_WDT_STG_SEL_RESET_RTC       4




////SIM SITL STUFF
//env SIM_ENABLED 1
#define AP_SIM_ENABLED 1

#define INS_MAX_INSTANCES 2
#define HAL_COMPASS_MAX_SENSORS 2

#define AP_GPS_BACKEND_DEFAULT_ENABLED 0
#define AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED 0

#define HAL_NAVEKF2_AVAILABLE 0
#define EK3_FEATURE_BODY_ODOM 0
#define EK3_FEATURE_EXTERNAL_NAV 0
#define EK3_FEATURE_DRAG_FUSION 0
#define HAL_ADSB_ENABLED 0
#define HAL_PROXIMITY_ENABLED 0
#define HAL_VISUALODOM_ENABLED 0
#define HAL_GENERATOR_ENABLED 0

#define MODE_SPORT_ENABLED 0
#define MODE_THROW_ENABLED 0
#define MODE_TURTLE_ENABLED 0
#define MODE_FLOWHOLD 0
#define MODE_POSHOLD_ENABLED 0
#define MODE_SYSTEMID_ENABLED 0
#define MODE_ACRO_ENABLED 0
#define MODE_FOLLOW_ENABLED 0
#define MODE_FLIP_ENABLED 0
#define MODE_DRIFT_ENABLED 0
#define MODE_THROW_ENABLED 0


#define HAL_MSP_OPTICALFLOW_ENABLED 0
#define HAL_SUPPORT_RCOUT_SERIAL 0
#define HAL_HOTT_TELEM_ENABLED 0
#define HAL_HIGH_LATENCY2 0

#define AP_SIM_INS_FILE_ENABLED 0