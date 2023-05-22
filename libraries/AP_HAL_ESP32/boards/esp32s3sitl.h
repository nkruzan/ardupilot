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

// make sensor selection clearer
#define PROBE_IMU_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,GET_I2C_DEVICE(bus, addr),##args))
#define PROBE_IMU_SPI(driver, devname, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname),##args))
#define PROBE_IMU_SPI2(driver, devname1, devname2, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname1),hal.spi->get_device(devname2),##args))

#define PROBE_BARO_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(GET_I2C_DEVICE(bus, addr)),##args))
#define PROBE_BARO_SPI(driver, devname, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(hal.spi->get_device(devname)),##args))

#define PROBE_MAG_I2C(driver, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(GET_I2C_DEVICE(bus, addr),##args))
#define PROBE_MAG_SPI(driver, devname, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(hal.spi->get_device(devname),##args))
#define PROBE_MAG_IMU(driver, imudev, imu_instance, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(imu_instance,##args))
#define PROBE_MAG_IMU_I2C(driver, imudev, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(GET_I2C_DEVICE(bus,addr),##args))
//------------------------------------



//SERIAL DEFAULTS
//#define DEFAULT_SERIAL0_PROTOCOL				SerialProtocol_MAVLink2			//A	UART0: Always: Console, MAVLink2
//#define DEFAULT_SERIAL0_BAUD				AP_SERIALMANAGER_CONSOLE_BAUD/1000	//115200

//#define DEFAULT_SERIAL1_PROTOCOL				SerialProtocol_MAVLink2			//C	WiFi:  TCP, UDP, or disable (depends on HAL_ESP32_WIFI)
//#define DEFAULT_SERIAL1_BAUD				AP_SERIALMANAGER_MAVLINK_BAUD/1000	//57600

//Inertial sensors
#define HAL_INS_DEFAULT				HAL_INS_MPU9250_SPI
#define AP_INERTIALSENSOR_ENABLED 1
#define AP_SIM_INS_ENABLED 1
#define HAL_INS_MPU9250_NAME "mpu9250"
#define HAL_INS_PROBE_LIST PROBE_IMU_SPI( Invensense, HAL_INS_MPU9250_NAME, ROTATION_NONE)

//-----COMPASS-----
#define AP_COMPASS_BACKEND_DEFAULT_ENABLED 1
#define HAL_COMPASS_DEFAULT HAL_COMPASS_AK8963_MPU9250
#define HAL_MAG_PROBE_LIST PROBE_MAG_IMU(AK8963, mpu9250, 0, ROTATION_NONE)
//#define HAL_PROBE_EXTERNAL_I2C_COMPASSES FALSE
#define ALLOW_ARM_NO_COMPASS				1
#define AP_COMPASS_AK8963_ENABLED TRUE

//-----BARO-----
//#define HAL_BARO_PROBE_LIST PROBE_BARO_I2C(BMP280, 0, 0x77)
#define HAL_BARO_ALLOW_INIT_NO_BARO 1
#define AP_SIM_BARO_ENABLED TRUE


//#define HAL_COMPASS_MAX_SENSORS 3

// SPI BUS setup, including gpio, dma, etc
// note... we use 'vspi' for the bmp280 and mpu9250
#define HAL_ESP32_SPI_BUSES \
    {.host=SPI3_HOST, .dma_ch=SPI_DMA_CH_AUTO, .mosi=GPIO_NUM_36, .miso=GPIO_NUM_37, .sclk=GPIO_NUM_35}
// tip:  VSPI_HOST  is an alternative name for esp's SPI3
//#define HAL_ESP32_SPI_BUSES {}

// SPI per-device setup, including speeds, etc.
#define HAL_ESP32_SPI_DEVICES \
    {.name="mpu9250", .bus=0, .device=0, .cs=GPIO_NUM_34,  .mode = 0, .lspeed=2*MHZ, .hspeed=8*MHZ}
//#define HAL_ESP32_SPI_DEVICES {}

//I2C bus list
#define HAL_ESP32_I2C_BUSES \
	{.port=I2C_NUM_0, .sda=GPIO_NUM_16, .scl=GPIO_NUM_15, .speed=400*KHZ, .internal=true}
//#define HAL_ESP32_I2C_BUSES {} // using this embty block appears to cause crashes?


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

//HARDWARE UARTS
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