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

#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_ESP32_S3PERIPH
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

//- these are missing from esp-idf......will not be needed later
#define RTC_WDT_STG_SEL_OFF             0
#define RTC_WDT_STG_SEL_INT             1
#define RTC_WDT_STG_SEL_RESET_CPU       2
#define RTC_WDT_STG_SEL_RESET_SYSTEM    3
#define RTC_WDT_STG_SEL_RESET_RTC       4

#define HAL_ESP32_BOARD_NAME "esp32s3periph"

#define HAL_ESP32_RMT_RX_PIN_NUMBER GPIO_NUM_14

#define AP_INERTIALSENSOR_ENABLED 0

//INS choices:
#define HAL_INS_DEFAULT HAL_INS_NONE

// BARO choices:
#define HAL_BARO_DEFAULT HAL_BARO_BMP280_I2C
#define HAL_BARO_BMP280_NAME "BMP280"
// MAG/COMPASS choices:
//#define HAL_COMPASS_DEFAULT HAL_COMPASS_AK8963_MPU9250
// allow boot without a baro
#define HAL_BARO_ALLOW_INIT_NO_BARO 1
// ADC is available on lots of pints on the esp32, but adc2 cant co-exist with wifi we choose to allow ADC on :
#define HAL_DISABLE_ADC_DRIVER 1
#define HAL_USE_ADC 0

// the pin number, the gain/multiplier associated with it, the ardupilot name for the pin in parameter/s.
#define HAL_ESP32_ADC_PINS {\
	{ADC1_CHANNEL_4, 11, 1},\
	{ADC1_CHANNEL_3, 11, 2},\
	{ADC1_CHANNEL_1, 11, 3},\
	{ADC1_CHANNEL_0, 11, 4}\
}

#define HAL_INS_MPU9250_NAME "mpu9250"

#define HAL_INS_PROBE_LIST PROBE_IMU_SPI( Invensense, HAL_INS_MPU9250_NAME, ROTATION_NONE)
#define AP_COMPASS_AK8963_ENABLED TRUE

//#define HAL_MAG_PROBE_LIST ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(0, ROTATION_NONE))
#define HAL_PROBE_EXTERNAL_I2C_COMPASSES 1

#define HAL_ESP32_WIFI 1

#define WIFI_SSID "ardupilot123"
#define WIFI_PWD "ardupilot123"

// see boards.py
#ifndef ENABLE_HEAP
#define ENABLE_HEAP 1
#endif

//RCOUT which pins are used?
#define HAL_ESP32_RCOUT { GPIO_NUM_11,GPIO_NUM_10, GPIO_NUM_9, GPIO_NUM_8, GPIO_NUM_7, GPIO_NUM_6 }

// SPI BUS setup, including gpio, dma, etc
#define HAL_ESP32_SPI_BUSES \
    {.host=SPI3_HOST, .dma_ch=SPI_DMA_CH_AUTO, .mosi=GPIO_NUM_36, .miso=GPIO_NUM_37, .sclk=GPIO_NUM_35}
// SPI per-device setup, including speeds, etc.
//#define HAL_ESP32_SPI_DEVICES 
//    {.name="mpu9250", .bus=0, .device=0, .cs=GPIO_NUM_34,  .mode = 0, .lspeed=2*MHZ, .hspeed=8*MHZ}
#define HAL_ESP32_SPI_DEVICES {}

//I2C bus list
#define HAL_ESP32_I2C_BUSES \
	{.port=I2C_NUM_0, .sda=GPIO_NUM_16, .scl=GPIO_NUM_15, .speed=400*KHZ, .internal=true}
//#define HAL_ESP32_I2C_BUSES {} // using this embty block appears to cause crashes?

// rcin on what pin?
#define HAL_ESP32_RCIN GPIO_NUM_14

//HARDWARE UARTS
#define HAL_ESP32_UART_DEVICES \
  {.port=UART_NUM_0, .rx=GPIO_NUM_44, .tx=GPIO_NUM_43 },{.port=UART_NUM_1, .rx=GPIO_NUM_17, .tx=GPIO_NUM_18 }

#define HAVE_FILESYSTEM_SUPPORT 1
#define HAL_ESP32_SDMMC 1
#define HAL_ESP32_SDCARD 1
#define LOGGER_MAVLINK_SUPPORT 1
#define HAL_BOARD_LOG_DIRECTORY "/SDCARD/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "/SDCARD/APM/TERRAIN"
#define HAL_BOARD_STORAGE_DIRECTORY "/SDCARD/APM/STORAGE"
#define HAL_OS_POSIX_IO 1
#define HAL_LOGGING_BACKENDS_DEFAULT 1
//================================================================
//================================================================
#define SCRIPTING_DIRECTORY "/SDCARD/APM/SCRIPTS"
#define AP_SCRIPTING_ENABLED 0
#define AP_CAN_SLCAN_ENABLED 0
#define HAL_PERIPH_ENABLE_BATTERY
#define HAL_PERIPH_ENABLE_GPS
#define HAL_PERIPH_ENABLE_MAG
#define HAL_PERIPH_ENABLE_BARO
#define HAL_PERIPH_ENABLE_RC_OUT
#define HAL_PERIPH_ENABLE_NOTIFY
//#define AP_INERTIALSENSOR_ENABLED 0
#define AP_KDECAN_ENABLED 0
#define GPS_MAX_RECEIVERS 1
#define GPS_MAX_INSTANCES 1
#define HAL_COMPASS_MAX_SENSORS 1
#define HAL_NO_MONITOR_THREAD
#define HAL_DISABLE_LOOP_DELAY
#define HAL_USE_RTC FALSE
#define DISABLE_SERIAL_ESC_COMM 1
#define HAL_NO_RCIN_THREAD
//#define HAL_BARO_ALLOW_INIT_NO_BARO
#define CAN_APP_NODE_NAME "org.ardupilot." HAL_ESP32_BOARD_NAME
#define HAL_CAN_DEFAULT_NODE_ID 0
#define HAL_NUM_CAN_IFACES 1

#define HAL_SCHEDULER_ENABLED 0
#define HAL_LOGGING_ENABLED 0
#define HAL_GCS_ENABLED 0
#define HAL_NO_ROMFS_SUPPORT 1

#define HAL_SERIAL1_PROTOCOL -1
#define HAL_SERIAL2_PROTOCOL -1
#define HAL_SERIAL3_PROTOCOL -1
#define HAL_SERIAL4_PROTOCOL -1

#define HAL_LOGGING_MAVLINK_ENABLED 0
#define HAL_MISSION_ENABLED 0
#define HAL_RALLY_ENABLED 0
#define HAL_CAN_DEFAULT_NODE_ID 0
#define PERIPH_FW 1
#define HAL_BUILD_AP_PERIPH 1
#define HAL_WATCHDOG_ENABLED_DEFAULT 1
#define AP_FETTEC_ONEWIRE_ENABLED 0
#define HAL_BARO_WIND_COMP_ENABLED 0
#define HAL_UART_STATS_ENABLED (HAL_GCS_ENABLED || HAL_LOGGING_ENABLED)

#define COMPASS_CAL_ENABLED 0
#define AP_VOLZ_ENABLED 0
#define HAL_SUPPORT_RCOUT_SERIAL 0
#define AP_AIRSPEED_AUTOCAL_ENABLE 0
#define AP_ROBOTISSERVO_ENABLED 0

#define COMPASS_LEARN_ENABLED 0
#define AP_RCPROTOCOL_ENABLED 0
#undef HAL_ESP32_RCIN