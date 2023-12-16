/**************************************************************************/ /**
 * @file      WifiHandler.h
 * @brief     File to handle HTTP Download and MQTT support
 * @author    Eduardo Garcia
 * @date      2020-01-01

 ******************************************************************************/

#pragma once

#pragma once
#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * Includes
 ******************************************************************************/
#include "MQTTClient/Wrapper/mqtt.h"
#include "SerialConsole.h"
#include "asf.h"
#include "driver/include/m2m_wifi.h"
#include "iot/http/http_client.h"
#include "main.h"
#include "socket/include/socket.h"
#include "stdio_serial.h"
#include "BME680/Bme680Thread.h"
#include "AirVelocity/AirThread.h"
#include "IMU/ImuThread.h"
#include "Stepper_control/A4988_StepperMD.h"
#include "CliThread/CliThread.h"

/******************************************************************************
 * Defines
 ******************************************************************************/

#define WIFI_MQTT_INIT 0        ///< State for Wifi handler to Initialize MQTT Connection
#define WIFI_MQTT_HANDLE 1      ///< State for Wifi handler to Handle MQTT Connection
#define WIFI_DOWNLOAD_INIT 2    ///< State for Wifi handler to Initialize Download Connection
#define WIFI_DOWNLOAD_HANDLE 3  ///< State for Wifi handler to Handle Download Connection

#define WIFI_TASK_SIZE 1000     //edited working with 800 //1000 
#define WIFI_PRIORITY (configMAX_PRIORITIES - 3)

/** Wi-Fi AP Settings. */
// Note: you can register your WiFi PCBAs with AirPennNet-Device and use the credentials below to connect to the internet:
// https://device.upenn.edu/
#define MAIN_WLAN_SSID "Detkin IOT North"  /**< Destination SSID. Change to your WIFI SSID */
#define MAIN_WLAN_AUTH M2M_WIFI_SEC_WPA_PSK /**< Security manner */
#define MAIN_WLAN_PSK "n0rthC0untry"        /**< Password for Destination SSID. Change to your password. Please dont hack my WiFi router */

//#define MAIN_WLAN_SSID "Honest46211_2G"  /**< Destination SSID. Change to your WIFI SSID */
//#define MAIN_WLAN_AUTH M2M_WIFI_SEC_WPA_PSK /**< Security manner */
//#define MAIN_WLAN_PSK "slowotter21"        /**< Password for Destination SSID. Change to your password. Please dont hack my WiFi router */

//#define MAIN_WLAN_SSID "AirPennNet-Device"  /**< Destination SSID. Change to your WIFI SSID */
//#define MAIN_WLAN_AUTH M2M_WIFI_SEC_WPA_PSK /**< Security manner */
//#define MAIN_WLAN_PSK "penn1740wifi"        /**< Password for Destination SSID. Change to your password. Please dont hack my WiFi router */

/** IP address parsing. */
#define IPV4_BYTE(val, index) ((val >> (index * 8)) & 0xFF)

/** Content URI for download. */
//#define MAIN_HTTP_FILE_URL "http://ww1.microchip.com/downloads/en/DeviceDoc/70005266B.pdf" ///< Change me to the URL to download your OTAU binary file from!
#define MAIN_HTTP_FILE_URL "http://172.178.45.14/Application.bin"

/** Maximum size for packet buffer. */
#define MAIN_BUFFER_MAX_SIZE (512)
/** Maximum file name length. */
#define MAIN_MAX_FILE_NAME_LENGTH (250)
/** Maximum file extension length. */
#define MAIN_MAX_FILE_EXT_LENGTH (8)
/** Output format with '0'. */
#define MAIN_ZERO_FMT(SZ) (SZ == 4) ? "%04d" : (SZ == 3) ? "%03d" : (SZ == 2) ? "%02d" : "%d"
#define GAME_SIZE 20  ///< Number of plays in game

typedef enum {
    NOT_READY = 0,         /*!< Not ready. */
    STORAGE_READY = 0x01,  /*!< Storage is ready. */
    WIFI_CONNECTED = 0x02, /*!< Wi-Fi is connected. */
    GET_REQUESTED = 0x04,  /*!< GET request is sent. */
    DOWNLOADING = 0x08,    /*!< Running to download. */
    COMPLETED = 0x10,      /*!< Download completed. */
    CANCELED = 0x20        /*!< Download canceled. */
} download_state;

// Structure definition that holds IMU data
struct ImuDataPacket {
    int16_t xmg;
    int16_t ymg;
    int16_t zmg;
};

struct ImuDataPacket_float {
	float xmg;
	float ymg;
	float zmg;
};

#define DEBUG_PIN                   PIN_PA10
#define DEBUG_ACTIVE                false
#define DEBUG_INACTIVE              !DEBUG_ACTIVE
#define DEBUG_EIC_PIN               PIN_PA10A_EIC_EXTINT10
#define DEBUG_EIC_MUX                MUX_PA10A_EIC_EXTINT10
#define DEBUG_EIC_PINMUX            PINMUX_PA10A_EIC_EXTINT10
#define DEBUG_EIC_LINE              10

/* Debugging button */
//#define BUTTON_0_NAME             "Debug"
//#define BUTTON_0_PIN              DEBUG_PIN
//#define BUTTON_0_ACTIVE           DEBUG_ACTIVE
//#define BUTTON_0_INACTIVE         DEBUG_INACTIVE
//#define BUTTON_0_EIC_PIN          DEBUG_EIC_PIN
//#define BUTTON_0_EIC_MUX          DEBUG_EIC_MUX
//#define BUTTON_0_EIC_PINMUX       DEBUG_EIC_PINMUX
//#define BUTTON_0_EIC_LINE         DEBUG_EIC_LINE

/* Max size of UART buffer. */
#define MAIN_CHAT_BUFFER_SIZE 64

/* Max size of MQTT buffer. */
#define MAIN_MQTT_BUFFER_SIZE 512

/* Limitation of user name. */
#define MAIN_CHAT_USER_NAME_SIZE 64

#define PLAYER1 1  ///< Comment me to compile for player 2. Uncomment me to define for player 1.

//#ifdef PLAYER1
///* Chat MQTT topic. */
//#define LED_TOPIC "P1_LED_ESE516_T0"                  // Students to change to an unique identifier for each device! LED Data
//#define GAME_TOPIC_IN "P1_GAME_ESE516_T0"             // Students to change to an unique identifier for each device! Game Data
//#define GAME_TOPIC_OUT "P2_GAME_ESE516_T0"            // Students to change to an unique identifier for each device! Game Data
//#define IMU_TOPIC "P1_IMU_ESE516_T0"                  // Students to change to an unique identifier for each device! IMU Data
//#define DISTANCE_TOPIC "P1_DISTANCE_ESE516_T0"        // Students to change to an unique identifier for each device! Distance Data
//#define TEMPERATURE_TOPIC "P1_TEMPERATURE_ESE516_T0"  // Students to change to an unique identifier for each device! Distance Data
//
//#else
///* Chat MQTT topic. */
//#define LED_TOPIC "P2_LED_ESE516_T0"                  // Students to change to an unique identifier for each device! LED Data
//#define GAME_TOPIC_IN "P2_GAME_ESE516_T0"             // Students to change to an unique identifier for each device! Game Data
//#define GAME_TOPIC_OUT "P1_GAME_ESE516_T0"            // Students to change to an unique identifier for each device! Game Data
//#define IMU_TOPIC "P2_IMU_ESE516_T0"                  // Students to change to an unique identifier for each device! IMU Data
//#define DISTANCE_TOPIC "P2_DISTANCE_ESE516_T0"        // Students to change to an unique identifier for each device! Distance Data
//#define TEMPERATURE_TOPIC "P2_TEMPERATURE_ESE516_T0"  // Students to change to an unique identifier for each device! Distance Data
//
//#endif

/* Chat MQTT topic. */
#define IMU_TOPIC "IMU_Data"
#define BME_TOPIC "Environmental_Data"
#define AUTOMATE_TOPIC "Automate"
#define AIR_TOPIC "Air_Velocity_Data"

#define LED_TOPIC_LED_OFF "false"
#define LED_TOPIC_LED_ON "true"

// Cloud MQTT User
#define CLOUDMQTT_USER_ID "Wind_Cheaters"

// Cloud MQTT PASSWORD
#define CLOUDMQTT_USER_PASSWORD "Wind_Cheaters"

#define CLOUDMQTT_PORT 1883

/*
 * A MQTT broker server which was connected.
 * m2m.eclipse.org is public MQTT broker.
 */
static const char main_mqtt_broker[] = "broker.hivemq.com";

#define STRING_EOL "\r\n"
#define STRING_HEADER                                                                 \
    "-- HTTP file downloader example --" STRING_EOL "-- " BOARD_NAME " --" STRING_EOL \
    "-- Compiled: "__DATE__                                                           \
    " "__TIME__                                                                       \
    " --" STRING_EOL

/******************************************************************************
 * Structures and Enumerations
 ******************************************************************************/

/******************************************************************************
 * Global Function Declaration
 ******************************************************************************/
void vWifiTask(void *pvParameters);
void init_storage(void);
void WifiHandlerSetState(uint8_t state);
int WifiAddDistanceDataToQueue(uint16_t *distance);
int WifiAddImuDataToQueue(struct ImuDataPacket_float *imuPacket);
int WifiAddAirDataToQueue(float *air_ms);
int WifiAddBmeDataToQueue(struct bme68x_data *bmePacket);

void configure_extint_channel(void);
void configure_extint_callbacks(void);

#ifdef __cplusplus
}
#endif
