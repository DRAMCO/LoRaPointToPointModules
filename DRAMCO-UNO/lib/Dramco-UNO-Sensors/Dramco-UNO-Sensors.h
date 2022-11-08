#ifndef __Dramco_UNO
#define __Dramco_UNO

#include <avr/sleep.h>
#include <avr/wdt.h>

#include <Arduino.h>

#include "DeepSleep_avr_definition.h"

#include "Wire.h"
#include "LIS2DW12.h"

#define DRAMCO_UNO_WDT_ENABLE 					  true
#define DRAMCO_UNO_WDT_TIMEOUT 					  WDTO_8S


#if HARDWARE_VERSION >= 2
#define DRAMCO_UNO_LED_NAME 					  10
#define DRAMCO_UNO_LED_PORT 					  PORTB
#define DRAMCO_UNO_LED_PIN 						  2
#else
#define DRAMCO_UNO_LED_NAME 					  4
#define DRAMCO_UNO_LED_PORT 					  PORTD
#define DRAMCO_UNO_LED_PIN 						  4
#endif

#define DRAMCO_UNO_BLINK_ON 					  100 // Time on in ms

#define DRAMCO_UNO_3V3_ENABLE_PIN 				  8

#define DRAMCO_UNO_ERROR_ACC	 				  2
#define DRAMCO_UNO_ERROR_LORA_JOIN 				  3
#define DRAMCO_UNO_ERROR_BUFFER 				  4

// LoRaWAN LMIC constants
#define DRAMCO_UNO_LMIC_NSS_PIN 				   6
#define DRAMCO_UNO_LMIC_RST_PIN 				   LMIC_UNUSED_PIN
#define DRAMCO_UNO_LMIC_DIO0_PIN 				   2
#define DRAMCO_UNO_LMIC_DIO1_PIN 				   3
#define DRAMCO_UNO_LMIC_DIO2_PIN 				   LMIC_UNUSED_PIN

#define DRAMCO_UNO_LORA_EUI_SIZE  				   8
#define DRAMCO_UNO_LORA_KEY_SIZE  				   16

#define DRAMCO_UNO_BUFFER_SIZE 					   20 // Should be enough for temp, lumin, accelerometer and soil moisture
#define DRAMCO_UNO_SERIAL_BAUDRATE			       115200
// Sensor constants
#define DRAMCO_UNO_LIGHT_SENSOR_PIN 			   A0
#define DRAMCO_UNO_TEMPERATURE_SENSOR_ENABLE_PIN   7
#define DRAMCO_UNO_TEMPERATURE_SENSOR_PIN 	       A1
#define DRAMCO_UNO_TEMPERATURE_AVERAGE		       100.0
#define DRAMCO_UNO_TEMPERATURE_CALIBRATE		   3.27

#define DRAMCO_UNO_ACCELEROMTER_INT_PIN 		   9
#define DRAMCO_UNO_ACCELEROMTER_INT_NAME 		   PINB1
#define DRAMCO_UNO_ACCELEROMTER_INT_PORT 		   PINB

#if HARDWARE_VERSION >= 2
#define DRAMCO_UNO_BUTTON_INT_PIN 				   4
#define DRAMCO_UNO_BUTTON_INT_NAME 				   PIND4
#define DRAMCO_UNO_BUTTON_INT_PORT 				   PIND
#else
#define DRAMCO_UNO_BUTTON_INT_PIN 				   10
#define DRAMCO_UNO_BUTTON_INT_NAME 				   PINB2
#define DRAMCO_UNO_BUTTON_INT_PORT 				   PINB
#endif
#define DRAMCO_UNO_BUTTON_DEBOUNCE_DELAY		   50

#if HARDWARE_VERSION >=2
#define DRAMCO_UNO_SOIL_PIN_EN 					   DRAMCO_UNO_LED_NAME
#else
#define DRAMCO_UNO_SOIL_PIN_EN 					   A3
#endif
#define DRAMCO_UNO_SOIL_PIN_ANALOG 				   A2
#define DRAMCO_UNO_SOIL_DIVIDER 				   20.0 // 2000 = max value *100 (percent)

// Low power payload constants
#define DRAMCO_UNO_LPP_DIGITAL_INPUT               0     // 1 byte
#define DRAMCO_UNO_LPP_ANALOG_INPUT                2     // 2 bytes, 0.01 signed
#define DRAMCO_UNO_LPP_GENERIC_SENSOR              100   // 4 bytes, unsigned
#define DRAMCO_UNO_LPP_LUMINOSITY                  101   // 2 bytes, 1 lux unsigned
#define DRAMCO_UNO_LPP_TEMPERATURE                 103   // 2 bytes, 0.1°C signed
#define DRAMCO_UNO_LPP_ACCELEROMETER               113   // 2 bytes per axis, 0.001G
#define DRAMCO_UNO_LPP_PERCENTAGE                  120   // 1 byte 1-100% unsigned

#define DRAMCO_UNO_LPP_DIGITAL_INPUT_SIZE          1
#define DRAMCO_UNO_LPP_ANALOG_INPUT_SIZE           2
#define DRAMCO_UNO_LPP_GENERIC_SENSOR_SIZE         4
#define DRAMCO_UNO_LPP_LUMINOSITY_SIZE             2
#define DRAMCO_UNO_LPP_TEMPERATURE_SIZE            2
#define DRAMCO_UNO_LPP_ACCELEROMETER_SIZE          6
#define DRAMCO_UNO_LPP_PERCENTAGE_SIZE             1

#define DRAMCO_UNO_LPP_DIGITAL_INPUT_MULT          1
#define DRAMCO_UNO_LPP_ANALOG_INPUT_MULT           100
#define DRAMCO_UNO_LPP_GENERIC_SENSOR_MULT         1
#define DRAMCO_UNO_LPP_LUMINOSITY_MULT             1
#define DRAMCO_UNO_LPP_TEMPERATURE_MULT            10
#define DRAMCO_UNO_LPP_ACCELEROMETER_MULT          1000
#define DRAMCO_UNO_LPP_PERCENTAGE_MULT         	   1

#define DRAMCO_UNO_INT__NONE			   		   0
#define DRAMCO_UNO_INT__ACCELEROMETER		       1
#define DRAMCO_UNO_INT__BUTTON		   	   		   1

#define DRAMCO_UNO_INT_ACTION_NONE			       0
#define DRAMCO_UNO_INT_ACTION_WAKE		   	       1
#define DRAMCO_UNO_INT_ACTION_SEND_ACC		       2

#define DRAMCO_UNO_3V3_ACTIVE				       true
#define DRAMCO_UNO_3V3_DISABLE				       false

// #define DEBUG


class DramcoUnoClass {
	public:
		void begin();
		void loop();
		
		// --- Utils ---
		void delay(uint32_t d);
		static void blink();
		unsigned long millisWithOffset();

		// --- Sensors ---
		// - Temperature
		void calibrateTemperature();
		float readTemperature();		// Gets temperature in degrees C
		
		// - Luminosity
		float readLuminosity();			// Gets luminosity in % (not calibrated in lux)

		// - Accelerometer
		float readAccelerationX();		// Gets motion in g
		float readAccelerationY();
		float readAccelerationZ();
		float readTemperatureAccelerometer();	// Gets temperature in degrees C of accelerometer

		int16_t readAccelerationXInt();		// Gets motion in g
		int16_t readAccelerationYInt();
		int16_t readAccelerationZInt();
		int16_t readTemperatureAccelerometerInt();	// Gets temperature in degrees C of accelerometer


		void interruptOnMotion();
		void interruptOnMovement();
		void interruptOnShake();
		void interruptOnFall();
		void interruptOnFreeFall();

		bool processInterrupt();

		void delayUntilShake();
		void delayUntilFall();
		void delayUntilFreeFall();
		void delayUntilMotion();
		void delayUntilMovement();

		// - Button
		void interruptOnButtonPress();
		void delayUntilButtonPress();

		// - Soil moisture
		float readSoilMoisture();
		float readSoil();

		uint8_t random();

		// --- Sleep ---
		unsigned long sleep(uint32_t d);
		unsigned long sleep(uint32_t d, bool keep3v3active, bool sleepOnce = false);
		void fastSleep (void );
		static void _isrWdt(); 
		static unsigned long _sleep(unsigned long maxWaitTimeMillis, bool sleepOnce = false);
		static unsigned long _wdtEnableForSleep(const unsigned long maxWaitTimeMillis);	
		static void _wdtEnableInterrupt();
	private:
		static void _lppAddToBuffer(float val, uint8_t channel, uint8_t type, uint8_t size, uint16_t mult);
		static void _lppAddAcceleration(uint8_t channel, float x, float y, float z);
};

void error(uint8_t errorcode);

// External names
extern DramcoUnoClass Board;

// Adjust for capitalizing mistakes
#define board (Board)
#define DramcoUno (Board)
#define Dramco_Uno (Board)
#define DramcoUNO (Board)
#define Dramco_UNO (Board)


#endif//__Dramco_UNO