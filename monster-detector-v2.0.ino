/*********************************************************************
  This is an example for our nRF51822 based Bluefruit LE modules

  Pick one up today in the adafruit shop!

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  MIT license, check LICENSE for more information
  All text above, and the splash screen below must be included in
  any redistribution
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#include "Adafruit_NeoPixel.h"

/*=========================================================================
    APPLICATION SETTINGS

      FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
     
                                Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                                running this at least once is a good idea.
     
                                When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                                Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
  -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE     1

/* SCAN BUTTON */
#define SCAN_BUTTON_PIN         6

/* BUZZER TEST BUTTON */
#define BUZZER_BUTTON_PIN       9

/* BUZZER PIN */
#define BUZZER_PIN              11

/* NeoPixel RING */
#define RING_PIN                12
#define RING_NUM_LEDS           16
Adafruit_NeoPixel ringStrip = Adafruit_NeoPixel(RING_NUM_LEDS, RING_PIN, NEO_RGBW + NEO_KHZ800);

/* NeoPixel CORE */
#define CORE_PIN                13
#define CORE_NUM_LEDS           7
Adafruit_NeoPixel coreStrip = Adafruit_NeoPixel(CORE_NUM_LEDS, CORE_PIN, NEO_RGB + NEO_KHZ800);

/*=========================================================================*/

/* Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);
Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN, BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
/* Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);
/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err)
{
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

void setup(void)
{
  /* Set up scan button */
  pinMode(SCAN_BUTTON_PIN, INPUT);
  digitalWrite(SCAN_BUTTON_PIN, HIGH);

  /* Set up buzzer test button */
  pinMode(BUZZER_BUTTON_PIN, INPUT);
  digitalWrite(BUZZER_BUTTON_PIN, HIGH);
  
  /* Setup buzzer */
  pinMode(BUZZER_PIN, OUTPUT);

  /* Initialize NeoPixel ring */
  ringStrip.begin();
  ringStrip.setBrightness(5);
  for (int x = 0; x < RING_NUM_LEDS; x++)
  {
    ringStrip.setPixelColor(x, ringStrip.Color(0, 0, 0, 0));
    ringStrip.show();
  }

  /* Initialize NeoPixel core */
  coreStrip.begin();
  coreStrip.setBrightness(5);
  for (int x = 0; x < CORE_NUM_LEDS; x++)
  {
    coreStrip.setPixelColor(x, coreStrip.Color(0, 0, 0));
    coreStrip.show();
  }

  /* Setup BLE stuff */
  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Neopixel Color Picker Example"));
  Serial.println(F("------------------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection
  // while (! ble.isConnected()) {
  //   delay(500);
  // }
  */

  Serial.println(F("***********************"));

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("***********************"));
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  doBLEProcessing();
  doPhysicalProcessing();
}

// ################################################################
// ################################################################
// ###
// ### BEGIN MISC FUNCTION DEFINITIONS...
// ###
// ################################################################
// ################################################################
void statusMonitor()
{
  
}

void doAlarm()
{
  for (int x = 100; x < 2000; x++)
  {
    tone(BUZZER_PIN, x);
  }
  noTone(BUZZER_PIN);

  for (int x = 2000; x > 100; x--)
  {
    tone(BUZZER_PIN, x);
  }
  noTone(BUZZER_PIN);
}

void doBLEProcessing()
{
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);

  if (packetbuffer[1] == 'B')
  {

    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    
    switch (buttnum)
    {
      case 1: break;
      case 2: break;
      case 3: break;
      case 4: break;
      case 5: break;
    }
    if (pressed)
    {
      doAlarm();
      Serial.println(" pressed");
    }
    else
    {
      Serial.println(" released");
    }
  }
}

void doPhysicalProcessing()
{
  int scanButton = 0;
  int buzzTestButton = 0;
  
  scanButton = digitalRead(SCAN_BUTTON_PIN);
  buzzTestButton = digitalRead(BUZZER_BUTTON_PIN);

  if (buzzTestButton == LOW)
  {
    doAlarm();
  }
}


