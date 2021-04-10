
/**TODO: Add menu for setting individual rgb values (sub menu)
         make switch between rgb value and static white (maybe a toggle menu)
         Add arduino sleep to when menu goes to idle
         Add saving of variables to eeprom?

**/

#include <Arduino.h>
#include <FastLED.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// Libraries for the menu system
#include <menu.h>
#include <menuIO/SSD1306AsciiOut.h>
#include <menuIO/serialIO.h>
#include <menuIO/chainStream.h>
#include <ClickEncoder.h>
#include <menuIO/clickEncoderIn.h>

//--- General Settings
//Status indicator


// Fast led settings
const uint16_t 
	Num_Leds   =  107;         // strip length
const uint8_t
	Brightness =  255;        // maximum brightness

// --- FastLED Setings
#define LED_TYPE     WS2812B  // led strip type for FastLED
#define COLOR_ORDER  GRB      // color order for bitbang
#define PIN_DATA     5        // led data output pin

CRGB leds[Num_Leds];

// LED Settings
uint8_t currBrightness = 10;
uint8_t oldBrightness = currBrightness;

uint8_t currColorTempInKilo = 45;
uint8_t oldColorTempInKilo = currColorTempInKilo;


// Color correction settings
#define R_CORRECTION_F 1
#define G_CORRECTION_F 0.69f
#define B_CORRECTION_F 0.39f

// Power Button Pin
#define POWER_BUTTON 2
#define POWER_BUTTON_LED 4
volatile boolean powerBtnPressed = false;
boolean powerOn = false;
uint32_t powerBtnPressedPrevTime = 0u;
void powerButtonIRQ();


// Setup a RoraryEncoder for pins A2 and A3:
#define ENCODER_PIN_ONE 9
#define ENCODER_PIN_TWO 8
#define ENCODER_BUTTON_PIN 7

ClickEncoder clickEncoder(ENCODER_PIN_ONE,ENCODER_PIN_TWO,ENCODER_BUTTON_PIN,2);
ClickEncoderStream encStream(clickEncoder,1);

//-------------------------------------------------------------------------------------------
using namespace Menu;
// Menu system settings and defines
#define menuFont X11fixed7x14
#define fontW 7
#define fontH 15

SSD1306AsciiWire oled;
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_TYPE Adafruit128x32
#define MENU_SLEEP_TIMEOUT_SEC 30

// Define functions ahead of time to be referenced
bool ledOn = false;

void turnOnLeds();
void turnOffLeds();
inline uint32_t kilKelvinToKelvin(uint8_t kiloKelvin);
CRGB getRGBFromTemperature(uint32_t kelvin);
void setColorAll(CRGB color);
CRGB getColorCorrection(float redCoef, float greenCoef, float blueCoef);

//-----------------------------------------------------------------------------------------------
// Defining all of our menus
//using namespace Menu;

result doAlert(eventMask e, prompt &item);

result showEvent(eventMask e, navNode& nav, prompt& item) {
  Serial.print("event: ");
  Serial.println(e);
  return proceed;
}

MENU(mainMenu,"LED main menu",doNothing,noEvent,wrapStyle
  ,FIELD(currBrightness,"Brightness","%",0,100,5,5,doNothing,enterEvent,wrapStyle)
  ,FIELD(currColorTempInKilo,"Color Temp","00K",5,100,2,2,doNothing,noEvent,noStyle)
  ,EXIT("Sleep")
);

#define MAX_DEPTH 2

// Define our output for the menu (the oled)
const panel panels[] MEMMODE = {{0, 0, (SCREEN_WIDTH / fontW), (SCREEN_HEIGHT / fontH)}};
navNode* nodes[sizeof(panels) / sizeof(panel)]; //navNodes to store navigation status
panelsList pList(panels, nodes, 1); //a list of panels and nodes
idx_t tops[MAX_DEPTH] = {0, 0}; //store cursor positions for each level
SSD1306AsciiOut outOLED(&oled, tops, pList, 5, 1+((fontH-1)>>3) ); //oled output device menu driver


// //Define all our outputs
// idx_t serialTops[MAX_DEPTH] = {0};
// serialOut outSerial(Serial, serialTops);

// menuOut* constMEM outputs[] MEMMODE = {&outOLED,&outSerial}; //list of output devices
// outputsList out(outputs, 2); //outputs list

// serialIn inSerial(Serial);//changed on version 4
// MENU_INPUTS(in,&encStream,&inSerial);

menuOut* constMEM outputs[] MEMMODE = {&outOLED}; //list of output devices
outputsList out(outputs, 1); //outputs list

serialIn inSerial(Serial);//changed on version 4
MENU_INPUTS(in,&encStream);

// //macro to create navigation control root object (nav) using mainMenu
NAVROOT(nav, mainMenu, MAX_DEPTH, in, out);

result alert(menuOut& o, idleEvent e) {
  if (e == idling) {
    o.setCursor(0, 0);
    o.print("alert test");
    o.setCursor(0, 1);
    o.print("press [select]");
    o.setCursor(0, 2);
    o.print("to continue...");
  }
  return proceed;
}

result doAlert(eventMask e, prompt &item) {
  nav.idleOn(alert);
  return proceed;
}

void setup()
{
  //Serial.begin(115200);

  // Set up timer one to be check the encoder state every ms
  cli();//stop interrupts
  //set timer1 interrupt at 1kHz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set timer count for 1khz increments
  OCR1A = 1999;// = (16*10^6) / (1000*8) - 1
  //had to use 16 bit timer1 for this bc 1999>255, but could switch to timers 0 or 2 with larger prescaler
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for 8 prescaler
  TCCR1B |= (1 << CS11);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();//allow interrupts

  //   // Set up power button interrupt
  pinMode(POWER_BUTTON,INPUT);
  attachInterrupt(digitalPinToInterrupt(POWER_BUTTON),powerButtonIRQ,FALLING);

  // Set up encoder button
  pinMode(ENCODER_BUTTON_PIN,INPUT_PULLUP);

  // Set up led status indicator
  pinMode(POWER_BUTTON_LED,OUTPUT);
  digitalWrite(POWER_BUTTON_LED,LOW);

  // Calculate starting color temp rgb
  CRGB colorTempCRGB = CRGB(getRGBFromTemperature(kilKelvinToKelvin(currColorTempInKilo)));

  // Setup leds
  FastLED.addLeds<LED_TYPE, PIN_DATA, COLOR_ORDER>(leds, Num_Leds);
  FastLED.setCorrection(getColorCorrection(R_CORRECTION_F,G_CORRECTION_F,B_CORRECTION_F));
  FastLED.setTemperature(colorTempCRGB);
  setColorAll(CRGB::White);

  turnOffLeds();

  // Set up menu system
  Wire.begin();
  oled.begin(&OLED_TYPE, SCREEN_ADDRESS);
  oled.setFont(menuFont);
  oled.clear();

  oled.setCursor(0, 0);
  oled.print("Ambient LED");
  oled.setCursor(0, 2);
  oled.print("Controller");
  delay(1000);
  oled.clear();

  // Set settings about the menu
  //navRoot::timeOut = MENU_SLEEP_TIMEOUT_SEC
  nav.showTitle = false;
  nav.timeOut = MENU_SLEEP_TIMEOUT_SEC;
}

ISR(TIMER1_COMPA_vect) {
  clickEncoder.service();
}

void loop()
{
  nav.poll();
  
  // Turn on or off LEDs based on their previous state
  if(powerBtnPressed && (millis() - powerBtnPressedPrevTime) > 200){
    if(powerOn) {
      // Currently on so turn off
      turnOffLeds();
      ledOn = false;
    } else {
      // Currently off so turn on
      turnOnLeds();

      // Also check if the menu is idling to wake up the oled
      if(nav.sleepTask) {
        nav.idleOff();
      }

      ledOn = true;
    }
    delay(10);
    powerOn = !powerOn;
    digitalWrite(POWER_BUTTON_LED,powerOn);
    powerBtnPressed = false;
    powerBtnPressedPrevTime = millis();
  }

  // Update the brightness of the LEDs in a semi-real time manner
  if(ledOn && (currBrightness != oldBrightness)) {
    // Update the brightness variable
    oldBrightness = currBrightness;

    // set the brightness to the leds
    // Turning them on again resets the brightness
    turnOnLeds();
  }

  if(ledOn && (currColorTempInKilo != oldColorTempInKilo)) {
    oldColorTempInKilo = currColorTempInKilo;

    CRGB colorTempCRGB = CRGB(getRGBFromTemperature(kilKelvinToKelvin(currColorTempInKilo)));
    FastLED.setTemperature(colorTempCRGB);
    FastLED.show();
  }
}

void turnOnLeds() {
  // Turn on button led
  digitalWrite(LED_BUILTIN, HIGH);

  // Turn on led strip
  FastLED.setBrightness(currBrightness);
  FastLED.show();
}
void turnOffLeds() {
  // Turn off button led
  digitalWrite(LED_BUILTIN, LOW);

  // Turn off led strip
  FastLED.setBrightness(0);
  FastLED.show();
}

void powerButtonIRQ() {
  powerBtnPressed = true;
}

inline uint32_t kilKelvinToKelvin(uint8_t kiloKelvin) {
  return kiloKelvin * 100;
}

CRGB getRGBFromTemperature(uint32_t kelvin) {
  //https://tannerhelland.com/2012/09/18/convert-temperature-rgb-algorithm-code.html
  // Clamp the kelvin values
  if(kelvin < 1000) kelvin = 1000;
  else if(kelvin > 40000) kelvin = 40000;

  float kelvin_f = kelvin / 100.0f;
  float temp = 0;
  uint32_t tempAsRgb;

  // Calcuate new red value
  if(kelvin_f <= 66.0f) {
    tempAsRgb |= ((uint32_t)0xFF << 16);
  } else {
    temp = kelvin_f - 60.0f;
    temp = 329.69f * pow(temp,-0.133f);

    // If temp is greater than zero then we need to convert it to a uint
    // If less then zero then don't change anything as tempAsRGB starts as zero
    if(temp > 0.0f) {
      uint8_t rValveUnit = (uint8_t) temp;
      tempAsRgb |= ((uint32_t)(rValveUnit & 0xFF) << 16);
    }
    
  }
  //Serial.println(tempAsRgb,HEX);

  // Calculate new green
  if(kelvin_f <= 66.0f) {
    temp = kelvin_f;
    temp = 99.471f * log(temp) - 161.12f;

    if(temp > 0.0f) {
      uint8_t gValveUnit = (uint8_t) temp;
      tempAsRgb |= (((uint16_t)(gValveUnit & 0xFF)) << 8);
    }
  } else {
    temp = kelvin_f - 60.0f;
    temp = 288.122f * pow(temp,-0.0755f);

    if(temp > 0.0f) {
      uint8_t gValveUnit = (uint8_t) temp;
      tempAsRgb |= (((uint16_t)(gValveUnit & 0xFF)) << 8);
    }
  }
  //Serial.println(tempAsRgb,HEX);

  // Calculate the new blue value
  if(kelvin_f >= 66.0f) {
    tempAsRgb |= 255;
  } else if(kelvin_f <= 19.0f) {
    tempAsRgb |= 0;
  } else {
    temp = kelvin_f - 10;
    temp = 138.52f * log(temp) - 305.044f;

    if(temp > 0.0f) {
      uint8_t bValveUnit = (uint8_t) temp;
      tempAsRgb |= (uint8_t)((bValveUnit & 0xFF) << 0);
    }
  }
  //Serial.println(tempAsRgb,HEX);

  return CRGB(tempAsRgb);
}

CRGB getColorCorrection(float redCoef, float greenCoef, float blueCoef) {
  uint8_t red = uint8_t(0xFF * redCoef);
  uint8_t green = uint8_t(0xFF * greenCoef);
  uint8_t blue = uint8_t(0xFF * blueCoef);

  return CRGB(red,green,blue);
}

void setColorAll(CRGB color) {
  for(unsigned int i = 0; i < Num_Leds; i++) {
    leds[i] = color;
  }
}