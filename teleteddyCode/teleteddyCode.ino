#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// for debugging
#define DEBUG           1

// touch sensor threshold (how sensitive the touch is)
#define TOUCH_SENSITIVITY 512

// neopixel definitions
#define PIN             6
#define NUMPIXELS       1

// pin defintions for fsr
#define FSR_LEFT_HAND   A0
#define FSR_RIGHT_HAND  A1
#define FSR_LEFT_FOOT   A2
#define FSR_RIGHT_FOOT  A3
#define FSR_LEFT_EAR    A4
#define FSR_RIGHT_EAR   A5
//#define FSR_TAG

// bit map for fsr readings
#define MAP_LEFT_HAND   0x80
#define MAP_RIGHT_HAND  0x40
#define MAP_LEFT_FOOT   0x20
#define MAP_RIGHT_FOOT  0x10
#define MAP_LEFT_EAR    0x80
#define MAP_RIGHT_EAR   0x40
//#define MAP_TAG       0x20

// define what LEDs are where
#define HEART_LED       1
#define LEFT_HAND_LED   5
#define LEFT_FOOT_LED   6
#define RIGHT_FOOT_LED  7
#define RIGHT_HAND_LED  8
#define RIGHT_EAR_LED   9
#define LEFT_EAR_LED    10

// acceleromter interrupt pin (set to 1 if the bear is picked up)
const byte accIntPin = 13;


// track if in two player mode
bool twoPlayerMode  = false;

// keep track of which sensor is registering a touch
byte my_touch_map   = 0;
byte your_touch_map = 0;

// tracking variable to determine if bear is picked up
bool picked_up      = false;

// XBee setup
SoftwareSerial XBee(2, 3); // RX, TX


Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Adafruit_MMA8451 mma = Adafruit_MMA8451();

/******************************************************************/
/************************ SETUP ***********************************/
/******************************************************************/
  
void setup() {
  // setup serial port
  Serial.begin(9600);

  // XBee setup
  XBee.begin(9600);

  // start neo pixels
  pixels.begin(); // This initializes the NeoPixel library.

  // accelerometer interupt setup
  pinMode(accIntPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(accIntPin), bearPickedUp, HIGH);

  // setup accelerometer
  mma.setRange(MMA8451_RANGE_2_G);

}

/******************************************************************/
/************************ LOOP ************************************/
/******************************************************************/
void loop() {
  // check all touch sensors and update touch sensor variable
  updateTouchMap();
  // send info on what is being touched on this bear
  sendTouchMap();

  if(isBearPickedUp()){
    #ifdef DEBUG 
      Serial.println("Bear is picked up");
    #endif
  }
}

/******************************************************************/
/************************ FUNCTIONS *******************************/
/******************************************************************/

void updateTouchMap(void) {
  if(analogRead(FSR_LEFT_HAND) > TOUCH_SENSITIVITY) {
    my_touch_map |= MAP_LEFT_HAND;
    
    #ifdef DEBUG 
    Serial.println("Left hand touched");
    #endif
  }

  if(analogRead(FSR_RIGHT_HAND) > TOUCH_SENSITIVITY) {
    my_touch_map |= MAP_RIGHT_HAND;

    #ifdef DEBUG 
    Serial.println("Right hand touched");
    #endif
  }

  if(analogRead(FSR_LEFT_FOOT) > TOUCH_SENSITIVITY) {
    my_touch_map |= MAP_LEFT_FOOT;

    #ifdef DEBUG 
    Serial.println("Left foot touched");
    #endif
  }

  if(analogRead(FSR_RIGHT_FOOT) > TOUCH_SENSITIVITY) {
    my_touch_map |= MAP_RIGHT_FOOT;

    #ifdef DEBUG 
    Serial.println("Right foot touched");
    #endif
  }

  if(analogRead(FSR_LEFT_EAR) > TOUCH_SENSITIVITY) {
    my_touch_map |= MAP_LEFT_EAR;

    #ifdef DEBUG 
    Serial.println("Left ear touched");
    #endif
  }

  if(analogRead(FSR_RIGHT_EAR) > TOUCH_SENSITIVITY) {
    my_touch_map |= MAP_RIGHT_EAR;

    #ifdef DEBUG 
    Serial.println("Right ear touched");
    #endif
  }

#ifdef FSR_TAG
  if(digitalRead(FSR_TAG) == HIGH) {
    my_touch_map |= MAP_TAG;

    #ifdef DEBUG 
    Serial.println("Tag touched");
    #endif
  }
#endif
}

void turnOnTouched(void){
  if( (my_touch_map & MAP_LEFT_HAND) == MAP_LEFT_HAND){
    turnOn(LEFT_HAND_LED);
  }
  if( (my_touch_map & MAP_LEFT_HAND) == MAP_LEFT_HAND){
    turnOn(LEFT_FOOT_LED);
  }
  if( (my_touch_map & MAP_RIGHT_FOOT) == MAP_RIGHT_FOOT){
    turnOn(RIGHT_FOOT_LED);
  }
  if( (my_touch_map & MAP_RIGHT_HAND) == MAP_RIGHT_HAND){
    turnOn(RIGHT_HAND_LED);
  }
  if( (my_touch_map & MAP_RIGHT_EAR) == MAP_RIGHT_EAR){
    turnOn(RIGHT_EAR_LED);
  }
  if( (my_touch_map & MAP_LEFT_EAR) == MAP_LEFT_EAR){
    turnOn(LEFT_EAR_LED);
  }
}

void turnOn(uint16_t location) {
  if(location == HEART_LED){
    pixels.setPixelColor(1, 255, 0, 0);
    pixels.setPixelColor(2, 255, 0, 0);
    pixels.setPixelColor(3, 255, 0, 0);
    pixels.setPixelColor(4, 255, 0, 0);
  }
  else{
    pixels.setPixelColor(location, 255, 0, 0);
  } 
}

void turnOff(uint16_t location) {
  if(location == HEART_LED){
    pixels.setPixelColor(1, 0, 0, 0);
    pixels.setPixelColor(2, 0, 0, 0);
    pixels.setPixelColor(3, 0, 0, 0);
    pixels.setPixelColor(4, 0, 0, 0);
  }
  else{
    pixels.setPixelColor(location, 0, 0, 0);
  } 
}

void setLight(uint16_t location, uint16_t r, uint16_t g, uint16_t b){
  if(location == HEART_LED){
    pixels.setPixelColor(1, r, g, b);
    pixels.setPixelColor(2, r, g, b);
    pixels.setPixelColor(3, r, g, b);
    pixels.setPixelColor(4, r, g, b);
  }
  else{
    pixels.setPixelColor(location, r, g, b);
  } 
}

void heartbeatTouchableLights(){
  for(uint16_t t = 0; t <= 255; t+5){
    setLight(HEART_LED,       t, 0, 0);
    setLight(LEFT_HAND_LED,   t, 0, 0);
    setLight(LEFT_FOOT_LED,   t, 0, 0);
    setLight(RIGHT_FOOT_LED,  t, 0, 0);
    setLight(RIGHT_HAND_LED,  t, 0, 0);
    setLight(RIGHT_EAR_LED,   t, 0, 0);
    setLight(LEFT_EAR_LED,    t, 0, 0);
    delay(5);
  }
  for(uint16_t t = 255; t >= 0; t-5){
    setLight(HEART_LED,       t, 0, 0);
    setLight(LEFT_HAND_LED,   t, 0, 0);
    setLight(LEFT_FOOT_LED,   t, 0, 0);
    setLight(RIGHT_FOOT_LED,  t, 0, 0);
    setLight(RIGHT_HAND_LED,  t, 0, 0);
    setLight(RIGHT_EAR_LED,   t, 0, 0);
    setLight(LEFT_EAR_LED,    t, 0, 0);
    delay(5);
  }
}

void playInitTune(void) {

}

void sendTouchMap(void) {
  XBee.print(my_touch_map);
}

bool receivedTouch(void){
  if (XBee.available())
  { // If data comes in from XBee, send it out to serial monitor
    your_touch_map = XBee.read();
    return true;
  }
  else{
    return false;
  }
}

void bearPickedUp(){
  // disable interrupt to avoid constantly activating
  detachInterrupt(digitalPinToInterrupt(accIntPin));
  picked_up = true;
}

bool isBearPickedUp(){
  if(picked_up){
    // reenable interrupt
    attachInterrupt(digitalPinToInterrupt(accIntPin), bearPickedUp, CHANGE);
    picked_up = false;
    return true;
  }
  else return false;
}
