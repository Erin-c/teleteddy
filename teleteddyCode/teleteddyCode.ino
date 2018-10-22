#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// for debugging
#define DEBUG           1

// activate accelerometer
#define ACC_ENABLE      1

// activate speaker
#define SPEAKER_ENABLE    1

// define heartbeat delay interval (reduce to make it quicker)
#define HEARTBEAT_DELAY 10

// neopixel definitions
#define PIN             6
#define NUMPIXELS       10

// pin defintions for fsr
#define FSR_LEFT_HAND   A3
#define FSR_RIGHT_HAND  A0
#define FSR_LEFT_FOOT   A2
#define FSR_RIGHT_FOOT  A1
#define FSR_LEFT_EAR    4
#define FSR_RIGHT_EAR   7
//#define FSR_TAG         5

// bit map for fsr readings
#define MAP_LEFT_HAND   0x80
#define MAP_RIGHT_HAND  0x40
#define MAP_LEFT_FOOT   0x20
#define MAP_RIGHT_FOOT  0x10
#define MAP_LEFT_EAR    0x08
#define MAP_RIGHT_EAR   0x04
//#define MAP_TAG       0x02

// define what LEDs are where
#define HEART_LED       0
#define LEFT_HAND_LED   4
#define LEFT_FOOT_LED   5
#define RIGHT_FOOT_LED  6
#define RIGHT_HAND_LED  7
#define RIGHT_EAR_LED   8
#define LEFT_EAR_LED    9

#define SPEAKER_PIN     11

// acceleromter interrupt pin (set to 1 if the bear is picked up)
const byte accIntPin = 3;


// track if in two player mode
volatile bool twoPlayerMode  = false;

// keep track of which sensor is registering a touch
volatile uint16_t my_touch_map   = 0;
volatile uint16_t your_touch_map = 0;

// tracking variable to determine if bear is picked up
volatile bool picked_up      = false;

// XBee setup
//SoftwareSerial XBee(2, 3); // RX, TX


Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#ifdef ACC_ENABLE
Adafruit_MMA8451 mma = Adafruit_MMA8451();
#endif
/******************************************************************/
/************************ SETUP ***********************************/
/******************************************************************/
  
void setup() {
  // setup serial port
  Serial.begin(9600);

  //setup pins
  pinSetup();

  // set speaker pin to output
  pinMode(SPEAKER_PIN, INPUT_PULLUP);

  
//  // XBee setup
//  XBee.begin(9600);

  // start neo pixels
  pixels.begin(); // This initializes the NeoPixel library.

  // accelerometer interupt setup
#ifdef ACC_ENABLE
  pinMode(accIntPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(accIntPin), bearPickedUp, RISING);
  
  // throw error if accelerometer not working
  if (! mma.begin()) {
    Serial.println("Couldnt start");
    while (1);
  }

  // setup accelerometer
  mma.setRange(MMA8451_RANGE_2_G);
  
  // enable accelerometer interrupt
  mma.writeRegister8(MMA8451_REG_CTRL_REG1, 0x00);            // deactivate
  mma.writeRegister8(0X2A, 0x18);
  mma.writeRegister8(0x15, 0x78 );
  mma.writeRegister8(0x17, 0x30 ); //SET THRESHOLD TO 48 COUNTS
  mma.writeRegister8(0x18, 0x0A ); // 100ms debounce timing
  mma.writeRegister8(0x2C, 0x02 | 0x01);
  mma.writeRegister8(MMA8451_REG_CTRL_REG4, 0x04 | 0x20);
  mma.writeRegister8(MMA8451_REG_CTRL_REG5, 0x04 | 0x20);
  mma.writeRegister8(MMA8451_REG_CTRL_REG1, 0x01 | 0x04);     // activate (hard coded)
  
#endif

  // function to call when want to do one cycle of the heartbeat
  //heartbeatTouchableLights();
}

/******************************************************************/
/************************ LOOP ************************************/
/******************************************************************/
void loop() {
  // check all touch sensors and update touch sensor variable
  updateTouchMap();
//  // send info on what is being touched on this bear
//  sendTouchMap();
//
  if(isBearPickedUp()){

  }
  turnOnTouched();
  turnOffNotTouched();
}

/******************************************************************/
/************************ FUNCTIONS *******************************/
/******************************************************************/

void updateTouchMap(void) {
  my_touch_map = 0;
  if(digitalRead(FSR_LEFT_HAND) == LOW) {
    my_touch_map |= MAP_LEFT_HAND;
    
    #ifdef DEBUG 
    Serial.println("Left hand touched");
    #endif
  }

  if(digitalRead(FSR_RIGHT_HAND) == LOW) {
    my_touch_map |= MAP_RIGHT_HAND;

    #ifdef DEBUG 
    Serial.println("Right hand touched");
    #endif
  }

  if(digitalRead(FSR_LEFT_FOOT) == LOW) {
    my_touch_map |= MAP_LEFT_FOOT;

    #ifdef DEBUG 
    Serial.println("Left foot touched");
    #endif
  }

  if(digitalRead(FSR_RIGHT_FOOT) == LOW) {
    my_touch_map |= MAP_RIGHT_FOOT;

    #ifdef DEBUG 
    Serial.println("Right foot touched");
    #endif
  }

  if(digitalRead(FSR_LEFT_EAR) == LOW) {
    my_touch_map |= MAP_LEFT_EAR;

    #ifdef DEBUG 
    Serial.println("Left ear touched");
    #endif
  }

  if(digitalRead(FSR_RIGHT_EAR) == LOW) {
    my_touch_map |= MAP_RIGHT_EAR;

    #ifdef DEBUG 
    Serial.println("Right ear touched");
    #endif
  }

#ifdef FSR_TAG
  if(digitalRead(FSR_TAG) == LOW) {
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
  if( (my_touch_map & MAP_LEFT_FOOT) == MAP_LEFT_FOOT){
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
    pixels.setPixelColor(1, pixels.Color(255, 0, 0));
    pixels.setPixelColor(2, pixels.Color(255, 0, 0));
    pixels.setPixelColor(3, pixels.Color(255, 0, 0));
    pixels.setPixelColor(4, pixels.Color(255, 0, 0));
  }
  else{
    pixels.setPixelColor(location, pixels.Color(255, 0, 0));
  } 
  pixels.show();
}

void turnOff(uint16_t location) {
  if(location == HEART_LED){
    pixels.setPixelColor(1, pixels.Color(0, 0, 0));
    pixels.setPixelColor(2, pixels.Color(0, 0, 0));
    pixels.setPixelColor(3, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4, pixels.Color(0, 0, 0));
  }
  else{
    pixels.setPixelColor(location, pixels.Color(0, 0, 0));
  } 
  pixels.show();
}

void setLight(uint16_t location, uint16_t r, uint16_t g, uint16_t b){
  if(location == HEART_LED){
    pixels.setPixelColor(0, pixels.Color(r, g, b));
    pixels.setPixelColor(1, pixels.Color(r, g, b));
    pixels.setPixelColor(2, pixels.Color(r, g, b));
    pixels.setPixelColor(3, pixels.Color(r, g, b));
  }
  else{
    pixels.setPixelColor(location, pixels.Color(r, g, b));
  } 
  pixels.show();
}

void heartbeatTouchableLights(){
  #ifdef DEBUG 
  Serial.println("Heartbeat active");
  #endif
  for(uint16_t t = 0; t <= 255; t+=5){
    setLight(HEART_LED,   t, 0, 0);
    setLight(LEFT_HAND_LED,   t, 0, 0);
    setLight(LEFT_FOOT_LED,   t, 0, 0);
    setLight(RIGHT_FOOT_LED,  t, 0, 0);
    setLight(RIGHT_HAND_LED,  t, 0, 0);
    setLight(RIGHT_EAR_LED,   t, 0, 0);
    setLight(LEFT_EAR_LED,    t, 0, 0);
    delay(HEARTBEAT_DELAY);
    
    #ifdef DEBUG 
    Serial.println(t);
    Serial.println("going up");
    #endif
  }
  for(int16_t t = 255; t >= 0; t-=5){
    setLight(HEART_LED,   t, 0, 0);
    setLight(LEFT_HAND_LED,   t, 0, 0);
    setLight(LEFT_FOOT_LED,   t, 0, 0);
    setLight(RIGHT_FOOT_LED,  t, 0, 0);
    setLight(RIGHT_HAND_LED,  t, 0, 0);
    setLight(RIGHT_EAR_LED,   t, 0, 0);
    setLight(LEFT_EAR_LED,    t, 0, 0);
    delay(HEARTBEAT_DELAY);

    #ifdef DEBUG
    Serial.println(t);
    Serial.println("going down");
    #endif
  }
}

void playInitTune(void) {

}

//void sendTouchMap(void) {
//  XBee.print(my_touch_map);
//}
//
//bool receivedTouch(void){
//  if (XBee.available())
//  { // If data comes in from XBee, send it out to serial monitor
//    your_touch_map = XBee.read();
//    return true;
//  }
//  else{
//    return false;
//  }
//}

void bearPickedUp(){
  // disable interrupt to avoid constantly activating
  detachInterrupt(digitalPinToInterrupt(accIntPin));
  
  if(picked_up != true){
    picked_up = true;
  }
  
  #ifdef DEBUG 
  Serial.println("interrupt!");
  #endif
  
}

bool isBearPickedUp(){
  if(picked_up){
    // reenable interrupt
    picked_up = false;

    #ifdef DEBUG 
    Serial.println("Bear is picked up");
    #endif
    
    attachInterrupt(digitalPinToInterrupt(accIntPin), bearPickedUp, RISING);
    return true;
  }
  else return false;
}


void pinSetup(){
  pinMode(FSR_LEFT_HAND, INPUT);
  pinMode(FSR_RIGHT_HAND, INPUT);
  pinMode(FSR_LEFT_FOOT, INPUT);
  pinMode(FSR_RIGHT_FOOT, INPUT);
  pinMode(FSR_LEFT_EAR, INPUT);
  pinMode(FSR_RIGHT_EAR, INPUT);
#ifdef FSR_TAG
  pinMode(FSR_TAG, INPUT);
#endif
}

void turnOffNotTouched(void){
  if( (my_touch_map & MAP_LEFT_HAND) != MAP_LEFT_HAND){
    turnOff(LEFT_HAND_LED);
  }
  if( (my_touch_map & MAP_LEFT_FOOT) != MAP_LEFT_FOOT){
    turnOff(LEFT_FOOT_LED);
  }
  if( (my_touch_map & MAP_RIGHT_FOOT) != MAP_RIGHT_FOOT){
    turnOff(RIGHT_FOOT_LED);
  }
  if( (my_touch_map & MAP_RIGHT_HAND) != MAP_RIGHT_HAND){
    turnOff(RIGHT_HAND_LED);
  }
  if( (my_touch_map & MAP_RIGHT_EAR) != MAP_RIGHT_EAR){
    turnOff(RIGHT_EAR_LED);
  }
  if( (my_touch_map & MAP_LEFT_EAR) != MAP_LEFT_EAR){
    turnOff(LEFT_EAR_LED);
  }
}

uint8_t readReg(uint8_t reg){
    Wire.beginTransmission(MMA8451_DEFAULT_ADDRESS);
    #if ARDUINO >= 100
     Wire.write((uint8_t)reg);
    #else
    Wire.send(reg);
     #endif
    Wire.endTransmission(false); // MMA8451 + friends uses repeated start!!
    Wire.requestFrom(MMA8451_DEFAULT_ADDRESS, 1);
    
    if (! Wire.available()) return -1;
    return (i2cread());
}

static inline uint8_t i2cread(void) {
  #if ARDUINO >= 100
  return Wire.read();
  #else
  return Wire.receive();
  #endif
}
