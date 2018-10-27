#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <toneAC.h>

// for debugging`
//#define DEBUG           1

// activate XBEE
#define XBEE_ACTIVE     1

// activate accelerometer
//#define ACC_ENABLE      1

// activate speaker
#define SPEAKER_ENABLE  1

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

#define SPEAKER_PIN     10

// my bear's color when I press bear
#define MY_COLOR_R      255
#define MY_COLOR_G      0
#define MY_COLOR_B      0

// my bear's color when you press bear
#define YOUR_COLOR_R      0
#define YOUR_COLOR_G      0
#define YOUR_COLOR_B      255

// acceleromter interrupt pin (set to 1 if the bear is picked up)
const byte accIntPin = 3;



// track if in two player mode
bool twoPlayerMode  = false;

// keep track of which sensor is registering a touch
byte my_touch_map   = 0;
byte your_touch_map = 0;

// keep track of what is sent
byte sent_touch_map = 0;

// tracking variable to determine if bear is picked up
volatile bool picked_up = false;

// variable that gets updated with current pixel values
uint8_t u8R, u8G, u8B;

// Melodies and tones
int F2 = 87;
int G2 = 98;
int A22 = 110;
int B2 = 125;
int C3 = 131;
int D3 = 147;
int E3 = 165;
int F3 = 175;
int G3 = 196;
int A33 = 220;
int B3 = 247;
int C4 = 262;
int D4 = 294;
int E4 = 330;
int melody1[] = { F2, A22, C3, E3, F3, D4 };
int mel_length1 = 7;
int noteDurations1[] = { 4, 4, 4, 4, 4, 4 };
int melody2[] = { G3, A33, D4, F3, E3, C3, A22, F2 };
int mel_length2 = 9;
int noteDurations2[] = { 2, 2, 2, 2, 4, 4, 4, 2 };
int melody3[] = { A22, C3, E3 };
int mel_length3 = 4;
int noteDurations3[] = { 2, 2, 2 };
int melody4[] = { E3, C3, A22 };
int mel_length4 = 4;
int noteDurations4[] = { 2, 4, 2 };
int melody5[] = { A22, B2, F2 };
int mel_length5 = 4;
int noteDurations5[] = { 1, 2, 4 };
int melody6[] = { D4, C3, F2 };
int mel_length6 = 4;
int noteDurations6[] = { 4, 2, 1 };

// XBee setup
//#ifdef XBEE_ACTIVE
//SoftwareSerial XBee(0, 1); // RX, TX
//#endif

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

  // play startup sound
  playSound(melody1, mel_length1, noteDurations1);
  playSound(melody2, mel_length2, noteDurations2);
  playSound(melody3, mel_length3, noteDurations3);
  playSound(melody4, mel_length4, noteDurations4);
  playSound(melody5, mel_length5, noteDurations5);
  playSound(melody6, mel_length6, noteDurations6);
  
//  // XBee setup
//#ifdef XBEE_ACTIVE
//  XBee.begin(9600);
//#endif

  // start neo pixels
  pixels.begin(); // This initializes the NeoPixel library.

  // accelerometer interupt setup
#ifdef ACC_ENABLE
  pinMode(accIntPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(accIntPin), bearPickedUp, RISING);
  
  // throw error if accelerometer not working
  if (! mma.begin()) {
    //Serial.println("Couldnt start");
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
  heartbeatTouchableLights();
}

/******************************************************************/
/************************ LOOP ************************************/
/******************************************************************/
void loop() {
  // check all touch sensors and update touch sensor variable
  updateTouchMap();
  // send info on what is being touched on this bear
  sendTouchMap();
  
  if(receivedTouch()){
    //turn on LEDs that are touched by you
    turnOnYourTouched(your_touch_map);

    //turn off LEDs that you let go
    turnOffYourNotTouched(your_touch_map);
  }

  // has my bear been lifted
  if(isBearPickedUp()){

  }

  // turn on LEDs that have been touched on my bear
  turnOnMyTouched(my_touch_map);

  // turn off LEDs that are not touched by my bear
  turnOffMyNotTouched(my_touch_map);
}

/******************************************************************/
/************************ FUNCTIONS *******************************/
/******************************************************************/

// Sound playback function
void playSound(int melody[], int mel_length, int noteDurations[]) {
  for (int thisNote = 0; thisNote < mel_length; thisNote++) {
    int noteDuration = 1000/noteDurations[thisNote];
    toneAC(melody[thisNote], 10, noteDuration, true);
    delay(noteDuration * 4 / 3);
  }
  noToneAC();
}

void updateTouchMap(void) {
  my_touch_map = 0x00;
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


// send map to this function to turn on anything that is being touched
void turnOnMyTouched(uint16_t touch_map){
  if( (touch_map & MAP_LEFT_HAND) == MAP_LEFT_HAND){
    getCurrentPixel(LEFT_HAND_LED);
    setLight(LEFT_HAND_LED, MY_COLOR_R, u8G, u8B);
  }
  if( (touch_map & MAP_LEFT_FOOT) == MAP_LEFT_FOOT){
    getCurrentPixel(LEFT_FOOT_LED);
    setLight(LEFT_FOOT_LED, MY_COLOR_R, u8G, u8B);
  }
  if( (touch_map & MAP_RIGHT_FOOT) == MAP_RIGHT_FOOT){
    getCurrentPixel(RIGHT_FOOT_LED);
    setLight(RIGHT_FOOT_LED, MY_COLOR_R, u8G, u8B);
  }
  if( (touch_map & MAP_RIGHT_HAND) == MAP_RIGHT_HAND){
    getCurrentPixel(RIGHT_HAND_LED);
    setLight(RIGHT_HAND_LED, MY_COLOR_R, u8G, u8B);
  }
  if( (touch_map & MAP_RIGHT_EAR) == MAP_RIGHT_EAR){
    getCurrentPixel(RIGHT_EAR_LED);
    setLight(RIGHT_EAR_LED, MY_COLOR_R, u8G, u8B);
  }
  if( (touch_map & MAP_LEFT_EAR) == MAP_LEFT_EAR){
    getCurrentPixel(LEFT_EAR_LED);
    setLight(LEFT_EAR_LED, MY_COLOR_R, u8G, u8B);
  }
}

void turnOnYourTouched(byte touch_map){
  if( (touch_map & MAP_LEFT_HAND) == MAP_LEFT_HAND){
    getCurrentPixel(LEFT_HAND_LED);
    setLight(LEFT_HAND_LED, u8R, YOUR_COLOR_G, YOUR_COLOR_B);
  }
  if( (touch_map & MAP_LEFT_FOOT) == MAP_LEFT_FOOT){
    getCurrentPixel(LEFT_FOOT_LED);
    setLight(LEFT_FOOT_LED, u8R, YOUR_COLOR_G, YOUR_COLOR_B);
  }
  if( (touch_map & MAP_RIGHT_FOOT) == MAP_RIGHT_FOOT){
    getCurrentPixel(RIGHT_FOOT_LED);
    setLight(RIGHT_FOOT_LED, u8R, YOUR_COLOR_G, YOUR_COLOR_B);
  }
  if( (touch_map & MAP_RIGHT_HAND) == MAP_RIGHT_HAND){
    getCurrentPixel(RIGHT_HAND_LED);
    setLight(RIGHT_HAND_LED, u8R, YOUR_COLOR_G, YOUR_COLOR_B);
  }
  if( (touch_map & MAP_RIGHT_EAR) == MAP_RIGHT_EAR){
    getCurrentPixel(RIGHT_EAR_LED);
    setLight(RIGHT_EAR_LED, u8R, YOUR_COLOR_G, YOUR_COLOR_B);
  }
  if( (touch_map & MAP_LEFT_EAR) == MAP_LEFT_EAR){
    getCurrentPixel(LEFT_EAR_LED);
    setLight(LEFT_EAR_LED, u8R, YOUR_COLOR_G, YOUR_COLOR_B);
  }
}

// send map to this function to turn off anything that is being touched
void turnOffMyNotTouched(byte touch_map){
  if( (touch_map & MAP_LEFT_HAND) != MAP_LEFT_HAND){
    getCurrentPixel(LEFT_HAND_LED);
    setLight(LEFT_HAND_LED, 0, u8G, u8B);
  }
  if( (touch_map & MAP_LEFT_FOOT) != MAP_LEFT_FOOT){
    getCurrentPixel(LEFT_FOOT_LED);
    setLight(LEFT_FOOT_LED, 0, u8G, u8B);
  }
  if( (touch_map & MAP_RIGHT_FOOT) != MAP_RIGHT_FOOT){
    getCurrentPixel(RIGHT_FOOT_LED);
    setLight(RIGHT_FOOT_LED, 0, u8G, u8B);
  }
  if( (touch_map & MAP_RIGHT_HAND) != MAP_RIGHT_HAND){
    getCurrentPixel(RIGHT_HAND_LED);
    setLight(RIGHT_HAND_LED, 0, u8G, u8B);
  }
  if( (touch_map & MAP_RIGHT_EAR) != MAP_RIGHT_EAR){
    getCurrentPixel(RIGHT_EAR_LED);
    setLight(RIGHT_EAR_LED, 0, u8G, u8B);
  }
  if( (touch_map & MAP_LEFT_EAR) != MAP_LEFT_EAR){
    getCurrentPixel(LEFT_EAR_LED);
    setLight(LEFT_EAR_LED, 0, u8G, u8B);
  }
}

// send map to this function to turn off anything that is being touched
void turnOffYourNotTouched(byte touch_map){
  if( (touch_map & MAP_LEFT_HAND) != MAP_LEFT_HAND){
    getCurrentPixel(LEFT_HAND_LED);
    setLight(LEFT_HAND_LED, u8R, u8G, 0);
  }
  if( (touch_map & MAP_LEFT_FOOT) != MAP_LEFT_FOOT){
    getCurrentPixel(LEFT_FOOT_LED);
    setLight(LEFT_FOOT_LED, u8R, u8G, 0);
  }
  if( (touch_map & MAP_RIGHT_FOOT) != MAP_RIGHT_FOOT){
    getCurrentPixel(RIGHT_FOOT_LED);
    setLight(RIGHT_FOOT_LED, u8R, u8G, 0);
  }
  if( (touch_map & MAP_RIGHT_HAND) != MAP_RIGHT_HAND){
    getCurrentPixel(RIGHT_HAND_LED);
    setLight(RIGHT_HAND_LED, u8R, u8G, 0);
  }
  if( (touch_map & MAP_RIGHT_EAR) != MAP_RIGHT_EAR){
    getCurrentPixel(RIGHT_EAR_LED);
    setLight(RIGHT_EAR_LED, u8R, u8G, 0);
  }
  if( (touch_map & MAP_LEFT_EAR) != MAP_LEFT_EAR){
    getCurrentPixel(LEFT_EAR_LED);
    setLight(LEFT_EAR_LED, u8R, u8G, 0);
  }
}

void turnOn(uint16_t location) {
  if(location == HEART_LED){
    pixels.setPixelColor(1, pixels.Color(MY_COLOR_R, MY_COLOR_G, MY_COLOR_B));
    pixels.setPixelColor(2, pixels.Color(MY_COLOR_R, MY_COLOR_G, MY_COLOR_B));
    pixels.setPixelColor(3, pixels.Color(MY_COLOR_R, MY_COLOR_G, MY_COLOR_B));
    pixels.setPixelColor(4, pixels.Color(MY_COLOR_R, MY_COLOR_G, MY_COLOR_B));
  }
  else{
    pixels.setPixelColor(location, pixels.Color(MY_COLOR_R, MY_COLOR_G, MY_COLOR_B));
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

#ifdef XBEE_ACTIVE
void sendTouchMap(void) {
  if(sent_touch_map != my_touch_map){
    Serial.write(my_touch_map);
    sent_touch_map = my_touch_map;
  }
}

//updates everytime player lets go or touches
bool receivedTouch(void){
  if (Serial.available())
  { // If data comes in from XBee, send it out to serial monitor
    Serial.readBytes(&your_touch_map,1);
    #ifdef DEBUG 
    Serial.println("RECEIVED TRANSMISSION");
    Serial.println(your_touch_map,BIN);
    #endif
    return true;
  }
  else{
    return false;
  }
}
#endif

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

void getCurrentPixel(uint8_t led){
   long lngRGB = pixels.getPixelColor(led);
   u8R = (uint8_t)((lngRGB >> 16) & 0xff);
   u8G = (uint8_t)((lngRGB >> 8) & 0xff);
   u8B = (uint8_t)(lngRGB & 0xff);
}
