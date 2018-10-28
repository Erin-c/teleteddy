#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ArduinoSTL.h>
#include <vector>
#include <iterator>

using namespace std; 

// for debugging`
//#define DEBUG           1

// activate XBEE
#define XBEE_ACTIVE     1

// activate accelerometer
#define ACC_ENABLE      1

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
#define FSR_TAG         5

// bit map for fsr readings
#define MAP_LEFT_HAND   0x80
#define MAP_RIGHT_HAND  0x40
#define MAP_LEFT_FOOT   0x20
#define MAP_RIGHT_FOOT  0x10
#define MAP_LEFT_EAR    0x08
#define MAP_RIGHT_EAR   0x04
#define MAP_TAG         0x02
#define MAP_ACC         0x01

// define what LEDs are where
#define HEART_LED       0
#define LEFT_HAND_LED   4
#define LEFT_FOOT_LED   5
#define RIGHT_FOOT_LED  6
#define RIGHT_HAND_LED  7
#define RIGHT_EAR_LED   8
#define LEFT_EAR_LED    9

#define SPEAKER_PIN     11

// my bear's color when I press bear
#define MY_COLOR_R      0
#define MY_COLOR_G      255
#define MY_COLOR_B      0

// your bear's color when you press bear
#define YOUR_COLOR_R      0
#define YOUR_COLOR_G      0
#define YOUR_COLOR_B      255

// if both touched color when you press bear
#define COMBO_COLOR_R      255
#define COMBO_COLOR_G      69
#define COMBO_COLOR_B      0

#define TIMEOUT_DUAL_MODE  10000


// acceleromter interrupt pin (set to 1 if the bear is picked up)
const byte accIntPin = 3;


// track if in two player mode
bool twoPlayerMode  = false;

// keep track of which sensor is registering a touch
byte my_touch_map   = 0;
byte your_touch_map = 0;

// keep track of what is sent
byte sent_touch_map   = 0;

// tracking variable to determine if bear is picked up
volatile bool picked_up      = false;

// variable that gets updated with current pixel values
uint8_t u8R, u8G, u8B;

bool your_bear_pickup_flag = false;

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

  
//  // XBee setup
//#ifdef XBEE_ACTIVE
//  XBee.begin(9600);
//#endif

  // start neo pixels
  pixels.begin(); // This initializes the NeoPixel library.

  // accelerometer interupt setup
#ifdef ACC_ENABLE
  delay(500);
  pinMode(accIntPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(accIntPin), bearPickedUp, FALLING);
  
  // throw error if accelerometer not working
  if (! mma.begin()) {
    //Serial.println("Couldnt start");
    while (1);
  }

  // setup accelerometer
  mma.setRange(MMA8451_RANGE_2_G);
  
  // enable accelerometer interrupt
  //setup acc
  setupAcc();
  
#endif

  // function to call when want to do one cycle of the heartbeat
  heartbeatTouchableLights(10);
  pinMode(13, OUTPUT);
}

/******************************************************************/
/************************ LOOP ************************************/
/******************************************************************/
bool start_phase = true;

void loop() {
  if(isYourBearPickedUp() || receivedTouch()){
    unsigned long init_time = millis();
    while(true){

      // this is a notification by your bear that you want to pair
      if(your_touch_map == 0xFF){
        your_touch_map = 0;
        dual_mode();
        setLight(HEART_LED,   0, 0, 0);
      }

      
      if( (millis() - init_time) >= TIMEOUT_DUAL_MODE){
        break;
      }

      heartbeatTouchableLights(100); 
  
  
      // I touched the bear so that I can interact with the other bear
      if( my_touch_map != 0){
        Serial.write(0xFF); //tell other bear to enter dual mode!
        dual_mode();
        setLight(HEART_LED,   0, 0, 0);
      } 
    }
  }else{
    
    // check all touch sensors and update touch sensor variable (also checks and sends if picked up)
    updateTouchMap();
    // send info on what is being touched on this bear
    #ifdef XBEE_ACTIVE
    sendTouchMap();
    #endif
  
    //init single player game
    if(touchedTag(my_touch_map)){
      simonGame();
    }
  
    // has my bear been lifted
    if(isMyBearPickedUp()){
      sendBearPickedUp();
      digitalWrite(13,HIGH);
    }else{
      digitalWrite(13,LOW);
    }
  
      // turn on LEDs that have been touched on my bear
    turnOnMyTouched(my_touch_map);
    // turn off LEDs that are not touched by my bear
    turnOffMyNotTouched(my_touch_map);
  
    delay(100);
  }


//    // check all touch sensors and update touch sensor variable (also checks and sends if picked up)
//    updateTouchMap();
//    // send info on what is being touched on this bear
//    #ifdef XBEE_ACTIVE
//    sendTouchMap();
//    #endif
//      // turn on LEDs that have been touched on my bear
//    turnOnMyTouched(my_touch_map);
//    // turn off LEDs that are not touched by my bear
//    turnOffMyNotTouched(my_touch_map);
//
//
//    #ifdef XBEE_ACTIVE
//    if(receivedTouch()){
//      //turn on LEDs that are touched by you
//      turnOnYourTouched(your_touch_map);
//      //turn off LEDs that you let go
//      turnOffYourNotTouched(your_touch_map);
//
//      if(touchedTag(your_touch_map)){
//        initSlaveGame();
//      }
//    #endif
//    }
}

/******************************************************************/
/************************ FUNCTIONS *******************************/
/******************************************************************/

void dual_mode(){
    unsigned long timeout_counter = 0;

    //turn on heart when paired
    setLight(HEART_LED,   255, 0, 0);

    while(true){
      // check all touch sensors and update touch sensor variable (also checks and sends if picked up)
      updateTouchMap();
      // send info on what is being touched on this bear
      #ifdef XBEE_ACTIVE
      sendTouchMap();
      #endif
        
      // turn on LEDs that have been touched on my bear
      turnOnMyTouched(my_touch_map);
  
      // turn off LEDs that are not touched by my bear
      turnOffMyNotTouched(my_touch_map);

      #ifdef XBEE_ACTIVE
      if(receivedTouch()){
        timeout_counter = millis();
        
        //turn on LEDs that are touched by you
        turnOnYourTouched(your_touch_map);
        //turn off LEDs that you let go
        turnOffYourNotTouched(your_touch_map);
  
        if(touchedTag(your_touch_map)){
          initSlaveGame();
        }
      }
      
      #endif
  
      if(touchedTag(your_touch_map)){
        initMasterGame();
      }

      //time out if no touch activity on your bear
      if((millis() - timeout_counter) >= TIMEOUT_DUAL_MODE){
        return;
      }
    }
}

vector<byte> waitToReceiveSequence(){
  
  uint8_t numberOfBytesToReceive = 0;
  while(numberOfBytesToReceive == 0){
    if(Serial.available())
    { // If data comes in from XBee, send it out to serial monitor
      Serial.readBytes(&numberOfBytesToReceive,1);
    }
  }

  uint8_t receivedSequence[numberOfBytesToReceive];
    if(Serial.available())
    { // If data comes in from XBee, send it out to serial monitor
      Serial.readBytes(receivedSequence,numberOfBytesToReceive);
    }else{
      #ifdef DEBUG 
      Serial.println("didn't receive anything");
      #endif
    }

  vector<byte> receivedMoves;
  for(int i = 0; i < numberOfBytesToReceive; i++){
    receivedMoves.push_back(receivedSequence[i]);
  }

  return receivedMoves;
}

void sendSequence(vector<byte> sequence){
  byte sequence_size = sequence.size();
  byte sequence_array[sequence_size+1];
  sequence_array[0] = sequence_size;
  
  for(int i = 0; i < sequence_size; i++){
    sequence_array[i+1] = sequence[i];
  }
  Serial.write(sequence_array, sequence_size + 1);
}

bool touchedTag(byte touch_map){
  if( (touch_map &= MAP_TAG) == MAP_TAG){
    return true;
  }
  else{
    return false;
  }
}

#ifdef ACC_ENABLE
void setupAcc(){
  mma.writeRegister8(MMA8451_REG_CTRL_REG1, 0x00);            // deactivate
  mma.writeRegister8(0X2A, 0x18);
  mma.writeRegister8(0x15, 0x78 );
  mma.writeRegister8(0x17, 0x30 ); //SET THRESHOLD TO 48 COUNTS
  mma.writeRegister8(0x18, 0x0A ); // 100ms debounce timing
  mma.writeRegister8(0x2C, 0x02 | 0x01);
  mma.writeRegister8(MMA8451_REG_CTRL_REG4, 0x04 | 0x20);
  mma.writeRegister8(MMA8451_REG_CTRL_REG5, 0x04 | 0x20);
  mma.writeRegister8(MMA8451_REG_CTRL_REG1, 0x01 | 0x04);     // activate (hard coded)
}
#endif

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
    if(u8B == YOUR_COLOR_B){
      setLight(LEFT_HAND_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else{
    setLight(LEFT_HAND_LED, u8R, MY_COLOR_G, u8B);
    }
  }
  if( (touch_map & MAP_LEFT_FOOT) == MAP_LEFT_FOOT){
    getCurrentPixel(LEFT_FOOT_LED);
    if(u8B == YOUR_COLOR_B){
      setLight(LEFT_FOOT_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else{
    setLight(LEFT_FOOT_LED, u8R, MY_COLOR_G, u8B);
    }
  }
  if( (touch_map & MAP_RIGHT_FOOT) == MAP_RIGHT_FOOT){
    getCurrentPixel(RIGHT_FOOT_LED);
    if(u8B == YOUR_COLOR_B){
      setLight(RIGHT_FOOT_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else{
    setLight(RIGHT_FOOT_LED, u8R, MY_COLOR_G, u8B);
    }
  }
  if( (touch_map & MAP_RIGHT_HAND) == MAP_RIGHT_HAND){
    getCurrentPixel(RIGHT_HAND_LED);
    if(u8B == YOUR_COLOR_B){
      setLight(RIGHT_HAND_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else{
    setLight(RIGHT_HAND_LED, u8R, MY_COLOR_G, u8B);
    }
  }
  if( (touch_map & MAP_RIGHT_EAR) == MAP_RIGHT_EAR){
    getCurrentPixel(RIGHT_EAR_LED);
    if(u8B == YOUR_COLOR_B){
      setLight(RIGHT_EAR_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else{
    setLight(RIGHT_EAR_LED, u8R, MY_COLOR_G, u8B);
    }
  }
  if( (touch_map & MAP_LEFT_EAR) == MAP_LEFT_EAR){
    getCurrentPixel(LEFT_EAR_LED);
    if(u8B == YOUR_COLOR_B){
      setLight(LEFT_EAR_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else{
    setLight(LEFT_EAR_LED, u8R, MY_COLOR_G, u8B);
    }
  }
}

void turnOnYourTouched(byte touch_map){
  if( (touch_map & MAP_LEFT_HAND) == MAP_LEFT_HAND){
    getCurrentPixel(LEFT_HAND_LED);
    if(u8G == MY_COLOR_B){
      setLight(LEFT_HAND_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else{
    setLight(LEFT_HAND_LED, u8R, u8G, YOUR_COLOR_B);
    }
  }
  if( (touch_map & MAP_LEFT_FOOT) == MAP_LEFT_FOOT){
    getCurrentPixel(LEFT_FOOT_LED);
    if(u8G == MY_COLOR_B){
      setLight(LEFT_FOOT_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else{
    setLight(LEFT_FOOT_LED, u8R, u8G, YOUR_COLOR_B);
    }
  }
  if( (touch_map & MAP_RIGHT_FOOT) == MAP_RIGHT_FOOT){
    getCurrentPixel(RIGHT_FOOT_LED);
    if(u8G == MY_COLOR_B){
      setLight(RIGHT_FOOT_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else{
    setLight(RIGHT_FOOT_LED, u8R, u8G, YOUR_COLOR_B);
    }
  }
  if( (touch_map & MAP_RIGHT_HAND) == MAP_RIGHT_HAND){
    getCurrentPixel(RIGHT_HAND_LED);
    if(u8G == MY_COLOR_B){
      setLight(RIGHT_HAND_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else{
    setLight(RIGHT_HAND_LED, u8R, u8G, YOUR_COLOR_B);
    }
  }
  if( (touch_map & MAP_RIGHT_EAR) == MAP_RIGHT_EAR){
    getCurrentPixel(RIGHT_EAR_LED);
    if(u8G == MY_COLOR_B){
      setLight(RIGHT_EAR_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else{
    setLight(RIGHT_EAR_LED, u8R, u8G, YOUR_COLOR_B);
    }
  }
  if( (touch_map & MAP_LEFT_EAR) == MAP_LEFT_EAR){
    getCurrentPixel(LEFT_EAR_LED);
    if(u8G == MY_COLOR_B){
      setLight(LEFT_EAR_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else{
    setLight(LEFT_EAR_LED, u8R, u8G, YOUR_COLOR_B);
    }
  }
}

// send map to this function to turn off anything that is being touched
void turnOffMyNotTouched(byte touch_map){
  if( (touch_map & MAP_LEFT_HAND) != MAP_LEFT_HAND){
    getCurrentPixel(LEFT_HAND_LED);
    if(u8R == COMBO_COLOR_R){
      setLight(LEFT_HAND_LED, 0, 0, YOUR_COLOR_B);
    }
    else{
      setLight(LEFT_HAND_LED, 0, 0, u8B);
    }
  }
  if( (touch_map & MAP_LEFT_FOOT) != MAP_LEFT_FOOT){
    getCurrentPixel(LEFT_FOOT_LED);
    if(u8R == COMBO_COLOR_R){
      setLight(LEFT_FOOT_LED, 0, 0, YOUR_COLOR_B);
    }
    else{
      setLight(LEFT_FOOT_LED, 0, 0, u8B);
    }
  }
  if( (touch_map & MAP_RIGHT_FOOT) != MAP_RIGHT_FOOT){
    getCurrentPixel(RIGHT_FOOT_LED);
    if(u8R == COMBO_COLOR_R){
      setLight(RIGHT_FOOT_LED, 0, 0, YOUR_COLOR_B);
    }
    else{
      setLight(RIGHT_FOOT_LED, 0, 0, u8B);
    }
  }
  if( (touch_map & MAP_RIGHT_HAND) != MAP_RIGHT_HAND){
    getCurrentPixel(RIGHT_HAND_LED);
    if(u8R == COMBO_COLOR_R){
      setLight(RIGHT_HAND_LED, 0, 0, YOUR_COLOR_B);
    }
    else{
      setLight(RIGHT_HAND_LED, 0, 0, u8B);
    }
  }
  if( (touch_map & MAP_RIGHT_EAR) != MAP_RIGHT_EAR){
    getCurrentPixel(RIGHT_EAR_LED);
    if(u8R == COMBO_COLOR_R){
      setLight(RIGHT_EAR_LED, 0, 0, YOUR_COLOR_B);
    }
    else{
      setLight(RIGHT_EAR_LED, 0, 0, u8B);
    }
  }
  if( (touch_map & MAP_LEFT_EAR) != MAP_LEFT_EAR){
    getCurrentPixel(LEFT_EAR_LED);
    if(u8R == COMBO_COLOR_R){
      setLight(LEFT_EAR_LED, 0, 0, YOUR_COLOR_B);
    }
    else{
      setLight(LEFT_EAR_LED, 0, 0, u8B);
    }
  }
}

// send map to this function to turn off anything that is being touched
void turnOffYourNotTouched(byte touch_map){
  if( (touch_map & MAP_LEFT_HAND) != MAP_LEFT_HAND){
    getCurrentPixel(LEFT_HAND_LED);
    if(u8R == COMBO_COLOR_R){
      setLight(LEFT_HAND_LED, 0, YOUR_COLOR_G, 0);
    }
    else{
      setLight(LEFT_HAND_LED, 0, u8G, 0);
    }
  }
  if( (touch_map & MAP_LEFT_FOOT) != MAP_LEFT_FOOT){
    getCurrentPixel(LEFT_FOOT_LED);
    if(u8R == COMBO_COLOR_R){
      setLight(LEFT_FOOT_LED, 0, YOUR_COLOR_G, 0);
    }
    else{
      setLight(LEFT_FOOT_LED, 0, u8G, 0);
    }
  }
  if( (touch_map & MAP_RIGHT_FOOT) != MAP_RIGHT_FOOT){
    getCurrentPixel(RIGHT_FOOT_LED);
    if(u8R == COMBO_COLOR_R){
      setLight(RIGHT_FOOT_LED, 0, YOUR_COLOR_G, 0);
    }
    else{
      setLight(RIGHT_FOOT_LED, 0, u8G, 0);
    }
  }
  if( (touch_map & MAP_RIGHT_HAND) != MAP_RIGHT_HAND){
    getCurrentPixel(RIGHT_HAND_LED);
    if(u8R == COMBO_COLOR_R){
      setLight(RIGHT_HAND_LED, 0, YOUR_COLOR_G, 0);
    }
    else{
      setLight(RIGHT_HAND_LED, 0, u8G, 0);
    }
  }
  if( (touch_map & MAP_RIGHT_EAR) != MAP_RIGHT_EAR){
    getCurrentPixel(RIGHT_EAR_LED);
    if(u8R == COMBO_COLOR_R){
      setLight(RIGHT_EAR_LED, 0, YOUR_COLOR_G, 0);
    }
    else{
      setLight(RIGHT_EAR_LED, 0, u8G, 0);
    }
  }
  if( (touch_map & MAP_LEFT_EAR) != MAP_LEFT_EAR){
    getCurrentPixel(LEFT_EAR_LED);
    if(u8R == COMBO_COLOR_R){
      setLight(LEFT_EAR_LED, 0, YOUR_COLOR_G, 0);
    }
    else{
      setLight(LEFT_EAR_LED, 0, u8G, 0);
    }
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

void heartbeatTouchableLights(uint8_t delay_time){
  #ifdef DEBUG 
  Serial.println("Heartbeat active");
  #endif
  for(uint16_t t = 0; t <= 255; t+=5){
    updateTouchMap();
    if(my_touch_map != 0){
      setLight(HEART_LED,   0, 0, 0);
      setLight(LEFT_HAND_LED,   0, 0, 0);
      setLight(LEFT_FOOT_LED,   0, 0, 0);
      setLight(RIGHT_FOOT_LED,  0, 0, 0);
      setLight(RIGHT_HAND_LED,  0, 0, 0);
      setLight(RIGHT_EAR_LED,   0, 0, 0);
      setLight(LEFT_EAR_LED,    0, 0, 0);
      break;
    }
    
    setLight(HEART_LED,   t, 0, 0);
    setLight(LEFT_HAND_LED,   t, 0, 0);
    setLight(LEFT_FOOT_LED,   t, 0, 0);
    setLight(RIGHT_FOOT_LED,  t, 0, 0);
    setLight(RIGHT_HAND_LED,  t, 0, 0);
    setLight(RIGHT_EAR_LED,   t, 0, 0);
    setLight(LEFT_EAR_LED,    t, 0, 0);
    delay(delay_time);
    
    #ifdef DEBUG 
    Serial.println(t);
    Serial.println("going up");
    #endif
  }
  for(int16_t t = 255; t >= 0; t-=5){
    updateTouchMap();
    if(my_touch_map != 0){
      setLight(HEART_LED,   0, 0, 0);
      setLight(LEFT_HAND_LED,   0, 0, 0);
      setLight(LEFT_FOOT_LED,   0, 0, 0);
      setLight(RIGHT_FOOT_LED,  0, 0, 0);
      setLight(RIGHT_HAND_LED,  0, 0, 0);
      setLight(RIGHT_EAR_LED,   0, 0, 0);
      setLight(LEFT_EAR_LED,    0, 0, 0);
      break;
    }
    
    setLight(HEART_LED,   t, 0, 0);
    setLight(LEFT_HAND_LED,   t, 0, 0);
    setLight(LEFT_FOOT_LED,   t, 0, 0);
    setLight(RIGHT_FOOT_LED,  t, 0, 0);
    setLight(RIGHT_HAND_LED,  t, 0, 0);
    setLight(RIGHT_EAR_LED,   t, 0, 0);
    setLight(LEFT_EAR_LED,    t, 0, 0);
    delay(delay_time);

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
    if(start_phase){
      start_phase = false;
    }else{
      Serial.write(my_touch_map);
    }
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

bool isMyBearPickedUp(){
  if(picked_up){
    // reenable interrupt
    picked_up = false;
    setupAcc();
    #ifdef DEBUG 
    Serial.println("Bear is picked up");
    #endif
    pinMode(accIntPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(accIntPin), bearPickedUp, FALLING);
    return true;
  }
  else return false;
}

void sendBearPickedUp(){
  byte send_picked_up = 0;
  send_picked_up |= MAP_ACC;
  Serial.write(send_picked_up);
}

bool isYourBearPickedUp(){
  if ((your_touch_map & MAP_ACC) == MAP_ACC) {
    your_touch_map = your_touch_map & (0xFF & ~MAP_ACC);
    your_bear_pickup_flag = true;
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

/******************************************************************/
/************************ GAME CODE *******************************/
/******************************************************************/

#define TIMEOUT 5000

void simonGame(void){
  #ifdef DEBUG 
  Serial.println("start game");
  #endif
  int round = 1;
  uint16_t GAME_R = 0;
  uint16_t GAME_G = 0;
  uint16_t GAME_B = 0;
  
  // play chime
  // light up heart (l-->r t-->b) green red yellow blue
  while(true){
    uint16_t gameInput[round];
     #ifdef DEBUG 
     Serial.println(round);
     #endif

     
    // initialize pattern to match
    for (int i = 0; i < round; i++){
      gameInput[i] = random(4,8);
      #ifdef DEBUG 
      Serial.println(gameInput[i]);
      #endif
      // red
      if(gameInput[i] == 4) {
        GAME_R = 255;
        GAME_G = 0;
        GAME_B = 0;
      }
      // green
      if(gameInput[i] == 5) {
        GAME_R = 0;
        GAME_G = 255;
        GAME_B = 0;
      }
      // blue
      if(gameInput[i] == 6) {
        GAME_R = 0;
        GAME_G = 0;
        GAME_B = 255;
      }
      // yellow
      if(gameInput[i] == 7) {
        GAME_R = 255;
        GAME_G = 255;
        GAME_B = 0;
      }
       #ifdef DEBUG 
       Serial.println("interrupt!");
       #endif
      setLight(gameInput[i], GAME_R, GAME_G, GAME_B);
      delay(500);
      // turn off
      setLight(gameInput[i], 0, 0, 0);
      delay(100);
    }
    
    // take input from the user and compare
    for (int j = 0; j < round; j++){
      #ifdef DEBUG 
       Serial.println("waiting for input");
       #endif
      unsigned long initTime = millis();
      do{
        updateTouchMap();
        if((millis() - initTime) >= TIMEOUT){
          return;
        }
        delay(50);
      }while(my_touch_map == 0);

      
      // turn on LEDs that have been touched on my bear
      turnOnMyTouched(my_touch_map);
      // turn off LEDs that are not touched by my bear
      turnOffMyNotTouched(my_touch_map);
  
      if(gameInput[j] == 4) {
        
        if(digitalRead(FSR_LEFT_HAND) == LOW) {
          #ifdef DEBUG 
          Serial.println("4 touched");
          #endif
          // play tune 4
          //continue;
        }
        else {
          gameOver();
          return;
        }
      }
      if(gameInput[j] == 5) {
        if(digitalRead(FSR_LEFT_FOOT) == LOW) {
          #ifdef DEBUG 
          Serial.println("5 touched");
          #endif
          // play tune 5
          //continue;
        }
        else {
          gameOver();
          return;
        }
      }
      if(gameInput[j] == 6) {
        if(digitalRead(FSR_RIGHT_FOOT) == LOW) {
          #ifdef DEBUG 
          Serial.println("6 touched");
          #endif
          // play tune 6
          //continue;
        }
        else {
          gameOver();
          return;
        }
      }
      if(gameInput[j] == 7) {
        if(digitalRead(FSR_RIGHT_HAND) == LOW) {
          #ifdef DEBUG 
          Serial.println("7 touched");
          #endif
           // play tune 7
           //continue;
         }
         else {
           gameOver();
           return;
         }
      }
      while(my_touch_map != 0){
        #ifdef DEBUG 
         Serial.println("waiting to let go");
         #endif
        updateTouchMap();
        delay(50);
      } 
        // turn on LEDs that have been touched on my bear
      turnOnMyTouched(my_touch_map);
      // turn off LEDs that are not touched by my bear
      turnOffMyNotTouched(my_touch_map);
    }
    round++;
  }
}

void gameOver(void){
  //play losing tune
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
}

void gameWon(void){
  //play winning tune
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
}

/* BearSeeBearDo */ 
 
// initialize pattern to match
void recordAndSendMoves(){
  uint16_t GAME_R = 0;
  uint16_t GAME_G = 0;
  uint16_t GAME_B = 0;

  unsigned long p1inputTime = 0;
  
  vector<byte> gameInput;  
  int i = 0;
  while(true){
    p1inputTime = millis();
    // dont do anything while not touching
    do{
      updateTouchMap();
      if((millis() - p1inputTime) >= TIMEOUT){
        sendSequence(gameInput);
      }
      delay(50);
    }while( (my_touch_map & 0xFC) == 0);
    
    if (digitalRead(FSR_LEFT_HAND) == LOW){
      gameInput.push_back(4);
      // play tune 4
      GAME_R = 255;
      GAME_G = 0;
      GAME_B = 0;
      setLight(FSR_LEFT_HAND, GAME_R, GAME_G, GAME_B);
      delay(500);
      // turn off
      setLight(FSR_LEFT_HAND, 0, 0, 0);
      delay(100);
      
    }
    if (digitalRead(FSR_LEFT_FOOT) == LOW){
      gameInput.push_back(5);
      // play tune 5
      GAME_R = 0;
      GAME_G = 255;
      GAME_B = 0;
      setLight(FSR_LEFT_FOOT, GAME_R, GAME_G, GAME_B);
      delay(500);
      // turn off
      setLight(FSR_LEFT_FOOT, 0, 0, 0);
      delay(100);
    }
    if (digitalRead(FSR_RIGHT_FOOT) == LOW){
      gameInput.push_back(6);
      // play tune 6
      GAME_R = 0;
      GAME_G = 0;
      GAME_B = 255;
      setLight(FSR_RIGHT_FOOT, GAME_R, GAME_G, GAME_B);
      delay(500);
      // turn off
      setLight(FSR_RIGHT_FOOT, 0, 0, 0);
      delay(100);
    }
    if (digitalRead(FSR_RIGHT_HAND)  == LOW){
      gameInput.push_back(7);
      // play tune 7
      GAME_R = 255;
      GAME_G = 255;
      GAME_B = 0;
      setLight(FSR_RIGHT_HAND, GAME_R, GAME_G, GAME_B);
      delay(500);
      // turn off
      setLight(FSR_RIGHT_HAND, 0, 0, 0);
      delay(100);
    }
    if (digitalRead(FSR_RIGHT_EAR)  == LOW) {
      gameInput.push_back(8);
      // play tune 8
      GAME_R = 0;
      GAME_G = 255;
      GAME_B = 255;
      setLight(FSR_RIGHT_EAR, GAME_R, GAME_G, GAME_B);
      delay(500);
      // turn off
      setLight(FSR_RIGHT_EAR, 0, 0, 0);
      delay(100);
    }
    if (digitalRead(FSR_LEFT_EAR)  == LOW) {
      gameInput.push_back(9);
      // play tune 9
      GAME_R = 255;
      GAME_G = 0;
      GAME_B = 255;
      setLight(FSR_LEFT_EAR, GAME_R, GAME_G, GAME_B);
      delay(500);
      // turn off
      setLight(FSR_LEFT_EAR, 0, 0, 0);
      delay(100);
    }
    
    
    // dont do anything while touching
    while(my_touch_map != 0){
      #ifdef DEBUG 
       Serial.println("waiting to let go");
       #endif
      updateTouchMap();
      delay(50);
    }

    i++;
  }
}

void response(vector<byte> gameInput) {
  // take input from the second user and compare
  for (int j = 0, n = gameInput.size(); j < n ; j++){
    
    #ifdef DEBUG 
     Serial.println("waiting for input");
     #endif
    unsigned long p2inputTime = millis();
    
    do{
      updateTouchMap();
      if((millis() - p2inputTime) >= TIMEOUT){
        return;
      }
      delay(50);
    }while(my_touch_map == 0);
    
    if(gameInput[j] == 4 && digitalRead(FSR_LEFT_HAND) != LOW) {
      gameOver();
      return;
    }
    if(gameInput[j] == 5 && digitalRead(FSR_LEFT_FOOT) != LOW) {
      gameOver();
      return;
    }
    if(gameInput[j] == 6 && digitalRead(FSR_RIGHT_FOOT) != LOW) {
      gameOver();
      return;
    }
    if(gameInput[j] == 7 && digitalRead(FSR_RIGHT_HAND) != LOW) {
      gameOver();
      return;
    }
    if(gameInput[j] == 8 && digitalRead(FSR_RIGHT_HAND) != LOW) {
      gameOver();
      return;
    }
    if(gameInput[j] == 9 && digitalRead(FSR_RIGHT_HAND) != LOW) {
       gameOver();
       return;
    }
    while(my_touch_map != 0){
      #ifdef DEBUG 
       Serial.println("waiting to let go");
       #endif
      updateTouchMap();
      delay(50);
    }
  }
  gameWon();
}

void initMasterGame(){
  recordAndSendMoves();
}

void initSlaveGame(){
  vector<byte> receivedMoves = waitToReceiveSequence();
  response(receivedMoves);
}
