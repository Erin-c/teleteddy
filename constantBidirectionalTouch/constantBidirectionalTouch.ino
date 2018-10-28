#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ArduinoSTL.h>
#include <vector>
#include <iterator>
#include <toneAC.h>

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

// tones
// Melodies and tones
#define C2 65
#define D2 73
#define E2 82
#define F2 87
#define G2 98
#define A22 110
#define B2 125
#define C3 131
#define D3 147
#define E3 165
#define F3 175
#define G3 196
#define GS3 208
#define A33 220
#define B3 247
#define C4 262
#define D4 294
#define DS4 311
#define E4 330
#define F4 349
#define G4 392
#define GS4 415
#define A44 440
#define B4 494
#define C5 523
#define C6 1046
#define C7 2093

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
#define INPUT_TIMEOUT      3000
#define TIMEOUT_RESPONSE   5000

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

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#ifdef ACC_ENABLE
Adafruit_MMA8451 mma = Adafruit_MMA8451();
#endif
/******************************************************************/
/************************ SETUP ***********************************/
/******************************************************************/
bool start_phase = true;

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
  pinMode(accIntPin, INPUT);

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
  delay(1000);
  attachInterrupt(digitalPinToInterrupt(accIntPin), bearPickedUp, FALLING);

#endif

  // function to call when want to do one cycle of the heartbeat
  heartbeatTouchableLights(10);
  pinMode(13, OUTPUT);

  // get rid of some start hardware funniness
  if (start_phase) {
    delay(500);
    picked_up = false;
    attachInterrupt(digitalPinToInterrupt(accIntPin), bearPickedUp, FALLING);
  }
}

/******************************************************************/
/************************ LOOP ************************************/
/******************************************************************/

void loop() {
  //heartbeatTouchableLights(50);

//
//  if (isYourBearPickedUp() || receivedTouch()) {
//    unsigned long init_time = millis();
//    while (true) {
//
//      // this is a notification by your bear that you want to pair
//      if (your_touch_map == 0xFF) {
//        your_touch_map = 0;
//        dual_mode();
//        setLight(HEART_LED,   0, 0, 0);
//        playUnpairedTone();
//        break;
//      }
//
//
//      if ( (millis() - init_time) >= TIMEOUT_DUAL_MODE) {
//        break;
//      }
//
//      heartbeatTouchableLights(100);
//
//
//      // I touched the bear so that I can interact with the other bear
//      if ( my_touch_map != 0) {
//        Serial.write(0xFF); //tell other bear to enter dual mode!
//        dual_mode();
//        setLight(HEART_LED,   0, 0, 0);
//        playUnpairedTone();
//        break;
//      }
//    }
//  } else {
//
//    // check all touch sensors and update touch sensor variable (also checks and sends if picked up)
//    updateTouchMap();
//    // send info on what is being touched on this bear
//#ifdef XBEE_ACTIVE
//    sendTouchMap();
//#endif
//
//    //init single player game
//    if (touchedTag(my_touch_map)) {
//#ifdef SPEAKER_ENABLE
//      my_touch_map &= 0xFD;
//      playGameStartTone();
//      delay(500);
//#endif
//      simonGame();
//    }
//
//    // has my bear been lifted
//    if (isMyBearPickedUp()) {
//      sendBearPickedUp();
//      digitalWrite(13, HIGH);
//    } else {
//      digitalWrite(13, LOW);
//    }
//
//    // turn on LEDs that have been touched on my bear
//    turnOnMyTouched(my_touch_map);
//    // turn off LEDs that are not touched by my bear
//    turnOffMyNotTouched(my_touch_map);
//
//    delay(100);
//  }


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
        //turn on LEDs that are touched by you
        turnOnYourTouched(your_touch_map);
        //turn off LEDs that you let go
        turnOffYourNotTouched(your_touch_map);
  
        if(touchedTag(&your_touch_map)){
          initSlaveGame();
        }
      #endif
      }
}

/******************************************************************/
/************************ FUNCTIONS *******************************/
/******************************************************************/
//game start
int mel1[] = { GS3, B3, DS4, B3, DS4, F4, A44 };
int m_len1 = 7;
int noteDurations1[] = { 2, 2, 2, 8, 8, 8, 8 };

//game over
int mel2[] = { E3, C3, A22 };
int m_len2 = 3;
int noteDurations2[] = { 2, 4, 1 };

//paired
int mel3[] = { A44, B4, A44, B4 };
int m_len3 = 4;
int noteDurations3[] = { 8, 16, 8, 16 };

//tone 1
int tone1[] = { C2 };
//tone 2
int tone2[] = { E2 };
//tone 3
int tone3[] = { G2 };
//tone 4
int tone4[] = { C3 };
//tone 5
int tone5[] = { E3 };
//tone 6
int tone6[] = { G3 };

int t_ND[] = { 2 };
int t_len = 1;

// Sound playback function
void playSound(int melody[], int mel_length, int noteDurations[]) {
  for (int thisNote = 0; thisNote < mel_length; thisNote++) {
    int noteDuration = 1000/noteDurations[thisNote];
    toneAC(melody[thisNote], 10, noteDuration, true);
    delay(noteDuration * 4 / 3);
  }
  noToneAC();
}

void playGameStartTone(){
  playSound(mel1, m_len1, noteDurations1);  
}

void playGameOverTone(){
  playSound(mel2, m_len2, noteDurations2);
}

void playPairedTone(){
  playSound(mel3, m_len3, noteDurations3);  
}


void playUnpairedTone(){
  playSound(tone1, t_len, t_ND);  
  playSound(tone2, t_len, t_ND);
  playSound(tone3, t_len, t_ND);  
  playSound(tone4, t_len, t_ND);
  playSound(tone5, t_len, t_ND);  
  playSound(tone6, t_len, t_ND);
}

void playTone1(){
  playSound(tone1, t_len, t_ND);  
}
void playTone2(){
  playSound(tone2, t_len, t_ND);
}
void playTone3(){
  playSound(tone3, t_len, t_ND);
}
void playTone4(){
  playSound(tone4, t_len, t_ND);
}
void playTone5(){
  playSound(tone5, t_len, t_ND);
}
void playTone6(){
  playSound(tone6, t_len, t_ND);
}
  
void dual_mode() {
#ifdef DEBUG
  Serial.println("paired");
#endif

#ifdef SPEAKER_ENABLE
  playPairedTone();
#endif
  unsigned long timeout_counter = millis();

  //turn on heart when paired
  setLight(HEART_LED,   255, 0, 0);

  while (true) {
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

    if (receivedTouch()) {
      timeout_counter = millis();

      //turn on LEDs that are touched by you
      turnOnYourTouched(your_touch_map);
      //turn off LEDs that you let go
      turnOffYourNotTouched(your_touch_map);

      if (touchedTag(your_touch_map)) {
        your_touch_map &= 0xFD;
#ifdef DEBUG
        Serial.println("slave mode");
#endif
#ifdef SPEAKER_ENABLE
      playGameStartTone();
#endif
        initSlaveGame();
        break;
        timeout_counter = millis();
      }
    }else if(touchedTag(my_touch_map)) {
      my_touch_map &= 0xFD;
#ifdef DEBUG
      Serial.println("master mode");
#endif
#ifdef SPEAKER_ENABLE
      playGameStartTone();
#endif
      initMasterGame();
      waitForSlave();
      break;
      timeout_counter = millis();
      
      
    }
    //time out if no touch activity on your bear
    else if((millis() - timeout_counter) >= TIMEOUT_DUAL_MODE) {
      return;
    }
  }
}

vector<byte> waitToReceiveSequence() {

  byte numberOfBytesToReceive = 0;
  while (numberOfBytesToReceive == 0) {
    if (Serial.available())
    { // If data comes in from XBee, send it out to serial monitor
       Serial.readBytes(&numberOfBytesToReceive, 1);
    }
  }

  byte receivedSequence[numberOfBytesToReceive];
  int bytes_read = 0 ;
  while (bytes_read < numberOfBytesToReceive)
  {
    if (Serial.available() > 0)
    {
      Serial.readBytes(&receivedSequence[bytes_read],1);
      bytes_read ++;
    }
  }

  vector<byte> receivedMoves;
  for (int i = 0; i < numberOfBytesToReceive; i++) {
    receivedMoves.push_back(receivedSequence[i]);
  }

  return receivedMoves;
}

void sendSequence(vector<byte> sequence) {
  byte sequence_size = sequence.size();
  byte sequence_array[sequence_size + 1];
  sequence_array[0] = sequence_size;
#ifdef DEBUG
  Serial.println("Will be sending");
  Serial.println(sequence_size);
#endif
  for (int i = 0; i < sequence_size; i++) {
#ifdef DEBUG
    Serial.println(sequence[i]);
#endif
    sequence_array[i + 1] = sequence[i];
  }

  delay(10000);
  Serial.write(sequence_array, sequence_size + 1);
}

bool touchedTag(byte touch_map) {

  // brute attempt to avoid any funniness
  if (touch_map == 0xFF) {
    return false;
  }

  if ( (touch_map &= MAP_TAG) == MAP_TAG) {
    return true;
  }
  else {
    return false;
  }
}

#ifdef ACC_ENABLE
void setupAcc() {
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

void waitForSlave() {
  byte garbage = 0;
  while (true) {
    if (Serial.available()) {
      Serial.readBytes(&garbage, 1);
      return;
    }
  }
}

void updateTouchMap(void) {
  my_touch_map = 0x00;
  if (digitalRead(FSR_LEFT_HAND) == LOW) {
    my_touch_map |= MAP_LEFT_HAND;

    //    #ifdef DEBUG
    //    Serial.println("Left hand touched");
    //    #endif
  }

  if (digitalRead(FSR_RIGHT_HAND) == LOW) {
    my_touch_map |= MAP_RIGHT_HAND;

    //    #ifdef DEBUG
    //    Serial.println("Right hand touched");
    //    #endif
  }

  if (digitalRead(FSR_LEFT_FOOT) == LOW) {
    my_touch_map |= MAP_LEFT_FOOT;

    //    #ifdef DEBUG
    //    Serial.println("Left foot touched");
    //    #endif
  }

  if (digitalRead(FSR_RIGHT_FOOT) == LOW) {
    my_touch_map |= MAP_RIGHT_FOOT;

    //    #ifdef DEBUG
    //    Serial.println("Right foot touched");
    //    #endif
  }

  if (digitalRead(FSR_LEFT_EAR) == LOW) {
    my_touch_map |= MAP_LEFT_EAR;

    //    #ifdef DEBUG
    //    Serial.println("Left ear touched");
    //    #endif
  }

  if (digitalRead(FSR_RIGHT_EAR) == LOW) {
    my_touch_map |= MAP_RIGHT_EAR;

    //    #ifdef DEBUG
    //    Serial.println("Right ear touched");
    //    #endif
  }

#ifdef FSR_TAG
  if (digitalRead(FSR_TAG) == LOW) {
    my_touch_map |= MAP_TAG;

    //    #ifdef DEBUG
    //    Serial.println("Tag touched");
    //    #endif
  }
#endif
}


// send map to this function to turn on anything that is being touched
void turnOnMyTouched(uint16_t touch_map) {
  if ( (touch_map & MAP_LEFT_HAND) == MAP_LEFT_HAND) {
    getCurrentPixel(LEFT_HAND_LED);
    if (u8B == YOUR_COLOR_B) {
      setLight(LEFT_HAND_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else {
      setLight(LEFT_HAND_LED, u8R, MY_COLOR_G, u8B);
    }
  }
  if ( (touch_map & MAP_LEFT_FOOT) == MAP_LEFT_FOOT) {
    getCurrentPixel(LEFT_FOOT_LED);
    if (u8B == YOUR_COLOR_B) {
      setLight(LEFT_FOOT_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else {
      setLight(LEFT_FOOT_LED, u8R, MY_COLOR_G, u8B);
    }
  }
  if ( (touch_map & MAP_RIGHT_FOOT) == MAP_RIGHT_FOOT) {
    getCurrentPixel(RIGHT_FOOT_LED);
    if (u8B == YOUR_COLOR_B) {
      setLight(RIGHT_FOOT_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else {
      setLight(RIGHT_FOOT_LED, u8R, MY_COLOR_G, u8B);
    }
  }
  if ( (touch_map & MAP_RIGHT_HAND) == MAP_RIGHT_HAND) {
    getCurrentPixel(RIGHT_HAND_LED);
    if (u8B == YOUR_COLOR_B) {
      setLight(RIGHT_HAND_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else {
      setLight(RIGHT_HAND_LED, u8R, MY_COLOR_G, u8B);
    }
  }
  if ( (touch_map & MAP_RIGHT_EAR) == MAP_RIGHT_EAR) {
    getCurrentPixel(RIGHT_EAR_LED);
    if (u8B == YOUR_COLOR_B) {
      setLight(RIGHT_EAR_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else {
      setLight(RIGHT_EAR_LED, u8R, MY_COLOR_G, u8B);
    }
  }
  if ( (touch_map & MAP_LEFT_EAR) == MAP_LEFT_EAR) {
    getCurrentPixel(LEFT_EAR_LED);
    if (u8B == YOUR_COLOR_B) {
      setLight(LEFT_EAR_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else {
      setLight(LEFT_EAR_LED, u8R, MY_COLOR_G, u8B);
    }
  }
}

void turnOnYourTouched(byte touch_map) {
  if ( (touch_map & MAP_LEFT_HAND) == MAP_LEFT_HAND) {
    getCurrentPixel(LEFT_HAND_LED);
    if (u8G == MY_COLOR_B) {
      setLight(LEFT_HAND_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else {
      setLight(LEFT_HAND_LED, u8R, u8G, YOUR_COLOR_B);
    }
  }
  if ( (touch_map & MAP_LEFT_FOOT) == MAP_LEFT_FOOT) {
    getCurrentPixel(LEFT_FOOT_LED);
    if (u8G == MY_COLOR_B) {
      setLight(LEFT_FOOT_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else {
      setLight(LEFT_FOOT_LED, u8R, u8G, YOUR_COLOR_B);
    }
  }
  if ( (touch_map & MAP_RIGHT_FOOT) == MAP_RIGHT_FOOT) {
    getCurrentPixel(RIGHT_FOOT_LED);
    if (u8G == MY_COLOR_B) {
      setLight(RIGHT_FOOT_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else {
      setLight(RIGHT_FOOT_LED, u8R, u8G, YOUR_COLOR_B);
    }
  }
  if ( (touch_map & MAP_RIGHT_HAND) == MAP_RIGHT_HAND) {
    getCurrentPixel(RIGHT_HAND_LED);
    if (u8G == MY_COLOR_B) {
      setLight(RIGHT_HAND_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else {
      setLight(RIGHT_HAND_LED, u8R, u8G, YOUR_COLOR_B);
    }
  }
  if ( (touch_map & MAP_RIGHT_EAR) == MAP_RIGHT_EAR) {
    getCurrentPixel(RIGHT_EAR_LED);
    if (u8G == MY_COLOR_B) {
      setLight(RIGHT_EAR_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else {
      setLight(RIGHT_EAR_LED, u8R, u8G, YOUR_COLOR_B);
    }
  }
  if ( (touch_map & MAP_LEFT_EAR) == MAP_LEFT_EAR) {
    getCurrentPixel(LEFT_EAR_LED);
    if (u8G == MY_COLOR_B) {
      setLight(LEFT_EAR_LED, COMBO_COLOR_R, COMBO_COLOR_G, COMBO_COLOR_B);
    }
    else {
      setLight(LEFT_EAR_LED, u8R, u8G, YOUR_COLOR_B);
    }
  }
}

// send map to this function to turn off anything that is being touched
void turnOffMyNotTouched(byte touch_map) {
  if ( (touch_map & MAP_LEFT_HAND) != MAP_LEFT_HAND) {
    getCurrentPixel(LEFT_HAND_LED);
    if (u8R == COMBO_COLOR_R) {
      setLight(LEFT_HAND_LED, 0, 0, YOUR_COLOR_B);
    }
    else {
      setLight(LEFT_HAND_LED, 0, 0, u8B);
    }
  }
  if ( (touch_map & MAP_LEFT_FOOT) != MAP_LEFT_FOOT) {
    getCurrentPixel(LEFT_FOOT_LED);
    if (u8R == COMBO_COLOR_R) {
      setLight(LEFT_FOOT_LED, 0, 0, YOUR_COLOR_B);
    }
    else {
      setLight(LEFT_FOOT_LED, 0, 0, u8B);
    }
  }
  if ( (touch_map & MAP_RIGHT_FOOT) != MAP_RIGHT_FOOT) {
    getCurrentPixel(RIGHT_FOOT_LED);
    if (u8R == COMBO_COLOR_R) {
      setLight(RIGHT_FOOT_LED, 0, 0, YOUR_COLOR_B);
    }
    else {
      setLight(RIGHT_FOOT_LED, 0, 0, u8B);
    }
  }
  if ( (touch_map & MAP_RIGHT_HAND) != MAP_RIGHT_HAND) {
    getCurrentPixel(RIGHT_HAND_LED);
    if (u8R == COMBO_COLOR_R) {
      setLight(RIGHT_HAND_LED, 0, 0, YOUR_COLOR_B);
    }
    else {
      setLight(RIGHT_HAND_LED, 0, 0, u8B);
    }
  }
  if ( (touch_map & MAP_RIGHT_EAR) != MAP_RIGHT_EAR) {
    getCurrentPixel(RIGHT_EAR_LED);
    if (u8R == COMBO_COLOR_R) {
      setLight(RIGHT_EAR_LED, 0, 0, YOUR_COLOR_B);
    }
    else {
      setLight(RIGHT_EAR_LED, 0, 0, u8B);
    }
  }
  if ( (touch_map & MAP_LEFT_EAR) != MAP_LEFT_EAR) {
    getCurrentPixel(LEFT_EAR_LED);
    if (u8R == COMBO_COLOR_R) {
      setLight(LEFT_EAR_LED, 0, 0, YOUR_COLOR_B);
    }
    else {
      setLight(LEFT_EAR_LED, 0, 0, u8B);
    }
  }
}

// send map to this function to turn off anything that is being touched
void turnOffYourNotTouched(byte touch_map) {
  if ( (touch_map & MAP_LEFT_HAND) != MAP_LEFT_HAND) {
    getCurrentPixel(LEFT_HAND_LED);
    if (u8R == COMBO_COLOR_R) {
      setLight(LEFT_HAND_LED, 0, YOUR_COLOR_G, 0);
    }
    else {
      setLight(LEFT_HAND_LED, 0, u8G, 0);
    }
  }
  if ( (touch_map & MAP_LEFT_FOOT) != MAP_LEFT_FOOT) {
    getCurrentPixel(LEFT_FOOT_LED);
    if (u8R == COMBO_COLOR_R) {
      setLight(LEFT_FOOT_LED, 0, YOUR_COLOR_G, 0);
    }
    else {
      setLight(LEFT_FOOT_LED, 0, u8G, 0);
    }
  }
  if ( (touch_map & MAP_RIGHT_FOOT) != MAP_RIGHT_FOOT) {
    getCurrentPixel(RIGHT_FOOT_LED);
    if (u8R == COMBO_COLOR_R) {
      setLight(RIGHT_FOOT_LED, 0, YOUR_COLOR_G, 0);
    }
    else {
      setLight(RIGHT_FOOT_LED, 0, u8G, 0);
    }
  }
  if ( (touch_map & MAP_RIGHT_HAND) != MAP_RIGHT_HAND) {
    getCurrentPixel(RIGHT_HAND_LED);
    if (u8R == COMBO_COLOR_R) {
      setLight(RIGHT_HAND_LED, 0, YOUR_COLOR_G, 0);
    }
    else {
      setLight(RIGHT_HAND_LED, 0, u8G, 0);
    }
  }
  if ( (touch_map & MAP_RIGHT_EAR) != MAP_RIGHT_EAR) {
    getCurrentPixel(RIGHT_EAR_LED);
    if (u8R == COMBO_COLOR_R) {
      setLight(RIGHT_EAR_LED, 0, YOUR_COLOR_G, 0);
    }
    else {
      setLight(RIGHT_EAR_LED, 0, u8G, 0);
    }
  }
  if ( (touch_map & MAP_LEFT_EAR) != MAP_LEFT_EAR) {
    getCurrentPixel(LEFT_EAR_LED);
    if (u8R == COMBO_COLOR_R) {
      setLight(LEFT_EAR_LED, 0, YOUR_COLOR_G, 0);
    }
    else {
      setLight(LEFT_EAR_LED, 0, u8G, 0);
    }
  }
}

void turnOn(uint16_t location) {
  if (location == HEART_LED) {
    pixels.setPixelColor(1, pixels.Color(MY_COLOR_R, MY_COLOR_G, MY_COLOR_B));
    pixels.setPixelColor(2, pixels.Color(MY_COLOR_R, MY_COLOR_G, MY_COLOR_B));
    pixels.setPixelColor(3, pixels.Color(MY_COLOR_R, MY_COLOR_G, MY_COLOR_B));
    pixels.setPixelColor(4, pixels.Color(MY_COLOR_R, MY_COLOR_G, MY_COLOR_B));
  }
  else {
    pixels.setPixelColor(location, pixels.Color(MY_COLOR_R, MY_COLOR_G, MY_COLOR_B));
  }
  pixels.show();
}

void turnOff(uint16_t location) {
  if (location == HEART_LED) {
    pixels.setPixelColor(1, pixels.Color(0, 0, 0));
    pixels.setPixelColor(2, pixels.Color(0, 0, 0));
    pixels.setPixelColor(3, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4, pixels.Color(0, 0, 0));
  }
  else {
    pixels.setPixelColor(location, pixels.Color(0, 0, 0));
  }
  pixels.show();
}

void setLight(uint16_t location, uint16_t r, uint16_t g, uint16_t b) {
  if (location == HEART_LED) {
    pixels.setPixelColor(0, pixels.Color(r, g, b));
    pixels.setPixelColor(1, pixels.Color(r, g, b));
    pixels.setPixelColor(2, pixels.Color(r, g, b));
    pixels.setPixelColor(3, pixels.Color(r, g, b));
  }
  else {
    pixels.setPixelColor(location, pixels.Color(r, g, b));
  }
  pixels.show();
}

void heartbeatTouchableLights(uint8_t delay_time) {
#ifdef DEBUG
  Serial.println("Heartbeat active");
#endif
  for (uint16_t t = 0; t <= 255; t += 5) {
    updateTouchMap();
    if (my_touch_map != 0) {
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

    //    #ifdef DEBUG
    //    Serial.println(t);
    //    Serial.println("going up");
    //    #endif
  }
  for (int16_t t = 255; t >= 0; t -= 5) {
    updateTouchMap();
    if (my_touch_map != 0) {
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

    //    #ifdef DEBUG
    //    Serial.println(t);
    //    Serial.println("going down");
    //    #endif
  }
}

void playInitTune(void) {

}

#ifdef XBEE_ACTIVE
void sendTouchMap(void) {
  if (sent_touch_map != my_touch_map) {
    if (start_phase) {
      start_phase = false;
    } else {
      Serial.write(my_touch_map);
#ifdef DEBUG
      Serial.println("SENDING TRANSMISSION");
      Serial.println(my_touch_map, BIN);
      Serial.println(my_touch_map, HEX);
#endif
    }
    sent_touch_map = my_touch_map;
  }
}

//updates everytime player lets go or touches
bool receivedTouch(void) {
  if (Serial.available())
  { // If data comes in from XBee, send it out to serial monitor
    Serial.readBytes(&your_touch_map, 1);
#ifdef DEBUG
    Serial.println("RECEIVED TRANSMISSION");
    Serial.println(temp, BIN);
    Serial.println(temp, HEX);
#endif
    return true;
  }
  else {
    return false;
  }
}
#endif

void bearPickedUp() {
  // disable interrupt to avoid constantly activating
  detachInterrupt(digitalPinToInterrupt(accIntPin));

  if (picked_up != true) {
    picked_up = true;
  }

#ifdef DEBUG
  Serial.println("interrupt!");
#endif

}

bool isMyBearPickedUp() {
  if (picked_up) {
    // reenable interrupt
    picked_up = false;
#ifdef DEBUG
    Serial.println("Bear is picked up");
#endif
    pinMode(accIntPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(accIntPin), bearPickedUp, FALLING);
    return true;
  }
  else return false;
}

void sendBearPickedUp() {
  byte send_picked_up = 0;
  send_picked_up |= MAP_ACC;
  Serial.write(send_picked_up);
}

bool isYourBearPickedUp() {
  if ((your_touch_map & MAP_ACC) == MAP_ACC) {
    your_touch_map = your_touch_map & (0xFF & ~MAP_ACC);
    your_bear_pickup_flag = true;
    return true;
  }
  else return false;
}

void pinSetup() {
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

uint8_t readReg(uint8_t reg) {
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

void getCurrentPixel(uint8_t led) {
  long lngRGB = pixels.getPixelColor(led);
  u8R = (uint8_t)((lngRGB >> 16) & 0xff);
  u8G = (uint8_t)((lngRGB >> 8) & 0xff);
  u8B = (uint8_t)(lngRGB & 0xff);
}

/******************************************************************/
/************************ GAME CODE *******************************/
/******************************************************************/

#define TIMEOUT 5000

void simonGame(void) {
#ifdef DEBUG
  Serial.println("start game");
#endif
  int round = 1;
  uint16_t GAME_R = 0;
  uint16_t GAME_G = 0;
  uint16_t GAME_B = 0;

  // play chime
  // light up heart (l-->r t-->b) green red yellow blue
  while (true) {
    uint16_t gameInput[round];
#ifdef DEBUG
    Serial.println(round);
#endif


    // initialize pattern to match
    for (int i = 0; i < round; i++) {
      gameInput[i] = random(4, 8);
#ifdef DEBUG
      Serial.println(gameInput[i]);
#endif
      // red
      if (gameInput[i] == 4) {
        setLight(gameInput[i], GAME_R, GAME_G, GAME_B);
#ifdef SPEAKER_ENABLE
        playTone1();
#endif
        GAME_R = 255;
        GAME_G = 0;
        GAME_B = 0;
      }
      // green
      if (gameInput[i] == 5) {
        setLight(gameInput[i], GAME_R, GAME_G, GAME_B);
#ifdef SPEAKER_ENABLE
        playTone2();
#endif
        GAME_R = 0;
        GAME_G = 255;
        GAME_B = 0;
      }
      // blue
      if (gameInput[i] == 6) {
        setLight(gameInput[i], GAME_R, GAME_G, GAME_B);
#ifdef SPEAKER_ENABLE
        playTone3();
#endif
        GAME_R = 0;
        GAME_G = 0;
        GAME_B = 255;
      }
      // yellow
      if (gameInput[i] == 7) {
        setLight(gameInput[i], GAME_R, GAME_G, GAME_B);
#ifdef SPEAKER_ENABLE
        playTone4();
#endif
        GAME_R = 255;
        GAME_G = 255;
        GAME_B = 0;
      }
#ifdef DEBUG
      Serial.println("interrupt!");
#endif
      delay(500);
      // turn off
      setLight(gameInput[i], 0, 0, 0);
      delay(100);
    }

    // take input from the user and compare
    for (int j = 0; j < round; j++) {
#ifdef DEBUG
      Serial.println("waiting for input");
#endif
      unsigned long initTime = millis();
      do {
        updateTouchMap();
        if ((millis() - initTime) >= TIMEOUT) {
          return;
        }
        delay(50);
      } while (my_touch_map == 0);


      // turn on LEDs that have been touched on my bear
      turnOnMyTouched(my_touch_map);
      // turn off LEDs that are not touched by my bear
      turnOffMyNotTouched(my_touch_map);

      if (gameInput[j] == 4) {

        if (digitalRead(FSR_LEFT_HAND) == LOW) {
#ifdef DEBUG
          Serial.println("4 touched");
#endif
          // play tune 4
#ifdef SPEAKER_ENABLE
        playTone1();
#endif
          //continue;
        }
        else {
          gameOver();
          return;
        }
      }
      if (gameInput[j] == 5) {
        if (digitalRead(FSR_LEFT_FOOT) == LOW) {
#ifdef DEBUG
          Serial.println("5 touched");
#endif
          // play tune 5
#ifdef SPEAKER_ENABLE
        playTone2();
#endif
          //continue;
        }
        else {
          gameOver();
          return;
        }
      }
      if (gameInput[j] == 6) {
        if (digitalRead(FSR_RIGHT_FOOT) == LOW) {
#ifdef DEBUG
          Serial.println("6 touched");
#endif
          // play tune 6
#ifdef SPEAKER_ENABLE
        playTone3();
#endif
          //continue;
        }
        else {
          gameOver();
          return;
        }
      }
      if (gameInput[j] == 7) {
        if (digitalRead(FSR_RIGHT_HAND) == LOW) {
#ifdef DEBUG
          Serial.println("7 touched");
#endif
          // play tune 7
#ifdef SPEAKER_ENABLE
        playTone4();
#endif
          //continue;
        }
        else {
          gameOver();
          return;
        }
      }
      while (my_touch_map != 0) {
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

void gameOver(void) {
  //play losing tune
#ifdef SPEAKER_ENABLE
  delay(100);
  playGameOverTone();
#endif
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
}

void gameWon(void) {
  //play winning tune
  playGameStartTone();
  
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
}

/* BearSeeBearDo */

// initialize pattern to match
void recordAndSendMoves() {
  pinMode(13,HIGH);
  delay(100);
  pinMode(13,LOW);
  delay(100);
  pinMode(13,HIGH);
  delay(100);
  pinMode(13,LOW);
  delay(100);
  uint16_t GAME_R = 0;
  uint16_t GAME_G = 0;
  uint16_t GAME_B = 0;

  unsigned long p1inputTime = 0;

  vector<byte> gameInput;
  int i = 0;
  while (true) {
    p1inputTime = millis();
    // dont do anything while not touching
    do {
      updateTouchMap();
      if ((millis() - p1inputTime) >= INPUT_TIMEOUT) {
        sendSequence(gameInput);
      }
      delay(50);
    } while ( (my_touch_map & 0xFC) == 0);

    if (digitalRead(FSR_LEFT_HAND) == LOW) {
      gameInput.push_back(4);
      // play tune 4
      GAME_R = 255;
      GAME_G = 0;
      GAME_B = 0;
      setLight(LEFT_HAND_LED, GAME_R, GAME_G, GAME_B);
      playTone1();
      delay(500);
      // turn off
      setLight(LEFT_HAND_LED, 0, 0, 0);
      delay(100);

    }
    if (digitalRead(FSR_LEFT_FOOT) == LOW) {
      gameInput.push_back(5);
      // play tune 5
      GAME_R = 0;
      GAME_G = 255;
      GAME_B = 0;
      setLight(LEFT_FOOT_LED, GAME_R, GAME_G, GAME_B);
      playTone2();
      delay(500);
      // turn off
      setLight(LEFT_FOOT_LED, 0, 0, 0);
      delay(100);
    }
    if (digitalRead(FSR_RIGHT_FOOT) == LOW) {
      gameInput.push_back(6);
      // play tune 6
      GAME_R = 0;
      GAME_G = 0;
      GAME_B = 255;
      setLight(RIGHT_FOOT_LED, GAME_R, GAME_G, GAME_B);
      playTone3();
      delay(500);
      // turn off
      setLight(RIGHT_FOOT_LED, 0, 0, 0);
      delay(100);
    }
    if (digitalRead(FSR_RIGHT_HAND)  == LOW) {
      gameInput.push_back(7);
      // play tune 7
      GAME_R = 255;
      GAME_G = 255;
      GAME_B = 0;
      setLight(RIGHT_HAND_LED, GAME_R, GAME_G, GAME_B);
      playTone4();
      delay(500);
      // turn off
      setLight(RIGHT_HAND_LED, 0, 0, 0);
      delay(100);
    }
    if (digitalRead(FSR_RIGHT_EAR)  == LOW) {
      gameInput.push_back(8);
      // play tune 8
      GAME_R = 0;
      GAME_G = 255;
      GAME_B = 255;
      setLight(RIGHT_EAR_LED, GAME_R, GAME_G, GAME_B);
      playTone5();
      delay(500);
      // turn off
      setLight(RIGHT_EAR_LED, 0, 0, 0);
      delay(100);
    }
    if (digitalRead(FSR_LEFT_EAR)  == LOW) {
      gameInput.push_back(9);
      // play tune 9
      GAME_R = 255;
      GAME_G = 0;
      GAME_B = 255;
      setLight(LEFT_EAR_LED, GAME_R, GAME_G, GAME_B);
      playTone6();
      delay(500);
      // turn off
      setLight(LEFT_EAR_LED, 0, 0, 0);
      delay(100);
    }


    // dont do anything while touching
    while (my_touch_map != 0) {
#ifdef DEBUG
      Serial.println("waiting to let go");
#endif
      updateTouchMap();
      delay(50);
    }

    i++;
  }
}

void showReceived(vector<byte> gameInput){
  uint16_t GAME_R = 255;
  uint16_t GAME_G = 0;
  uint16_t GAME_B = 0;
      
  for (int j = 0, n = gameInput.size(); j < n ; j++){
    if (gameInput[j] == 4) {
      // play tune 4
      GAME_R = 255;
      GAME_G = 0;
      GAME_B = 0;
      setLight(LEFT_HAND_LED, GAME_R, GAME_G, GAME_B);
      playTone1();
      delay(500);
      // turn off
      setLight(LEFT_HAND_LED, 0, 0, 0);
      delay(200);
    }
    if (gameInput[j] == 5) {
      // play tune 5
      GAME_R = 0;
      GAME_G = 255;
      GAME_B = 0;
      setLight(LEFT_FOOT_LED, GAME_R, GAME_G, GAME_B);
      playTone2();
      delay(500);
      // turn off
      setLight(LEFT_FOOT_LED, 0, 0, 0);
      delay(200);
    }
    if (gameInput[j] == 6) {
      // play tune 6
      GAME_R = 0;
      GAME_G = 0;
      GAME_B = 255;
      setLight(RIGHT_FOOT_LED, GAME_R, GAME_G, GAME_B);
      playTone3();
      delay(500);
      // turn off
      setLight(RIGHT_FOOT_LED, 0, 0, 0);
      delay(200);
    }
    if (gameInput[j] == 7) {
      // play tune 7
      GAME_R = 255;
      GAME_G = 255;
      GAME_B = 0;
      setLight(RIGHT_HAND_LED, GAME_R, GAME_G, GAME_B);
      playTone4();
      delay(500);
      // turn off
      setLight(RIGHT_HAND_LED, 0, 0, 0);
      delay(200);
    }
    if (gameInput[j] == 8) {
      // play tune 8
      GAME_R = 0;
      GAME_G = 255;
      GAME_B = 255;
      setLight(RIGHT_EAR_LED, GAME_R, GAME_G, GAME_B);
      playTone5();
      delay(500);
      // turn off
      setLight(RIGHT_EAR_LED, 0, 0, 0);
      delay(200);
    }
    if (gameInput[j] == 9) {
      // play tune 9
      GAME_R = 255;
      GAME_G = 0;
      GAME_B = 255;
      setLight(LEFT_EAR_LED, GAME_R, GAME_G, GAME_B);
      playTone6();
      delay(500);
      // turn off
      setLight(LEFT_EAR_LED, 0, 0, 0);
      delay(200);
    }
  }
}

void response(vector<byte> gameInput) {
  // take input from the second user and compare
  for (int j = 0, n = gameInput.size(); j < n ; j++) {

#ifdef DEBUG
    Serial.println("waiting for input");
#endif
    unsigned long p2inputTime = millis();

    do {
      updateTouchMap();
      if ((millis() - p2inputTime) >= TIMEOUT_RESPONSE) {
        return;
      }
      delay(50);
    } while ( (my_touch_map & 0xFC) == 0);

    // turn on LEDs that have been touched on my bear
    turnOnMyTouched(my_touch_map);
    // turn off LEDs that are not touched by my bear
    turnOffMyNotTouched(my_touch_map);
    delay(100);

    if (gameInput[j] == 4 && ((my_touch_map & MAP_LEFT_HAND) == MAP_LEFT_HAND) ) {
      gameOver();
      return;
    }
    else if (gameInput[j] == 5  && ((my_touch_map & MAP_LEFT_FOOT) == MAP_LEFT_FOOT) ) {
      gameOver();
      return;
    }
    else if (gameInput[j] == 6  && ((my_touch_map & MAP_RIGHT_FOOT) == MAP_RIGHT_FOOT) ) {
      gameOver();
      return;
    }
    else if (gameInput[j] == 7  && ((my_touch_map & MAP_RIGHT_HAND) == MAP_RIGHT_HAND) ) {
      gameOver();
      return;
    }
    else if (gameInput[j] == 8  && ((my_touch_map & MAP_RIGHT_EAR) == MAP_RIGHT_EAR) ) {
      gameOver();
      return;
    }
    else if (gameInput[j] == 9  && ((my_touch_map & MAP_LEFT_EAR) == MAP_LEFT_EAR) ) {
      gameOver();
      return;
    }
    while ( (my_touch_map & 0xFC) != 0) {
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
    delay(1000);
  }
  gameWon();
}

void initMasterGame() {
  recordAndSendMoves();
}

void initSlaveGame() {
  vector<byte> receivedMoves = waitToReceiveSequence();
  showReceived(receivedMoves);
  response(receivedMoves);
}
