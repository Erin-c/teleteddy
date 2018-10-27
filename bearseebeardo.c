#include <StandardCplusplus.h>
#include <serstream>
#include <string>
#include <vector>
#include <iterator>

void bearSeeBearDo(void){
  #ifdef DEBUG 
  Serial.println("start game");
  #endif
  
  int round = 10;
  uint16_t GAME_R = 0;
  uint16_t GAME_G = 0;
  uint16_t GAME_B = 0;

  uint16_t gameInput[round];
  unsigned long p1inputTime;
  // play chime
  recordAndSendMoves();
  
 
// initialize pattern to match
void recordAndSendMoves(){
  vector<int> gameInput;  
  int i = 0;
  while(true){
    p1inputTime = millis();
    // dont do anything while not touching
    do{
      updateTouchMap();
      if((millis() - p1inputTime) >= TIMEOUT){
        sendMovesToBear(gameInput[]);
      }
      delay(50);
    }while(my_touch_map == 0);
    
    if (digitalRead(FSR_LEFT_HAND) == LOW){
      gameInput.push_back(4);
      // play tune 4
      GAME_R = 255;
      GAME_G = 0;
      GAME_B = 0;
      setLight(gameInput, GAME_R, GAME_G, GAME_B);
      delay(500);
      // turn off
      setLight(gameInput, 0, 0, 0);
      delay(100);
      
    }
    if (digitalRead(FSR_LEFT_FOOT) == LOW){
      gameInput.push_back(5);
      // play tune 5
      GAME_R = 0;
      GAME_G = 255;
      GAME_B = 0;
      setLight(gameInput, GAME_R, GAME_G, GAME_B);
      delay(500);
      // turn off
      setLight(gameInput, 0, 0, 0);
      delay(100);
    }
    if (digitalRead(FSR_RIGHT_FOOT) == LOW){
      gameInput.push_back(6);
      // play tune 6
      GAME_R = 0;
      GAME_G = 0;
      GAME_B = 255;
      setLight(gameInput, GAME_R, GAME_G, GAME_B);
      delay(500);
      // turn off
      setLight(gameInput, 0, 0, 0);
      delay(100);
    }
    if (digitalRead(FSR_RIGHT_HAND)  == LOW{
      gameInput.push_back(7);
      // play tune 7
      GAME_R = 255;
      GAME_G = 255;
      GAME_B = 0;
      setLight(gameInput, GAME_R, GAME_G, GAME_B);
      delay(500);
      // turn off
      setLight(gameInput, 0, 0, 0);
      delay(100);
    }
    if (digitalRead(FSR_RIGHT_EAR)  == LOW) {
      gameInput.push_back(8);
      // play tune 8
      GAME_R = 0;
      GAME_G = 255;
      GAME_B = 255;
      setLight(gameInput, GAME_R, GAME_G, GAME_B);
      delay(500);
      // turn off
      setLight(gameInput, 0, 0, 0);
      delay(100);
    }
    if (digitalRead(FSR_LEFT_EAR)  == LOW) {
      gameInput.push_back(9);
      // play tune 9
      GAME_R = 255;
      GAME_G = 0;
      GAME_B = 255;
      setLight(gameInput, GAME_R, GAME_G, GAME_B);
      delay(500);
      // turn off
      setLight(gameInput, 0, 0, 0);
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
  

void response(vector<int> *gameInput) {
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
