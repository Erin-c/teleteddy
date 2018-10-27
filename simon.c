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