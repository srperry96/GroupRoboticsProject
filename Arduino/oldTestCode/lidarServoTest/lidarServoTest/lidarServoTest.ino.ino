#include <DynamixelSerial1.h> //AX12A servo library

void setup() {
  Serial.begin(9600);
  Dynamixel.begin(1000000, 2); //Initialise servo at 1Mbps. Control pin 2
  delay(1000); //give the servo some time to set itself up
}

void loop() {
  for(int i = -45; i <= 45; i++){
    moveToPosDegrees(i); 
    delay(10);
  }

  for(int i = 45; i > -45; i--){
    moveToPosDegrees(i);
    delay(10);
  }
  
}

void moveToPosDegrees(float posDegrees){
  //give degrees in range -150 to +150
  //(zero point is at 150 degrees (512))
  //calulation adds 150 to correct for range, then scales
  double pos;
  
  pos = ((posDegrees + 150) / 300) * 1024;
  
  //ensure value is within limits
  if(pos >= 1024) pos = 1023;
  else if(pos < 0) pos = 0;

  //tell servo to move
  Dynamixel.move(1, (int)pos);
  delay(10);

}
