#include <DynamixelSerial1.h> //AX12a servo library

void setup() {
  Dynamixel.begin(1000000, 4);
  Serial.begin(9600);
}

void loop() {
 /* Dynamixel.move(1, 200);
  delay(500);
  Dynamixel.move(2,300);
  delay(500);
  Dynamixel.move(3, 22);
  delay(500);
  Dynamixel.move(4, 46);
  delay(500);
  */
  Serial.println(Dynamixel.readPosition(1));
  Serial.println(Dynamixel.readPosition(2));
  Serial.println(Dynamixel.readPosition(3));
  Serial.println(Dynamixel.readPosition(4));
  delay(500);
}
