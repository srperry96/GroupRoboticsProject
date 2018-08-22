//motor pin and direction defines
#define leftDirectionPin 7
#define rightDirectionPin 8
#define leftPWMPin 9
#define rightPWMPin 10
#define FORWARD HIGH
#define BACKWARD LOW

void setup() {
  Serial.begin(9600);

  //setup motor driver pins
  pinMode(leftDirectionPin, OUTPUT); 
  pinMode(rightDirectionPin, OUTPUT);
  pinMode(leftPWMPin, OUTPUT);
  pinMode(rightPWMPin, OUTPUT);
}

void loop() {

  
  //right motor forward full speed
  analogWrite(rightPWMPin, 100);
  digitalWrite(rightDirectionPin, FORWARD);
/*  
  //left motor forward full speed
  analogWrite(leftPWMPin, 255);
  digitalWrite(leftDirectionPin, FORWARD);
  
  //right motor backward full speed
  analogWrite(rightPWMPin, 255);
  digitalWrite(rightDirectionPin, BACKWARD);
  
  //left motor backward full speed
  analogWrite(rightPWMPin, 255);
  digitalWrite(rightDirectionPin, BACKWARD);

  //stop both
  analogWrite(rightPWMPin, 0);
  analogWrite(leftPWMPin, 0);

*/


}
