//define sharp IR sensor select pins
#define IRS0 6
#define IRS1 7
#define IRS2 8
//define sharp IR input pin
#define IRIN 0

void setup() {
  Serial.begin(9600);
  pinMode(IRS0, OUTPUT);
  pinMode(IRS1, OUTPUT);
  pinMode(IRS2, OUTPUT);
}

/* Set the IR multiplexer select pins corresponding to the sensor we wish to read */
void writeSelectPins(int num){
  switch(num){
    case 0: digitalWrite(IRS0, LOW);
            digitalWrite(IRS1, LOW);
            digitalWrite(IRS2, LOW);
            break;
    case 1: digitalWrite(IRS0, HIGH);
            digitalWrite(IRS1, LOW);
            digitalWrite(IRS2, LOW);
            break;    
    case 2: digitalWrite(IRS0, LOW);
            digitalWrite(IRS1, HIGH);
            digitalWrite(IRS2, LOW);
            break;
    case 3: digitalWrite(IRS0, HIGH);
            digitalWrite(IRS1, HIGH);
            digitalWrite(IRS2, LOW);
            break;
    case 4: digitalWrite(IRS0, LOW);
            digitalWrite(IRS1, LOW);
            digitalWrite(IRS2, HIGH);
            break;
    case 5: digitalWrite(IRS0, HIGH);
            digitalWrite(IRS1, LOW);
            digitalWrite(IRS2, HIGH);
            break;
    case 6: digitalWrite(IRS0, LOW);
            digitalWrite(IRS1, HIGH);
            digitalWrite(IRS2, HIGH);
            break;
    case 7: digitalWrite(IRS0, HIGH);
            digitalWrite(IRS1, HIGH);
            digitalWrite(IRS2, HIGH);
            break;
    default:digitalWrite(IRS0, LOW);
            digitalWrite(IRS1, LOW);
            digitalWrite(IRS2, LOW);
            break;
  }
}

int readIRSensor(int sensorNum){

  writeSelectPins(sensorNum);
  
  float val = analogRead(IRIN) * 0.0048828125;

  return (int)(13 * pow(val, -1));
}

void loop() {
  
  for(int i = 0; i < 8; i++){
    Serial.print(i);
    Serial.print(": ");
    Serial.print(readIRSensor(i));
    Serial.print(" -- ");
  }
  Serial.println();
  delay(100);
}
