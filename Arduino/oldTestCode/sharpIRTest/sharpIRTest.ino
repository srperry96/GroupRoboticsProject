static const uint8_t sensorPins[] = {0, 1, 2, 3, 4};

void setup() {
  Serial.begin(9600);
}

void loop() {
  static float voltages[5];
  static int distances[5];
  static char tempPrint[6];
  
  for(int i = 0; i < 5; i++){
    voltages[i] = analogRead(sensorPins[i]) * 0.0048828125;    
    distances[i] = 13 * pow(voltages[i], -1);
    Serial.print("Sensor ");
    Serial.print(i);
    sprintf(tempPrint, ": %5d; ", distances[i]);
    Serial.print(tempPrint);
  }

  Serial.println();
  delay(100);
}
