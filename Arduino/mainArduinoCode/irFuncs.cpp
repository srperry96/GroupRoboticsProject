/* Functions for using the IR sensors on the ground robot.
Written by Samuel Perry */

#include "irFuncs.h"

//Array holding the most recent set of IR readings
uint8_t irDistances[8] = {0,0,0,0,0,0,0,0};

/* Set up the three MUX select pins as outputs */
void irSetupMultiplexerPins(){
    //setup IR multiplexer select pins
    pinMode(IRS0, OUTPUT);
    pinMode(IRS1, OUTPUT);
    pinMode(IRS2, OUTPUT);
}

/* Set MUX select pins for a given sensor, take a reading and scale to get a distance value in cm from
the voltage level reading. Distance calculation taken from datasheet of Sharp IR sensor */
uint8_t irReadSingleSensor(int sensorNum){

  //set mux pins for the sensor being read
  irWriteSelectPins(sensorNum);

  //take a voltage reading, calculate and return the corresponding distance
  float val = analogRead(IRIN) * 0.0048828125;
  return (uint8_t)(13 * pow(val, -1));
}

/* Loop through all 8 IR sensors taking readings from each one */
void irReadSensors(){
  for(int i = 0; i < 8; i++){
    irDistances[i] = irReadSingleSensor(i);
  }
}

/* Set the IR multiplexer select pins corresponding to the sensor we wish to read */
void irWriteSelectPins(int num){
  //select pins 0,1,2 are bits 0,1,2 in binary
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
