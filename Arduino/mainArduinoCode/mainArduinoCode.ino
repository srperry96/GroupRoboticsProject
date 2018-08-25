/* The main arduino code for the ground robot. Sets up sensors and gripper and
handles messages from the Raspberry Pi through SPI.
Written by Samuel Perry */

#include "laserSensor.h"
#include "irFuncs.h"
#include "gripper.h"


//timing variables used for polling ir sensors and laser 5 times a second
unsigned long startMillis;
unsigned long currentMillis;

//variable to keep track of what SPI operation is taking place. 'z' is no operation
char currentOperation = 'z';

//marker for keeping track of where we are in an SPI instruction
int marker = 0;


/* Write a single char to the SPI interface */
void spiWriteChar(char data) {
  SPDR = data;
}

void setup (void) {
  //start serial for usb comms
  Serial.begin(9600);

  //setup SPI (in slave mode)
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);

  //setup IR mux
  irSetupMultiplexerPins();

  //setup laser time of flight distance sensor
  laserSetup();

  //setup the AX12A servos of the gripper/arm
  setup_gripper();

  //get start value for timing count used for IR polling
  startMillis = millis();
}

/* Loop until SPI flag (SPIF) is set, which means a byte has been received,
then do something with that byte. Also, poll IR sensors and laser every 200ms */
void loop (void) {

  //update timer count
  currentMillis = millis();

  //read ir sensors and laser every 200ms
  if (currentMillis - startMillis >= 200) {
    irReadSensors();
    laserGetMeasurement();
    startMillis = currentMillis;
  }

  //if SPI message has been receieved, process it
  if ((SPSR & (1 << SPIF)) != 0){
    spiHandler();
  }
}

/* Handler for any SPI message that is receieved. Calls the corresponding function for whatever message or series
of messages that are received */
void spiHandler(){
  //if not currently in an operation, set the currentOperation value to the byte received
  if (currentOperation == 'z'){
    currentOperation = SPDR;
  }

  /* This switch statement handles all the functionality for each message type. 'currentOperation' and 'marker'
  are used to track what operation is being carried out, and how far through the operation we are. They are reset
  at the end of each operation so the function does not repeat. Much of the code works the same for different cases so
  comments are only written for the first occurence of a process. */
  switch (currentOperation){
    //Startup handshake (SPI connection test)
    case 'a': spiWriteChar('a'); //Send acknowledge byte
              currentOperation = 'z'; //reset current operation so this doesnt repeat infinitely
              break;

    //Transmit the most recent set of IR readings via SPI
    case 'b': if(marker == 0){      //first part is the acknowledgement
                spiWriteChar('b');
              }else{                //next we send the data one byte at a time (this repeats 8 times, once for each IR sensor)
                spiWriteChar((char)irDistances[marker - 1]);
                if (marker >= 8) {    //once 8 bytes of data have been sent, we exit this SPI function
                  currentOperation = 'z';
                  marker = 0;             //reset marker so the next operation starts in the right place
                  break;                  //break here so marker doesnt increase to 1
                }
              }
              marker++; //increase marker so we move onto the next section of the operation
              break;

    //Gripper - grip high
    case 'c': if(marker == 0){
                spiWriteChar('c');
                marker++;
              }else{
                carTeddy(); //call the function for gripping the teddy when it is on top of the car
                marker = 0;
                currentOperation = 'z';
              }
              break;

    //Gripper - grip low
    case 'd': if(marker == 0){
                spiWriteChar('d');
                marker++;
              }else{
                floorTeddy(); //call the function for gripping the teddy when it is on the ground
                marker = 0;
                currentOperation = 'z';
              }
              break;

    //Gripper - reset
    case 'e': if(marker == 0){
                spiWriteChar('e');
                marker++;
              }else{
                reset_gripper(); //call the function to reset the gripper to its start position
                marker = 0;
                currentOperation = 'z';
              }
              break;

    //Gripper - tilt camera down
    case 'f': if(marker == 0){
                spiWriteChar('f');
                marker++;
              }else{
                camera_down(); //call the function to move the arm so the camera is tilted down
                marker = 0;
                currentOperation = 'z';
              }
              break;

    //Gripper - tilt camera center
    case 'g': if(marker == 0){
                spiWriteChar('g');
                marker++;
              }else{
                camera_center(); // call the function to move the arm so the camera is central
                marker = 0;
                currentOperation = 'z';
              }
              break;

    //Laser - get most recent measurement
    case 'h': if(marker == 0){
                spiWriteChar('h');
                marker++;
              }else if(marker == 1){
                spiWriteChar((char)lastLaserReading); //send the most recent laser reading to the Pi via SPI
                marker = 0;
                currentOperation = 'z';
              }
              break;

    //default case just resets the markers. This shouldnt ever be reached, but may be if the SPI interface receives erroneous data/noise
    //and interprets it as a message
    default:  currentOperation = 'z'; //
              marker = 0;
              break;
  }
}
