/* An old version of the main Arduino code. Included as there was significant
code written to work with the LIDAR which was scrapped late in the project.
Written by Samuel Perry */

#include "lidar.h"
#include "irFuncs.h"
#include "gripper.h"

//The LIDAR
Lidar* lidar;

//timing variables used for polling ir sensors 5 times a second
unsigned long startMillis;
unsigned long currentMillis;

//angle increment used in the most recent LIDAR scan
uint8_t lastScanIncrement = 1;


//variable to keep track of what SPI function is taking place. 'z' is no operation
char currentOperation = 'z';
//marker for keeping track of where we are in an SPI instruction
int marker = 0;

//angle for LIDAR to move to in single scan mode
uint16_t desiredAngle = 0;


/* Write a single char to the SPI interface */
void spiWriteChar(char data) {
  SPDR = data;
}

void setup (void) {
  //start serial for usb comms
  Serial.begin(9600);

  lidar = new Lidar();

  //Setup SPI (in slave mode)
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);

  irSetupMultiplexerPins();

  //setup the AX12A servos of the gripper/arm
  setup_gripper();

  //get start value for timing count used for IR polling
  startMillis = millis();

}

/* Loop until SPI flag (SPIF) is set, which means a byte has been received,
then do something with that byte. Also, poll IR sensors every 200ms */
void loop (void) {

  //update timer count
  currentMillis = millis();

  //read ir sensors every 200ms
  if (currentMillis - startMillis >= 200) {
    irReadSensors();
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

  //This switch statement handles all the functionality for each message type. 'currentOperation' and 'marker'
  //are used to track what operation is being carried out, and how far through the operation we are. They are reset
  //at the end of each operation so the function does not repeat.
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
                if (marker >= 8) {
                  currentOperation = 'z';
                  marker = 0;             //reset marker so the next operation starts in the right place
                  break;                  //break here so marker doesnt increase to 1
                }
              }
              marker++; //increase marker so we move onto the next section of the operation
              break;

    //LIDAR - trigger a full scan using a given angle increment
    case 'c': if(marker == 0){
                spiWriteChar('c');
                marker++;
              }else{
                lastScanIncrement = (uint8_t)SPDR; //get increment size from SPI
                lidar->fullScan(lastScanIncrement);//do a full lidar scan at the given increment
                //NOTE: The LIDAR scan stops the arduino doing anything else at the same time
                //NOTE: Raspberry Pi must wait around 5 seconds for the scan to finish before asking for the new readings
                currentOperation = 'z';
                marker = 0;
              }
              break;

    //LIDAR - get most recent full set of readings
    case 'd': if(marker == 0){
                spiWriteChar('d');
                marker++;
              }else{
                spiWriteChar((char)lidar->scan[marker - 1]); //write next LIDAR scan value to SPI
                marker += lastScanIncrement;                 //increase marker by increment size so only new data is transmitted
                if(marker > 360){
                  currentOperation = 'z';
                  marker = 0;
                }
              }
              break;

    //LIDAR - go to angle
    case 'e': if(marker == 0){
                spiWriteChar('e');
                marker++;
              }else if(marker == 1){ //angle is 16 bits, so is split and transmitted as two 8 bit values
                desiredAngle = ((uint8_t)SPDR) << 8; //get upper 8 bits of angle, shift up 8 bits
                marker++;
              }else if(marker == 2){
                desiredAngle += (uint8_t)SPDR; //add lower 8 bits to get actual angle value
                marker++;
              }else if(marker == 3){
                spiWriteChar('w');//write w for waiting (pi pushes through zeros here until movement is complete)
                lidar->turnToAngle(desiredAngle); //tell lidar to move to the angle
                if(lidar->moving != true){ //if movement is finished, continue
                  marker++;
                }
              }else if(marker == 4){
                spiWriteChar('x'); //transmit an 'x' to show we have reached the correct angle, then reset markers
                marker = 0;
                currentOperation = 'z';
              }
              break;

    //LIDAR - trigger single scan (assume we're already at the correct angle)
    case 'f': if(marker == 0){
                spiWriteChar('f');
                marker++;
              }else if(marker == 1){
                lidar->singleScan((uint8_t)SPDR); //start a single scan using sensor defined by SPDR value
                marker = 0;
                currentOperation = 'z';
              }
              break;

    //LIDAR - send single scan reading over SPI
    case 'g': if(marker == 0){
                spiWriteChar('g');
                marker++;
              }else if(marker == 1){
                spiWriteChar((char)lidar->singleScanReading); //transmit single scan reading value over SPI
                marker = 0;
                currentOperation = 'z';
              }
              break;

    //Set LIDAR range - long or short
    case 'h': if(marker == 0){
                spiWriteChar('h');//ack byte
                marker++;
              }else{
                //set LIDAR range accordingly
                if((uint8_t)SPDR == 1){
                  lidar->setLongRange(false);
                }else if((uint8_t)SPDR == 2){
                  lidar->setLongRange(true);
                }
                currentOperation = 'z';
                marker = 0;
              }
              break;

    //Set LIDAR time budget
    case 'i': if(marker == 0){
                spiWriteChar('i');//ack
                marker++;
              }else{
                //set LIDAR time budget as given value *10^3
                lidar->setTimingBudget((uint8_t)SPDR * 1000);
                marker = 0;
                currentOperation = 'z';
              }
              break;

    //Gripper - grip high
    case 'j': if(marker == 0){
                spiWriteChar('j');
                marker++;
              }else{
                carTeddy();
                marker = 0;
                currentOperation = 'z';
              }
              break;

    //Gripper - grip low
    case 'k': if(marker == 0){
                spiWriteChar('k');
                marker++;
              }else{
                floorTeddy();
                marker = 0;
                currentOperation = 'z';
              }
              break;

    //Gripper - reset
    case 'l': if(marker == 0){
                spiWriteChar('l');
                marker++;
              }else{
                reset_gripper();
                marker = 0;
                currentOperation = 'z';
              }
              break;

    //Gripper - tilt the camera down
    case 'm': if(marker == 0){
                spiWriteChar('m');
                marker++;
              }else{
                camera_down();
                marker = 0;
                currentOperation = 'z';
              }
              break;

    //Gripper - tilt camera up
    case 'n': if(marker == 0){
                spiWriteChar('n');
                marker++;
              }else{
                camera_center();
                marker = 0;
                currentOperation = 'z';
              }
              break;

    //default operation just resets the markers. This shouldnt ever be reached, but may be if the SPI interface receives erroneous data/noise
    //and interprets it as a message
    default:  currentOperation = 'z'; //
              marker = 0;
              break;
  }
}
