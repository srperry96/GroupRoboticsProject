/* ROS node which handles the whole process of grabbing the teddy. First it lines up,
then moves towards it, then grabs it. This relies on being able to take control from
the highlevelController node to perform the sequence, then give control back once the
bear is grabbed.
Written by Samuel Perry */

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Empty.h"

//resolution (width) of the camera on the ground robot
#define CAMERARESX 480

//gains for lining up with the bear, and moving to the right distance to grab it
float kLineUp = 3.0, kDist = 0.05;
//most recent laser reading received
int lastLaserReading = 0;
//flag for whether or not the camera sees green (the teddys jumper)
int seeGreen = 0;

//state machine state
int currentState = 1;
//count used for timing in the state machine (allows the gripper time to move)
int gripperCount = 0;
//size of the teddy blob when gripped. If this decreases significantly, we have dropped the bear
float teddySizeWhenGripped = 0;

//ros publishers for robot movement, gripper commands, taking control and requesting laser readings
ros::Publisher movementPublisher;
ros::Publisher gripperInstructionsPublisher;
ros::Publisher takeControlPublisher;
ros::Publisher laserRequestReadingPublisher;

/* Function which informs the highlevel controller who should be in control. This
function either takes or relinquishes control depending on the value of controlState.
0 is no teddy, 1 is found teddy so initiating grip sequence, 2 is have teddy in grip */
void takeControl(int controlState){
  std_msgs::Int8 ctrlMsg;
  //give up control
  if(controlState == 0){
    ctrlMsg.data = 0;
    seeGreen = 0;
    //stop any movement that may have been occuring
    geometry_msgs::Twist stopMoving;
    stopMoving.angular.x = 0; stopMoving.angular.y = 0; stopMoving.angular.z = 0;
    stopMoving.linear.x = 0; stopMoving.linear.y = 0; stopMoving.linear.z = 0;
    movementPublisher.publish(stopMoving);
  //else take control so grip sequence can begin
}else if(controlState == 1){
    ctrlMsg.data = 1;
    seeGreen = 1;
  //else give up control while informing the highlevel controller that the teddy is in our grasp
  }else{
    ctrlMsg.data = 2;
    seeGreen = 1;
  }

  //publish the control state message
  takeControlPublisher.publish(ctrlMsg);
}

/* Simple proportional controller for turning on the spot to line up with the teddy */
void lineUpTeddyController(float xPos){
  geometry_msgs::Twist ctrlMsg;
  float error;

  //If the centre of the bear is in the middle 60 pixels of the image, that is lined up
  if((xPos > -30) && (xPos < 30)){
    error = 0;
  }else{
    error = -(xPos / CAMERARESX);
  }
  //publish control message
  ctrlMsg.angular.z = error * kLineUp;
  ctrlMsg.angular.x = 0; ctrlMsg.angular.y = 0;
  ctrlMsg.linear.x = 0; ctrlMsg.linear.y = 0; ctrlMsg.linear.z = 0;
  movementPublisher.publish(ctrlMsg);
}

/* Two simple proportional controllers working together. One controls the distance from
the bear, the other is the same as lineUpTeddyController to ensure we stay aligned as
we move towards the bear */
void moveToTeddyController(float xPos){
  geometry_msgs::Twist ctrlMsg;
  float errorDist, errorAngle;

  //ideal distance is 16cm, so control to near enough there
  if((lastLaserReading > 14) && (lastLaserReading < 18)){
    errorDist = 0;
  }else{
    errorDist = lastLaserReading - 16;
  }

  //ensure we are always aligned with the bear while moving
  if((xPos > -30) && (xPos < 30)){
    errorAngle = 0;
  }else{
    errorAngle = -(xPos / CAMERARESX);
  }

  //publish commands for linear speed using distance controller and angular speed using lineUpTeddyController
  ctrlMsg.linear.x = errorDist * kDist;
  ctrlMsg.angular.z = errorAngle * kLineUp;
  ctrlMsg.linear.y = 0; ctrlMsg.linear.z = 0;
  ctrlMsg.angular.x = 0; ctrlMsg.angular.y = 0;
  movementPublisher.publish(ctrlMsg);
}

/* Callback function for the position of the teddy blob from the blob detection
node. This function runs a state machine for the process of picking up the teddy,
First, we line up with it, then move towards it, then pick it up, then give control
back to the highlevel controller. The robot checks if the bear is still in its grip,
if it is lost, the state machine restarts.
This function runs at ~30Hz since the camera is running at ~30fps. This is used for
timing in two of the states. */
void teddyPosCallback(const geometry_msgs::Point::ConstPtr& msg){
std_msgs::Int8 gripCommand;

  //if we can see green, we must be in control (instead of the highlevel controller)
  if(seeGreen == 1){
    switch(currentState){
      //line up teddy state (-CAMERARESX/2 so the centre of the screen is at 0 (nicer to write controllers for))
      case 1: lineUpTeddyController(msg->x - (CAMERARESX / 2));
              //if teddy is lined up, we can move on to the next state
              if(((msg->x - (CAMERARESX / 2)) > -30) && ((msg->x - (CAMERARESX / 2)) < 30)){
                currentState = 2;
              }
              break;

      //move to within grabbing distance of teddy
      case 2: moveToTeddyController(msg->x - (CAMERARESX / 2));
              //if we're in range for grabbing the teddy, move to next state
              if((lastLaserReading < 18) && (lastLaserReading > 14)){
                currentState = 3;
              }
              break;
      //grip teddy state
      case 3: //tell the gripper to grip once
              if(gripperCount == 0){
                gripCommand.data = 2; //grip teddy on floor
                gripperInstructionsPublisher.publish(gripCommand);
              //if we have waited long enough for the gripper to grip (8 seconds)
              //~30fps video feed means a delay of 8 seconds requires a count timeout of 30 * 8 = 240
              }else if(gripperCount > 240){
                teddySizeWhenGripped = msg->z;//get the size of teddy blob for checking later
                gripperCount = 0;//reset gripperCount
                currentState = 4;//move to the next state
                break;//break here so gripperCount doesnt increase again
              }
              gripperCount++;
              break;

      //give back control, informing the high level controller that the teddy is gripped
      case 4: takeControl(2);
              currentState = 5;
              break;

      //checking we still have the bear state
              //if laser reading is too long, or if size of teddy blob is significantly smaller,
              //we have probably dropped the bear
      case 5: if((lastLaserReading > 30) || (msg->z < (teddySizeWhenGripped / 4))){
                currentState = 6;
                takeControl(1);
                printf("Lost the bear!\n");
              }
              break;
      //reset gripper as we have lost the teddy
      case 6: if(gripperCount == 0){ //publish grip command once
                gripCommand.data = 0;
                gripperInstructionsPublisher.publish(gripCommand);
              }else if(gripperCount > 240){ //count up for a while before moving on (gives gripper time to move)
                currentState = 1;
                gripperCount = 0;
                printf("Gripper reset. Restarting the pickup process\n");
                break;
              }
              gripperCount++;
              break;
    }
  }
}

/* Callback function for when a new laser reading is receieved. Simply updates
the lastLaserReading value */
void laserReadingCallback(const std_msgs::Int16::ConstPtr& msg){
//ERROR SOMEWHERE IN LASER READING CODE MEANS !)$ COMES OUT A LOT. IGNORE THAT
  if(msg->data != 104){
    lastLaserReading = msg->data;
  }
}

/* Callback for when green is detected. Takes or gives up control correspondingly */
void seeGreenCallback(const std_msgs::Int8::ConstPtr& msg){
  //if green is not being detected
  if(msg->data == 0){
    //if previously, we were seeing green but have lost it, relinquish control and reset the state machine
    if(seeGreen == 1){
      takeControl(0);
      currentState = 1;
    }
  //else green is being detected
  }else{
    //if previously we were not seeing green and now we are, take control and ensure the state machine starts in state 1
    if(seeGreen == 0){
      takeControl(1);
      currentState = 1;
    }
  }
}

/* Periodically request a laser distance reading by publishing to the corresponding topic */
void laserReadingsTimerCallback(const ros::TimerEvent&){
  std_msgs::Empty emptyMsg;
  laserRequestReadingPublisher.publish(emptyMsg);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "groundRobotGetTeddyNode");

  ros::NodeHandle nh;

  //Subscriber for the biggest blob position on screen (should be the teddy) - x, y is centre coord and z is size of blob
  ros::Subscriber teddyPosSubscriber = nh.subscribe("/camera/BiggestBlob", 10, teddyPosCallback);

  //Subscriber for readings from the laser distance sensor
  ros::Subscriber laserReadingSubscriber = nh.subscribe("/groundRobot/LaserReadings", 10, laserReadingCallback);

  //Subscriber for the seeGreen topic, so we know if we can see the bear or not
  ros::Subscriber seeGreenSubscriber = nh.subscribe("/camera/SeeGreen", 10, seeGreenCallback);

  //Timer used to poll the laser distance sensor at a rate of 4Hz
  ros::Timer laserReadingsTimer = nh.createTimer(ros::Duration(0.25), laserReadingsTimerCallback);

  //Publisher to tell the highlevelController to give up control, as we can see the bear
  takeControlPublisher = nh.advertise<std_msgs::Int8>("/groundRobot/TakeControl", 10);

  //Publisher to request laser sensor distance readings
  laserRequestReadingPublisher = nh.advertise<std_msgs::Empty>("/groundRobot/RequestLaserReading", 10);

  //Publisher for movement of the ground robot wheels
  movementPublisher = nh.advertise<geometry_msgs::Twist>("/groundRobot/MovementControl", 10);

  //Publisher for telling the gripper to carry out a movement
  gripperInstructionsPublisher = nh.advertise<std_msgs::Int8>("/groundRobot/GripperInstructions", 10);

  ros::spin();

  return 0;
}
