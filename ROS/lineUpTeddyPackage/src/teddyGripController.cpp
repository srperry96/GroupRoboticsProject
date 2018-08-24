#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Empty.h"

#define CAMERARESX 480

float kLineUp = 3.0, kDist = 0.05;
int currentState = 1;
int bearHeight = 2; //2 is ground, 1 is on top of car
int lastLaserReading = 0;
int seeGreen = 0;


ros::Publisher movementPublisher;
ros::Publisher gripperInstructionsPublisher;
ros::Publisher takeControlPublisher;
ros::Publisher laserRequestReadingPublisher;

void takeControl(int controlState){
  std_msgs::Int8 ctrlMsg;
  //give up control
  if(controlState == 0){
    ctrlMsg.data = 0;
    takeControlPublisher.publish(ctrlMsg);
    seeGreen = 0;
    //stop any movement that may have been occuring
    geometry_msgs::Twist stopMoving;
    stopMoving.angular.x = 0; stopMoving.angular.y = 0; stopMoving.angular.z = 0;
    stopMoving.linear.x = 0; stopMoving.linear.y = 0; stopMoving.linear.z = 0;
    movementPublisher.publish(stopMoving);
  //else take control
  }else{
    ctrlMsg.data = 1;
    takeControlPublisher.publish(ctrlMsg);
    seeGreen = 1;
  }
}

void lineUpTeddyController(float xPos){
  geometry_msgs::Twist ctrlMsg;
  float error;

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

void moveToTeddyController(float xPos){
  geometry_msgs::Twist ctrlMsg;
  float errorDist, errorAngle;

  if((lastLaserReading > 14) && (lastLaserReading < 18)){
    errorDist = 0;
  }else{
    errorDist = lastLaserReading - 16;
  }

  if((xPos > -30) && (xPos < 30)){
    errorAngle = 0;
  }else{
    errorAngle = -(xPos / CAMERARESX);
  }

  ctrlMsg.linear.x = errorDist * kDist;
  ctrlMsg.linear.y = 0; ctrlMsg.linear.z = 0;
  ctrlMsg.angular.x = 0; ctrlMsg.angular.y = 0;
  ctrlMsg.angular.z = errorAngle * kLineUp;
  movementPublisher.publish(ctrlMsg);
}

int gripperCount = 0;
float teddySizeWhenGripped = 0;

void teddyPosCallback(const geometry_msgs::Point::ConstPtr& msg){
std_msgs::Int8 gripCommand;

  //if we can see green, we must be in control (not the highlevel controller)
  if(seeGreen == 1){
    switch(currentState){
      //line up teddy state (- CAMERARESX/2 so the centre of the scree )
      case 1: lineUpTeddyController(msg->x - (CAMERARESX / 2));
              //if teddy is lined up, we can move on
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
      case 3: gripCommand.data = 2; //grip teddy on floor
              gripperInstructionsPublisher.publish(gripCommand);
              currentState = 4;
              break;
      //wait for gripper command to finish
      case 4: gripperCount++;
              //~30fps means a delay of 8 seconds requires count timeout of 30 * 8 = 240
              if(gripperCount > 240){
                currentState = 5;
                teddySizeWhenGripped = msg->z;
                gripperCount = 0;
              }
              break;
      //give back control, but keep seeGreen as 1 since we have the bear
      case 5: takeControl(0);
              seeGreen = 1;
              currentState = 6;
              break;

      //checking we still have the bear state
              //if laser reading is too long, or if size of teddy blob is significantly smaller,
              //we have probably dropped the bear
      case 6: if((lastLaserReading > 30) || (msg->z < (teddySizeWhenGripped / 4))){
                currentState = 7;
                takeControl(1);
                printf("Lost the bear!\n");
              }
              break;
      //reset gripper
      case 7: if(gripperCount == 0){
                gripCommand.data = 0;
                gripperInstructionsPublisher.publish(gripCommand);
              }else if(gripperCount > 240){
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
    //if previously, we were seeing green but have lost it, give back control to highlevel controller
    if(seeGreen == 1){
      takeControl(0);
    }
  //else green is being detected
  }else{
    //if previously we were not seeing green, take control from highlevel controller
    if(seeGreen == 0){
      takeControl(1);
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

  //state machine subscriber
  ros::Subscriber rescueTeddySubscriber = nh.subscribe("", 10, rescueTeddyCallback);
  //biggest blob position on screen - x, y is centre coord and z is size of blob
  ros::Subscriber teddyPosSubscriber = nh.subscribe("/camera/BiggestBlob", 10, teddyPosCallback);

  ros::Subscriber laserReadingSubscriber = nh.subscribe("/groundRobot/LaserReadings", 10, laserReadingCallback);

  ros::Subscriber seeGreenSubscriber = nh.subscribe("/camera/SeeGreen", 10, seeGreenCallback);

  //Timer used to poll the laser distance sensor at a rate of 4Hz when we can see green
  ros::Timer laserReadingsTimer = nh.createTimer(ros::Duration(0.25), laserReadingsTimerCallback);

  takeControlPublisher = nh.advertise<std_msgs::Int8>("/groundRobot/TakeControl", 10);

  laserRequestReadingPublisher = nh.advertise<std_msgs::Empty>("/groundRobot/RequestLaserReading", 10);

  movementPublisher = nh.advertise<geometry_msgs::Twist>("/groundRobot/MovementControl", 10);
  gripperInstructionsPublisher = nh.advertise<std_msgs::Int8>("/groundRobot/GripperInstructions", 10);

  ros::spin();

  return 0;
}
