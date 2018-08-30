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
float kLineUp = 1.2, kDist = 0.03;
//most recent laser reading received
int lastLaserReading = 0;
//flag for whether or not the camera sees green (the teddys jumper)
int seeGreen = 0;
//timer and flags used for ensuring the gripper has ample time to carry out its motion
int timerOn = 0, gripping = 0, gripperCount = 0;
//most recent x position of the teddy on screen
float teddyPosX = 0;
//counter for a timeout for the teddy x position (stops controller going mental if the bear goes out of view quickly)
int teddyPosCount = 0;
//state machine state
int currentState = 1;

//ros publishers for robot movement, gripper commands, taking control and requesting laser readings
ros::Publisher movementPublisher;
ros::Publisher gripperInstructionsPublisher;
ros::Publisher takeControlPublisher;
ros::Publisher laserRequestReadingPublisher;

/* Publishes to the movement topic telling the robot to stop all movement */
void stopMoving(){
  geometry_msgs::Twist stopMoving;
  stopMoving.angular.x = 0; stopMoving.angular.y = 0; stopMoving.angular.z = 0;
  stopMoving.linear.x = 0; stopMoving.linear.y = 0; stopMoving.linear.z = 0;
  movementPublisher.publish(stopMoving);
}

/* Function which informs the highlevel controller who should be in control. This
function either takes or relinquishes control depending on the value of controlState.
0 is no teddy, 1 is found teddy so initiating grip sequence, 2 is have teddy in grip */
void takeControl(int controlState){
  std_msgs::Int8 ctrlMsg;
  //give up control
  if(controlState == 0){
    printf("control 0\n");
    ctrlMsg.data = 0;
    seeGreen = 0;
    //stop any movement that may have been occuring
    stopMoving();
  //else take control so grip sequence can begin
  }else if(controlState == 1){
    ctrlMsg.data = 1;
        printf("control 1\n");
    seeGreen = 1;
  //else give up control while informing the highlevel controller that the teddy is in our grasp
  }else if(controlState == 2){
    ctrlMsg.data = 2;
    seeGreen = 1;
    printf("control 2\n");
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

/* Callback function for when a blob (representing the teddy) is detected and its data
received by the node. Updates a most recent position value and resets the timeout counter */
void teddyPosCallback(const geometry_msgs::Point::ConstPtr& msg){
  teddyPosX = msg->x;
  teddyPosCount = 0;
}

/* Callback function for when a new laser reading is receieved. Simply updates
the lastLaserReading value */
void laserReadingCallback(const std_msgs::Int16::ConstPtr& msg){
//if SPI connections is busy when a reading is requested, 104 (the start byte) is returned. Ignore the reading if this is the case
  if(msg->data != 104){
    lastLaserReading = msg->data;
  }
}

/* Callback for when green is detected. Takes or gives up control correspondingly */
void seeGreenCallback(const std_msgs::Int8::ConstPtr& msg){
  //if green is not being detected
  if(msg->data == 0){
    //if previously, we were seeing green but have lost it, relinquish control
    if(seeGreen == 1){
      //takeControl(0);
    }
  //else green is being detected
  }else{
    //if previously we were not seeing green and now we are, take control
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

/* Timer used to allow the gripper to carry out its motion. Runs at 4Hz. There are
two durations possible, the first is for gripping the teddy, the second is for resetting
the gripper to its start position. */
void gripperTimerCallback(const ros::TimerEvent&){
  //gripping the bear
  if(timerOn == 1){
    gripperCount++;
    //if time limit is reached, set the appropriate flag and reset the counter
    if(gripperCount > 25){
      timerOn = 0;
      gripperCount = 0;
    }
  //resetting the gripper
  }else if(timerOn == 2){
    gripperCount++;
    //if time limit is reached, set the appropriate flag and reset the counter
    if(gripperCount > 20){
      timerOn = 0;
      gripperCount = 0;
    }
  }
}

/* Timer callback function containing the state machine which handles the entire sequence of picking
up the teddy. The machine is started when green is detected. State 1 lines up the robot with the bear,
state two moves towards the bear while remaining aligned, state 3 grabs the bear, state 4 checks the
bear is still gripped as the robot returns home. State 5 resets the gripper if the bear is dropped. */
void stateMachineTimerCallback(const ros::TimerEvent&){
  std_msgs::Int8 gripCommand;

  teddyPosCount++;
  //if no update on teddy pos for half a second, assume its lost
  if(teddyPosCount > 10){
    teddyPosX = 0;
  }

    //if we can see green, we must be in control (instead of the highlevel controller)
    if(seeGreen == 1){
      switch(currentState){
        //line up teddy state (-CAMERARESX/2 so the centre of the screen is at 0 (nicer to write controllers for))
        case 1: lineUpTeddyController(teddyPosX - (CAMERARESX / 2));
                //if teddy is lined up, we can move on to the next state
                if(((teddyPosX - (CAMERARESX / 2)) > -30) && ((teddyPosX - (CAMERARESX / 2)) < 30)){
                  currentState = 2;
                  printf("state 2\n");
                }
                break;

        //move to within grabbing distance of teddy
        case 2: moveToTeddyController(teddyPosX - (CAMERARESX / 2));
                //if we're in range for grabbing the teddy, move to next state
                if((lastLaserReading < 18) && (lastLaserReading > 14)){
                  currentState = 3;
                  //stop the robot moving
                  stopMoving();
                  printf("state 3\n");
                }
                break;

        //grip teddy state
        case 3: //tell the gripper to grip once
                if(gripping == 0){
                  gripCommand.data = 2; //grip teddy on floor
                  gripperInstructionsPublisher.publish(gripCommand);
                  //start countdown timer for gripper movement
                  timerOn = 1;
                  gripping = 1;
                //if timer has finished
                }else if(gripping == 1){
                  if(timerOn == 0){
                    //give back control to high level controller
                    takeControl(2);
                    currentState = 4;
                    gripping = 0;
                    printf("state 4\n");
                  }
                }
                break;

        //checking we still have the bear state
                //if laser reading is too large we have probably dropped the bear
        case 4: if(lastLaserReading > 28){
                  currentState = 5;
                  //take control from high level controller
                  takeControl(1);
                  printf("state 5\n");
                }
                break;

        //reset gripper as we have lost the teddy
        case 5: if(gripping == 0){ //publish grip command once
                  gripCommand.data = 0;
                  gripperInstructionsPublisher.publish(gripCommand);
                  //set timer going
                  timerOn = 2;
                  gripping = 1;
                //if timer is finished, move on
                }else if(gripping == 1){
                  if(timerOn == 0){
                    printf("state 1\n");
                    gripping = 0;
                    currentState = 1;
                  }
                }
                break;
      }
    }
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

  //Timer used for ensuring the gripper has enough time to perform a movement
  ros::Timer gripperTimer = nh.createTimer(ros::Duration(0.25), gripperTimerCallback);

  //Timer running a state machine 20 times a second
  ros::Timer stateMachineTimer = nh.createTimer(ros::Duration(0.05), stateMachineTimerCallback);

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
