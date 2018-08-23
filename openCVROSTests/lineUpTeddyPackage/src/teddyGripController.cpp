#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int8.h"

#define CAMERARESX 480

float kLineUp = 3.0, kDist = 0.01;
int currentState = 1;
int bearHeight = 2; //2 is ground, 1 is on top of car

ros::Publisher movementPublisher;
ros::Publisher gripperInstructionsPublisher;


void teddyPosCallback(const geometry_msgs::Point::ConstPtr& msg){
  geometry_msgs::Twist ctrlMsg;
  float error;

  //if we are in the line up with teddy state
  if(currentState == 1){

    if((msg->x > -30) && (msg->x < 30)){
      error = 0;
    }else{
      error = -(msg->x / CAMERARESX);
    }
    //publish control message
    ctrlMsg.angular.z = error * kLineUp;
    ctrlMsg.angular.x = 0; ctrlMsg.angular.y = 0;
    ctrlMsg.linear.x = 0; ctrlMsg.linear.y = 0; ctrlMsg.linear.z = 0;
    movementPublisher.publish(ctrlMsg);
  }else if(currentState == 4){
    if(msg->z >= 24){
      error = msg->z - 18;
    }else{
      error = 0;
    }
    //publish control message
    ctrlMsg.linear.x = error * kDist;
    ctrlMsg.angular.x = 0; ctrlMsg.angular.y = 0; ctrlMsg.angular.z = 0;
    ctrlMsg.linear.y = 0; ctrlMsg.linear.z = 0;
    movementPublisher.publish(ctrlMsg);
  }else if(currentState == 2){
    if((msg->z > 35) && (msg->z < 40)){
      error = 0;
    }else{
      error = msg->z - 37;
    }
    //publish control message
    ctrlMsg.linear.x = error * kDist;
    ctrlMsg.angular.x = 0; ctrlMsg.angular.y = 0; ctrlMsg.angular.z = 0;
    ctrlMsg.linear.y = 0; ctrlMsg.linear.z = 0;
    movementPublisher.publish(ctrlMsg);
  }
}

void rescueTeddyCallback(const std_msgs::Int8::ConstPtr& msg){

  std_msgs::Int8 gripCommand;

  switch(msg->data){
    case 0: currentState = 0;
            gripCommand.data = 0;
            gripperInstructionsPublisher.publish(gripCommand);
    //line up teddy state
    case 1: currentState = 1;
            //DEACTIVATE HIGHLEVEL CONTROLLER
            break;
    //move to correct distance so camera angle can be set
    case 2: currentState = 2;
            break;

    //determine position of bear (on car or on ground)
    case 3: currentState = 3;
            break;
    //move forward until we are 18cm from the bear
    case 4: currentState = 4;
            break;
    //grab the bear
    case 5: currentState = 5;
            //Tell gripper to grip (high or low depending on bearHeight)
this bit doesnt run when it should
            gripCommand.data = bearHeight;
            gripperInstructionsPublisher.publish(gripCommand);
            printf("shouldve gripped\n");
            break;
    //drive home, checking the bear is still gripped
    case 6: currentState = 6;
            //TELL HIGHLEVEL CONTROLLER TO ACTIVATE AGAIN
            break;

    //teddy has been dropped. stop and return to state 1
    case 7: //BRAKE
            //DEACTIVATE HIGHLEVEL CONTROLLER
            gripCommand.data = 0;
            gripperInstructionsPublisher.publish(gripCommand);
            currentState = 1;
            break;

    //bear is on the car
    case 10:  bearHeight = 1;
              gripCommand.data = 4;
              gripperInstructionsPublisher.publish(gripCommand);
              break;
    //bear is on the ground
    case 11:  bearHeight = 2;
              gripCommand.data = 3;
              gripperInstructionsPublisher.publish(gripCommand);
              break;
    default: break;
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "groundRobotGetTeddyNode");

  ros::NodeHandle nh;

  ros::Subscriber rescueTeddySubscriber = nh.subscribe("/grippercommand", 10, rescueTeddyCallback);
  ros::Subscriber teddyPosSubscriber = nh.subscribe("/groundRobot/SeeTeddy", 10, teddyPosCallback);

  movementPublisher = nh.advertise<geometry_msgs::Twist>("/groundRobot/MovementControl", 10);
  gripperInstructionsPublisher = nh.advertise<std_msgs::Int8>("/groundRobot/GripperInstructions", 10);

  ros::spin();

  return 0;
}
