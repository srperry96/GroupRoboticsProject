/* Blob detection node for detecting the green of the teddy bear's jumper.
Publishes the location of the bear with regards to the camera for use in controllers.
Written by Samuel Perry (noiseRemoval() function written by Finlay Hudson)*/

#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int8.h>


using namespace cv;
namespace enc = sensor_msgs::image_encodings;

//Publisher for the details of the biggest blob detected
ros::Publisher biggestBlobPublisher;

//Publisher to tell the system if green is currently being detected
ros::Publisher seeGreenPublisher;
//Publisher for the processed video
image_transport::Publisher blobITPub;

//HSV values for filtering out everything but green (teddys jumper colour)
int lowGreenH  = 50, lowGreenS  = 50,  lowGreenV  = 44;
int highGreenH = 90, highGreenS = 161, highGreenV = 100;

/* Noise removal / filtering function - written by Finlay Hudson */
void noiseRemoval(Mat &mask)
{
  Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3,3), Point(-1, -1)); //Any white part thats less than 7x7 px's gets ignored as it is seen as noise
  Mat dilateElement = getStructuringElement(MORPH_RECT, Size(10,10)); //Fill in gaps in objects so they are solid

  //These work best when done twice
  //erode - removes any pixels too small (removing noise)
  erode(mask, mask, erodeElement);
  erode(mask, mask, erodeElement);

  //dilate - expands the remaing pixels, hopefully filling in the gaps to make a solid object
  dilate(mask, mask, dilateElement);
  dilate(mask, mask, dilateElement);
}

/* Get the ID of the largest blob in the keypoints vector - return -1 if no blobs are in the list */
int getBiggestBlob(const std::vector<KeyPoint>& keypoints){
  int currentLargest = 0;

  //if there are keypoints (blobs) in the list
  if(keypoints.size() > 0){
    //if there is more than one blob, we need to find the biggest
    if(keypoints.size() != 1){
      //loop through list comparing sizes of blobs, if bigger, change the value of currentLargest
      for(int i = 1; i < keypoints.size(); i++){
        if(keypoints[i].size > keypoints[currentLargest].size){
          currentLargest = i;
        }
      }
    }
    return currentLargest;
  //else there are no blobs in the list, so return -1
  }else{
    return -1;
  }
}

/* Callback function for every frame of video received. Translates to opencv format, then runs blob detection.
Calculates the biggest blob and publishes its location and size in a ros::Point message */
void blobDetectCallback(const sensor_msgs::ImageConstPtr& originalImage){
    std_msgs::Int8 seeGreenMsg;
    cv_bridge::CvImagePtr cv_ptr;

    //try to convert the frame of video to OpenCV format
    try{
      cv_ptr = cv_bridge::toCvCopy(originalImage, sensor_msgs::image_encodings::BGR8);
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("ROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
    //image converted from RGB to HSV
    Mat img_hsv;
    //filtered image (all green from the original is now white, everything else is black)
    Mat img_mask;

    //filter out all except the green of the teddys jumper, then convert to a black and white image where white represents green
    cvtColor(cv_ptr->image, img_hsv, CV_BGR2HSV);
    inRange(img_hsv, cv::Scalar(lowGreenH, lowGreenS, lowGreenV), cv::Scalar(highGreenH, highGreenS, highGreenV), img_mask);

    //vector of keypoints (blobs)
    std::vector<KeyPoint> keypoints;

    //setup blob detection parameters
    SimpleBlobDetector::Params params;
    params.filterByColor = true;
    params.blobColor = 255;     //only detect blobs of white
    params.filterByArea = true;
    params.minArea = 300;
    params.maxArea = 1000000000000;
    params.filterByConvexity = false;
    params.filterByCircularity = false;

    //remove noise from the black and white image
    noiseRemoval(img_mask);
    //create and run the blob detection process on the black and white image
    Ptr<cv::SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
    detector->detect(img_mask, keypoints);

    //create a new image which shows the black and white image and circles drawn around the detected blobs
    Mat imWithKeypoints;
    drawKeypoints(img_mask, keypoints, imWithKeypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    //get the ID of the biggest blob on screen
    int biggestBlobID = getBiggestBlob(keypoints);

    //if -1, no blobs detected. If blobs are detected, publish a point holding details of the biggest
    if(biggestBlobID != -1){
      geometry_msgs::Point biggestBlob;
      biggestBlob.x = keypoints[biggestBlobID].pt.x;
      biggestBlob.y = keypoints[biggestBlobID].pt.y;
      biggestBlob.z = keypoints[biggestBlobID].size;
      biggestBlobPublisher.publish(biggestBlob);

      //publish a 1 on seeGreen topic so we know green is currently being detected
      seeGreenMsg.data = 1;
    }else{
      //else publish a zero on seeGreen topic as we see no green
      seeGreenMsg.data = 0;
    }
    seeGreenPublisher.publish(seeGreenMsg);

    //display the image with blobs marked on (No longer used as we run the pi headless and publish the image to a ros topic)
    //imshow("BlobDetect", imWithKeypoints);

    //translate the processed image back to ROS format, then publish it
    sensor_msgs::ImagePtr processedVideo;
    processedVideo = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imWithKeypoints).toImageMsg();
    blobITPub.publish(processedVideo);

    //wait a short time
    waitKey(3);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "blobDetectNode");

  ros::NodeHandle nh;

  //create the video feed handler
  image_transport::ImageTransport it(nh);

  //Subscriber for listening to the raspberry pi camera topic
  image_transport::Subscriber sub = it.subscribe("raspicam_node/image", 1, blobDetectCallback, ros::VoidPtr(),image_transport::TransportHints("compressed"));

  //Publisher for details of the biggest blob detected on screen
  biggestBlobPublisher = nh.advertise<geometry_msgs::Point>("/camera/BiggestBlob", 10);

  //Publisher to tell the system that green is currently being detected
  seeGreenPublisher = nh.advertise<std_msgs::Int8>("/camera/SeeGreen", 10);

  //Publisher for the processed image so we can see what blobs are being detected on other machines
  blobITPub = it.advertise("camera/ProcessedWithBlobs", 1);

  ros::spin();

  return 0;
}
