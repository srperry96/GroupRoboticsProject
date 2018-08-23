#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>

using namespace cv;
namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image Processed";

image_transport::Publisher blobITPub;

int lowGreenH  = 60, lowGreenS  = 40,  lowGreenV  = 25;
int highGreenH = 80, highGreenS = 105, highGreenV = 102;

void noiseRemoval(Mat &mask)
{
 Mat erodeElement = getStructuringElement(MORPH_RECT, Size(6,6), Point(-1, -1)); //Any white part thats less than 7x7 px's gets ignored as it is seen as noise
 Mat dilateElement = getStructuringElement(MORPH_RECT, Size(10,10)); //Fill in gaps in objects so they are solid

 //Each step needs doing twice for it to be successful
 erode(mask, mask, erodeElement);
 erode(mask, mask, erodeElement);

 dilate(mask, mask, dilateElement);
 dilate(mask, mask, dilateElement);
}


void blobDetectCallback(const sensor_msgs::ImageConstPtr& originalImage){
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(originalImage, sensor_msgs::image_encodings::BGR8);
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("ROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

    Mat im = cv_ptr->image;

    Mat img_mask, img_hsv;

    cvtColor(cv_ptr->image, img_hsv, CV_BGR2HSV);
    inRange(img_hsv, cv::Scalar(lowGreenH, lowGreenS, lowGreenV), cv::Scalar(highGreenH, highGreenS, highGreenV), img_mask);

    std::vector<KeyPoint> keypoints;

    SimpleBlobDetector::Params params;
    params.filterByColor = true;
    params.blobColor = 255;

    params.filterByArea = true;
    params.minArea = 300;
    params.maxArea = 1000000000000;

    params.filterByConvexity = false;

    params.filterByCircularity = false;

    noiseRemoval(img_mask);
    Ptr<cv::SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
    detector->detect(img_mask, keypoints);

    Mat imWithKeypoints;
    drawKeypoints(img_mask, keypoints, imWithKeypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    imshow("Processed video n that", imWithKeypoints);

    waitKey(3);
  //  blobITPub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char** argv){
  ros::init(argc, argv, "blobDetectNode");

  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);

  namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);

  image_transport::Subscriber sub = it.subscribe("raspicam_node/image", 1, blobDetectCallback, ros::VoidPtr(),image_transport::TransportHints("compressed"));
  blobITPub = it.advertise("camera/image_processed", 1);

  destroyWindow(WINDOW);

  ros::spin();

  return 0;
}
