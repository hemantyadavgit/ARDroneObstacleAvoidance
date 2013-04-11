#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>     
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <cvaux.h>
#include<math.h>
#include <cxcore.h>
#include "turtlesim/Velocity.h"
#include <turtlesim/Pose.h>
#include <highgui.h>
 
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;
static double r=0,g=0,b=0;
static const char WINDOW[] = "Image window";
 
class walldetect
{
  ros::NodeHandle nh_;
  ros::NodeHandle n;
 ros::Publisher pub ;
 
  image_transport::ImageTransport it_;    
  image_transport::Subscriber image_sub_; 
  image_transport::Publisher image_pub_; 
  std_msgs::String msg;
public:
 walldetect()
    : it_(nh_)
  {
 
      pub= n.advertise<turtlesim::Pose>("/drone/walldis", 500);
      image_sub_ = it_.subscribe("/ardrone/image_raw", 1, &walldetect::imageCb, this);
     image_pub_= it_.advertise("/arcv/Image",1);
 
 
  }
 
  ~walldetect()
  {
    cv::destroyWindow(WINDOW);
  }
 
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
 
     sensor_msgs::CvBridge bridge;
 
 
	IplImage* img = bridge.imgMsgToCv(msg,"bgr8"); 
 
 
 
 
 
turtlesim::Pose dist;
	int i1,j1,i2,j2;
        CvSeq* lines = 0;
       int i,c,d;
      int j;
       float c1[50]; 
       float m;
       float dis;
       int k=0,k1=0; 
      static int count=0;  
     float m1[50];
      float xv;
      float yv;
      int vc=0;
     float xvan=0,yvan=0;
      static float xvan1=0,yvan1=0;
    float num=0;
   static float count1=0;
  float dv;
float vxx,vyy;
 
 
      IplImage* out1 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 3 );  
      IplImage* gray_out = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 ); 
      IplImage* canny_out = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 );
      IplImage* gray_out1=cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 3 );
      IplImage* img1 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 3 ); 
 
 
 
 
 CvMemStorage* storage = cvCreateMemStorage(0);
   cvCvtColor(img, gray_out, CV_BGR2GRAY);
    cvSmooth(gray_out, gray_out, CV_GAUSSIAN, 9, 9); 
 
 
 
	int nFrames = 50;
 
 
 
CvSeq* circles = cvHoughCircles(gray_out, storage, 
        CV_HOUGH_GRADIENT, 2, 1200,1,1 ,0,80);
 
for (i = 0; i < circles->total; i++) 
    {
 
         float* p = (float*)cvGetSeqElem( circles, i );
	printf("\n%d,%d",cvRound(p[0]),cvRound(p[1]));
	if(cvRound(p[0])>=140 && cvRound(p[0])<=160)
		{
		if(cvRound(p[1])>=90 && cvRound(p[1])<=110)
			{
			printf("\nCIRCLE!!!!!!!!!!!FOUND!!!!!!!!!!!!!!!!!");
			dist.x =1;
	 		pub.publish(dist);
			}
		}	
       else
                {
                  dist.x =0;
	 		pub.publish(dist);
                  }
         cvCircle( img, cvPoint(cvRound(p[0]),cvRound(p[1])), 
             3, CV_RGB(0,255,0), -1, 8, 0 );
	cvCircle( img, cvPoint(cvRound(p[0]),cvRound(p[1])), 
             3, CV_RGB(0,255,0), 3, 8, 0 );
 
    }
 
 
cvShowImage( "img",img);
cvShowImage( "grayall",gray_out);
  cvWaitKey(2);   
 
}
};
 
 
 
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "walldetect");
  walldetect ic;
  ros::spin();
 
  return 0;
}