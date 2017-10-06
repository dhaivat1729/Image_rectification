#include <ros/ros.h>
//#include <image_transport/image_transport.h>
#include "opencv2/highgui/highgui.hpp"
//#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/video/tracking.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <cv.h>
/*
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
*/

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
using namespace std;
using namespace cv;

int main(int argc, char** argv)
{

// SlamDunk calibrated paramters
float camera_matrix_l[3][3]={{432.08320937135778, 0, 337.54198488711262}, {0, 433.00345265051834, 247.31701986204351},{0, 0, 1}};  //Projection Matrix Ps3 camera
//float distortion_coefficients_l[4][1]={{-0.329795791832902}, {0.171354481891701} ,{-0.062497174979430}, {0.0004983932270126202}, {0.000008364886208859099}};            //Camera Matrix o // Matlab paramters

float distortion_coefficients_l[4][1]={{-0.0015355299072718684}, {0}, {0}, {0}};            //Camera Matrix o // From slamdunk calibration

float camera_matrix_r[3][3]={{431.33413810692417, 0., 310.36762202912143}, {0, 431.80163244232176, 238.12093323431566}, {0, 0, 1}};  //Projection Matrix Ps3 camera
//float distortion_coefficients_r[5][1]={{-0.326057790808347}, {0.159192356320030}, {-0.048466891556342}, {0.0003907877454749005},{-0.0001838990796207646}};            //Camera Matrix o // Matlab paramters

float distortion_coefficients_r[4][1]={{0.00066376640137790193}, {0}, {0}, {0}};            //Camera Matrix o // From slamdunk calibration


float relative_rotation[3][3]={ {0.999994225711118, -0.000363334208339934,
       0.00337883599397535}, {0.000358761767974083,
       0.999999019302383, 0.00135376669571662},
       {-0.00337932455010961, -0.00135254668150175,
       0.999993375369587}};

float relative_translation[3][1] = {{-20.086131778527300},{-0.008257986510928},{-0.463644348932970}};

float r1[3][3]={{0.99990407923595359, 0.0010767264667982538,
       0.013808438992708470}, {-0.0010670306415240173,
       0.99999917901684920, -0.00070951479016085478}, {-0.013809191609565872, 0.00069471270544324895,   0.99990440723168483}};

float r2[3][3]={{0.99980495989032137, 0.00075826190965174985,
       0.019734923804998708}, {-0.00077211889201979442,
       0.99999946071952683, 0.00069454522682344498},
       {-0.019734386515149627e-02, -0.00070964747014863573,
       0.99980500618337498}};

float p1[3][4]={ {351.98612023404002, 0, 315.66696919048394, 0}, {0.,
       351.98612023404002, 243.61191598413927, 0}, {0, 0, 1, 0}};

float p2[3][4]={{351.98612023404002, 0, 315.66696919048394, -70.715289735638265}, {0,
       351.9861202340400, 243.61191598413927, 0}, 
     	{0, 0, 1,0}};

float q1[4][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};

Size imageSize;

Mat matrix_l = Mat(3, 3, CV_32FC1,  camera_matrix_l);
Mat distortion_l = Mat(4, 1, CV_32FC1, distortion_coefficients_l);

Mat matrix_r = Mat(3, 3, CV_32FC1,  camera_matrix_r);
Mat distortion_r = Mat(4, 1, CV_32FC1, distortion_coefficients_r);

Mat rel_rotation = Mat(3, 3, CV_32FC1, relative_rotation);
Mat rel_translation = Mat(3, 3, CV_32FC1, relative_translation);


Mat R1=Mat(3, 3, CV_32FC1,  r1);
Mat P1=Mat(3, 4, CV_32FC1,  p1);
Mat R2=Mat(3, 3, CV_32FC1,  r2);
Mat P2=Mat(3, 4, CV_32FC1,  p2);
Mat Q1=Mat(4, 4, CV_32FC1,  q1);
Mat map1_l,map2_l,map1_r,map2_r;

int alpha;

//cv::Rect myROI(0, 0, 1280, 720);
//cv::Rect myROI1(1280, 0, 1280, 720);
//ros::init(argc, argv, "image_publisher");
//ros::NodeHandle nh;
//image_transport::ImageTransport it(nh);
//image_transport::Publisher right_msg = it.advertise("zed/rectified_images", 1);
//image_transport::Publisher current_msg = it.advertise("current/image", 1);
//namedWindow("Original",CV_WINDOW_AUTOSIZE);
//namedWindow("Calibrated",CV_WINDOW_AUTOSIZE);

/*VideoCapture cap(1);
if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the video file" << endl;
         return -1;
    }*/

Mat image,left_im,right_im,left_im_undist,right_im_undist,frame;
Mat R_1, R_2, P_1, P_2, Q;
//bool bSuccess1 = cap.read(frame);
// cvtColor(frame, frame, COLOR_BGR2GRAY);
//cv::Mat img_l =frame(myROI);
Mat img_l = imread("/media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/catkin_sync_images/src/Sync_messages/left_images/left_1.jpg",1);
imageSize=img_l.size();
Size new_image_size(576,576);
namedWindow( "Original window", WINDOW_AUTOSIZE );// Create a window for display.
namedWindow( "Undistorted window", WINDOW_AUTOSIZE );// Create a window for display.
//cout << "before SR" << endl;
//stereoRectify(matrix_l, distortion_l, matrix_r, distortion_r, img_l.size(), rel_rotation, rel_translation, R1, R2, P1, P2, Q, CV_CALIB_ZERO_DISPARITY, 0, img_l.size(), 0, 0);
//stereoRectify(matrix_l, distortion_l, matrix_r, distortion_r, imageSize, rel_rotation, rel_translation, R1, R2, P1, P2, Q1, 0, 1.0, new_image_size, 0, 0);
//cout << "After SR" << endl;

/*R1.convertTo(R1, CV_32FC1);
R2.convertTo(R2, CV_32FC1);
P1.convertTo(P1, CV_32FC1);
P2.convertTo(P2, CV_32FC1);
matrix_l.convertTo(matrix_l, CV_32FC1);
matrix_r.convertTo(matrix_r, CV_32FC1);
distortion_l.convertTo(distortion_l, CV_32FC1);
distortion_r.convertTo(distortion_r, CV_32FC1);
rel_rotation.convertTo(rel_rotation, CV_32FC1);
rel_translation.convertTo(rel_translation, CV_32FC1);*/

cout << "before Undistort!" << endl;
initUndistortRectifyMap(matrix_l,distortion_l, R1,P1,imageSize, CV_16SC2, map1_l, map2_l);
initUndistortRectifyMap(matrix_r,distortion_r, R2,P2,imageSize, CV_16SC2, map1_r, map2_r);
cout << map1_l.size() << endl;
cout << map2_l.size() << endl;

for(int i = 1; i < 236; i++){

  cout << i << endl;
  stringstream out;                 
  string image_number;              
  out << i;
  image_number = out.str();
  //string left_image_name = "/media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/catkin_sync_images/src/Sync_messages/left_images/";
  //string right_image_name = "/media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/catkin_sync_images/src/Sync_messages/right_images/";
  string left_image_name = "/media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/Image_rectification/Chess_board_data/left/";
  string right_image_name = "/media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/Image_rectification/Chess_board_data/right/";
  left_image_name = left_image_name + "left_" + image_number + ".jpg";
  right_image_name = right_image_name + "right_" + image_number + ".jpg";
  
//cout << left_image_name << endl;
  //cout << right_image_name << endl;
  left_im = imread(left_image_name, 1);
  right_im = imread(right_image_name, 1);
  Size s(576,576);  

  fisheye::initUndistortRectifyMap(matrix_l,distortion_l, R1,P1, imageSize, CV_16SC2, map1_l, map2_l);
  fisheye::initUndistortRectifyMap(matrix_r,distortion_r, R2,P2, imageSize, CV_16SC2, map1_r, map2_r);
  
  left_im = imread(left_image_name, 1);
  right_im = imread(right_image_name, 1);
  //resize(left_im, left_im, s);//, 0, 0, interpolation=CV_INTER_CUBIC);
  //resize(right_im, right_im, s);//, 0, 0, interpolation=CV_INTER_CUBIC);
  cout << "Before remapping!" << endl;
  remap(left_im,left_im_undist, map1_l, map2_l, 2);
  remap(right_im, right_im_undist, map1_r, map2_r, 2);
  cout << "After remapping!" << endl;
  //undistort(left_im,left_im_undist,matrix_l,distortion_l,matrix_l);
  //undistort(right_im,right_im_undist,matrix_r,distortion_r,matrix_r);
  /*Mat left = left_im_undist.clone();
  Mat right = right_im_undist.clone();
  remap(left_im,left , map1_l, map2_l, 2);
  remap(right_im, right, map1_r, map2_r, 2);*/
  string left_image_save = "/media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/Image_rectification/left/";
  string right_image_save = "/media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/Image_rectification/right/";
  left_image_save = left_image_save + "left_" + image_number + ".jpg";
  right_image_save = right_image_save + "right_" + image_number + ".jpg";
  cout << right_image_save << endl;
  cout << left_image_save << endl;
  imshow( "Original window", left_im);                   // Show our image inside it.
  imshow( "Undistorted window", left_im_undist);                   // Show our rectified image inside it.
  waitKey(0);
  //usleep(1000000);
  imwrite(left_image_save, left_im_undist);
  imwrite(right_image_save, right_im_undist);
  //imwrite("./test.ppm", left_im_undist);

}

}
