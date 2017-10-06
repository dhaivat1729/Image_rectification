/*#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <stdio.h>
#include <string.h>
#include <iostream>*/

#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include <iostream>
#include <string>


using namespace cv;
using namespace cv::ximgproc;
using namespace std;

int main(int argc, char** argv) {

	Mat left_im, right_im;
	Mat disp;

	Mat disp_;

	double matching_time = (double)getTickCount();
			// Running SGBM
	int sadSize = 7;
//	StereoSGBM sbm;
	Ptr<StereoSGBM> left_matcher  = StereoSGBM::create();
	/*
	sbm.SADWindowSize = sadSize;
	sbm.numberOfDisparities = 144;//144; 128
	sbm.preFilterCap = 63; //63
	sbm.minDisparity = -39; //-39; 0
	sbm.uniquenessRatio = 7.0;
	sbm.speckleWindowSize = 20;
	sbm.speckleRange = 16;
	sbm.disp12MaxDiff = 0;
	sbm.fullDP = true;
	sbm.P1 = 200; // sadSize*sadSize*4;
	sbm.P2 = 1600; // sadSize*sadSize*32;
		*/
	left_matcher->setP1(200);
	left_matcher->setP2(1600);
	left_matcher->setPreFilterCap(63);
	left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
	left_matcher->setMinDisparity(-25);
	left_matcher->setNumDisparities(112);
	left_matcher->setBlockSize(sadSize);
	left_matcher->setUniquenessRatio(7);
	left_matcher->setSpeckleRange(16);
	left_matcher->setSpeckleWindowSize(20);
	left_matcher->setDisp12MaxDiff(0);

        //wls_filter = createDisparityWLSFilter(left_matcher);

	for(int i = 1; i < 2; i++){

  		//cout << i << endl;
	        stringstream out;
	        string image_number;
	        out << i;
	        image_number = out.str();
	        string left_image_name ="/media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/Image_rectification/left/";
                string right_image_name = "/media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/Image_rectification/right/";
         	// left_image_name = left_image_name + "left_" + image_number + ".jpg";
	        // right_image_name = right_image_name + "right_" + image_number + ".jpg";

         	left_image_name = left_image_name + image_number + ".jpg";
	        right_image_name = right_image_name + image_number + ".jpg";
		left_im = imread(left_image_name, 1);
	        right_im = imread(right_image_name, 1);

		left_matcher->compute(left_im, right_im, disp);
		//sbm(left_im, right_im, disp);
		/*
		// Disparity filtering
		wls_filter->setLambda(8000.0);
	        wls_filter->setSigmaColor(1.5);
        	filtering_time = (double)getTickCount();
	        wls_filter->filter(left_disp,left,filtered_disp,right_disp);
     	        filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();
        	//! [filtering]
        	conf_map = wls_filter->getConfidenceMap();
		*/

		string disp_final_image = "/media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/Image_rectification/Disparity_out/";
		disp_final_image = disp_final_image + image_number + ".bmp";
		imwrite(disp_final_image, disp);
		Mat raw_disp_vis;
	        getDisparityVis(disp,raw_disp_vis,2.0);
        	namedWindow("raw disparity", WINDOW_AUTOSIZE);
       	 	imshow("raw disparity", raw_disp_vis);
		waitKey(0);

	}

	matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
	cout<<"Matching time:  "<<matching_time<<"s"<<endl;
}

/*
	// for (int i = 0; i < 103; i++) {

		// char filename_left_original[128], filename_right_original[128];
		// sprintf(filename_left_original, "/home/diav/Data/ICRA17/ZED/Indoors/Scaled/left/frame%06d.png", i);
		// sprintf(filename_right_original, "/home/diav/Data/ICRA17/ZED/Indoors/Scaled/right/frame%06d.png", i);

		// char filename_disparity[128];
		// sprintf(filename_disparity, "/home/diav/openCV_Workspace/disparity_sgbm/disp_in/frame%06d.png", i); // Set Folder

		// img1 = imread(filename_left_original);
		// img2 = imread(filename_right_original);

		// cvtColor(img1, g1, CV_BGR2GRAY);
		// cvtColor(img2, g2, CV_BGR2GRAY);

		g1 = imread("l1.jpg");
		g2 = imread("r1.jpg");

		// StereoSGBM sgbm;
		// sgbm.SADWindowSize = 15;
		// sgbm.numberOfDisparities = 256;
		// sgbm.preFilterCap = 5;
		// sgbm.minDisparity = 0;
		// sgbm.uniquenessRatio = 5;
		// sgbm.speckleWindowSize = 500;
		// sgbm.speckleRange = 2;
		// sgbm.disp12MaxDiff = -1;
		// sgbm.fullDP = false;
		// sgbm.P1 = 2000;
		// sgbm.P2 = 10000;

		// sgbm(g2, g1, disp);

		int sadSize = 7;
		StereoSGBM sbm;
		sbm.SADWindowSize = sadSize;
		sbm.numberOfDisparities = 64;//144; 128
		sbm.preFilterCap = 63; //63
		sbm.minDisparity = -40; //-39; 0
		sbm.uniquenessRatio = 7.0;
		sbm.speckleWindowSize = 20;
		sbm.speckleRange = 16;
		sbm.disp12MaxDiff = 0;
		sbm.fullDP = true;
		sbm.P1 = 200; // sadSize*sadSize*4;
		sbm.P2 = 1600; // sadSize*sadSize*32;
		sbm(g1, g2, disp);

		Mat dispSGBMscale;
    	disp.convertTo(dispSGBMscale,CV_32F, 1./16);

		// normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

		// imshow("left", img1);
		// imshow("right", img2);
		// imshow("disp", disp8);
		// imshow("disp_act", disp);
		// waitKey(0);

		// printf("%u\n", disp.at<uchar>(460, 740));
		// printf("%u\n", disp8.at<uchar>(460, 740));

		// imwrite(filename_disparity, disp);
		imwrite("actual_.png", dispSGBMscale);
		// imwrite("this_one_norm.png", disp8);

	// }


	////////////////////// ADDED BM PART ///////////////////////

	StereoBM bm;
	Mat g1_, g2_;

	cvtColor(g1, g1_, CV_BGR2GRAY);
	cvtColor(g2, g2_, CV_BGR2GRAY);

	bm(g1_, g2_, disp_);
	Mat dispSGBMscale_;
    	disp_.convertTo(dispSGBMscale_,CV_32F, 1./16);
	imwrite("actual__.png", dispSGBMscale_);

	return 0;
}*/
