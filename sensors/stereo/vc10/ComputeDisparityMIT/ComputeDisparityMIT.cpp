// ComputeDisparityMIT.cpp : 定义控制台应用程序的入口点。
//


#include "opencv2/opencv.hpp"
using namespace cv;

#include "pushbroom-stereo.hpp"
#include "opencv-stereo-util.hpp"

#include "helper_timer.h"

#define ConfigFile	"aaazzz.conf"
#define CalibrationDir	"..\\..\\calib-02-20-2014"

int configCD(OpenCvStereoCalibration& stereoCalibration, PushbroomStereoState& state) 
{
	
	
    // load calibration from calibexe
	CalibParams calibparam;
	if( !readCalibrationFiles(calibparam) )
	{
		printf("Cannot find calibration file!\n");
		//EtronDI_CloseDevice(m_hEtronDI);
		return 0 ;
	}
	//OpenCvStereoCalibration stereoCalibration;
	
		if (TransCalibration(calibparam, &stereoCalibration) != true)
	{
		cerr << "Error: failed to read calibration files. Quitting." << endl;
		return -1;
	}
/*
	if (LoadCalibration(CalibrationDir, &stereoCalibration) != true)
	{
		cerr << "Error: failed to read calibration files. Quitting." << endl;
		return -1;
	}
	*/
	




	int inf_disparity_tester, disparity_tester;
	//para of Distance
	disparity_tester = GetDisparityForDistance(3000, stereoCalibration, &inf_disparity_tester);

	//std::cout << "computed disparity is = " << disparity_tester << ", inf disparity = " << inf_disparity_tester << std::endl;

	float random_results = -1.0;
	bool show_display = true;

	// sensors\stereo\aaazzz.conf 
	//state.disparity = -105;
	//state.zero_dist_disparity = -95;
	state.disparity = -41;
	state.zero_dist_disparity = -10;
	state.sobelLimit = 860;
	state.horizontalInvarianceMultiplier = 0.5;
	state.blockSize = 5;
	state.random_results = random_results;
	state.check_horizontal_invariance = true;

	if (state.blockSize > 10 || state.blockSize < 1)
	{
		fprintf(stderr, "Warning: block size is very large "
			"or small (%d).  Expect trouble.\n", state.blockSize);
	}

	state.sadThreshold = 54;

	state.mapxL = stereoCalibration.mx1fp;
	state.mapxR = stereoCalibration.mx2fp;
	state.Q = stereoCalibration.qMat;
	state.show_display = show_display;

	state.lastValidPixelRow =  -1;

	return 0;
}

int main( )
{
	Mat matL = imread("11_L.jpg", CV_8UC1);
	Mat matR = imread("11_R.jpg", CV_8UC1);

	cv::vector<Point3f> pointVector3d;
	cv::vector<uchar> pointColors;
	cv::vector<Point3i> pointVector2d; // for display
	cv::vector<Point3i> pointVector2d_inf; // for display


	OpenCvStereoCalibration stereoCalibration;
	PushbroomStereoState state; // HACK
	configCD( stereoCalibration, state);

	PushbroomStereo pushbroom_stereo;


	StopWatchInterface	*timer;
	sdkCreateTimer( &timer );

	sdkResetTimer( &timer );
	sdkStartTimer( &timer );

	pushbroom_stereo.ProcessImages(matL, matR, &pointVector3d, &pointColors, &pointVector2d, state);

	sdkStopTimer( &timer );
	printf("timer: %.2f ms \n", sdkGetTimerValue( &timer) );

	cout << pointVector2d.size() << "points " <<  endl;

	// output
	Mat matDisp, remapL, remapR;
#if 1
	remapL = matL;
	remapR = matR;
	remapL.copyTo(matDisp);
#else
	if (state.show_display) {
		// we remap again here because we're just in display
		Mat remapLtemp(matL.rows, matL.cols, matL.depth());
		Mat remapRtemp(matR.rows, matR.cols, matR.depth());

		remapL = remapLtemp;
		remapR = remapRtemp;

		remap(matL, remapL, stereoCalibration.mx1fp, Mat(), INTER_NEAREST);
		remap(matR, remapR, stereoCalibration.mx2fp, Mat(), INTER_NEAREST);

		remapL.copyTo(matDisp);

		//process LCM until there are no more messages
		// this allows us to drop frames if we are behind
	} // end show_display
#endif

	// global for where we are drawing a line on the image
	bool visualize_stereo_hits = true;
	bool show_unrectified = false;

	if (state.show_display) {

		for (unsigned int i=0;i<pointVector2d.size();i++) {
			int x2 = pointVector2d[i].x;
			int y2 = pointVector2d[i].y;
			//int sad = pointVector2d[i].z;
			//rectangle(matDisp, Point(x2,y2), Point(x2+state.blockSize, y2+state.blockSize), 0,  CV_FILLED);
				rectangle(matL, Point(x2,y2), Point(x2+state.blockSize, y2+state.blockSize), 0,  CV_FILLED);
			//rectangle(matDisp, Point(x2+1,y2+1), Point(x2+state.blockSize-1, y2-1+state.blockSize), 255);
				rectangle(matL, Point(x2+1,y2+1), Point(x2+state.blockSize-1, y2-1+state.blockSize), 255);
		}

		if (visualize_stereo_hits == true) {

			// draw the points on the unrectified image (to see these
			// you must pass the -u flag)
			Draw3DPointsOnImage(matL, &pointVector3d, stereoCalibration.M1, stereoCalibration.D1, stereoCalibration.R1, 128);

		}

	}

	if (show_unrectified == false) {

		//imshow("matDisp", matDisp);
			imshow("matL", matL);
	} else {
		imshow("matL", matL);
	}

	waitKey();

	return 0;
}

