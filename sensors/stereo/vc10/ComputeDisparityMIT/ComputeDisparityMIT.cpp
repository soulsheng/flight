// ComputeDisparityMIT.cpp : 定义控制台应用程序的入口点。
//


#include "opencv2/opencv.hpp"
using namespace cv;

#include "pushbroom-stereo.hpp"
#include "opencv-stereo-util.hpp"

#define ConfigFile	"aaazzz.conf"
#define CalibrationDir	"..\\..\\calib-02-20-2014"


int main( )
{
	Mat matL = imread("11_Ls.jpg", CV_8UC1);
	Mat matR = imread("11_Rs.jpg", CV_8UC1);

	cv::vector<Point3f> pointVector3d;
	cv::vector<uchar> pointColors;
	cv::vector<Point3i> pointVector2d; // for display
	cv::vector<Point3i> pointVector2d_inf; // for display


	// load calibration
	OpenCvStereoCalibration stereoCalibration;

	if (LoadCalibration(CalibrationDir, &stereoCalibration) != true)
	{
		cerr << "Error: failed to read calibration files. Quitting." << endl;
		return -1;
	}

	int inf_disparity_tester, disparity_tester;
	disparity_tester = GetDisparityForDistance(10, stereoCalibration, &inf_disparity_tester);

	std::cout << "computed disparity is = " << disparity_tester << ", inf disparity = " << inf_disparity_tester << std::endl;

	float random_results = -1.0;
	bool show_display = false;

	PushbroomStereoState state; // HACK

	// sensors\stereo\aaazzz.conf 
	state.disparity = -105;
	state.zero_dist_disparity = -95;
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

	state.lastValidPixelRow =  205;

	PushbroomStereo pushbroom_stereo;

	pushbroom_stereo.ProcessImages(matL, matR, &pointVector3d, &pointColors, &pointVector2d, state);

	return 0;
}

