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
	// load calibration
	//OpenCvStereoCalibration stereoCalibration;

	if (LoadCalibration(CalibrationDir, &stereoCalibration) != true)
	{
		cerr << "Error: failed to read calibration files. Quitting." << endl;
		return -1;
	}

	int inf_disparity_tester, disparity_tester;
	disparity_tester = GetDisparityForDistance(10, stereoCalibration, &inf_disparity_tester);

	std::cout << "computed disparity is = " << disparity_tester << ", inf disparity = " << inf_disparity_tester << std::endl;

	float random_results = -1.0;
	bool show_display = true;

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

	return 0;
}

void DisplayPixelBlocks(Mat left_image, Mat right_image, int left, int top, 
	PushbroomStereoState state, PushbroomStereo *pushbroom_stereo);

void DrawLines(Mat leftImg, Mat rightImg, Mat stereoImg, 
	int lineX, int lineY, int disparity, int inf_disparity);

int main( )
{
	Mat matL = imread("11_Ls.jpg", CV_8UC1);
	Mat matR = imread("11_Rs.jpg", CV_8UC1);

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
	bool visualize_stereo_hits = false;
	bool show_unrectified = false;

	if (state.show_display) {

		for (unsigned int i=0;i<pointVector2d.size();i++) {
			int x2 = pointVector2d[i].x;
			int y2 = pointVector2d[i].y;
			//int sad = pointVector2d[i].z;
			rectangle(matDisp, Point(x2,y2), Point(x2+state.blockSize, y2+state.blockSize), 0,  CV_FILLED);
			rectangle(matDisp, Point(x2+1,y2+1), Point(x2+state.blockSize-1, y2-1+state.blockSize), 255);

		}

		if (visualize_stereo_hits == true) {

			// draw the points on the unrectified image (to see these
			// you must pass the -u flag)
			Draw3DPointsOnImage(matL, &pointVector3d, stereoCalibration.M1, stereoCalibration.D1, stereoCalibration.R1, 128);

		}

	}

	if (show_unrectified == false) {

		imshow("matDisp", matDisp);

	} else {
		imshow("matL", matL);
	}

	waitKey();

	return 0;
}


/**
 * Displays a very zoomed in version of the two pixel blocks being looked at
 *
 * @param left_image the left image
 * @param right_image the right image
 * @param left left coordinate of the box
 * @param top top coordinate of the box
 * @param state PushbroomStereoState containing stereo information
 * @param pushbroom_stereo stereo object so we can run GetSAD
 *
 */
void DisplayPixelBlocks(Mat left_image, Mat right_image, int left, int top, PushbroomStereoState state, PushbroomStereo *pushbroom_stereo) {
    if (left + state.blockSize > left_image.cols || top+state.blockSize > left_image.rows
        || left + state.blockSize > right_image.cols || top+state.blockSize > right_image.rows
        || left + state.disparity < 0) { // remember, disparity can be negative

        // invalid spot

        return;
    }

    Mat left_block = left_image.rowRange(top, top+state.blockSize).colRange(left, left+state.blockSize);
    Mat right_block = right_image.rowRange(top, top+state.blockSize).colRange(left+state.disparity, left+state.disparity+state.blockSize);


    Mat laplacian_left;
    Laplacian(left_image, laplacian_left, -1, 3, 1, 0, BORDER_DEFAULT);

    Mat laplacian_right;
    Laplacian(right_image, laplacian_right, -1, 3, 1, 0, BORDER_DEFAULT);

    // compute stats about block
    int left_interest, right_interest, raw_sad;
    int sad = pushbroom_stereo->GetSAD(left_image, right_image, laplacian_left, laplacian_right, left, top,
        state, &left_interest, &right_interest, &raw_sad);

    float diff_score = 100*(float)abs(left_interest - right_interest)/(float)(left_interest+right_interest);

    // make the blocks visible by making them huge
    const int scale_factor = 100;

    Size output_size = Size(state.blockSize * scale_factor, state.blockSize * scale_factor);
    resize(left_block, left_block, output_size, 0, 0, INTER_NEAREST);
    resize(right_block, right_block, output_size, 0, 0, INTER_NEAREST);

    char sad_str_char[100], left_interest_char[100], right_interest_char[100],
        diff_score_char[100], raw_sad_char[100];
    sprintf(sad_str_char, "SAD = %d", sad);
    sprintf(left_interest_char, "Left interest = %d", left_interest);
    sprintf(right_interest_char, "Right interest = %d", right_interest);
    sprintf(raw_sad_char, "Raw sad = %d", raw_sad);
    sprintf(diff_score_char, "Diff score = %.02f", diff_score);

    putText(right_block, raw_sad_char, Point(310, 425), FONT_HERSHEY_PLAIN, 1, 0);
    putText(right_block, sad_str_char, Point(400, 445), FONT_HERSHEY_PLAIN, 1, 0);
    putText(right_block, left_interest_char, Point(310, 465), FONT_HERSHEY_PLAIN, 1, 0);
    putText(right_block, right_interest_char, Point(305, 485), FONT_HERSHEY_PLAIN, 1, 0);
    putText(right_block, diff_score_char, Point(305, 495), FONT_HERSHEY_PLAIN, 1, 0);

    imshow("Left Block", left_block);
    imshow("Right Block", right_block);



}


/**
 * Draws lines on the images for stereo debugging.
 *
 * @param rightImg right image
 * @param stereoImg stereo image
 * @param lineX x position of the line to draw
 * @param lineY y position of the line to draw
 * @param disparity disparity move the line on the right image over by
 * @param inf_disparity disparity corresponding to "infinite distance"
 *  used to filter out false-positives.  Usually availible in
 *   state.zero_dist_disparity.
 */
void DrawLines(Mat leftImg, Mat rightImg, Mat stereoImg, int lineX, int lineY, int disparity, int inf_disparity) {
    int lineColor = 128;
    if (lineX >= 0)
    {
        // print out the values of the pixels where they clicked
        //cout << endl << endl << "Left px: " << (int)leftImg.at<uchar>(lineY, lineX)
        //    << "\tRight px: " << (int)rightImg.at<uchar>(lineY, lineX + disparity)
        //    << endl;

        line(leftImg, Point(lineX, 0), Point(lineX, leftImg.rows), lineColor);
        line(stereoImg, Point(lineX, 0), Point(lineX, leftImg.rows), lineColor);
        line(rightImg, Point(lineX + disparity, 0), Point(lineX + disparity, rightImg.rows), lineColor);

        line(rightImg, Point(lineX + inf_disparity, 0), Point(lineX + inf_disparity, rightImg.rows), lineColor);

        line(leftImg, Point(0, lineY), Point(leftImg.cols, lineY), lineColor);
        line(stereoImg, Point(0, lineY), Point(leftImg.cols, lineY), lineColor);
        line(rightImg, Point(0, lineY), Point(rightImg.cols, lineY), lineColor);
    }
}
