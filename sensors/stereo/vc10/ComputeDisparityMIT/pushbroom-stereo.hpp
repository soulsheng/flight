/**
 * Implements pushbroom stereo, a fast, single-disparity stereo algorithm.
 *
 * Copyright 2013-2015, Andrew Barry <abarry@csail.mit.edu>
 *
 */

#ifndef PUSHBROOM_STEREO_HPP
#define PUSHBROOM_STEREO_HPP

#include "pushbroom-stereo-def.hpp"
#include "getSADCUDA.cuh"

class PushbroomStereo {
    private:
        void RunStereoPushbroomStereo(PushbroomStereoStateThreaded *statet);
		void PushbroomStereo::RunStereoPushbroomStereo( Mat leftImage, Mat rightImage, Mat laplacian_left, Mat laplacian_right,
	cv::vector<Point3f> *pointVector3d, cv::vector<Point3i> *pointVector2d, cv::vector<uchar> *pointColors,
	int row_start,  int row_end, PushbroomStereoState state );

        void RunRemapping(RemapThreadState *remap_state);

        void RunInterestOp(InterestOpState *interest_state);

        bool CheckHorizontalInvariance(Mat leftImage, Mat rightImage, Mat sobelL, Mat sobelR, int pxX, int pxY, PushbroomStereoState state);

		int RoundUp(int numToRound, int multiple);

        PushbroomStereoStateThreaded thread_states_[NUM_THREADS+1];
        RemapThreadState remap_thread_states_[NUM_THREADS+1];
        InterestOpState interest_op_states_[NUM_THREADS+1];


    public:
        PushbroomStereo();

        void ProcessImages(InputArray _leftImage, InputArray _rightImage, cv::vector<Point3f> *pointVector3d, cv::vector<uchar> *pointColors, cv::vector<Point3i> *pointVector2d, PushbroomStereoState state);

		int GetSAD(Mat leftImage, Mat rightImage, Mat laplacianL, Mat laplacianR, int pxX, int pxY, PushbroomStereoState state, int *left_interest = NULL, int *right_interest = NULL, int *raw_sad = NULL);

		GetSadCUDA	m_sadCalculator;
};



#endif
