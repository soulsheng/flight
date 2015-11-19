// ComputeDisparityMIT.cpp : 定义控制台应用程序的入口点。
//


#include "opencv2/opencv.hpp"
using namespace cv;

#include "pushbroom-stereo.hpp"
#include "opencv-stereo-util.hpp"

#include "helper_timer.h"

#define	IMAGE_FILE_MP4_L				"昆虫总动员-预告L.mp4"
#define	IMAGE_FILE_MP4_R				"昆虫总动员-预告L.mp4"
#define	IMAGE_FILE_MP4_OUT		"CollisionAvoidance30s.avi"

int main( )
{
	Mat matL ;//= imread("11_L.jpg", CV_8UC1);
	Mat matR ;//= imread("11_R.jpg", CV_8UC1);
	CvCapture* capture_L = 0;
	CvCapture* capture_R = 0;

	capture_L = cvCaptureFromAVI( IMAGE_FILE_MP4_L );
	capture_R = cvCaptureFromAVI( IMAGE_FILE_MP4_R );

	double fps = cvGetCaptureProperty(capture_L,CV_CAP_PROP_FPS);   
	CvSize size = cvSize(
		(int)cvGetCaptureProperty( capture_L, CV_CAP_PROP_FRAME_WIDTH),  
		(int)cvGetCaptureProperty( capture_L, CV_CAP_PROP_FRAME_HEIGHT));    

	CvVideoWriter* writer = cvCreateVideoWriter(  
		IMAGE_FILE_MP4_OUT, CV_FOURCC('D', 'I', 'V', 'X'),fps,size);  


	OpenCvStereoCalibration stereoCalibration;
	PushbroomStereoState state; // HACK
	configCD( stereoCalibration, state);

	PushbroomStereo pushbroom_stereo;

	IplImage* iplImgL1 = cvCreateImage( size, IPL_DEPTH_8U, 1 );  
	IplImage* iplImgR1 = cvCreateImage( size, IPL_DEPTH_8U, 1 );  

	StopWatchInterface	*timer;
	sdkCreateTimer( &timer );


	if( capture_L && capture_R )
	{
		cout << "In capture ..." << endl;
		int nFrameCount = 0;
		int nFrameCountMax = fps*28;
		for(;nFrameCount<nFrameCountMax;nFrameCount++)
		{
			IplImage* iplImgL = cvQueryFrame( capture_L );
			IplImage* iplImgR = cvQueryFrame( capture_R );

			cvCvtColor( iplImgL, iplImgL1, CV_BGR2GRAY);
			matL = iplImgL1;

			cvCvtColor( iplImgR, iplImgR1, CV_BGR2GRAY);
			matR = iplImgR1;
				
			cv::vector<Point3f> pointVector3d;
			cv::vector<uchar> pointColors;
			cv::vector<Point3i> pointVector2d; // for display

			sdkResetTimer( &timer );
			sdkStartTimer( &timer );

			pushbroom_stereo.ProcessImages( matL, matR, &pointVector3d, &pointColors, &pointVector2d, state);

			sdkStopTimer( &timer );
			printf("timer: %.2f ms \n", sdkGetTimerValue( &timer) );

			cout << pointVector2d.size() << "points " <<  endl;

			// global for where we are drawing a line on the image
			for (unsigned int i=0;i<pointVector2d.size();i++) {
					int x2 = pointVector2d[i].x;
					int y2 = pointVector2d[i].y;
					rectangle(matL, Point(x2,y2), Point(x2+state.blockSize, y2+state.blockSize), 0,  CV_FILLED);
					rectangle(matL, Point(x2+1,y2+1), Point(x2+state.blockSize-1, y2-1+state.blockSize), 255);
			}

			cv::imshow("matL", matL );
	
			*iplImgL1 = matL;
			cvCvtColor( iplImgL1, iplImgL, CV_GRAY2BGR);

			cvWriteToAVI( writer, iplImgL );  

			if( waitKey( 10 ) >= 0 )
				goto _cleanup_;
		}

		waitKey(0);

_cleanup_:
		cvReleaseVideoWriter( &writer ); 
		cvReleaseCapture( &capture_L );
		cvReleaseCapture( &capture_R );
	}


	return 0;
}

