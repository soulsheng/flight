
#ifndef		GETSADCUDA_CUH
#define		GETSADCUDA_CUH
#include "pushbroom-stereo-def.hpp"


class GetSadCUDA
{
public:
	int GetSADCPP(uchar* leftImage, uchar* rightImage, uchar* laplacianL, uchar* laplacianR, int nstep, int pxX, int pxY, PushbroomStereoState state )
	{
		// init parameters
		int blockSize = state.blockSize;
		int disparity = state.disparity;
		int sobelLimit = state.sobelLimit;

		// top left corner of the SAD box
		int startX = pxX;
		int startY = pxY;

		// bottom right corner of the SAD box
		int endX = pxX + blockSize - 1;
		int endY = pxY + blockSize - 1;

		//printf("startX = %d, endX = %d, disparity = %d, startY = %d, endY = %d, rows = %d, cols = %d\n", startX, endX, disparity, startY, endY, leftImage.rows, leftImage.cols);

		int leftVal = 0, rightVal = 0;

		int sad = 0;

		for (int i=startY;i<=endY;i++) {
			// get a pointer for this row
			uchar *this_rowL = leftImage + i * nstep;
			uchar *this_rowR = rightImage + i * nstep;

			uchar *this_row_laplacianL = laplacianL + i * nstep;
			uchar *this_row_laplacianR = laplacianR + i * nstep;


				for (int j=startX;j<=endX;j++) {
					// we are now looking at a single pixel value
					/*uchar pxL = leftImage.at<uchar>(i,j);
					uchar pxR = rightImage.at<uchar>(i,j + disparity);

					uchar sL = laplacianL.at<uchar>(i,j);
					uchar sR = laplacianR.at<uchar>(i,j + disparity);
					*/


					uchar sL = this_row_laplacianL[j];//laplacianL.at<uchar>(i,j);
					uchar sR = this_row_laplacianR[j + disparity]; //laplacianR.at<uchar>(i,j + disparity);

					leftVal += sL;
					rightVal += sR;

					uchar pxL = this_rowL[j];
					uchar pxR = this_rowR[j + disparity];

					sad += abs(pxL - pxR);
				}
		}

		//cout << "(" << leftVal << ", " << rightVal << ") vs. (" << leftVal2 << ", " << rightVal2 << ")" << endl;

		int laplacian_value = leftVal + rightVal;

		//cout << "sad with neon: " << sad << " without neon: " << sad2 << endl;


		if (leftVal < sobelLimit || rightVal < sobelLimit)// || diff_score > state.interest_diff_limit)
		{
			return -1;
		}

		// weight laplacian_value into the score

		//return sobel;
		return NUMERIC_CONST*(float)sad/(float)laplacian_value;
	}
};

#endif