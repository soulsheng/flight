
#ifndef		GETSADCUDA_CUH
#define		GETSADCUDA_CUH
#include "pushbroom-stereo-def.hpp"


class GetSadCUDA
{
public:
	void GetSAD_kernel(uchar* leftImage, uchar* rightImage, uchar* laplacianL, uchar* laplacianR, int nstep, int pxX, int pxY, 
		int blockSize, int disparity, int sobelLimit,
		int x, int y, int blockDim, int *sadArray );

	void runGetSAD( int row_start, int row_end, int startJ, int stopJ, int * sadArray, uchar* leftImage, uchar* rightImage, uchar* laplacianL, uchar* laplacianR, int nstep, int blockSize, int disparity, int sobelLimit );
};

#endif