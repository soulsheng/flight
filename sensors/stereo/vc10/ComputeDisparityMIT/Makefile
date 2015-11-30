
# Common includes and paths for CUDA
INCLUDES  = -std=c++11 -I../../common/inc
CFLAGS	  = -I/usr/include/opencv
LIBS	  = -lopencv_core -lopencv_highgui -lopencv_calib3d -lopencv_imgproc -lopencv_features2d -lopencv_gpu
################################################################################

# Target rules
all: build

build: ComputeDisparity

check.deps:
ifeq ($(SAMPLE_ENABLED),0)
	@echo "Sample will be waived due to the above missing dependencies"
else
	@echo "Sample is ready - all dependencies have been met"
endif

getSADCUDA.o:getSADCUDA.cu
	nvcc -ccbin g++ $(CFLAGS) $(INCLUDES) -o $@ -c $<

opencv-stereo-util.o:opencv-stereo-util.cpp
	nvcc -ccbin g++ $(CFLAGS) $(INCLUDES) -o $@ -c $< $(LIBS)

pushbroom-stereo.o:pushbroom-stereo.cpp
	nvcc -ccbin g++ $(CFLAGS) $(INCLUDES) -o $@ -c $< $(LIBS)

ComputeDisparityMIT-mp4.o:ComputeDisparityMIT-mp4.cpp
	nvcc -ccbin g++ $(CFLAGS) $(INCLUDES) -o $@ -c $< $(LIBS)

ComputeDisparity: getSADCUDA.o opencv-stereo-util.o pushbroom-stereo.o ComputeDisparityMIT-mp4.o
	nvcc -ccbin g++ -o $@ $+ $(LIBS)

run: build
	./ComputeDisparity

clean:
	rm -f ComputeDisparity opencv-stereo-util.o cppIntegration_gold.o main.o

clobber: clean
