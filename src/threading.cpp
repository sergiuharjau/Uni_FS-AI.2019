//#ifndef FSPYCAN_H #
//#define FSPYCAN_H #

#include <iostream>
#include <thread> 
#include <chrono>
#include <string>

 // ZED includes
#include <sl_zed/Camera.hpp>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <pyboostcvconverter/pyboostcvconverter.hpp>

class ZedCam
{
private:
	volatile bool running = false; 
	std::thread captureThread ; 

 	sl::Mat image_zed;
	sl::Mat depth_image_zed;

	sl::RuntimeParameters runtime_parameters;

    	sl::Camera zed;

public:

	ZedCam() : 
		image_zed(1920, 1080, sl::MAT_TYPE_8U_C4),
		depth_image_zed(1920, 1080, sl::MAT_TYPE_8U_C4)
	{}

	ZedCam( const ZedCam& other ) {
		zed = other.zed;	
	};

	void init()
	{ 
		sl::InitParameters init_params;
		init_params.camera_resolution = sl::RESOLUTION_HD1080;
		init_params.depth_mode = sl::DEPTH_MODE_ULTRA;
		init_params.coordinate_units = sl::UNIT_METER;
		init_params.camera_fps = 30;		
		init_params.depth_minimum_distance = 0.3;

	    sl::ERROR_CODE err = zed.open(init_params);
   		if (err != sl::SUCCESS) {
       		printf("%s\n", toString(err).c_str());
        	zed.close();
       		return; // Quit if an error occurred
    		}

  	    runtime_parameters.sensing_mode = sl::SENSING_MODE_STANDARD;

			running = true;
			captureThread = std::thread(&ZedCam::capture, this);
			std::cout << "Starting capturing loop" << std::endl;
	}

	void capture()
	{
		while (running)
		{
			if (zed.grab(runtime_parameters) == sl::SUCCESS) {
		        zed.retrieveImage(image_zed, sl::VIEW_LEFT, sl::MEM_CPU);
		        zed.retrieveMeasure(depth_image_zed, sl::MEASURE_DEPTH, sl::MEM_CPU);
			}
		}
	}

	PyObject* latestImage()
	{
		return  pbcvt::fromMatToNDArray(slMat2cvMat(image_zed)) ; 
	}

	PyObject* latestDepth()
	{
		//return slMat2cvMat(depth_image_zed); 
	}

	void exit()
	{
		running = false;
		captureThread.join();
		zed.close();
	}
	
	cv::Mat slMat2cvMat(sl::Mat& input) {
	    // Mapping between MAT_TYPE and CV_TYPE
	    int cv_type = -1;
	    switch (input.getDataType()) {
		case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
		case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
		case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
		case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
		case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
		case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
		case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
		case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
		default: break;
	    }

	    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	    // cv::Mat and sl::Mat will share a single memory structure
	    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
	}
	
};

//#endif
