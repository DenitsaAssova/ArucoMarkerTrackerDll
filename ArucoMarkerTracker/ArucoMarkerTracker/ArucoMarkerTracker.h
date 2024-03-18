#pragma once

/*******************
* ArucoMarkerTracker.h: Defines the exported functions for the DLL.
*******************/

#ifndef ArucoMarkerTracker_H
#define ArucoMarkerTracker_H

// #include "pch.h" Uncomment for Visual Studio 2017 and earlier
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
//#include <opencv2/core/cuda.hpp>
//#include <opencv2/cudaarithm.hpp>
//#include "opencv2/cudaimgproc.hpp"
//#include <opencv2/cudafilters.hpp>
//#include <opencv2/aruco.hpp>
#include<opencv2/objdetect/objdetect.hpp>

#define _USE_MATH_DEFINES
#include <math.h>

//============================================
// marshalling between Unity and C#
//============================================ 

#define DllExport   __declspec( dllexport )
#define DllImport   __declspec( dllimport )

#ifdef ARUCOMARKERTRACKER_EXPORTS
#define ARUCOMARKERTRACKER_API DllExport
#else
#define ARUCOMARKERTRACKER_API DllImport
#endif

struct Color32
{
	uchar red;
	uchar green;
	uchar blue;
	uchar alpha;
};


#define MAXMARKERS 100
#define CORNERS 4

typedef struct _MYPOINT
{
	float nX;
	float nY;
} MYPOINT;

typedef struct _CAMERAPARAMS
{
	float focalLength;
	float cx;
	float cy;
	float kappa1;
	float kappa2;
} CAMERAPARAMS;

typedef struct _OCVPOSELIST
{
	int   num;
	int   id[MAXMARKERS];
	float pmat[MAXMARKERS * 16];
} OCVPOSELIST;

extern "C"
{
	ARUCOMARKERTRACKER_API int findAllArucoMarkersMarshalled(
		Color32** rawImage, int width, int height,
		OCVPOSELIST* pList,
		MYPOINT* corners[],
		CAMERAPARAMS* cameraParams,
		float singleMarkerSize,
		float boardMarkerSize,
		float markerSeparaion,
		int tableID,
		int cameraID,
		int smartphoneID,
		int boardID1,
		int boardID2,
		bool showResults);

	ARUCOMARKERTRACKER_API void cropMarkerRegion(
		Color32** rawImage, int width, int height,
		MYPOINT* point1, MYPOINT* point2, int padding);

	/*ARUCOMARKERTRACKER_API void cropMarkerRegionGpu(
		Color32** rawImage, int width, int height,
		MYPOINT* point1, MYPOINT* point2, int padding);*/

	ARUCOMARKERTRACKER_API void cutOutFace(
		Color32** rawImage, int width, int height
	);

	ARUCOMARKERTRACKER_API void OCVtest_Canny(Color32** rawImage, int width, int height);
	/*ARUCOMARKERTRACKER_API void OCVtest_Gpu(Color32** rawImage, int width, int height);*/
	ARUCOMARKERTRACKER_API void OCVtest_Cpu(Color32** rawImage, int width, int height);
}

//====================================
// regular marker tracking (C++)
//====================================

// colour constants
#define vGRAY(v) cv::Scalar(v,v,v,v)
#define vRED(v) cv::Scalar(0,0,v)
#define vGREEN(v) cv::Scalar(0,v,0)
#define vBLUE(v) cv::Scalar(v,0,0)
#define vYELLOW(v) cv::Scalar(0,v,v)
#define vMAGENTA(v) cv::Scalar(v,0,v)
#define vCYAN(v) cv::Scalar(v,v,0)
#define vORANGE(v) cv::Scalar(0,(int)(v/2),v)
#define vLEMONGREEN(v) cv::Scalar(0,v,(int)(v/2))

//-----------------------------------------
class ARUCOMARKERTRACKER_API ArucoMarkerTracker
	//-----------------------------------------
{
public:
	ArucoMarkerTracker(float kMarkerSize)
	{
		init();
		_kMarkerSize = kMarkerSize;
	}

	ArucoMarkerTracker(float kMarkerSize, int thresh, int bw_thresh)
	{
		init();
		_kMarkerSize = kMarkerSize;
	}

	~ArucoMarkerTracker() {
		cleanup();
	}

	int findAllArucoMarkers(
		cv::Mat& colorImage,
		std::vector<int>& MarkerID,
		std::vector<cv::Mat>& MarkerPmat,
		std::vector<std::vector<cv::Point2f>>& MarkerCorners,
		std::vector<float> intrinsicCamParams = { 0.0f, 0.0f, 0.0f },
		std::vector<float> distortionCoefficients = { 0.0f, 0.0f, 0.0f, 0.0f },
		float singleMarkerSize = 0.0f,
		float boardMarkerSize = 0.0f,
		float markerSeparation = 0.0f,
		int tableID = 0,
		int cameraID = 0,
		int smartphoneID = 0,
		int boardID1 = 0,
		int boardID2 = 0,
		bool showResults = false);

	int findArucoBoard(
		cv::Mat& colorImage,
		std::vector<int>& MarkerID,
		std::vector<cv::Mat>& MarkerPmat,
		std::vector<std::vector<cv::Point2f>>& MarkerCorners,
		float focalLength = 0.0f,
		std::vector<float> distortionCoefficients = { 0.0f, 0.0f, 0.0f, 0.0f },
		float markerSize = 0.0f,
		float markerSeparation = 0.0f,
		bool showResults = false);

	cv::Mat KMatrix() { return _Kmatf; }

	cv::Rect enlargeROI(int rows, int cols, cv::Rect rect, int padding);

	void calibrateCamera(std::string imgPath, int boardWidth, int boardHeight, int fieldSize);

protected:
	void init() {};
	void cleanup() {};

	float _kMarkerSize = 1.0f;

	cv::Mat _Kmatd = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat _Kmatf = cv::Mat::eye(3, 3, CV_32F);

	void initKalmanFilter(cv::KalmanFilter& KF, int nStates, int nMeasurements, int nInputs, float dt);
	void fillMeasurements(cv::Mat& measurements,
		const cv::Mat& translation_measured, const cv::Mat& rotation_measured);
	void updateKalmanFilter(cv::KalmanFilter& KF, cv::Mat& measurement,
		cv::Mat& translation_estimated, cv::Mat& rotation_estimated);

	cv::Mat rot2euler(const cv::Mat& rotationMatrix);
	cv::Mat euler2rot(const cv::Mat& euler);


};

#endif 
