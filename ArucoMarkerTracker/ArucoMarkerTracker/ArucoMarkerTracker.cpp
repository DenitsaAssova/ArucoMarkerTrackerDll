/*
* ArucoMarkerTracker.cpp : Defines the exported functions for the DLL.
*
*/

#include "pch.h" // use stdafx.h in Visual Studio 2017 and earlier

#include <utility>
#include <limits.h>
#include "ArucoMarkerTracker.h"

//============================================
// marshalling between Unity and C#
//============================================ 

// Source https://amin-ahmadi.com/2019/06/01/how-to-pass-images-between-opencv-and-unity/

ARUCOMARKERTRACKER_API void OCVtest_Canny(Color32** rawImage, int width, int height)
{
	cv::Mat image(height, width, CV_8UC4, *rawImage);
	cv::flip(image, image, -1); // flip in both directions
	cv::Mat temp;
	/*cv::Mat edges;
	cv::Canny(image, edges, 50, 200);
	cv::dilate(edges, edges, (5, 5));
	cv::cvtColor(edges, edges, cv::COLOR_GRAY2RGBA);
	cv::normalize(edges, edges, 0, 1, cv::NORM_MINMAX);
	cv::multiply(image, edges, image);*/
	cv::cvtColor(image, temp, cv::COLOR_BGR2GRAY);
	image = temp.clone();
	cv::flip(image, image, 0); // flip again to get the right orientation
}
//
//ARUCOMARKERTRACKER_API void OCVtest_Gpu(Color32** rawImage, int width, int height)
//{
//	cv::Mat image(height, width, CV_8UC4, *rawImage);
//
//	try
//	{
//		cv::cuda::GpuMat src(height, width, CV_8UC4);
//		cv::cuda::GpuMat temp(height, width, CV_8UC4);
//		src.upload(image);
//		cv::cuda::cvtColor(src, temp, cv::COLOR_BGRA2GRAY);
//		temp.download(image);
//		cv::imshow("Result", image);
//		cv::waitKey();
//
//	}
//	catch (const cv::Exception& ex)
//	{
//		std::string s = ex.what();
//		std::ofstream myfile;
//		myfile.open("error.txt");
//		myfile << s;
//		myfile.close();
//
//	}
//
//}

ARUCOMARKERTRACKER_API void OCVtest_Cpu(Color32** rawImage, int width, int height)
{
	cv::Mat image(height, width, CV_8UC4, *rawImage);
	//cv::cuda::GpuMat src;
	//src.upload(image);
	cv::flip(image, image, -1);
	cv::threshold(image, image, 128.0, 255.0, cv::THRESH_BINARY);
	/*cv::cuda::flip(image, image, -1);
	cv::cuda::cvtColor(image, image, cv::COLOR_BGR2GRAY);
	cv::cuda::flip(image, image, 0);*/
}

// Marshalling for Marker Tracking

ARUCOMARKERTRACKER_API int findAllArucoMarkersMarshalled(Color32** rawImage, int width, int height,
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
	bool showResults)
{

	const float kMarkerSize = 0.05f;
	ArucoMarkerTracker markerTracker(kMarkerSize);

	cv::Mat image(height, width, CV_8UC4, *rawImage);
	cv::flip(image, image, 0); // flip vertically (around x-axis); i.e.: upside-down

	std::vector <int> MarkerIDs;
	std::vector<cv::Mat> MarkerPmats;
	std::vector<std::vector<cv::Point2f>> MarkerCorners;
	std::vector<float> intrinsicCamParams = { cameraParams->focalLength, cameraParams->cx, cameraParams->cy };
	std::vector<float> distortionCoefficients = { cameraParams->kappa1, cameraParams->kappa2, 0.0f, 0.0f };


	int vals = MAXMARKERS;

	int numMarkers = 0;
	try {
		numMarkers = markerTracker.findAllArucoMarkers(
			image, MarkerIDs, MarkerPmats, MarkerCorners,
			intrinsicCamParams, distortionCoefficients,
			singleMarkerSize, boardMarkerSize, markerSeparaion, tableID, cameraID,
			smartphoneID, boardID1, boardID2, showResults);

		if (numMarkers < MAXMARKERS) vals = numMarkers;

		pList->num = vals;

		for (int i = 0; i < vals; i++)
		{
			pList->id[i] = MarkerIDs[i];
			if (showResults)
				cv::putText(image, std::to_string(MarkerIDs[i]),
					cv::Point(400, 50 + 50 * i),
					cv::FONT_HERSHEY_PLAIN, 1, vGRAY(255), 1);


			cv::Mat pmat = MarkerPmats[i];
			for (int row = 0; row < 4; row++)
				for (int col = 0; col < 4; col++)
				{
					pList->pmat[(row * 4 + col) + i * 16] = pmat.at<float>(row, col);
				}
		}


		for (int i = 0; i < vals; i += 4)
		{
			std::vector<cv::Point2f> v = MarkerCorners[i];
			for (size_t c = 0; c < CORNERS; c++) {
				corners[i + c]->nX = v[c].x;
				corners[i + c]->nY = v[c].y;
				//if(showResults)
				//cv::circle(image, cv::Point2f(corners[i + c]->nX, corners[i + c]->nY), 3, cv::Scalar(0, 255, 0, 255), 3, cv::LINE_8);
			}
		}

	}
	catch (const cv::Exception& ex)
	{
		std::string s = ex.what();
		std::ofstream myfile;
		myfile.open("error.txt");
		myfile << s;
		myfile.close();

	}
	cv::flip(image, image, 0); // flip back - vertically (around x-axis); i.e.: upside-down

	return vals;

}

ARUCOMARKERTRACKER_API void cropMarkerRegion(Color32** rawImage, int width, int height, MYPOINT* point1, MYPOINT* point2, int padding)
{
	const float kMarkerSize = 0.045f;
	ArucoMarkerTracker markerTracker(kMarkerSize);

	cv::Mat image(height, width, CV_8UC4, *rawImage);
	cv::Mat mask(height, width, CV_8UC4, cv::Scalar(0, 0, 0, 255));
	cv::flip(image, image, 0); // flip vertically (around x-axis); i.e.: upside-down

	//cv::circle(image, cv::Point2f(point1->nX, point1->nY), 3, cv::Scalar(255, 255, 255, 255), 3, cv::LINE_8);
	//cv::circle(image, cv::Point2f(point2->nX, point2->nY), 3, cv::Scalar(255, 255, 255, 255), 3, cv::LINE_8);
	cv::Rect rect(cv::Point(point1->nX, point1->nY), cv::Point(point2->nX, point2->nY));
	cv::Rect enlargedROI = markerTracker.enlargeROI(image.rows, image.cols, rect, padding);
	cv::rectangle(mask, enlargedROI, cv::Scalar(255, 255, 255, 255), -1);
	bitwise_and(image, mask, image);
	cv::circle(image, cv::Point2f(point1->nX, point1->nY), 3, cv::Scalar(255, 255, 255, 255), 3, cv::LINE_8);
	cv::circle(image, cv::Point2f(point2->nX, point2->nY), 3, cv::Scalar(255, 255, 255, 255), 3, cv::LINE_8);

	cv::Mat temp, thresholded;
	cv::cvtColor(image, temp, cv::COLOR_BGR2GRAY);
	cv::threshold(temp, thresholded, 0, 255, cv::THRESH_BINARY);
	std::vector<cv::Mat> rgbChannels(3);
	cv::split(image, rgbChannels);
	std::vector<cv::Mat> channels;
	channels.push_back(rgbChannels[0]);
	channels.push_back(rgbChannels[1]);
	channels.push_back(rgbChannels[2]);
	channels.push_back(thresholded);
	cv::merge(channels, image);

	cv::flip(image, image, 0); // flip back - vertically (around x-axis); i.e.: upside-down
}

//ARUCOMARKERTRACKER_API void cropMarkerRegionGpu(Color32** rawImage, int width, int height, MYPOINT* point1, MYPOINT* point2, int padding)
//{
//	try
//	{
//		const float kMarkerSize = 0.045f;
//		ArucoMarkerTracker markerTracker(kMarkerSize);
//
//		cv::Mat image(height, width, CV_8UC4, *rawImage);
//		cv::Mat mask(height, width, CV_8UC4, cv::Scalar(0, 0, 0, 255));
//
//		cv::Rect rect(cv::Point(point1->nX, point1->nY), cv::Point(point2->nX, point2->nY));
//		cv::Rect enlargedROI = markerTracker.enlargeROI(image.rows, image.cols, rect, padding);
//		cv::rectangle(mask, enlargedROI, cv::Scalar(255, 255, 255, 255), -1);
//
//		cv::cuda::GpuMat imgSrc, maskSrc;
//		imgSrc.upload(image);
//		maskSrc.upload(mask);
//
//		cv::cuda::flip(imgSrc, imgSrc, 0); // flip vertically (around x-axis); i.e.: upside-down
//		cv::cuda::bitwise_and(imgSrc, maskSrc, imgSrc);
//
//		cv::cuda::GpuMat  temp, thresholded;
//		cv::cuda::cvtColor(imgSrc, temp, cv::COLOR_BGRA2GRAY);
//		cv::cuda::threshold(temp, thresholded, 0, 255, cv::THRESH_BINARY);
//
//		std::vector<cv::cuda::GpuMat> rgbChannels(3);
//		cv::cuda::split(imgSrc, rgbChannels);
//		std::vector<cv::cuda::GpuMat> channels;
//		channels.push_back(rgbChannels[0]);
//		channels.push_back(rgbChannels[1]);
//		channels.push_back(rgbChannels[2]);
//		channels.push_back(thresholded);
//		cv::cuda::merge(channels, imgSrc);
//
//		cv::cuda::flip(imgSrc, imgSrc, 0); // flip back - vertically (around x-axis); i.e.: upside-down
//
//		imgSrc.download(image);
//
//
//	}
//	catch (const cv::Exception& ex)
//	{
//		std::string s = ex.what();
//		std::ofstream myfile;
//		myfile.open("error.txt");
//		myfile << s;
//		myfile.close();
//
//	}
//
//
//}

ARUCOMARKERTRACKER_API void cutOutFace(Color32** rawImage, int width, int height)
{
	cv::Mat image(height, width, CV_8UC4, *rawImage);
	cv::Mat mask(height, width, CV_8UC4, cv::Scalar(0, 0, 0, 255));
	cv::flip(image, image, 0); // flip vertically (around x-axis); i.e.: upside-down

	std::string trained_classifier_location = "path_to/GeoTravel/Assets/Resources/haarcascade_frontalface_default.xml";//Defining the location our XML Trained Classifier in a string//
	cv::CascadeClassifier faceDetector;//Declaring an object named 'face detector' of CascadeClassifier class//
	faceDetector.load(trained_classifier_location);//loading the XML trained classifier in the object//
	std::vector<cv::Rect>faces;//Declaring a rectangular vector named faces//

	cv::Mat gray;
	cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

	faceDetector.detectMultiScale(gray, faces, 1.1,
		4, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));//Detecting the faces in 'image_with_humanfaces' matrix//

	for (int i = 0; i < faces.size(); i++)
	{ //for locating the face

		cv::Rect r = faces[i];
		cv::Point center;

		center.x = cvRound((r.x + r.width * 0.5));
		center.y = cvRound((r.y + r.height * 0.5));

		//Face
		ellipse(mask, center, cv::Size(r.width / 2, r.height * 0.75), 0, 0, 360, cv::Scalar(255, 255, 255, 255), -1);
		//Shoulders
		ellipse(mask, cv::Point(center.x, image.rows), cv::Size(r.height, r.width * 0.75), 0, 180, 360, cv::Scalar(255, 255, 255, 255), -1);
		//Neck
		rectangle(mask, cv::Point(center.x - (r.width / 3), center.y), cv::Point(center.x + (r.width / 3), image.cols), cv::Scalar(255, 255, 255, 255), -1);

		cv::bitwise_and(image, mask, image);
		cv::Mat temp, thresholded;
		cv::cvtColor(image, temp, cv::COLOR_BGR2GRAY);
		cv::threshold(temp, thresholded, 0, 255, cv::THRESH_BINARY);

		std::vector<cv::Mat> maskChannels(3), imgChannels(3);
		cv::split(mask, maskChannels);
		cv::split(image, imgChannels);
		std::vector<cv::Mat> channels;
		channels.push_back(imgChannels[0]);
		channels.push_back(imgChannels[1]);
		channels.push_back(imgChannels[2]);
		channels.push_back(maskChannels[0]);
		cv::merge(channels, image);

	}

	cv::flip(image, image, 0); // flip back - vertically (around x-axis); i.e.: upside-down
}

//====================================
// regular marker tracking (C++)
//====================================

int ArucoMarkerTracker::findAllArucoMarkers(cv::Mat& colorImage, std::vector<int>& MarkerID, std::vector<cv::Mat>& MarkerPmat, std::vector<std::vector<cv::Point2f>>& MarkerCorners,
	std::vector<float> intrinsicCameraParams, std::vector<float> distortionCoefficients, float singleMarkerSize, float boardMarkerSize, float markerSeparation, int tableID, int cameraID,
	int smartphoneID, int boardID1, int boardID2, bool showResults)
{
	double cx = colorImage.cols / 2.0;
	double cy = colorImage.rows / 2.0;
	double max_d = (double)cv::max(colorImage.rows, colorImage.cols);
	double _fx = max_d; // typical setup, suggested in OCV tutorials
	double _fy = max_d;

	if (intrinsicCameraParams[0] > 0.0f && intrinsicCameraParams[1] > 0.0f && intrinsicCameraParams[2] > 0.0f)  // use data, provided by external calibration data
	{
		_fx = (double)intrinsicCameraParams[0];
		_fy = (double)intrinsicCameraParams[0];
		cx = intrinsicCameraParams[1];
		cy = intrinsicCameraParams[2];
	}

	std::ofstream myfile;
	myfile.open("intrinsic.txt");
	myfile << "cx: " << cx << ", cy: " << cy << ", fx/fy: " << _fx << std::endl;

	_Kmatd = (cv::Mat_<double>(3, 3) <<
		_fx, 0., cx,
		0., _fy, cy,
		0., 0., 1.);
	_Kmatf = (cv::Mat_<float>(3, 3) <<
		_fx, 0., cx,
		0., _fy, cy,
		0., 0., 1.);


	int coeffs = distortionCoefficients.size();
	cv::Mat distCoeffs = cv::Mat(coeffs, 1, CV_64FC1);
	for (int i = 0; i < coeffs; i++) {
		distCoeffs.at<double>(i) = (double)distortionCoefficients[i];
		myfile << (double)distortionCoefficients[i] << std::endl;

	}
	myfile.close();
	//Aruco Marker Setup
	cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
	detectorParams.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers
	cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250); //change to 4x4
	cv::aruco::ArucoDetector detector(dictionary, detectorParams);

	// manually create board objects
	float half = boardMarkerSize / 2.f;
	std::vector<int> idsBoard1 = { 94, 95, 96 ,97 ,98, 99 };
	std::vector<int> idsBoard2 = { 89, 90, 91 ,92 ,93 };
	std::vector<std::vector<cv::Point3f> > board1CornerPoints;
	std::vector<std::vector<cv::Point3f> > board2CornerPoints;

	cv::Point3f corner891 = cv::Point3f(-half, -0.03, half);
	cv::Point3f corner892 = cv::Point3f(half, -0.03, half);
	cv::Point3f corner893 = cv::Point3f(half, -0.03, -half);
	cv::Point3f corner894 = cv::Point3f(-half, -0.03, -half);
	std::vector<cv::Point3f> marker89;
	marker89.push_back(corner891);
	marker89.push_back(corner892);
	marker89.push_back(corner893);
	marker89.push_back(corner894);

	cv::Point3f corner901 = cv::Point3f(-0.03, -half, half);
	cv::Point3f corner902 = cv::Point3f(-0.03, -half, -half);
	cv::Point3f corner903 = cv::Point3f(-0.03, half, -half);
	cv::Point3f corner904 = cv::Point3f(-0.03, half, half);
	std::vector<cv::Point3f> marker90;
	marker90.push_back(corner901);
	marker90.push_back(corner902);
	marker90.push_back(corner903);
	marker90.push_back(corner904);

	cv::Point3f marker911 = cv::Point3f(-half, -half, -0.03);
	cv::Point3f corner912 = cv::Point3f(half, -half, -0.03);
	cv::Point3f corner913 = cv::Point3f(half, half, -0.03);
	cv::Point3f corner914 = cv::Point3f(-half, half, -0.03);
	std::vector<cv::Point3f> marker91;
	marker91.push_back(marker911);
	marker91.push_back(corner912);
	marker91.push_back(corner913);
	marker91.push_back(corner914);

	cv::Point3f marker921 = cv::Point3f(0.03, -half, -half);
	cv::Point3f corner922 = cv::Point3f(0.03, -half, half);
	cv::Point3f corner923 = cv::Point3f(0.03, half, half);
	cv::Point3f corner924 = cv::Point3f(0.03, half, -half);
	std::vector<cv::Point3f> marker92;
	marker92.push_back(marker921);
	marker92.push_back(corner922);
	marker92.push_back(corner923);
	marker92.push_back(corner924);

	cv::Point3f marker931 = cv::Point3f(-half, 0.03, -half);
	cv::Point3f corner932 = cv::Point3f(half, 0.03, -half);
	cv::Point3f corner933 = cv::Point3f(half, 0.03, half);
	cv::Point3f corner934 = cv::Point3f(-half, 0.03, half);
	std::vector<cv::Point3f> marker93;
	marker93.push_back(marker931);
	marker93.push_back(corner932);
	marker93.push_back(corner933);
	marker93.push_back(corner934);

	cv::Point3f corner941 = cv::Point3f(-half, half, 0.03);
	cv::Point3f corner942 = cv::Point3f(half, half, 0.03);
	cv::Point3f corner943 = cv::Point3f(half, -half, 0.03);
	cv::Point3f corner944 = cv::Point3f(-half, -half, 0.03);
	std::vector<cv::Point3f> marker94;
	marker94.push_back(corner941);
	marker94.push_back(corner942);
	marker94.push_back(corner943);
	marker94.push_back(corner944);
	cv::Point3f corner951 = cv::Point3f(-half, -0.03, half);
	cv::Point3f corner952 = cv::Point3f(half, -0.03, half);
	cv::Point3f corner953 = cv::Point3f(half, -0.03, -half);
	cv::Point3f corner954 = cv::Point3f(-half, -0.03, -half);
	std::vector<cv::Point3f> marker95;
	marker95.push_back(corner951);
	marker95.push_back(corner952);
	marker95.push_back(corner953);
	marker95.push_back(corner954);

	cv::Point3f corner961 = cv::Point3f(-0.03, -half, half);
	cv::Point3f corner962 = cv::Point3f(-0.03, -half, -half);
	cv::Point3f corner963 = cv::Point3f(-0.03, half, -half);
	cv::Point3f corner964 = cv::Point3f(-0.03, half, half);
	std::vector<cv::Point3f> marker96;
	marker96.push_back(corner961);
	marker96.push_back(corner962);
	marker96.push_back(corner963);
	marker96.push_back(corner964);

	cv::Point3f marker971 = cv::Point3f(-half, -half, -0.03);
	cv::Point3f corner972 = cv::Point3f(half, -half, -0.03);
	cv::Point3f corner973 = cv::Point3f(half, half, -0.03);
	cv::Point3f corner974 = cv::Point3f(-half, half, -0.03);
	std::vector<cv::Point3f> marker97;
	marker97.push_back(marker971);
	marker97.push_back(corner972);
	marker97.push_back(corner973);
	marker97.push_back(corner974);

	cv::Point3f marker981 = cv::Point3f(0.03, -half, -half);
	cv::Point3f corner982 = cv::Point3f(0.03, -half, half);
	cv::Point3f corner983 = cv::Point3f(0.03, half, half);
	cv::Point3f corner984 = cv::Point3f(0.03, half, -half);
	std::vector<cv::Point3f> marker98;
	marker98.push_back(marker981);
	marker98.push_back(corner982);
	marker98.push_back(corner983);
	marker98.push_back(corner984);

	cv::Point3f marker991 = cv::Point3f(-half, 0.03, -half);
	cv::Point3f corner992 = cv::Point3f(half, 0.03, -half);
	cv::Point3f corner993 = cv::Point3f(half, 0.03, half);
	cv::Point3f corner994 = cv::Point3f(-half, 0.03, half);
	std::vector<cv::Point3f> marker99;
	marker99.push_back(marker991);
	marker99.push_back(corner992);
	marker99.push_back(corner993);
	marker99.push_back(corner994);

	board1CornerPoints.push_back(marker94);
	board1CornerPoints.push_back(marker95);
	board1CornerPoints.push_back(marker96);
	board1CornerPoints.push_back(marker97);
	board1CornerPoints.push_back(marker98);
	board1CornerPoints.push_back(marker99);

	board2CornerPoints.push_back(marker89);
	board2CornerPoints.push_back(marker90);
	board2CornerPoints.push_back(marker91);
	board2CornerPoints.push_back(marker92);
	board2CornerPoints.push_back(marker93);


	cv::aruco::Board board1 = cv::aruco::Board(board1CornerPoints, dictionary, idsBoard1);
	cv::aruco::Board board2 = cv::aruco::Board(board2CornerPoints, dictionary, idsBoard2);

	//Kalma filter setup
	cv::KalmanFilter KF; // instantiate Kalman Filter
	int nStates = 18; // the number of states
	int nMeasurements = 6; // the number of measured states
	int nInputs = 0; // the number of action control
	float dt = 0.125; // time between measurements (1/FPS)
	initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt); // init function
	cv::Mat measurements(nMeasurements, 1, CV_32FC1); measurements.setTo(cv::Scalar(0));

	cv::Mat tempImg;
	cv::cvtColor(colorImage, tempImg, cv::COLOR_BGRA2BGR);

	std::vector<int> ids, boardIDs1, boardIDs2, singleMarkersIDs;
	std::vector<std::vector<cv::Point2f>> cornersAruco, rejected, cornersBoard1, cornersBoard2, cornersSingleMarkers;

	detector.detectMarkers(tempImg, cornersAruco, ids, rejected);

	int nMarkers = ids.size();

	int markersOfBoardDetected = 0;

	if (nMarkers > 0) {
		//cv::aruco::drawDetectedMarkers(tempImg, cornersAruco, ids);

		for (int i = 0; i < nMarkers; i++) {

			//Sort out the ids
			if (ids[i] == tableID || ids[i] == cameraID || ids[i] == smartphoneID) { //Could be expanded for more object

				singleMarkersIDs.push_back(ids[i]);
				cornersSingleMarkers.push_back(cornersAruco.at(i));

			}

			else if (std::find(idsBoard1.begin(), idsBoard1.end(), ids[i]) != idsBoard1.end()) { //TODO: generalize for n boards
				boardIDs1.push_back(ids[i]);
				cornersBoard1.push_back(cornersAruco.at(i));

			}
			else if (std::find(idsBoard2.begin(), idsBoard2.end(), ids[i]) != idsBoard2.end()) {
				boardIDs2.push_back(ids[i]);
				cornersBoard2.push_back(cornersAruco.at(i));
			}

		}

		int singleMarkersSize = singleMarkersIDs.size();
		int board1Size = (boardIDs1.size() > 0 ? 1 : 0);
		int board2Size = (boardIDs2.size() > 0 ? 1 : 0);
		nMarkers = singleMarkersSize + board1Size + board2Size;

		cv::Mat objPoints(4, 1, CV_32FC3), imgPoints, raux, taux;
		for (int i = 0; i < nMarkers; i++) {

			//Single markers
			if (i < singleMarkersSize) {
				MarkerCorners.push_back(cornersSingleMarkers[i]);
				MarkerID.push_back(singleMarkersIDs[i]);
				objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-singleMarkerSize / 2.f, singleMarkerSize / 2.f, 0);
				objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(singleMarkerSize / 2.f, singleMarkerSize / 2.f, 0);
				objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(singleMarkerSize / 2.f, -singleMarkerSize / 2.f, 0);
				objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-singleMarkerSize / 2.f, -singleMarkerSize / 2.f, 0);
				solvePnP(objPoints, cornersSingleMarkers.at(i), _Kmatd, distCoeffs, raux, taux);
			}
			//Board markers
			else if (i < singleMarkersSize + board1Size) {
				MarkerCorners.push_back(cornersBoard1[i - singleMarkersSize]);
				MarkerID.push_back(boardID1);
				board1.matchImagePoints(cornersBoard1, boardIDs1, objPoints, imgPoints);
				cv::solvePnP(objPoints, imgPoints, _Kmatd, distCoeffs, raux, taux);
			}
			else {
				MarkerCorners.push_back(cornersBoard2[i - singleMarkersSize - board1Size]);
				MarkerID.push_back(boardID2);
				board2.matchImagePoints(cornersBoard2, boardIDs2, objPoints, imgPoints);
				cv::solvePnP(objPoints, imgPoints, _Kmatd, distCoeffs, raux, taux);
			}


			cv::Mat Rvec;
			raux.convertTo(Rvec, CV_32F);
			cv::Mat_<float> Rmat(3, 3);
			Rodrigues(Rvec, Rmat);

			cv::Mat_<float> Tvec;
			taux.convertTo(Tvec, CV_32F);

			fillMeasurements(measurements, Tvec, Rmat);
			// Instantiate estimated translation and rotation
			cv::Mat translation_estimated(3, 1, CV_32F);
			cv::Mat rotation_estimated(3, 3, CV_32F);
			// update the Kalman filter with good measurements
			updateKalmanFilter(KF, measurements,
				translation_estimated, rotation_estimated);

			cv::Mat Pmat = (cv::Mat_<float>(4, 4) <<
				rotation_estimated.at<float>(0, 0), rotation_estimated.at<float>(0, 1), rotation_estimated.at<float>(0, 2), translation_estimated.at<float>(0),
				rotation_estimated.at<float>(1, 0), rotation_estimated.at<float>(1, 1), rotation_estimated.at<float>(1, 2), translation_estimated.at<float>(1),
				rotation_estimated.at<float>(2, 0), rotation_estimated.at<float>(2, 1), rotation_estimated.at<float>(2, 2), translation_estimated.at<float>(2),
				0.0f, 0.0f, 0.0f, 1.0f
				);

			MarkerPmat.push_back(Pmat);
			//cv::drawFrameAxes(tempImg, _Kmatd, distCoeffs, raux, taux, 0.1);
		}


	}

	//cv::imshow("out", tempImg);

	return nMarkers;

}

int ArucoMarkerTracker::findArucoBoard(cv::Mat& colorImage, std::vector<int>& MarkerID, std::vector<cv::Mat>& MarkerPmat, std::vector<std::vector<cv::Point2f>>& MarkerCorners, float focalLength, std::vector<float> distortionCoefficients, float markerSize, float markerSeparation, bool showResults)
{
	double cx = colorImage.cols / 2.0;
	double cy = colorImage.rows / 2.0;
	double max_d = (double)cv::max(colorImage.rows, colorImage.cols);
	double _fx = max_d; // typical setup, suggested in OCV tutorials
	double _fy = max_d;
	if (focalLength > 0.0f)  // use focal length, provided by external calibration data
	{
		_fx = (double)focalLength;
		_fy = (double)focalLength;
	}

	_Kmatd = (cv::Mat_<double>(3, 3) <<
		_fx, 0., cx,
		0., _fy, cy,
		0., 0., 1.);
	_Kmatf = (cv::Mat_<float>(3, 3) <<
		_fx, 0., cx,
		0., _fy, cy,
		0., 0., 1.);


	int coeffs = distortionCoefficients.size();
	cv::Mat distCoeffs = cv::Mat(coeffs, 1, CV_64FC1);
	for (int i = 0; i < coeffs; i++)
		distCoeffs.at<double>(i) = (double)distortionCoefficients[i];

	//Aruco Marker Setup
	cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
	detectorParams.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers
	cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::aruco::ArucoDetector detector(dictionary, detectorParams);
	//create board object
	cv::Ptr<cv::aruco::GridBoard> gridboard = new cv::aruco::GridBoard(cv::Size(1, 2), markerSize, markerSeparation, dictionary);
	cv::Ptr<cv::aruco::Board> board = gridboard.staticCast<cv::aruco::Board>();

	//Kalma filter setup
	cv::KalmanFilter KF; // instantiate Kalman Filter
	int nStates = 18; // the number of states
	int nMeasurements = 6; // the number of measured states
	int nInputs = 0; // the number of action control
	float dt = 0.125; // time between measurements (1/FPS)
	initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt); // init function
	cv::Mat measurements(nMeasurements, 1, CV_32FC1); measurements.setTo(cv::Scalar(0));

	cv::Mat tempImg;
	cv::cvtColor(colorImage, tempImg, cv::COLOR_BGRA2BGR);

	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f>> cornersAruco, rejected;

	detector.detectMarkers(tempImg, cornersAruco, ids, rejected);

	int markersOfBoardDetected = 0;

	if (!ids.empty()) {

		cv::aruco::drawDetectedMarkers(tempImg, cornersAruco, ids);

		for (int i = 0; i < ids.size(); i++) {

			MarkerCorners.push_back(cornersAruco[i]);
			MarkerID.push_back(ids[i]);
		}

		cv::Mat objPoints, imgPoints, raux, taux;
		board->matchImagePoints(cornersAruco, ids, objPoints, imgPoints);

		// Find pose
		cv::solvePnP(objPoints, imgPoints, _Kmatd, distCoeffs, raux, taux);

		markersOfBoardDetected = (int)objPoints.total() / 4;

		if (markersOfBoardDetected > 0)
			cv::drawFrameAxes(tempImg, _Kmatd, distCoeffs, raux, taux, 0.1);

		cv::Mat Rvec;
		raux.convertTo(Rvec, CV_32F);
		cv::Mat_<float> Rmat(3, 3);
		Rodrigues(Rvec, Rmat);

		cv::Mat_<float> Tvec;
		taux.convertTo(Tvec, CV_32F);

		fillMeasurements(measurements, Tvec, Rmat);
		// Instantiate estimated translation and rotation
		cv::Mat translation_estimated(3, 1, CV_32F);
		cv::Mat rotation_estimated(3, 3, CV_32F);
		// update the Kalman filter with good measurements
		updateKalmanFilter(KF, measurements,
			translation_estimated, rotation_estimated);

		cv::Mat Pmat = (cv::Mat_<float>(4, 4) <<
			rotation_estimated.at<float>(0, 0), rotation_estimated.at<float>(0, 1), rotation_estimated.at<float>(0, 2), translation_estimated.at<float>(0),
			rotation_estimated.at<float>(1, 0), rotation_estimated.at<float>(1, 1), rotation_estimated.at<float>(1, 2), translation_estimated.at<float>(1),
			rotation_estimated.at<float>(2, 0), rotation_estimated.at<float>(2, 1), rotation_estimated.at<float>(2, 2), translation_estimated.at<float>(2),
			0.0f, 0.0f, 0.0f, 1.0f
			);

		MarkerPmat.push_back(Pmat);
		cv::drawFrameAxes(tempImg, _Kmatd, distCoeffs, raux, taux, 0.1);
		cv::imshow("out", tempImg);
	}
	return markersOfBoardDetected;
}

cv::Rect ArucoMarkerTracker::enlargeROI(int rows, int cols, cv::Rect rect, int padding)
{
	cv::Rect returnRect = cv::Rect(rect.x - padding, rect.y - padding, rect.width + (padding * 2), rect.height + (padding * 2));
	if (returnRect.x < 0)returnRect.x = 0;
	if (returnRect.y < 0)returnRect.y = 0;
	if (returnRect.x + returnRect.width >= cols)returnRect.width = cols - returnRect.x;
	if (returnRect.y + returnRect.height >= rows)returnRect.height = rows - returnRect.y;

	return returnRect;
}

void ArucoMarkerTracker::calibrateCamera(std::string imgPath, int boardWidth, int boardHeight, int fieldSize)
{
	//Adapted from https://www.youtube.com/watch?v=E5kHUs4npX4 
	std::vector<cv::String> fileNames;
	cv::glob(imgPath + "/*.jpg", fileNames, false);
	cv::Size patternSize(boardWidth - 1, boardHeight - 1);
	std::vector<std::vector<cv::Point2f>> q(fileNames.size());

	std::vector<std::vector<cv::Point3f>> Q;

	int checkerBoard[2] = { boardWidth, boardHeight };

	std::vector<cv::Point3f> objp;
	for (int i = 1; i < checkerBoard[1]; i++) {
		for (int j = 1; j < checkerBoard[0]; j++) {
			objp.push_back(cv::Point3f(j * fieldSize, i * fieldSize, 0));
		}
	}

	std::vector<cv::Point2f> imgPoint;
	std::size_t i = 0;
	int rows = 0, cols = 0;
	for (auto const& f : fileNames) {

		cv::Mat img = cv::imread(fileNames[i]);
		cv::Mat gray;

		cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
		rows = gray.rows;
		cols = gray.cols;

		bool patternFound = cv::findChessboardCorners(gray, patternSize, q[i], cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

		if (patternFound) {
			cv::cornerSubPix(gray, q[i], cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
			Q.push_back(objp);
		}

		cv::drawChessboardCorners(img, patternSize, q[i], patternFound);
		cv::imshow("chessboard detection", img);
		cv::waitKey(0);

		i++;
	}

	cv::Matx33f K(cv::Matx33f::eye());
	cv::Vec<float, 5> k(0, 0, 0, 0, 0);

	std::vector<cv::Mat> rvecs, tvecs;
	std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
	int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 + cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;
	cv::Size frameSize(cols, rows);

	float error = cv::calibrateCamera(Q, q, frameSize, K, k, rvecs, tvecs, flags);

	std::ofstream myfile;
	myfile.open("calibparams.txt");
	myfile << "Reprojection error = " << error << "\nK = \n" << K << "\nk =\n" << k << std::endl;
	myfile.close();

	cv::Mat mapX, mapY;
	cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1, mapX, mapY);

	for (auto const& f : fileNames) {

		cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);
		cv::Mat imgUndistorted;
		cv::remap(img, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);

		cv::imshow("undistorted image", imgUndistorted);
		cv::waitKey(0);
	}
}


//====================================
// Kalman Filter functions
//====================================

//Source: https://github.com/Pold87/aruco-localization


void ArucoMarkerTracker::initKalmanFilter(cv::KalmanFilter& KF, int nStates, int nMeasurements, int nInputs, float dt)
{
	KF.init(nStates, nMeasurements, nInputs, CV_32F); // init Kalman Filter
	cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5)); // set process noise
	cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1)); // set measurement noise
	cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1)); // error covariance
	/* DYNAMIC MODEL */
	// [1 0 0 dt 0 0 dt2 0 0 0 0 0 0 0 0 0 0 0]
	// [0 1 0 0 dt 0 0 dt2 0 0 0 0 0 0 0 0 0 0]
	// [0 0 1 0 0 dt 0 0 dt2 0 0 0 0 0 0 0 0 0]
	// [0 0 0 1 0 0 dt 0 0 0 0 0 0 0 0 0 0 0]
	// [0 0 0 0 1 0 0 dt 0 0 0 0 0 0 0 0 0 0]
	// [0 0 0 0 0 1 0 0 dt 0 0 0 0 0 0 0 0 0]
	// [0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0]
	// [0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0]
	// [0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0]
	// [0 0 0 0 0 0 0 0 0 1 0 0 dt 0 0 dt2 0 0]
	// [0 0 0 0 0 0 0 0 0 0 1 0 0 dt 0 0 dt2 0]
	// [0 0 0 0 0 0 0 0 0 0 0 1 0 0 dt 0 0 dt2]
	// [0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 dt 0 0]
	// [0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 dt 0]
	// [0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 dt]
	// [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0]
	// [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0]
	// [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1]
	// position
	KF.transitionMatrix.at<float>(0, 3) = dt;
	KF.transitionMatrix.at<float>(1, 4) = dt;
	KF.transitionMatrix.at<float>(2, 5) = dt;
	KF.transitionMatrix.at<float>(3, 6) = dt;
	KF.transitionMatrix.at<float>(4, 7) = dt;
	KF.transitionMatrix.at<float>(5, 8) = dt;
	KF.transitionMatrix.at<float>(0, 6) = 0.5 * pow(dt, 2);
	KF.transitionMatrix.at<float>(1, 7) = 0.5 * pow(dt, 2);
	KF.transitionMatrix.at<float>(2, 8) = 0.5 * pow(dt, 2);
	// orientation
	KF.transitionMatrix.at<float>(9, 12) = dt;
	KF.transitionMatrix.at<float>(10, 13) = dt;
	KF.transitionMatrix.at<float>(11, 14) = dt;
	KF.transitionMatrix.at<float>(12, 15) = dt;
	KF.transitionMatrix.at<float>(13, 16) = dt;
	KF.transitionMatrix.at<float>(14, 17) = dt;
	KF.transitionMatrix.at<float>(9, 15) = 0.5 * pow(dt, 2);
	KF.transitionMatrix.at<float>(10, 16) = 0.5 * pow(dt, 2);
	KF.transitionMatrix.at<float>(11, 17) = 0.5 * pow(dt, 2);
	/* MEASUREMENT MODEL */
	// [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
	// [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
	// [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
	// [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
	// [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
	// [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]
	KF.measurementMatrix.at<float>(0, 0) = 1; // x
	KF.measurementMatrix.at<float>(1, 1) = 1; // y
	KF.measurementMatrix.at<float>(2, 2) = 1; // z
	KF.measurementMatrix.at<float>(3, 9) = 1; // roll
	KF.measurementMatrix.at<float>(4, 10) = 1; // pitch
	KF.measurementMatrix.at<float>(5, 11) = 1; // yaw
}

void ArucoMarkerTracker::fillMeasurements(cv::Mat& measurements, const cv::Mat& translation_measured, const cv::Mat& rotation_measured)
{
	// Convert rotation matrix to euler angles
	cv::Mat measured_eulers(3, 1, CV_32F);
	measured_eulers = rot2euler(rotation_measured);
	// Set measurement to predict
	measurements.at<float>(0) = translation_measured.at<float>(0); // x
	measurements.at<float>(1) = translation_measured.at<float>(1); // y
	measurements.at<float>(2) = translation_measured.at<float>(2); // z
	measurements.at<float>(3) = measured_eulers.at<float>(0); // roll
	measurements.at<float>(4) = measured_eulers.at<float>(1); // pitch
	measurements.at<float>(5) = measured_eulers.at<float>(2); // yaw
}

void ArucoMarkerTracker::updateKalmanFilter(cv::KalmanFilter& KF, cv::Mat& measurement, cv::Mat& translation_estimated, cv::Mat& rotation_estimated)
{
	// First predict, to update the internal statePre variable
	cv::Mat prediction = KF.predict();
	// The "correct" phase that is going to use the predicted value and our measurement
	cv::Mat estimated = KF.correct(measurement);
	// Estimated translation
	translation_estimated.at<float>(0) = estimated.at<float>(0);
	translation_estimated.at<float>(1) = estimated.at<float>(1);
	translation_estimated.at<float>(2) = estimated.at<float>(2);
	// Estimated euler angles
	cv::Mat eulers_estimated(3, 1, CV_32F);
	eulers_estimated.at<float>(0) = estimated.at<float>(9);
	eulers_estimated.at<float>(1) = estimated.at<float>(10);
	eulers_estimated.at<float>(2) = estimated.at<float>(11);
	// Convert estimated quaternion to rotation matrix
	rotation_estimated = euler2rot(eulers_estimated);
}

cv::Mat ArucoMarkerTracker::rot2euler(const cv::Mat& rotationMatrix)
{
	cv::Mat euler(3, 1, CV_32F);

	float m00 = rotationMatrix.at<float>(0, 0);
	float m02 = rotationMatrix.at<float>(0, 2);
	float m10 = rotationMatrix.at<float>(1, 0);
	float m11 = rotationMatrix.at<float>(1, 1);
	float m12 = rotationMatrix.at<float>(1, 2);
	float m20 = rotationMatrix.at<float>(2, 0);
	float m22 = rotationMatrix.at<float>(2, 2);

	float bank, attitude, heading;

	// Assuming the angles are in radians.
	if (m10 > 0.998) { // singularity at north pole
		bank = 0;
		attitude = CV_PI / 2;
		heading = atan2(m02, m22);
	}
	else if (m10 < -0.998) { // singularity at south pole
		bank = 0;
		attitude = -CV_PI / 2;
		heading = atan2(m02, m22);
	}
	else
	{
		bank = atan2(-m12, m11);
		attitude = asin(m10);
		heading = atan2(-m20, m00);
	}

	euler.at<float>(0) = bank;
	euler.at<float>(1) = attitude;
	euler.at<float>(2) = heading;

	return euler;
}

cv::Mat ArucoMarkerTracker::euler2rot(const cv::Mat& euler)
{
	cv::Mat rotationMatrix(3, 3, CV_32F);

	float bank = euler.at<float>(0);
	float attitude = euler.at<float>(1);
	float heading = euler.at<float>(2);

	// Assuming the angles are in radians.
	float ch = cos(heading);
	float sh = sin(heading);
	float ca = cos(attitude);
	float sa = sin(attitude);
	float cb = cos(bank);
	float sb = sin(bank);

	float m00, m01, m02, m10, m11, m12, m20, m21, m22;

	m00 = ch * ca;
	m01 = sh * sb - ch * sa * cb;
	m02 = ch * sa * sb + sh * cb;
	m10 = sa;
	m11 = ca * cb;
	m12 = -ca * sb;
	m20 = -sh * ca;
	m21 = sh * sa * cb + ch * sb;
	m22 = -sh * sa * sb + ch * cb;

	rotationMatrix.at<float>(0, 0) = m00;
	rotationMatrix.at<float>(0, 1) = m01;
	rotationMatrix.at<float>(0, 2) = m02;
	rotationMatrix.at<float>(1, 0) = m10;
	rotationMatrix.at<float>(1, 1) = m11;
	rotationMatrix.at<float>(1, 2) = m12;
	rotationMatrix.at<float>(2, 0) = m20;
	rotationMatrix.at<float>(2, 1) = m21;
	rotationMatrix.at<float>(2, 2) = m22;

	return rotationMatrix;
}


