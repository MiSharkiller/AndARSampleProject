// A simple demo of JNI interface to implement SIFT detection for Android application using nonfree module in OpenCV4Android.
// Created by Guohui Wang
// Email: robertwgh_at_gmail_com
// Data: 2/26/2014


#include <jni.h>
#include <android/log.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <iostream>
#include <ctime>
#include "SurfMatching.h"
#include "SurfMatching.cpp"
//#include "Marker.h"

using namespace cv;
using namespace std;

#define  LOG_TAG    "nonfree_jni_demo"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)

typedef unsigned char uchar;



cv::Mat			patt_img;
double          patt_width = 2.0;
//Marker			marker;
std::vector<cv::KeyPoint> keypoints;
cv::Mat			desc_patt;
double          patt_center[2] = {0.0, 0.0};



void homographyRefine(std::vector<cv::Point3f> patt_vertex, std::vector<cv::Point2f> marker_vertex, double threshold,
		std::vector<cv::Point3f> &objectPts, std::vector<cv::Point2f> &imagePts,
		std::vector<cv::DMatch> &matches);

void matchPoints(std::vector<cv::DMatch> &matches, Mat des1, Mat des2);
cv::Point2f nomalizeLib2Img( cv::Point2f &libPt );


//JNIEXPORT jint JNICALL Java_nativeLib_NativeLib_runDemo
//JNIEXPORT jint JNICALL Java_edu_dhbw_andar_sample_NonfreeJNILib_runDemo
extern "C" {
    JNIEXPORT jint JNICALL Java_nativeLib_NativeLib_runDemo(JNIEnv * env, jobject obj, jfloatArray contourPts, jfloatArray returnTMat);
};

JNIEXPORT jint JNICALL Java_nativeLib_NativeLib_runDemo(JNIEnv * env, jobject obj, jfloatArray contourPts, jfloatArray returnTMat)
{
	LOGI( "run demo? \n");
	
	//timer
	std::clock_t start;
    double duration;
	
    start = std::clock();
	
	
	// Input and output image path.
	const char * imgInFile = "/storage/extSdCard/pattlena_big.jpg";
	const char * imgInFile2 = "/storage/extSdCard/COOL.jpg";
	
	const char * imgOutFile = "/storage/extSdCard/getMatch.jpg";
	const char * imgOutFile2 = "/storage/extSdCard/haha.jpg";
	const char * imgOutTest = "/storage/extSdCard/cut.jpg";
	
	
	float markerVertex[8];
	float returnMat[16];
	
	//get the 4 contour points
	env->GetFloatArrayRegion(contourPts, 0, 8, markerVertex);
	
	std::vector<cv::DMatch> matches;
	std::vector<Point3f> objectPts;
	std::vector<Point2f> imagePts;
	cv::Mat rvecs(3, 1, CV_64F), tvecs(3, 1, CV_64F);
	cv::Mat rvecs_old, tvecs_old;
	cv::Mat R(3, 3, CV_64F);
	cv::Mat cparam_mat(3, 3, CV_64F);
	double	patt_trans[3][4];
	vector<KeyPoint> keypoints;
	Mat descriptors;
	Mat image, image2;
	image = imread(imgInFile, CV_LOAD_IMAGE_COLOR);
	image2 = imread(imgInFile2, CV_LOAD_IMAGE_COLOR);
	patt_img = image.clone();
	
	
	if(! image.data )
	{
		LOGI("Could not open or find the image!\n");
		return -1;
	}
	if(! image2.data )
	{
		LOGI("Could not open or find the image!\n");
		return -1;
	}
	int topLeftX, topLeftY, lowRightX, lowRightY, cutRow, cutCol;
	topLeftX = topLeftY = 10000;
	lowRightX = lowRightY = 0;
	for(int i = 0 ; i < 4 ; i++){
		if(markerVertex[i*2] < topLeftX){
			topLeftX = markerVertex[i*2];
		}
		if(markerVertex[i*2] > lowRightX){
			lowRightX = markerVertex[i*2];
		}
		if(markerVertex[i*2 + 1] < topLeftY){
			topLeftY = markerVertex[i*2 + 1];
		}
		if(markerVertex[i*2 + 1] > lowRightY){
			lowRightY = markerVertex[i*2 + 1];
		}
	}
	cutRow = lowRightY - topLeftY;
	cutCol = lowRightX - topLeftX;
	Mat CutImg(image2, Rect(topLeftX, topLeftY, cutCol, cutRow) );
	
	duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
	LOGI( "initial time %f\n", duration);
	
	
	//Mat CutImg = image2.clone();
	//imwrite(imgOutTest, CutImg);
	
	// Create a SIFT keypoint detector.
	cv::resize(image, image, cv::Size(0,0), 0.5, 0.5);
	SurfFeatureDetector detector;
	detector.detect(image, keypoints);
	LOGI("Detected %d keypoints\n", (int) keypoints.size());

	/****************/
	vector<KeyPoint> keypoints2;
	Mat descriptors2;

	//cv::resize(image2, image2, cv::Size(0,0), 0.5, 0.5);
	// Create a SIFT keypoint detector.
	SurfFeatureDetector detector2;
	detector2.detect(CutImg, keypoints2);
	LOGI("Detected %d keypoints\n", (int) keypoints2.size());
	/****************/
	
	duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
	LOGI( "detecting time %f\n", duration);
	
	
	// Compute feature description.
	SurfDescriptorExtractor extractor;
	extractor.compute(image,keypoints, descriptors);
	LOGI("Compute feature.\n");
	LOGI("a0.\n");
	/****************/
	SurfDescriptorExtractor extractor2;
	extractor2.compute(CutImg,keypoints2, descriptors2);
	LOGI("Compute feature.\n");
	/****************/
	
	duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
	LOGI( "extract time %f\n", duration);
	
	
	
	LOGI("a0.0\n");
	// Show keypoints in the output image.
	Mat outputImg;
	//Scalar keypointColor = Scalar(255, 0, 0);
	//drawKeypoints(image2, keypoints2, outputImg, keypointColor, DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	//LOGI("Drew keypoints in output image file.\n");
	
	LOGI("a0.1\n");
	Mat matchImg;
	SurfMatching testMatch(true);
	testMatch.match(descriptors, descriptors2, matches);
	LOGI("a0.2\n");
	//matches.at(0).
	duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
	LOGI( "matching time %f\n", duration);
	
	/*******HERE********/
	
	/* 只用四個邊角計算RT (Artoolkit內建) */

	//get tranlationMat
	//arGetTransMat(marker.markerInfo, patt_center, patt_width, patt_trans);
	//patt_trans[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
	patt_trans[0][0] = 1.0;
	patt_trans[0][1] = 0.0;
	patt_trans[0][2] = 0.0;
	
	patt_trans[1][0] = 0.0;
	patt_trans[1][1] = 1.0;
	patt_trans[1][2] = 0.0;
	
	patt_trans[2][0] = 0.0;
	patt_trans[2][1] = 0.0;
	patt_trans[0][2] = 1.0;
	
	
	LOGI("a1.\n");
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++){
			R.at<double>(i,j) = patt_trans[i][j];
		}
	
	Rodrigues(R, rvecs);
	for(int i=0;i<3;i++)
		tvecs.at<double>(i) = patt_trans[i][3];
	rvecs_old = rvecs.clone();
	tvecs_old = tvecs.clone();
	
	LOGI("a2.\n");
	
	/* 尋找對應點 */
	//std::vector<DMatch> matches;
	//matchPoints(matches, descriptors, descriptors2);
	/* 新增對應點到清單中 */
	LOGI("matches size = %d", matches.size());
	if(matches.size() == 0){
		LOGI("BYE");
		
	}
	for(unsigned int i=0 ; i < matches.size(); i++){
		LOGI("c0 %d\n", matches.size());
		////////signal 11
		LOGI("c0.1 = %f", keypoints.at(matches.at(i).trainIdx).pt.x);
		LOGI("c0.2 = %f", keypoints.at(matches.at(i).trainIdx).pt.y);
		cv::Point2f patt_keypos = nomalizeLib2Img(keypoints.at(matches.at(i).trainIdx).pt);
		LOGI("c1\n");
		//??????????????????????????????????????  (35, 41) (88, 86) (103, 6)
		LOGI("ksize = %d %d\n",keypoints2.size(), matches.at(i).queryIdx);
		
		cv::Point2f marker_keypos(keypoints2.at(matches.at(i).queryIdx).pt);
		LOGI("c2\n");
		marker_keypos.x += (float)markerVertex[i*2];
		marker_keypos.y += (float)markerVertex[i*2 + 1];
		//??????????????????????????????????????
		LOGI("c3\n");
		objectPts.push_back( cv::Point3f(patt_keypos.x, patt_keypos.y, 0.0) );
		//??????????????????????????????????????
		imagePts.push_back( cv::Point2f(marker_keypos.x, marker_keypos.y) );
		//??????????????????????????????????????
		/*std::cout<<cv::Point3f(patt_keypos.x, patt_keypos.y, 0.0);
		std::cout<<" ----> ";
		std::cout<<marker_keypos<<std::endl;*/
		LOGI("c4\n");
	}

	LOGI("a3!!!\n");
	/* 用四頂點算的homography去除outlier */
	/*********rewrite***********/
	std::vector<cv::Point3f> patt_vertex;
	std::vector<cv::Point2f> marker_vertex;
	LOGI("a3.0\n");
	patt_vertex.push_back(cv::Point3f((float)(patt_center[0]-patt_width/2), (float)(patt_center[1]+patt_width/2), 0.0));
	patt_vertex.push_back(cv::Point3f((float)(patt_center[0]+patt_width/2), (float)(patt_center[1]+patt_width/2), 0.0));
	patt_vertex.push_back(cv::Point3f((float)(patt_center[0]+patt_width/2), (float)(patt_center[1]-patt_width/2), 0.0));
	patt_vertex.push_back(cv::Point3f((float)(patt_center[0]-patt_width/2), (float)(patt_center[1]-patt_width/2), 0.0));
	LOGI("a3.1\n");
	
	/*???????????????????????????????4 points*/
	marker_vertex.push_back(cv::Point2f((float)markerVertex[1*2], (float)markerVertex[1*2 + 1]));
	marker_vertex.push_back(cv::Point2f((float)markerVertex[0*2], (float)markerVertex[0*2 + 1]));
	marker_vertex.push_back(cv::Point2f((float)markerVertex[3*2], (float)markerVertex[3*2 + 1]));
	marker_vertex.push_back(cv::Point2f((float)markerVertex[2*2], (float)markerVertex[2*2 + 1]));
	
	
	
	
	
	LOGI("a3.2\n");
	/*CRASH!!!!!!!!*/
	if(matches.size() > 0)
		homographyRefine(patt_vertex, marker_vertex, 2.0, objectPts, imagePts, matches);
	
	LOGI("a3.3\n");
	/******************************/
	/* 對應點清單加入四個邊角 */
	for(unsigned int i=0;i<patt_vertex.size();i++){
		objectPts.push_back(patt_vertex[i]);
		imagePts.push_back(marker_vertex[i]);
		
		LOGI("%.4f %.4f===> %.4f %.4f", patt_vertex[i].x, patt_vertex[i].y, marker_vertex[i].x, marker_vertex[i].y);
		/*std::cout<<"("<<patt_vertex[i].x<<", "<<patt_vertex[i].y<<","<<patt_vertex[i].z<<")"<<" ---> ";
		std::cout<<"("<<marker_vertex[i].x<<", "<<marker_vertex[i].y<<")"<<std::endl;*/
	}
	//showMatches(matches);
	LOGI("a4.\n");

	/*camera matrix*/
	cv::Mat distCoeffs_mat = cv::Mat::zeros(4, 1, CV_64F);
	//The intrinsic camera parameters.
	double kMat[9] = { 3.3727007117788119e+02, 0.00, 160, 0.00, 3.3727507565724787e+02, 120, 0.00, 0.00, 1.00 };
	
	for(int i = 0 ; i < 3 ; i++)
		for(int j = 0 ; j < 3 ; j++){
			cparam_mat.at<double>(i,j) = kMat[i*3 + j];
		}
	
	
	/*camera matrix END*/
	if(objectPts.size()>=4 && imagePts.size()>=4){
		
		cv::solvePnP(objectPts, imagePts, cparam_mat, distCoeffs_mat, rvecs, tvecs, false);
		
		//cv::solvePnPRansac(objectPts, imagePts, cparam_mat, distCoeffs_mat, rvecs, tvecs, true);
		
		for(int j = 0 ; j < objectPts.size() ; j++){
			LOGI("%.4f %.4f >>>>> %.4f %.4f", objectPts[j].x, objectPts[j].y, imagePts[j].x, imagePts[j].y);
		}
		
		
		
		Rodrigues(rvecs, R);
		for(int i=0;i<3;i++){

			patt_trans[i][0] = R.at<double>(i, 0);
			patt_trans[i][1] = R.at<double>(i, 1);
			patt_trans[i][2] = R.at<double>(i, 2);
			patt_trans[i][3] = tvecs.at<double>(i);

		}
		/*std::cout<<"after:\n"<<cv::Mat(3, 4, CV_64F, patt_trans)<<std::endl;
		std::cout<<rvecs<<"\n"<<tvecs<<std::endl;*/
		/*later
		if(objectPts.size()>4){
			float reproErr;
			double rms_ARTK = computeReprojectionErrors(objectPts, imagePts, rvecs_old, tvecs_old, cparam_mat, distCoeffs_mat, reproErr);
			double rms = computeReprojectionErrors(objectPts, imagePts, rvecs, tvecs, cparam_mat, distCoeffs_mat, reproErr);
			//std::cout<<"Reprojection Error:\t"<<rms<<"\tARTK: "<<rms_ARTK<<std::endl;
		}
		*/
	}
	LOGI("a5.\n");
	for(int i = 0 ; i < 3; i++){
		for(int j = 0 ; j < 4 ; j++){
			returnMat[i*4 + j] = patt_trans[i][j];
			//LOGI("mat[%d] = %f.\n", i*4+j, returnMat[i*4 + j]);
		}
	}
	
	int cont = 0;
	returnMat[cont++] = patt_trans[0][0];
	returnMat[cont++] = patt_trans[1][0];
	returnMat[cont++] = patt_trans[2][0];
	returnMat[cont++] = 0.0;
	returnMat[cont++] = patt_trans[0][1];
	returnMat[cont++] = patt_trans[1][1];
	returnMat[cont++] = patt_trans[2][1];
	returnMat[cont++] = 0.0;
	returnMat[cont++] = patt_trans[0][2];
	returnMat[cont++] = patt_trans[1][2];
	returnMat[cont++] = patt_trans[2][2];
	returnMat[cont++] = 0.0;
	returnMat[cont++] = patt_trans[0][3];
	returnMat[cont++] = patt_trans[1][3];
	returnMat[cont++] = patt_trans[2][3];
	returnMat[cont++] = 1.0;
	
	
	
	
	
	
	env->SetFloatArrayRegion(returnTMat, 0, 16, returnMat);
	
	LOGI("a6.\n");
	
	/********************/
	matchImg = testMatch.getMatchesImg(image, CutImg, keypoints, keypoints2);

	imwrite(imgOutFile2, outputImg);


#ifdef WIN32
	namedWindow("Output image", CV_WINDOW_AUTOSIZE );
	imshow("Output image", outputImg);
	waitKey(0);
#endif

	LOGI("Generate the output image.\n");
	imwrite(imgOutFile, matchImg);

	LOGI("Done.\n");
	
	
	duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
	LOGI( "computing time %f\n", duration);
	
	/*run_demo END*/
	LOGI( "End run_demo!\n");
	return 0;
}


void homographyRefine(std::vector<cv::Point3f> patt_vertex, std::vector<cv::Point2f> marker_vertex, double threshold,
		std::vector<cv::Point3f> &objectPts, std::vector<cv::Point2f> &imagePts,
		std::vector<cv::DMatch> &matches)
{
		std::vector<cv::Point2f> patt_vertex2f;
		std::vector<cv::Point2f> objectPts2f;
		std::vector<cv::Point2f> objectPts2f_H;
		std::vector<cv::Point2f> imagePts_buf;;
		std::vector<cv::DMatch> matches_old(matches);

		for(unsigned int i=0;i<patt_vertex.size();i++)
			patt_vertex2f.push_back(cv::Point2f(patt_vertex[i].x, patt_vertex[i].y));
		for(unsigned int i=0;i<objectPts.size();i++){
			objectPts2f.push_back(cv::Point2f(objectPts[i].x, objectPts[i].y));
			imagePts_buf.push_back(cv::Point2f(imagePts[i].x, imagePts[i].y));
		}
		LOGI("b1.\n");
		for(int i = 0 ; i < 4 ; i++){
			LOGI("P = %f %f --> %f %f\n",patt_vertex[i].x, patt_vertex[i].y, marker_vertex[i].x, marker_vertex[i].y);
		}
		
		
		cv::Mat H = findHomography(patt_vertex2f, marker_vertex, CV_RANSAC);
		for(int i = 0 ; i < 3 ; i++){
			LOGI("H = %f %f %f\n",H.at<double>(i, 0), H.at<double>(i, 1), H.at<double>(i, 2));
		}
		
		LOGI("b2??\n");
		LOGI("b2 = %d\n", objectPts2f.size());
		
		for(int i = 0 ; i < objectPts2f.size() ; i++){
			LOGI("O = %f %f\n",objectPts2f[i].x , objectPts2f[i].y);
		}
		
		
		cv::perspectiveTransform(objectPts2f, objectPts2f_H, H);
		LOGI("b3.\n");
		objectPts.clear();
		imagePts.clear();
		matches.clear();
		for(unsigned int i=0;i<objectPts2f_H.size();i++){
			double err= sqrt(pow(objectPts2f_H[i].x-imagePts_buf[i].x, 2)+pow(objectPts2f_H[i].y-imagePts_buf[i].y, 2));
			if(err < threshold){
				objectPts.push_back(cv::Point3f(objectPts2f[i].x, objectPts2f[i].y, 0.0));
				imagePts.push_back(imagePts_buf[i]);
				matches.push_back(matches_old[i]);
				if(err!=0)
					std::cout<<err<<std::endl;
			}
		}
}
void matchPoints(std::vector<cv::DMatch> &matches, Mat des1, Mat des2)
{
	/* 用FLANN的方法match feature */
		std::vector<cv::DMatch> matches_temp;
		FlannBasedMatcher matcher;
		////////////////////??????????????????????????????????
		matcher.match(des1, des2, matches_temp);
		////////////////////??????????????????????????????????

		double max_dist = 0, min_dist = 100;
		std::cout<<"match:\t"<<matches_temp.size()<<std::endl;
		for(unsigned int i=0;i<matches_temp.size(); i++){
			double dist = matches_temp[i].distance;
			if(dist < min_dist) min_dist = dist;
			if(dist > max_dist) max_dist = dist;
		}
		std::cout<<"-- Min dist : "<<min_dist<<std::endl;
		std::cout<<"-- Max dist : "<<max_dist<<std::endl;
		for(unsigned int i=0;i<matches_temp.size();i++){
			cv::Point2f patt_keypos = nomalizeLib2Img(keypoints.at(matches_temp.at(i).trainIdx).pt);
			if(matches_temp[i].distance < 3*min_dist)
				if(patt_keypos.inside(cv::Rect((int)(-patt_width/4), (int)(-patt_width/4), (int)(patt_width/2), (int)(patt_width/2) )))
					matches.push_back(matches_temp[i]);
		}
		std::cout<<"good match:\t"<<matches.size()<<std::endl;
}
cv::Point2f nomalizeLib2Img( cv::Point2f &libPt )
{
		cv::Point2f imgPt;
		imgPt.x = (float)((libPt.x*patt_width/patt_img.cols)-(patt_width/2));
		imgPt.y = (float)((libPt.y*patt_width/patt_img.cols)-(patt_width/2));
		return imgPt;
}


