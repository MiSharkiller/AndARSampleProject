#ifndef MARKER_H
#define MARKER_H
#include <iostream>
#include <iomanip>
#include <opencv2\opencv.hpp>
#include <opencv2\nonfree\features2d.hpp>

class Marker
{
public:
	Marker(){}
	Marker(ARMarkerInfo *marker_info){setMarkerInfo(marker_info);}
	void setMarkerInfo( ARMarkerInfo *marker_info ){
		markerInfo = marker_info;
		area = marker_info->area;
		id = marker_info->id;
		dir = marker_info->dir;
		cf = marker_info->cf;
		memcpy(pos, marker_info->pos, 2*8);
		memcpy(line, marker_info->line, 12*8);
		memcpy(vertex, marker_info->vertex, 8*8);
	}
	void showInfo(){
		std::cout<<"marker infomation:"<<std::endl;
		std::cout<<"========================================================="<<std::endl;
		std::cout<<"area = "<<area<<std::endl;
		std::cout<<"id = "<<id<<std::endl;
		std::cout<<"dir = "<<dir<<std::endl;
		std::cout<<"cf = "<<cf<<std::endl;
		for(int i=0;i<4;i++)
			std::cout<<"line "<<i<<" = ("<<line[i][0]<<", "<<line[i][1]<<", "<<line[i][2]<<")"<<std::endl;
	
		std::cout<<"vertex:"<<std::endl;;
		for(int i=0;i<4;i++)
			std::cout<<"("<<vertex[i][0]<<", "<<vertex[i][1]<<")\n";
		std::cout<<"\n"<<std::endl;
	}
	static void showInfo(ARMarkerInfo *marker_info){
		Marker temp(marker_info);
		temp.showInfo();
	}
	void computeFeature( ARUint8 *frame ){
		/* ARtoolkit轉換到opencv */
		cv::Mat mat_frame(arImYsize, arImXsize, CV_8UC4);
		memcpy( mat_frame.data, frame, mat_frame.rows*mat_frame.cols*mat_frame.channels() );

		/* 求marker的bounding box */
		double xMin=10000.0, xMax=-1.0, yMin=10000.0, yMax=-1.0;
		for(int i=0;i<4;i++){
			if(xMin>vertex[i][0])
				xMin = vertex[i][0];
			if(xMax<vertex[i][0])
				xMax = vertex[i][0];
			if(yMin>vertex[i][1])
				yMin = vertex[i][1];
			if(yMax<vertex[i][1])
				yMax = vertex[i][1];
		}
		if(xMax >= mat_frame.cols) xMax = mat_frame.cols-1;
		if(yMax >= mat_frame.rows) yMax = mat_frame.rows-1;

		/* 將marker部分的圖獨立抓出來 */
		cv::Range xRange( (int)(xMin+0.5), (int)(xMax+0.5) );
		cv::Range yRange( (int)(yMin+0.5), (int)(yMax+0.5) );
		cv::cvtColor(mat_frame(yRange, xRange), patch_img, CV_BGRA2BGR);
		//cv::threshold(patch_img, patch_img, 128.0, 255.0, cv::THRESH_BINARY);

		/* 找出SURF Feature */
		getSurfFeature(patch_img, keypoints, desc_marker);
	}
	
	static void getSurfFeature(cv::Mat img, std::vector<cv::KeyPoint> &keypoint, cv::Mat &descriptor)
	{
		keypoint.clear();
		cv::SurfFeatureDetector surfdetector(400);
		surfdetector.detect(img, keypoint);
		
		cv::SurfDescriptorExtractor extractor;
		extractor.compute(img, keypoint, descriptor);

		/*cv::Mat keyimg;
		drawKeypoints(img, keypoint, keyimg, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
		imshow("keypoints", keyimg);*/
	}

	int     area;
    int     id;
    int     dir;
    double  cf;
    double  pos[2];
    double  line[4][3];
    double  vertex[4][2];
	double  plane[4];
	ARMarkerInfo *markerInfo;
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat patch_img;
	cv::Mat	desc_marker;
};
#endif
