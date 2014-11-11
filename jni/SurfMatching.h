#pragma once
#include <opencv2/opencv.hpp>
#define SURFM_CPU 1
#define SURFM_GPU 0

class SurfMatching
{
public:
	SurfMatching(bool uCPU);
	~SurfMatching(void);
	void match(cv::Mat descriptors1, cv::Mat descriptors2, std::vector<cv::DMatch> &matches);
	cv::Mat getMatchesImg(cv::Mat img1, cv::Mat img2, std::vector<cv::KeyPoint> keypoints1,std::vector<cv::KeyPoint> keypoints2);

private:
	bool useCPU;                  //true => CPU, false => GPU
	std::vector<cv::KeyPoint> keypoints1, keypoints2;
    std::vector<cv::DMatch> matches;
};

