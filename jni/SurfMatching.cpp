#include "SurfMatching.h"

//#include <opencv2\nonfree\ocl.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
using namespace cv;
//using namespace cv::ocl;

template<class KPMatcher>
struct SURFMatcher
{
    KPMatcher matcher;
    template<class T>
    void match(const T& in1, const T& in2, vector<cv::DMatch>& matches)
    {
        matcher.match(in1, in2, matches);
    }
};

SurfMatching::SurfMatching(bool uCPU):useCPU(uCPU)
{
	if(!useCPU){

		/*std::cout
                << "Device name:"
                << cv::ocl::Context::getContext()->getDeviceInfo().deviceName
                << std::endl;*/
	}
}


SurfMatching::~SurfMatching(void)
{
}

void SurfMatching::match(cv::Mat descriptors1, cv::Mat descriptors2, std::vector<cv::DMatch> &match)
{
    SURFMatcher<BFMatcher>      cpp_matcher;
    //SURFMatcher<BFMatcher_OCL>  ocl_matcher;
    double min_dist, max_dist;

	if(useCPU){
		cpp_matcher.match(descriptors1, descriptors2, matches);
	}
	else{
		//oclMat descriptors1_GPU(descriptors1), descriptors2_GPU(descriptors2);
		//ocl_matcher.match(descriptors1_GPU, descriptors2_GPU, matches);
	}
	match = matches;

	//-- Quick calculation of max and min distances between keypoints
	  for( int i = 0; i < descriptors1.rows; i++ )
	  { double dist = matches[i].distance;
	    if( dist < min_dist ) min_dist = dist;
	    if( dist > max_dist ) max_dist = dist;
	  }

	  printf("-- Max dist : %f \n", max_dist );
	  printf("-- Min dist : %f \n", min_dist );

	  //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
	  //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
	  //-- small)
	  //-- PS.- radiusMatch can also be used here.
	  std::vector< DMatch > good_matches;

	  for( int i = 0; i < descriptors1.rows; i++ )
	  { if( matches[i].distance <= max(2*min_dist, 0.1) )
	    { good_matches.push_back( matches[i]); }
	  }
	  match = good_matches;
	  matches = good_matches;

}

Mat SurfMatching::getMatchesImg(Mat img1, Mat img2, std::vector<cv::KeyPoint> keypoints1,std::vector<cv::KeyPoint> keypoints2)
{
	Mat img_matches;
	drawMatches( img1, keypoints1, img2, keypoints2,
		matches, img_matches, Scalar::all(-1), Scalar::all(-1),
		vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	return img_matches;
}
