#ifndef dataStructures_h
#define dataStructures_h
using namespace std;
#include <vector>
#include <opencv2/core.hpp>


struct DataFrame { // represents the available sensor information at the same time instance
    
    cv::Mat cameraImg; // camera image
    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
};

struct ParameterList {
    string detectorType;
    string descriptorType;
    string matchingType;
    string selectorType;
    int numKeyPointsDetected[10];
    int numKeyPointsOnVehicle[10];
    int numMatchedKeyPoints[10];
    double detectorTime[10];
    double descriptorTime[10];
    double MatcherTime[10];
    };

struct ResultOutput {
        int numPoints;
        double timeTaken_ms;
};


#endif /* dataStructures_h */
