/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"
#include <fstream>

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time 
    bool bVis = false;            // visualize results

    //Ring Buffer Implementation using vector type

    //deque<DataFrame> dataBuffer;

    vector<string> detectorTypes   = { "SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};  
    vector<string> descriptorTypes = { "BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};                        
    vector<string> matcherTypes    = { "MAT_BF"}; //MAT_FLANN (implemented but not used)
    vector<string> selectorTypes   = { "SEL_KNN"}; // SEL_NN (not used)

    vector<ParameterList> CollectionOfTypes;
    for(const auto& det_type : detectorTypes)
    {
        for(const auto& desc_type : descriptorTypes)
            {
                for(const auto& matcher_type : matcherTypes)
                {
                    for(const auto& selector_type : selectorTypes)
                    {
                        ParameterList singleType;
                        singleType.detectorType   = det_type;
                        singleType.descriptorType = desc_type;
                        singleType.matchingType   = matcher_type;
                        singleType.selectorType   = selector_type;
                        for( int i=0 ; i < 10; i++)
                        {
                            singleType.detectorTime[i] =0.0f;
                            singleType.descriptorTime[i] =0.0f;
                            singleType.MatcherTime[i]= 0.0f;
                            singleType.numKeyPointsDetected[i] = 0;
                            singleType.numMatchedKeyPoints[i]  = 0;
                            singleType.numKeyPointsOnVehicle[i]   = 0;
                        }
                        CollectionOfTypes.push_back(singleType);

                    }
                }
            }       
    }


    for( auto& oneType : CollectionOfTypes )
    {
        string detectorType   = oneType.detectorType;
        string descriptorType = oneType.descriptorType;
        string matcherType    = oneType.matchingType;
        string selectorType   = oneType.selectorType;

        if ((detectorType.compare("AKAZE") != 0 && descriptorType.compare("AKAZE") == 0) 
            || (detectorType.compare("SIFT") == 0 && descriptorType.compare("ORB") == 0))
        continue;

    /* MAIN LOOP OVER ALL IMAGES */

    dataBuffer.clear();
    cout << endl <<"Detector - Descriptor Combination: " << detectorType <<"--" << descriptorType << "--"<< endl;

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ResultOutput output = {0,0.0f};
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        //cout<<dataBuffer.size()<<endl;
        if(dataBuffer.size() < dataBufferSize)
            {
                dataBuffer.push_back(frame);
            }
            else
            {
                dataBuffer.erase(dataBuffer.begin());
                dataBuffer.push_back(frame);
            } 
        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image

        //SHITOMASI, HARRIS, FAST, BRISK, ORB AKAZE, SIFT detectors
        //string detectorType = "HARRIS";

        /* EXTRACT KEYPOINT DESCRIPTORS */
        //cv::Mat descriptors;
        //string descriptorType = "BRIEF"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT descriptors

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        if (detectorType.compare("SHITOMASI") == 0)
        {
            output = detKeypointsShiTomasi(keypoints, imgGray, false);
        }
        else if (detectorType.compare("HARRIS") == 0)
        {
            output = detKeypointsHarris(keypoints, imgGray, false);            
        }
        else if (detectorType.compare("FAST") == 0||detectorType.compare("BRISK") == 0||
        detectorType.compare("ORB") == 0||detectorType.compare("AKAZE") == 0||
        detectorType.compare("SIFT") == 0)
        {
            output = detKeypointsModern(keypoints, imgGray, detectorType , false);
        }
        else
        {
            throw invalid_argument("Entered detector " + detectorType + " is not a valid type");
        }

        oneType.numKeyPointsDetected[imgIndex] = output.numPoints;
        oneType.detectorTime[imgIndex] = output.timeTaken_ms;
        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            vector<cv::KeyPoint> selectedKeyPoints;
            for(auto kpts : keypoints)
            {
                if(vehicleRect.contains(kpts.pt))
                {
                    selectedKeyPoints.push_back(kpts);
                }
            }
            keypoints = selectedKeyPoints;
            cout<<"Number of Keypoints on the preceding vehicle "<<keypoints.size()<<endl;
            oneType.numKeyPointsOnVehicle[imgIndex] = keypoints.size();
        }

        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        //string descriptorType = "SIFT"; // BRIEF, ORB, FREAK, AKAZE, SIFT
        output = descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        oneType.descriptorTime[imgIndex] = output.timeTaken_ms;
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) 
        //cout<<"I am not getting this"<<endl;// wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            
            vector<cv::DMatch> matches;
            //string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
            string descriptorSType = "DES_BINARY";
            if (descriptorType.compare("SIFT") == 0)
            {
                descriptorSType = "DES_HOG"; // DES_BINARY, DES_HOG
                //cout<<"Selected HOG type for SIFT"<<endl;
            }
            
            
            //string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN
            
            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            output = matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorSType, matcherType, selectorType);

            //// EOF STUDENT ASSIGNMENT

            oneType.numMatchedKeyPoints[imgIndex] = output.numPoints;
            oneType.MatcherTime[imgIndex] = output.timeTaken_ms;

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            bVis = false;
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
            }
        }

    } // eof loop over all images

    std::string filename = "../Performance Evaluation.xlsx";
    std::ofstream outputFile(filename, ios::out);

    outputFile << "Frame Number"<<","<< "Detector Type"<< ","<< "Detection Time(ms)"<< ","
               << "Number of keypoints detected"<< ","<< "Number of KeyPoints n preceding vehicle "<< ","
               << "Descriptor Type"<<","<< "Descriptor Time(ms)"<< ","
               << "Number of Matches"<< ","<< "MatchingTime(ms))"<< std::endl;

    for( const auto& oneType : CollectionOfTypes)
    {
        for( int i= 0 ; i < 10; i++)
        {
            outputFile << i+1 << "," 
                       << oneType.detectorType<< "," 
                       << fixed << std::setprecision(5) << oneType.detectorTime[i]<< "," 
                       << oneType.numKeyPointsDetected[i]<< "," 
                       << oneType.numKeyPointsOnVehicle[i]<< "," 
                       << oneType.descriptorType<< "," 
                       << fixed << std::setprecision(5) << oneType.descriptorTime[i]<< "," 
                       << oneType.numMatchedKeyPoints[i]<< "," 
                       << std::fixed << std::setprecision(5) << oneType.MatcherTime[i]<< "," 
                       << endl;
        }
        outputFile << endl;
    }
    outputFile.close();

    return 0;
}

