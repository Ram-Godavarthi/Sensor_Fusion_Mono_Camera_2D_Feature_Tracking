# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />



MP.1 Data Buffer Optimization: Implement a vector for dataBuffer objects whose size does not exceed a limit (e.g. 2 elements) This can be achieved  by  pushing  in new  elements on one  end  and removing elements on  the other end.

    -----Ring buffer of vector type has been implemented with dataBufferSize = 2. New image is fed from one        side and old image is erased from other side by always keeping the buffer size 2.


MP.2 Keypoint Detection: Implement detectors HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT and make them selectable by setting a string accordingly.

            3 Keypoint detector functions have been implemented.  
            1. Shitomasi detector
            2. Harris detector
            3. Modern detectors (HARIS, FAST, BRISK, ORB, AKAZE, SIFT)
             OpenCV build-in detector class will be initialized and whole image will be scaned to detect key-points. Number of Keypoints and detection time are calculated for performance evaluation.

MP.3 Keypoint Removal: Remove all keypoints outside of a pre-defined rectangle and only use the keypoints within the rectangle for further processing.

        Predefined rectangular area that covers the vehicle has been used to remove the keypoints outside that area. Here, the keypoints withing the rectangle indicates the points on the preceding vehicle. The number of keypoints on preceding vehicle are also estimated.

MP.4 Keypoint Descriptors: Implement descriptors BRIEF, ORB, FREAK, AKAZE and SIFT and make them selectable by setting a string accordingly.

        Different descriptors like BRIEF, ORB, FREAK, AKAZE and SIFT  including the pre-built BRISK are implemented and used in this project. OpenCV built in descriptor class with defualt parameters have been used and the perfomrance is evaluated.

MP.5 Descriptor Matching: Implement FLANN matching as well as k-nearest neighbor selection. Both methods must be selectable using the respective strings in the main function.

        String based selection for both Brute-force matcher(MAT_BF) and FLANN matcher(MAT_FLANN) is implemented. In the project MAT_BF is used for evaluation purpose. Histogram of orientation or Binary type will be activated automatically depending on the descriptor type. Also implemented KNN selection with default of k = 2 nearest neighors.

MP.6 Descriptor Distance Ratio: Use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.

        String based selection for both Nearest neighbor and K-Nearest Neighbor is implemented. KNN is used in the project implementation with Descriptor Distance Ratio of 0.8 to filter out the keypoints to make best match.

MP.7 Performance Evaluation 1: Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.

        Detector Type 	Number of Keypoints on preceding vehicle for all 10 images
        SHITOMASI 	            1179
        HARRIS 	                248
        FAST 	                1491
        BRISK 	                2762
        ORB 	                1161
        AKAZE 	                1670
        SIFT 	                1386

MP.8 Performance Evaluation 2: Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.
                                    Descriptors
        Detector            	BRISK 	BRIEF 	ORB 	FREAK 	AKAZE 	SIFT
        SHITOMASI 	             767 	 944 	 908 	 768 	   0     927
        HARRIS 	                 142 	 173 	 162 	 144 	   0 	 163
        FAST 	                 899 	1099 	1071 	 878 	   0 	1046
        BRISK 	                1570 	1704 	1514 	1524 	   0 	1646
        ORB 	                 751 	 545 	 763 	 420 	   0     763
        AKAZE 	                1215 	1266 	1182 	1187 	1259 	1270
        SIFT 	                 592 	 702 	   0 	 593 	   0 	 800

MP.9 Performance Evaluation 2: Log the time it takes for keypoint detection  and descriptor extraction.
The result must be entered into a spreadsheet  and based  on this  data, the TOP3  detector / descriptor combinations  must be recommanded as the best choice for  out purpose  of detecting  keypoiints on  vehicles.

Total detection and extraction time of Detector and descriptor on all 10 images for all combinations is listed below (All time values in ms).

                                                    Descriptors
        Detector            	BRISK 	    BRIEF 	    ORB 	    FREAK 	    AKAZE   	SIFT
        SHITOMASI 	            19.040 	    18.529 	    17.971 	    54.723 	    -           13.953
        HARRIS 	                16.360 	    15.499 	    15.190 	    53.379 	    -           30.018
        FAST 	                 3.045 	    1.771 	    1.922	    42.362      -           21.607	
        BRISK 	                44.117 	    41.692 	    45.239 	    83.886 	    -           85.420
        ORB 	                9.238 	    8.070 	    11.913 	    47.540 	    -           54.913
        AKAZE 	                75.228 	    79.988	    79.708 	    118.158     132.906	    97.347
        SIFT 	                127.863 	127.969 	   - 	    166.634     - 	        187.878


Top 3 detector/descriptor combinations: These have been selected based on their performance merits such as highest number of matched keypoints and less processing time.


        DETECTOR/DESCRIPTOR 	    NUMBER OF KEYPOINTS 	    TIME
        FAST+BRIEF 	                1099 keypoints             	1.771 ms
        FAST+ORB 	                1071 keypoints 	            1.922 ms
        FAST+BRISK 	                899 keypoints 	            3.045 ms