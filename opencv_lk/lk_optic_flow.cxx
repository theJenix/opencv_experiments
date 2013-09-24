#include <cv.h>
#include <highgui.h>
#include "math.h"

using namespace cv;
const int MAX_CORNERS = 10;

int trackAndAnnotateMat(Mat& imgA, Mat& maskA, Mat& imgB, Mat& imgC);
void cvShowImageMat(const char *name, Mat& mat);

void flipHorizAndVert(Mat& img) {
	int flipMode = -1;
	IplImage ipl = img;
	cvFlip(&ipl, &ipl, flipMode);
}

void pullFrame(VideoCapture& cap, Mat& img, Mat& imgGray, void (*adjustFunc)(Mat& img)) {
	Mat frame;
    cap >> frame; // get a new frame from camera
	img = frame;

	if (adjustFunc) {
		adjustFunc(img);
	}
    imgGray = img.clone();
    cvtColor(imgGray, imgGray, CV_BGR2GRAY);
}

// Hardcoded values for green android folder
Scalar LOW_HSV = Scalar(60, 50, 50);
Scalar HIGH_HSV = Scalar(90, 255, 255);


// Get binary thresholded image
// low_HSV, hi_HSV - low, high range values for threshold as a list [H,S,V]
// debug= True to display the binary image generated
void getBinary(Mat& src, Scalar& low_HSV, Scalar& hi_HSV, Mat& dest) {
    Mat frame = src.clone();
    cvtColor(frame, frame, CV_BGR2HSV);
    inRange(frame, low_HSV, hi_HSV, dest);
}
            
int main ( int argc, char **argv )
{
	// Load two images and allocate other structures
	VideoCapture cap("/Users/theJenix/Development/opencv_experiments/opencv_lk/recording2.mov"); // open the default camera
	if(!cap.isOpened())  // check if we succeeded
        return -1;
	
	cvNamedWindow( "ImageA", 0 );
	cvNamedWindow( "ImageB", 0 );
	cvNamedWindow( "LKpyr_OpticalFlow", 0 );

	int flipMode = -1;
	Mat imgA, imgGrayA, imgB, imgGrayB, imgC;
	pullFrame(cap, imgA, imgGrayA, flipHorizAndVert);
	imgC = imgA;
	while(true) {
        Mat maskA, discard;
        getBinary(imgA, LOW_HSV, HIGH_HSV, maskA);
		pullFrame(cap, imgB, imgGrayB, flipHorizAndVert);
        trackAndAnnotateMat(imgGrayA, maskA, imgGrayB, imgGrayA);

		cvShowImageMat( "ImageA", imgGrayA );
        cvShowImageMat( "ImageB", maskA );
		cvShowImageMat( "LKpyr_OpticalFlow", imgGrayA );
		imgA = imgB;
        imgGrayA = imgGrayB;
		imgC = imgB;
		usleep(1000 * 1000);
	}
}

void cvShowImageMat(const char *name, Mat& mat) {
	IplImage img = mat;
	cvShowImage(name, &img);
}

int roundAndMap(double data, int precision, int minVal, int maxVal) {
    int factor     = pow(10.0, precision);
    double rounded = (int)(data * factor) / factor;
    return fmin(fmax(minVal, rounded), maxVal) - minVal;
}

int binBySimpleAssignment(double samples[], int nSamples, int bins[], int nBins, int precision) {
    
    for (int i = 0; i != nSamples; ++i) {
        bins[roundAndMap(samples[i], precision, -3, 3)]++;
    }
    return 0;
}

/*
int binByKmeans(double samples[], int nSamples, int bins[], int nBins) {
Mat samples(src.rows * src.cols, 3, CV_32F);
  for( int y = 0; y < src.rows; y++ )
    for( int x = 0; x < src.cols; x++ )
      for( int z = 0; z < 3; z++) {
        samples.at<float>(y + x*src.rows, z) = src.at<Vec3b>(y,x)[z];
    }
  int clusterCount = 3;
  Mat labels;
  int attempts = 5;
  Mat centers;
  kmeans(samples, clusterCount, labels, TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10000, 0.0001), attempts, KMEANS_PP_CENTERS, centers );
}
*/

// I think this uses the population mean and stddev formulae
// Samples in the form of angles
// C++ MAGIC (nSamples)
// Filters samples to accept 
void filterByMeanAndStdDev(double samples[], int nSamples, double filteredSamples[], int& numFilteredSamples, int numStdDevAllowed = 1) {
	double *mean;
	double *std_dev;
    cvAvgSdv(samples, mean, std_dev);

    numFilteredSamples = 0;
    for (int i = 0; i < nSamples; i++) {
    	if (abs(samples[i] - mean) < std_dev * numStdDevAllowed) {
    		filteredSamples[numFilteredSamples] = samples[i];
    		numFilteredSamples++;
    	}
    }
}

int trackAndAnnotateMat(Mat& imgA, Mat& maskA, Mat& imgB, Mat& imgC) {
	CvSize img_sz = imgA.size();
	int win_size = 15;

	// Get the features for tracking
	IplImage* eig_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
	IplImage* tmp_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );

	int corner_count = MAX_CORNERS;
	CvPoint2D32f* cornersA = new CvPoint2D32f[ MAX_CORNERS ];

	IplImage iplA = imgA;
	IplImage iplB = imgB;
        IplImage maskIplA = maskA;
	cvGoodFeaturesToTrack( &iplA, eig_image, tmp_image, cornersA, &corner_count,
		0.05, 5.0, &maskIplA, 3, 0, 0.04 );

//	printf("Found %d good features..", corner_count);
//	for (int ii =0; ii < corner_count; ii++) {
//		printf("%d: %f, %f\n", ii, cornersA[ii].x, cornersA[ii].y);
//	}

	cvFindCornerSubPix( &iplA, cornersA, corner_count, cvSize( win_size, win_size ),
		cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03 ) );

	// Call Lucas Kanade algorithm
	char features_found[ MAX_CORNERS ];
	float feature_errors[ MAX_CORNERS ];

	CvSize pyr_sz = cvSize( img_sz.width+8, img_sz.height/3 );

	IplImage* pyrA = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
	IplImage* pyrB = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );

	CvPoint2D32f* cornersB = new CvPoint2D32f[ MAX_CORNERS ];

	cvCalcOpticalFlowPyrLK( &iplA, &iplB, pyrA, pyrB, cornersA, cornersB, corner_count, 
		cvSize( win_size, win_size ), 5, features_found, feature_errors,
		 cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3 ), 0 );

	// Make an image of the results
 	IplImage iplC = imgC;
    double angles[MAX_CORNERS] = {0};
    int angle_count = 0;
	for( int i=0; i < MAX_CORNERS; i++ ) {
        if (!features_found[i]) {
            // printf("Feature not found\n");
            continue;
        }
		if (feature_errors[i] > 10) {
//			printf("%d: Error is %f\n", i, feature_errors[i]);
			continue;
		}
//		printf("%d: Got it, error: %f\n", i, feature_errors[i]);
		CvPoint p0 = cvPoint( cvRound( cornersA[i].x ), cvRound( cornersA[i].y ) );
		CvPoint p1 = cvPoint( cvRound( cornersB[i].x ), cvRound( cornersB[i].y ) );
        double dist = sqrt(pow(cornersA[i].x - cornersB[i].x, 2) + pow(cornersA[i].y - cornersB[i].y, 2));
        if (dist > 100) {
            // printf("Vector length: %f\n", dist);
            // printf("%d,%d to %d,%d\n", p0.x, p0.y, p1.x, p1.y);
        } else {
            double angle = atan2(p1.y - p0.y, p1.x - p0.x);
            angles[i] = angle;
            angle_count++;
            cvLine( &iplC, p0, p1, CV_RGB(255,0,0), 2 );
        }
	}
    for (int i = 0; i < angle_count; i++) {
        printf("%d: %f\n", i, angles[i]);
    }
    
    // int counts[7] = {0};
    // binBySimpleAssignment(angles, angle_count, counts, 7, 0);
    // for (int i = 0; i < 7; i++) {
    //     printf("%d: %d\n", i - 3, counts[i]);
    // }

    double filteredSamples[MAX_CORNERS];
    int numFilteredSamples;
    filterByMeanAndStdDev(angles, angle_count, filteredSamples, numFilteredSamples);

	cvReleaseImage(&eig_image);
	cvReleaseImage(&tmp_image);
	cvReleaseImage(&pyrA);
	cvReleaseImage(&pyrB);
	delete [] cornersA;
	delete [] cornersB;
	printf("Finished\n");

	return 0;

}
