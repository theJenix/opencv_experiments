#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/legacy/legacy.hpp"
#include <iostream>
 
using namespace cv;
using namespace std;
 
 
#include <cv.h>
#include <highgui.h>

using namespace cv;
const int MAX_CORNERS = 50;

int trackAndAnnotate(IplImage *imgA, IplImage *imgB, IplImage *imgC);
int trackAndAnnotateMat(Mat& imgA, Mat& imgB, Mat& imgC);
void cvShowImageMat(const char *name, Mat& mat);
int segmentMat(Mat& imgA, Mat& imgC);

void groundTransformProj(const Mat& input, Mat& output) {

    // resolution of the input image in the camera frame
    const Size input_size = input.size();
    const int input_x_res = input_size.width;
    const int input_y_res = input_size.height;

    // resolution of the output image in the ground frame
    const int output_x_res = 800;
    const int output_y_res = 800;

    // dimensions of the ground frame we are sampling from
    // ground frame is the rectangle from
    //   (-ground_output_x_dim / 2, 0)
    //   to (ground_output_x_dim, ground_output_y_dim)
    //   i.e. (-15 ft, 0 ft) to (15 ft, 30 ft)
    const double ground_output_y_dim = 120.0; // ft
    const double ground_output_x_dim = 120.0; // ft

    // general scale factor for the camera frame the input is in
    const double camera_scale = 1000.0; // pixels/focal-length -- ADJUST ME
    const double camera_pitch = M_PI_2 + .26; // radians from vertical -- ADJUST ME

    output = Mat::zeros(Size(output_x_res, output_y_res), input.type());
    
    // compute trig outside of the loop for such fastness
    const double cos_camera_pitch = cos(camera_pitch);
    const double sin_camera_pitch = sin(camera_pitch);
    
    for (int output_x = 0; output_x < output_x_res; output_x++) {
        for (int output_y = 0; output_y < output_y_res; output_y++) {

            // ground frame coordinates of sample point (in ft, it's all relative)
            const double x_ground = ((double) output_x / output_x_res) * ground_output_x_dim - (ground_output_x_dim / 2);
            const double y_ground = ((double) output_y / output_y_res) * ground_output_y_dim;
            const double z_ground = - 21.0 / 12.0;

            // camera frame (post-transform) coordinates of ground sample point
            const double x_camera = x_ground;
            const double y_camera = y_ground * cos_camera_pitch - z_ground * sin_camera_pitch;
            const double z_camera = y_ground * sin_camera_pitch + z_ground * cos_camera_pitch;

            if (z_camera <= 0.0) {
                // don't divide by zero or look behind the camera
                continue;
            }

            // input image coordinates
            const int input_x = camera_scale * (x_camera / z_camera) + (input_x_res / 2);
            const int input_y = camera_scale * (y_camera / z_camera) + (input_y_res / 2);
            
            if (input_y < 0 || input_y >= input_y_res || input_x < 0 || input_x >= input_x_res) {
                // don't sample outside the camera frame
                continue;
            }
            
            output.at<Vec3b>(output_y_res - output_y, output_x) =
                input.at<Vec3b>(input_y, input_x);

            // output.at<unsigned char>(output_y_res - output_y, output_x_res - output_x) =
            //     input.at<unsigned char>(input_y, input_x);
        }
    }
}
void flipHorizAndVert(Mat& img) {
	int flipMode = -1;
	IplImage ipl = img;
	cvFlip(&ipl, &ipl, flipMode);
}

void pullFrame(VideoCapture& cap, Mat& img, void (*adjustFunc)(Mat& img)) {
	Mat frame;
    cap >> frame; // get a new frame from camera
   // cvtColor(frame, frame, CV_BGR2GRAY);
	img = frame;

	if (adjustFunc) {
		adjustFunc(img);
	}
}

int main ( int argc, char **argv )
{
	// Load two images and allocate other structures
	VideoCapture cap(argv[1]); // open the default camera
	if(!cap.isOpened())  // check if we succeeded
        return -1;
	
	cvNamedWindow( "ImageA", 0 );
	cvNamedWindow( "ImageB", 0 );
	cvNamedWindow( "LKpyr_OpticalFlow", 0 );

	int flipMode = -1;
	Mat imgA, imgB, imgC;
	pullFrame(cap, imgA, NULL); //flipHorizAndVert);
	imgC = imgA;
	while(true) {	
        Mat imgAp, imgBp;
		pullFrame(cap, imgB, NULL); //flipHorizAndVert);
	Mat proj;
	groundTransformProj(imgA, proj);
	imgA = proj;
    cvtColor(imgA, imgA, CV_BGR2GRAY);
        segmentMat(imgA, imgC);
  //      cvtColor(imgAp, imgAp, CV_BGR2GRAY);
        // segmentMat(imgB, imgBp);
      //  cvtColor(imgBp, imgBp, CV_BGR2GRAY);
        // trackAndAnnotateMat(imgA, imgB, imgC);
        

		cvShowImageMat( "ImageA", imgA );
        // cvShowImageMat( "ImageB", imgB );
		cvShowImageMat( "LKpyr_OpticalFlow", imgC );
		imgA = imgB;
		imgC = imgB;
		//usleep(30 * 1000);
	}
}

void cvShowImageMat(const char *name, Mat& mat) {
	IplImage img = mat;
	cvShowImage(name, &img);
}

int trackAndAnnotate(IplImage *imgA, IplImage *imgB, IplImage *imgC) {
	Mat matA = Mat(imgA);
	Mat matB = Mat(imgB);
	Mat matC = Mat(imgC);
	return trackAndAnnotateMat(matA, matB, matC);
}

int trackAndAnnotateMat(Mat& imgA, Mat& imgB, Mat& imgC) {
	CvSize img_sz = imgA.size();
	int win_size = 15;

	// Get the features for tracking
	IplImage* eig_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
	IplImage* tmp_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );

	int corner_count = MAX_CORNERS;
	CvPoint2D32f* cornersA = new CvPoint2D32f[ MAX_CORNERS ];

	IplImage iplA = imgA;
	IplImage iplB = imgB;
	cvGoodFeaturesToTrack( &iplA, eig_image, tmp_image, cornersA, &corner_count,
		0.05, 5.0, 0, 3, 0, 0.04 );

	printf("Found %d good features..", corner_count);
	for (int ii =0; ii < corner_count; ii++) {
		printf("%d: %f, %f\n", ii, cornersA[ii].x, cornersA[ii].y);
	}

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
	for( int i=0; i < MAX_CORNERS; i++ ) {
		if (feature_errors[i] > 10) {
			printf("%d: Error is %f\n", i, feature_errors[i]);
			continue;
		}
		printf("%d: Got it, error: %f\n", i, feature_errors[i]);
		CvPoint p0 = cvPoint( cvRound( cornersA[i].x ), cvRound( cornersA[i].y ) );
		CvPoint p1 = cvPoint( cvRound( cornersB[i].x ), cvRound( cornersB[i].y ) );
		cvLine( &iplC, p0, p1, CV_RGB(255,0,0), 2 );
	}

	cvReleaseImage(&eig_image);
	cvReleaseImage(&tmp_image);
	cvReleaseImage(&pyrA);
	cvReleaseImage(&pyrB);
	delete [] cornersA;
	delete [] cornersB;
	printf("Finished\n");

	return 0;

}

Mat res, element;
 
int segmentMat(Mat& imgA, Mat& imgC) {
    /*
    CvSeq *comp;
    CvMemStorage *storage;
    IplImage src, *dst;
    int levels, block_size;
    double thresh1, thresh2;
 
    src = imgA;
    dst = cvCreateImage(cvGetSize(&src), src.depth, src.nChannels);
 
    cout << "depth: " << src.depth << " channels: " << src.nChannels << endl;
 
    block_size = 1000;
    storage = cvCreateMemStorage(block_size);
    comp = NULL;
    levels = 4;
    thresh1 = 50;
    thresh2 = 50;
 
    /* this is mandatory *//*
    src.width = dst->width = (src.width & -(1 << levels));
    src.height = dst->height = (src.height & -(1 << levels));
 
    cvPyrSegmentation(&src, dst,
    storage, &comp,
    levels, thresh1, thresh2);
 
    int n_comp = comp->total;
 
    // mapping of color value to component id
    map<int, int> mapping;
    for (int i = 0; i < n_comp; i++) {
        // only the first value of the scalar contains information as we
        // handle 1 channel gray scale images
        CvConnectedComp* cc = (CvConnectedComp*)cvGetSeqElem(comp, i);
        mapping.insert(pair<int, int>(cc->value.val[0], i));
    }
/*
uchar *data = (uchar *)dst->imageData;
int step = dst->widthStep;
for (int i = 0; i < dst->height; i++) {
for (int j = 0; j < dst->width; j++) {
uchar val = data[i*step+j];
cout << "x,y,C: " << j << "," << i << "," << mapping[val] << endl;
}
}
 *//*
    cvReleaseMemStorage(&storage);
 
    imgC = Mat(dst);
    return 0;*/
    
    Mat src = imgA;
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

      Mat new_image( src.size(), src.type() );
      for( int y = 0; y < src.rows; y++ )
        for( int x = 0; x < src.cols; x++ )
        { 
          int cluster_idx = labels.at<int>(y + x*src.rows,0);
          new_image.at<Vec3b>(y,x)[0] = centers.at<float>(cluster_idx, 0);
          new_image.at<Vec3b>(y,x)[1] = centers.at<float>(cluster_idx, 1);
          new_image.at<Vec3b>(y,x)[2] = centers.at<float>(cluster_idx, 2);
        }
        imgC = new_image;
      return 0;
}
