#include <cv.h>
#include <highgui.h>
#include "math.h"

using namespace cv;
const int MAX_CORNERS = 100;

int trackAndAnnotateMat(Mat& imgA, Mat& maskA, Mat& imgB, Mat& imgC);
void cvShowImageMat(const char *name, Mat& mat);
void getBinary(Mat& src, Scalar& low_HSV, Scalar& hi_HSV, Mat& dest);
void cannyThreshold(Mat& src, Mat& srcGray, Mat& dst, Mat& dstMask);
void removeSmall(Mat& src, Mat& dest, double minRadius);
void groundTransform(Mat& src, Mat& ground);

Scalar LOW_HSV_EDGE = Scalar(20, 0, 0);
Scalar HIGH_HSV_EDGE = Scalar(40, 255, 255);

/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */
void cannyThreshold(Mat& src, Mat& srcGray, Mat& dst, Mat& dstMask)
{
    int edgeThresh = 1;
    int lowThreshold = 60; //120;
    int const max_lowThreshold = 200;
    int ratio = 3;
    int kernel_size = 3;
    Mat detected_edges;
  /// Reduce noise with a kernel 3x3
  blur( srcGray, detected_edges, Size(3,3) );
  /// Canny detector
  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  /// Using Canny's output as a mask, we display our result
  dst = Scalar::all(0);

  src.copyTo( dst, detected_edges);
  Mat mask;
  // getBinary(dst, LOW_HSV_EDGE, HIGH_HSV_EDGE, dstMask);
  // imshow( window_name, dst );
  cvtColor(dst, dstMask, CV_BGR2GRAY);
  
 }

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
// Scalar LOW_HSV = Scalar(60, 50, 50);
// Scalar HIGH_HSV = Scalar(90, 255, 255);
// Yellow line
Scalar LOW_HSV = Scalar(25, 120, 160);
Scalar HIGH_HSV = Scalar(35, 255, 255);
// Scalar LOW_HSV = Scalar(20, 200, 250);
// Scalar HIGH_HSV = Scalar(40, 255, 255);

void removeSmall(Mat& src, Mat& dest, double minRadius) {
    vector<vector<Point> > contours;
    findContours(src.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    vector<vector<Point> > erase;

    /// Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point> > contours_poly( contours.size() );
    // vector<Rect> boundRect( contours.size() );
    // vector<Point2f>center( contours.size() );
    // vector<float>radius( contours.size() );

    for( int i = 0; i < contours.size(); i++ ) {
        Point2f center;
        float radius;
        approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
        // boundRect[i] = boundingRect( Mat(contours_poly[i]) );
        minEnclosingCircle( (Mat)contours_poly[i], center, radius );
        if (radius < minRadius) {
            erase.push_back(contours[i]);
        }
    }
    
    // 
    // dest = Mat::zeros(bw.size(), bw.type());
    dest = src.clone();
    drawContours(dest, erase, -1, Scalar::all(0), CV_FILLED);    
}

// Get binary thresholded image
// low_HSV, hi_HSV - low, high range values for threshold as a list [H,S,V]
// debug= True to display the binary image generated
void getBinary(Mat& src, Scalar& low_HSV, Scalar& hi_HSV, Mat& dest) {
    Mat frame = src.clone();
    cvtColor(frame, frame, CV_BGR2HSV);
    Mat bw;
    inRange(frame, low_HSV, hi_HSV, bw);
    
    removeSmall(bw, dest, 10);
}
            
int main ( int argc, char **argv )
{
    
    char resolved_path[256];
    realpath(argv[1], resolved_path);
    printf("Opening movie file \n%s\n",resolved_path);
	
    // Load two images and allocate other structures
    VideoCapture cap(resolved_path); // open the default camera
    // VideoCapture cap("/Users/theJenix/Development/opencv_experiments/opencv_lk/recording1.mov"); // open the default camera
	if(!cap.isOpened())  // check if we succeeded
        return -1;
	
	cvNamedWindow( "ImageB", 0 );
    cvNamedWindow( "TransformB", 0 );
    // cvNamedWindow( "LKpyr_OpticalFlow", 0 );

	int flipMode = -1;
	Mat imgA, imgGrayA, imgB, imgGrayB, imgC;
	pullFrame(cap, imgA, imgGrayA, NULL); //flipHorizAndVert);
	imgC = imgA;
	while(true) {
        Mat maskA, discard;
        getBinary(imgA, LOW_HSV, HIGH_HSV, maskA);
        
        Mat thold, edge, edgeMask;
        //binary threshold for the really bright stuff
        
        cannyThreshold(imgA, imgGrayA, edge, edgeMask);
        // removeSmall(edgeMask, edgeMask, 50);
        threshold( imgGrayA, thold, 220, 255, 0);
        // printf("%d, %d, %d", thold.size().width, thold.size().height, thold.channels());
        // printf("%d, %d, %d", maskA.size().width, maskA.size().height, maskA.channels());
        maskA = maskA + thold + edgeMask;
        removeSmall(maskA, maskA, 100);
        
        Mat gt;
        groundTransform(imgA, gt);
        
		pullFrame(cap, imgB, imgGrayB, NULL); //flipHorizAndVert);

        // cvShowImageMat( "ImageA", thold );
        cvShowImageMat( "ImageB", imgA );
        cvShowImageMat( "TransformB", gt);
        
        // cvShowImageMat( "LKpyr_OpticalFlow", imgGrayA );
		imgA = imgB;
        imgGrayA = imgGrayB;
		imgC = imgB;
        // usleep(30 * 1000);
	}
}

// void lensTransform(Point& pt, double h, double cameraAngle, double& vec[2]) {
// }
// void groundTransform(Point& framePt, Point& groundPt, double h, double cameraAngle) {
//     double vec[2];
//     lensTransform(framePt, h, cameraAngle, vec);
//     double x = sqrt( (pow(-h / cos(vec[0]), 2) - pow(h, 2))
//                       / (1 + pow(tan(vec[1]), 2)) );
//     double y = x * tan(phi);
//     
// }

// void groundTransform(Mat& src, Mat& ground) {
//     int width = 1280;
//     int height = 720;
//     
//     int x_res = 400;
//     int y_res = 400;
// 
//     ground = Mat::zeros(cvSize(x_res, y_res), src.type());
// 
//     double groundWidth = 20.0; //ft
//     double groundHeight = 20.0; //ft
//     double h = 21.0/12.0; //ft
//     double deg_fov = 120.0;
//     double phi_scale = 100000 * (M_PI / 180) * (deg_fov / width);
//     double theta_scale = 100000 * (M_PI / 180) * (deg_fov / width);
//     int x_frame_off = width/2;
//     int y_frame_off = 0;
//     
//     for (int x = 0; x < x_res; x++) {
//         for (int y = 0; y < y_res; y++) {
//             double x_ground = x / groundWidth - (groundWidth/2);
//             double y_ground = y / groundHeight;
//             
//             double theta = acos(-h/ sqrt(pow(x_ground, 2) + pow(y_ground, 2) + pow(h, 2)));
//             double phi = atan2(y_ground, x_ground);
//             int x_frame = phi * phi_scale + x_frame_off;
//             int y_frame = theta * theta_scale + y_frame_off;
//             ground.at<double>(y, x) = src.at<double>(y_frame, x_frame);
//             if (x == 0 && y == 0) {
//                 printf("%d, %d, %f, %f, %f, %f, %f\n", x_frame, y_frame, src.at<double>(y_frame, x_frame), theta, phi, theta_scale, phi_scale);
//             }
//         }
//     }   
// }

void groundTransform(Mat& src, Mat& ground) {
    int width = 1280;
    int height = 720;
    
    int x_res = 800;
    int y_res = 800;

    ground = Mat::zeros(cvSize(x_res, y_res), src.type());

    double groundWidth = 30.0; //ft
    double groundHeight = 30.0; //ft
    double h = 21.0/12.0; //ft
    double deg_fov = (120.0 / 180.0) * M_PI;
    double phi_scale = width / deg_fov;
    double theta_scale = height / deg_fov * 1.0;
    double x_frame_off = x_res/2;
    double y_frame_off = y_res/10;
    double phi_off = -1.15;
    double theta_off = -1.55;
    
    for (int x = 0; x < x_res; x++) {
        for (int y = 0; y < y_res; y++) {
            double x_ground = x * groundWidth / x_res - (groundWidth / 2);
            double y_ground = y * groundHeight / y_res;
            
            double theta = acos(-h/ sqrt(pow(x_ground, 2) + pow(y_ground, 2) + pow(h, 2)));
            double phi = atan2(y_ground, x_ground);
            int x_frame = (phi + phi_off) * phi_scale + x_frame_off;
            int y_frame = (theta + theta_off) * theta_scale + y_frame_off;
            // if (x == 0 && y == 0) {
            //     printf("%d, %d, %f, %f, %f, %f\n", x_frame, y_frame, theta, phi, theta_scale, phi_scale);
            // }
            if (y_frame < 0 || y_frame >= height || x_frame < 0 || x_frame >= width) {
                // printf("%d, %d\n", x, y);
//                ground.at<double>(y, x) = 0;
            } else {
                // printf("%d, %d, %f\n", x, y,  src.at<Vec3b>(y_frame, x_frame));
                
                ground.at<Vec3b>(y_res-y, x_res-x) = src.at<Vec3b>(y_frame, x_frame);
            }
        }
    }
    
}
void cvShowImageMat(const char *name, Mat& mat) {
	IplImage img = mat;
	cvShowImage(name, &img);
}
