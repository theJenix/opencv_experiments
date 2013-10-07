#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>

using namespace cv;
using namespace std;

/// Global Variables
Mat src; Mat hsv; Mat hue;
int bins = 25;

/// Function Headers
void Hist_and_Backproj(Mat& hist);
void Hist_and_Backproj_Trackbar(int, void* );

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

void cvShowImageMat(const char *name, Mat& mat) {
	IplImage img = mat;
	cvShowImage(name, &img);
}

/** @function main */
int main( int argc, char** argv )
{
  /// Read the image
    VideoCapture cap("/Users/theJenix/Development/opencv_experiments/opencv_lk/bigcarrun1.mov"); // open the default camera
    // VideoCapture cap("/Users/theJenix/Development/opencv_experiments/opencv_lk/recording1.mov"); // open the default camera
	if(!cap.isOpened())  // check if we succeeded
        return -1;

    char* window_image = "Source image";
    namedWindow( window_image, CV_WINDOW_AUTOSIZE );
    createTrackbar("* Hue  bins: ", window_image, &bins, 180, Hist_and_Backproj_Trackbar );
	cvNamedWindow(window_image, 0 );
	cvNamedWindow("BackProj", 0 );
	cvNamedWindow("Histogram", 0 );
//	cvNamedWindow("Diff", 0 );

    Mat src, srcGray;
    //     int burn = 300;
    // while(burn--) {
    //     pullFrame(cap, src, srcGray, NULL); //flipHorizAndVert);
    //     }

	pullFrame(cap, src, srcGray, NULL); //flipHorizAndVert);
	CvSize img_sz = src.size();

    cvtColor( src, src, CV_BGR2HSV );
    
    Mat stuff = cv::Mat::zeros(img_sz.height, img_sz.width, 0) ;
    
    // printf("%d, %d, %d\n", img_sz.height, img_sz.width, stuff.type());
    Mat last;
    bool first = true;
	while(true) {

		pullFrame(cap, src, srcGray, NULL); //flipHorizAndVert);
        /// Transform it to HSV
        cvtColor( src, hsv, CV_BGR2HSV );

        /// Use only the Hue value
        hue.create( hsv.size(), hsv.depth() );
        int ch[] = { 0, 0 };
        mixChannels( &hsv, 1, &hue, 1, ch, 1 );

        /// Create Trackbar to enter the number of bins
        Mat bp;
        Hist_and_Backproj(bp);
        if (!first) {
            Mat diff = bp-last;
            img_sz = diff.size();
            // printf("%d, %d, %d\n", img_sz.height, img_sz.width, diff.type());
            stuff = stuff + diff;
            Mat cln = diff.clone();
            // for(int row = 0; row < cln.rows; ++row) {
            //     uchar* p = cln.ptr(row);
            //     for(int col = 0; col < cln.cols; ++col) {
            //          *p = 1-*p; //*p++  //points to each pixel value in turn assuming a CV_8UC1 greyscale image 
            //          p++;
            //     }
            // 
            //     // or 
            //     // for(int col = 0; col < img.cols*3; ++col) {
            //     //      *p++  //points to each pixel B,G,R value in turn assuming a CV_8UC3 color image 
            //     // }
            // 
            // }   
            //cvShowImageMat( "Diff", cln);
        }
        first = false;
        last = bp;
        // stuff = bp;
        /// Show the image
        // imshow( window_image, src );
		cvShowImageMat(window_image, src );
        usleep(30 * 1000);
    }
    /// Wait until user exits the program
    waitKey(0);
    return 0;
}


/**
 * @function Hist_and_Backproj
 * @brief Callback to Trackbar
 */
void Hist_and_Backproj_Trackbar(int, void* )
{
}

void Hist_and_Backproj(Mat& backprojOut)
{

  MatND hist;
  int histSize = MAX( bins, 2 );
  float hue_range[] = { 24, 27 };
  const float* ranges = { hue_range };

  /// Get the Histogram and normalize it
  calcHist( &hue, 1, 0, Mat(), hist, 1, &histSize, &ranges, true, false );
  normalize( hist, hist, 0, 255, NORM_MINMAX, -1, Mat() );

  /// Get Backprojection
  MatND backproj;
  calcBackProject( &hue, 1, 0, hist, backproj, &ranges, 1, true );

  /// Draw the backproj
  cvShowImageMat( "BackProj", backproj );

  backprojOut = backproj;
  /// Draw the histogram
  int w = 400; int h = 400;
  int bin_w = cvRound( (double) w / histSize );
  Mat histImg = Mat::zeros( w, h, CV_8UC3 );

  for( int i = 0; i < bins; i ++ )
     { rectangle( histImg, Point( i*bin_w, h ), Point( (i+1)*bin_w, h - cvRound( hist.at<float>(i)*h/255.0 ) ), Scalar( 0, 0, 255 ), -1 ); }

  cvShowImageMat( "Histogram", histImg );
}
