#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <unistd.h>

using namespace cv;

void cvShowImageMat(const char *name, Mat& mat) {
	IplImage img = mat;
	cvShowImage(name, &img);
}

int findCenter(Vec3b value, Mat centers) {
    long minDist = 0x7FFFFFFFFFFFFFFF;
    int center;
    for (int ii = 0; ii < centers.rows; ii++) {
        Vec3b potential = Vec3b(centers.at<float>(ii, 0), centers.at<float>(ii, 1), centers.at<float>(ii, 2));
        long dist = sum(potential - value).val[0] * sum(potential - value).val[0];
//        printf("%ld ", dist);
        if (dist < minDist) {
            minDist = dist;
            center = ii;
        }
    }
  //  printf("center: %d\n", center);
    return center;
}

int main( int argc, char** argv )
{
    printf("bwoo");
    cvNamedWindow("clustered image", 0);

    VideoCapture cap(argv[1]);
    Mat src;
    cap >> src; // = imread( argv[1], 1 );
    Mat samples(src.rows * src.cols, 3, CV_32F);
    for( int y = 0; y < src.rows; y++ )
        for( int x = 0; x < src.cols; x++ )
          for( int z = 0; z < 3; z++)
            samples.at<float>(y + x*src.rows, z) = src.at<Vec3b>(y,x)[z];

    int clusterCount = 3;
    Mat labels;
    int attempts = 1;
    Mat centers;
    kmeans(samples, clusterCount, labels, TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10000, 0.0001), attempts, KMEANS_PP_CENTERS, centers );

    Size size(src.size().width / 4, src.size().height / 4);
    while(true) {
        cap >> src;
        printf("Frame\n"); 
#if 1
        Mat dst;
        resize(src, src, size);
        Mat new_image( src.size(), src.type() );
        for( int y = 0; y < src.rows; y++ )
        for( int x = 0; x < src.cols; x++ )
        {
            int cluster_idx = findCenter(src.at<Vec3b>(y, x), centers); 
        //    int cluster_idx = labels.at<int>(y + x*src.rows,0);
            new_image.at<Vec3b>(y,x)[0] = centers.at<float>(cluster_idx, 0);
            new_image.at<Vec3b>(y,x)[1] = centers.at<float>(cluster_idx, 1);
            new_image.at<Vec3b>(y,x)[2] = centers.at<float>(cluster_idx, 2);
        }
#endif
        cvShowImageMat("clustered image", new_image);
        usleep(30 * 1000); 
    }
    waitKey( 0 );
}
