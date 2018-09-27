#ifndef MARKERDETECTION_H
#define MARKERDETECTION_H
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include<math.h>
#define PatternSide 400
#define PI 3.14159265
using namespace cv;
using namespace std;
class MarkerDetection
{
public:
    MarkerDetection();

    bool DetectANDmatchMarker(vector<int> Dp, Mat img,int dotNumber );
    bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2,Point2f &r);

    ///////////////filtering and gradian functions functions///////////////
    void mask1dx(int n,float s,cv::Mat &mask);
    void  mask1dy(int n,float s,cv::Mat &mask);
    void conv1x(cv::Mat input,cv::Mat &output,cv::Mat mask );
    void conv1y(cv::Mat input,cv::Mat &output,cv::Mat mask );
    void mask1derx(int n,float s,cv::Mat &mask);
    void mask1dery(int n,float s,cv::Mat &mask);






    ~MarkerDetection();
private:
    vector<vector<Point> > contours;


};

#endif // MARKERDETECTION_H
