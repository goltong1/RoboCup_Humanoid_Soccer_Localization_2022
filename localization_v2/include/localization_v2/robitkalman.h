#ifndef ROBIT_KALMAN_H
#define ROBIT_KALMAN_H

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <fstream>
#include <iostream>

using namespace std;
using namespace cv;

const double  __PI= 3.1415926535897932385;

class KalmanFilterC
{
public:
        KalmanFilter* kalman;
        KalmanFilterC(Point2f p);
        ~KalmanFilterC();
        Point2f GetPrediction();
        Point2f Update(Point2f p);
        void init(Point2f pt1);

};

#endif // ROBIT_KALMAN_H
