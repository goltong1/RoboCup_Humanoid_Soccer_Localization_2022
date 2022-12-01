#ifndef LIKELIHOOD_H
#define LIKELIHOOD_H
#include <opencv2/core.hpp>
#include <iostream>
#include "kfc.h"

using namespace std;
using namespace cv;

class Likelihood
{
public:
    Likelihood();
    Likelihood(const Mat img, double Zhit, double Zrand, double Sigma_hit, double Zmax, double Zgrid);
    void likelihoodField();
    void Set(double Zhit, double Zrand, double Sigma_hit, double Zmax, double Zgrid)
    {
        _Zhit = Zhit; _Zrand = Zrand; _Sigma_hit = Sigma_hit; _Zmax = Zmax; _Zgrid = Zgrid;
    }

public:
    double _Zhit, _Zrand, _Sigma_hit, _Zmax;
    double _Zgrid;
    int ObstacleNum;
    vector<vector<cv::Point>> _linePlot;
    vector<vector<double>> mapLikelihood;
    Mat fieldImg;
    int width = 0;
    int height = 0;

};

#endif // LIKELIHOOD_H
