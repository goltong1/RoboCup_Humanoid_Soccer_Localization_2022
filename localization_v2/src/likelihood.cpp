#include "../include/localization_v2/likelihood.h"

Likelihood::Likelihood()
{

}

Likelihood::Likelihood(const Mat img, double Zhit, double Zrand, double Sigma_hit, double Zmax, double Zgrid)
{
    width = img.cols;
    height = img.rows;
    fieldImg = img.clone();
    _linePlot.clear();
//    for(int y = 0; y < height; y++)
//    {
//        for(int x = 0; x <width; x++)
//        {
//            if(img.data[width*y + x] == 255) //if(data == 255) -> line
////                _linePlot.push_back(cv::Point(x,y));

//        }
//    }
    _Zhit = Zhit; _Zrand = Zrand; _Sigma_hit = Sigma_hit; _Zmax = Zmax; _Zgrid = Zgrid;


    mapLikelihood.clear();
    vector<double> mapcol(width);
    mapLikelihood.assign(height,mapcol);

}

void Likelihood::likelihoodField()
{
    for(int j=0; j<height; j++)
    {
        for(int i=0; i<width; i++)
        {
            mapLikelihood[j][i] = 0;
        }
    }


    double gPhit;
    double realDist;
    double gConstDen=1.0/(sqrt(_2PI)*_Sigma_hit);
    double gConstExp = -1.0/(2*_SQR(_Sigma_hit));

    double minDist, tempDist;
    cout<<"likelihoodField()"<<endl;
    cout<<"_linePlot.size(): "<<_linePlot.size()<<endl;
    for(int y = 0; y < height; y++)
    {
        for(int x = 0; x <width; x++)
        {
            if(fieldImg.data[width*y + x] == 0) //
            {
                for(int i=0; i<_linePlot.size(); i++)
                {

                    minDist = 99999;
                    for(int j=0; j<_linePlot[i].size(); j++)
                    {
                        tempDist = sqrt(_SQR(_linePlot[i][j].x - x) + _SQR(_linePlot[i][j].y - y));

                        if(tempDist < minDist)
                            minDist = tempDist;
                    }
                    realDist = minDist*_Zgrid;

                    if(realDist < _Zmax)
                    {
                        cout<<"realDist: "<<realDist<<endl;

                        gPhit = gConstDen*exp((_SQR(realDist))*gConstExp);
                        mapLikelihood[y][x] += _Zhit*gPhit + _Zrand/_Zmax;
                    }
                }
            }

        }
    }

    double max = 0;

    for(int j=0; j<height; j++)
    {
        for(int i=0; i<width; i++)
        {
            if(mapLikelihood[j][i] > max)
                max = mapLikelihood[j][i];
        }
    }

    cout<<"max: "<<max<<endl;


    for(int y=0; y<height; y++)
    {
        for(int x=0; x<width; x++)
        {
            if(fieldImg.data[width*y + x] == 255) //if(data == 255) -> line
            {
                mapLikelihood[y][x] = max;

            }
        }
    }

}
