#ifndef ROBIT_MCL_H
#define ROBIT_MCL_H
#include <iostream>
#include "kfc.h"
#include <vector>
#include "likelihood.h"
struct RobotPose
{
    int x = 0;
    int y = 0;
    double theta = 0;
    RobotPose(const double &Xo,const double &Yo, const double &Thetao): x(Xo), y(Yo), theta(Thetao){}

    RobotPose operator + (RobotPose& p) {
         x = x + p.x;
         y = y + p.y;
         theta = theta + p.theta;
         return RobotPose(x, y,theta);
     }

    RobotPose operator = (RobotPose& p) {
         x = p.x;
         y = p.y;
         theta = p.theta;

         return RobotPose(x, y,theta);
     }
};


struct MeasurementData
{
    double measurement =0;
    int featureX = 0;
    int featureY = 0;
};
using namespace std;


class OdometryMotion
{
public:
    OdometryMotion(){}
    void initOdometry(double x, double y, double theta);
    void Run(RobotPose oPose, RobotPose lastPosition);

public:
    RobotPose *nowPosition;
    double delta_trans = 0.0;
    double delta_rot1 = 0.0;
    double delta_rot2 = 0.0;
    int n_sample;

};

class robit_mcl: public OdometryMotion
{
public:
    robit_mcl();
    ~robit_mcl();
    void initMCL(double x, double y, double theta, int particle, int sizeX, int sizeY);
    void predictionMCL(double new_x, double new_y, double new_theta, RobotPose lastPose, vector<RobotPose> tempPose);
    void measurementMCL(vector<MeasurementData> data, double sense_noise); // feature
    void measurementMCL(vector<double> dist, vector<double> theta, Likelihood likelihood);
    void resamplingMCL();
    void resamplingMCL(vector<RobotPose> tempPose);
    bool check_particle = false;

    int sibasl = 0;

    void randomSet();
    static double calcGaussianProbability(double mu, double sigma,double x)
    {
        return exp(- (mu - x)*(mu - x) / (sigma*sigma * 2.0) ) / (sqrt(2.0 * M_PI) * sigma);
    }

    vector<RobotPose> getParticlePose()
    {
        if(check_particle == true){
//            sibasl ++;
            for(int i = 0; i <m_pf.size(); i++)
            {
                int j = i/54;
                m_pf[i].x = 100+3*i-162*j;
                m_pf[i].y = 190+2*j;
std::cout<<"ufckyou ==  "<<i<<endl;
            }
        }
        return m_pf;
    }
    RobotPose getRobotPose()
    {
        return *m_pose;
    }

public:
   int z_max;
   vector<RobotPose> m_pf;
   int m_particleNum;


private:
    OdometryMotion *m_odometry;
    Likelihood m_likelihood;
    vector<double> m_weight;
    int m_sizeX = 0;
    int m_sizeY = 0;
    KGaussian _gSampler;
    KRandom _rnSampler;

    double m_dMax;

    double w_slow = 0.0;
    double w_fast = 0.0;

    double m_alpha1 = 1;
    double m_alpha2 = 1;
    double m_alpha3 = 1;
    double m_alpha4 = 1;

//    double m_alpha1 = 0.01;
//    double m_alpha2 = 0.02;
//    double m_alpha3 = 0.01;
//    double m_alpha4 = 0.02;
    RobotPose *m_pose;
    RobotPose *temp_pose;


    // alpha_slow < alpha_fast
    // ex) alpha_slow = 0.1 < alpha_fast = 0.7
    double alpha_slow = 0.1; // 0~1
    double alpha_fast = 0.9; // 0~1

};

#endif // ROBIT_MCL_H
