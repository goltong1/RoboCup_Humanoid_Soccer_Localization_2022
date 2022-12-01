#include "../include/localization_v2/robit_mcl.h"

#define alpha1 0.001
#define alpha2 0.001
#define alpha3 0.001
#define alpha4 0.001

#define deg(r)  (180.0/M_PI) * r // radian to degree
#define rad(d)  (M_PI/180.0) * d // degree to radian

#define AUGMENTED 0
#define ROBOTAROUND 1
#define AROUNDRANGE 20
bool drawline=false;

// ------------------------------------------------ Odometry Motion Model

void OdometryMotion::initOdometry(double x, double y, double theta)
{
    nowPosition = new RobotPose(x,y,theta);

    //    n_sample = particle;
    //    sample = new RobotPose[n_sample];
    //    for(int i=0; i<n_sample;i++)
    //    {
    //        sample[i].x = x;
    //        sample[i].y = y;
    //        sample[i].theta = theta;
    //    }
}

//void OdometryMotion::doNextProcess(double new_x, double new_y, double new_theta)
//{
//    delta_trans = sqrt(pow(new_x - nowPosition.x,2) + pow(new_y - nowPosition.y,2));
//    delta_rot1 = atan2(new_y - nowPosition.y, new_x - nowPosition.x) - nowPosition.theta;
//    delta_rot2 = new_theta - nowPosition.theta - delta_rot1;
//}


void OdometryMotion::Run(RobotPose oPose, RobotPose lastPosition)
{
    // uint is radian // do not used degree!!
    delta_trans = sqrt(pow(oPose.x - lastPosition.x,2) + pow(oPose.y - lastPosition.y,2));
    delta_rot1 = atan2(oPose.y - lastPosition.y, oPose.x - lastPosition.x) - lastPosition.theta;
    delta_rot2 = oPose.theta - rad(lastPosition.theta) - delta_rot1;
}



// ------------------------------------------------ Monte Carlo Localization

robit_mcl::robit_mcl(){}

robit_mcl::~robit_mcl()
{
    delete m_odometry;
    delete m_pose;
}
void robit_mcl::initMCL(double x, double y, double theta, int particle, int sizeX, int sizeY)
{

    m_odometry = new OdometryMotion;
    m_pose = new RobotPose(0,0,0);
    _gSampler.OnRandom(particle*2);
    _rnSampler.OnRandom(particle*2, 0);
    m_sizeX= sizeX;
    m_sizeY = sizeY;
    theta = rad(theta);
    m_odometry->initOdometry(x,y,theta);

    m_particleNum = particle;

    cout<<"initMCL-"<<endl;
    m_pf.clear();
    for(int i = 0; i < m_particleNum; i++)
    {
        RobotPose a(x,y,rad(theta));
        m_pf.push_back(a);
    }
    m_pose->x = m_pf[0].x;
    m_pose->y = m_pf[0].y;
    m_pose->theta = m_pf[0].theta;
    cout<<"m_pf[0].x: "<< m_pf[0].x<<endl;


    //    int ran = (_rnSampler.Generate()*m_particleNum);
    //    *m_pose = m_pf[ran];


}
void robit_mcl::randomSet()
{
    for(int i = 0; i < m_particleNum; i++)
    {
        m_pf[i].x = _rnSampler.Generate()*m_sizeX;
        m_pf[i].y = _rnSampler.Generate()*m_sizeY;
    }
}

void robit_mcl::predictionMCL(double new_x, double new_y, double new_theta, RobotPose lastPose, vector<RobotPose> tempPose)
{
    double dRot1, dTr, dRot2;
    lastPose.theta = rad(lastPose.theta);
    m_odometry->Run(RobotPose(new_x, new_y, rad(new_theta)), lastPose);

    m_pose->x = new_x;
    m_pose->y = new_y;

    m_pose->theta = new_theta;

    //    KGaussian gSampler;
    //    gSampler.OnRandom(m_particleNum);

    for(int i = 0; i < m_particleNum; i++)
    {

        dRot1 = m_odometry->delta_rot1 + _gSampler.Generate(0.0,
                                                            m_alpha1*m_odometry->delta_rot1 + m_alpha2*m_odometry->delta_trans);

        dRot2 = m_odometry->delta_rot2 + _gSampler.Generate(0.0,
                                                            m_alpha1*m_odometry->delta_rot2 + m_alpha2*m_odometry->delta_trans);

        dTr =   m_odometry->delta_trans + _gSampler.Generate(0.0,
                                                             m_alpha3*m_odometry->delta_trans + m_alpha4*(m_odometry->delta_rot1 + m_odometry->delta_rot2));

        m_pf[i].x += dTr * cos(rad(m_pf[i].theta) + dRot1);
        m_pf[i].y += dTr * sin(rad(m_pf[i].theta) + dRot1);

        m_pf[i].theta += deg(dRot1 + dRot2);
        m_pf[i].theta = fmod(m_pf[i].theta, (2*M_PI));

        if(m_pf[i].x > m_sizeX) m_pf[i].x = m_sizeX - 1;
        else if(m_pf[i].x < 0) m_pf[i].x = 1;

        if(m_pf[i].y > m_sizeY) m_pf[i].y = m_sizeY - 1;
        else if(m_pf[i].y < 0) m_pf[i].y = 1;
    }
    if(tempPose.size()>5)
    {
        for(int i = 0; i < tempPose.size(); i++)
        {
            for(int j = 0; j < 5; j++)
            {

                m_pf[i*5+j].x =tempPose[i].x + _rnSampler.Generate()*150-75;
                m_pf[i*5+j].y =tempPose[i].y + _rnSampler.Generate()*150-75;
            }
        }
    }
    int index = tempPose.size()*5;
    for(int j = index; j < m_particleNum; j++)
    {

        while(1)
        {
            m_pf[j].x =m_pose->x + _rnSampler.Generate()*200 - 100;
            m_pf[j].y =m_pose->y + _rnSampler.Generate()*200 - 100;

            double dist = sqrtf((m_pose->x - m_pf[j].x)*(m_pose->x - m_pf[j].x) + (m_pose->y - m_pf[j].y)*(m_pose->y - m_pf[j].y));

            if(m_pf[j].x > m_sizeX) m_pf[j].x = m_sizeX - 1;
            else if(m_pf[j].x < 0) m_pf[j].x = 1;

            if(m_pf[j].y > m_sizeY) m_pf[j].y = m_sizeY - 1;
            else if(m_pf[j].y < 0) m_pf[j].y = 1;
            if(dist < 100) // 50cm
            {
                //cout<<m_pf[j].x<<','<<m_pf[j].y<<endl;
                break;
            }
        }


    }

}
void robit_mcl::measurementMCL(vector<MeasurementData> data, double sense_noise)  // weight
{
    double dSum, dWt, dMax;
    m_weight.clear();
    double w_avg = 0;
    for(int i = 0; i < m_particleNum; i++)
    {
        dWt = 1.0;
        for(int j = 0; j < data.size(); j++)
        {
            double dist = sqrt((m_pf[i].x - data[j].featureX)*(m_pf[i].x - data[j].featureX)
                               + (m_pf[i].y - data[j].featureY)*(m_pf[i].y - data[j].featureY));
            dWt *= calcGaussianProbability(dist, sense_noise, data[j].measurement);
        }

        m_weight.push_back(dWt);
        //        cout<<"dWt[ "<<i<<"]: "<<dWt<<endl;

        if(m_weight[i] > dMax)
        {
            drawline=true;
            dMax = m_weight[i];

            m_pose->x = m_pf[i].x;
            m_pose->y = m_pf[i].y;
            m_pose->theta = m_pf[i].theta;
        }
        dSum += m_weight[i];
    }

    for(int i = 0; i < m_particleNum; i++)
    {
#if AUGMENTED
        w_avg += m_weight[i]/m_particleNum;
#endif

        m_weight[i] /= dSum;
    }
    m_dMax = dMax;
#if AUGMENTED
    w_slow += alpha_slow*(w_avg - w_slow);
    w_fast += alpha_fast*(w_avg - w_fast);
#endif
}


void robit_mcl::measurementMCL(vector<double> dist, vector<double> theta, Likelihood likelihood)  // weight
{
 //   cout<<"/***************MeasurementMCL***************/"<<endl;
  //  cout<<"/***************MeasurementMCL***************/"<<endl;
  //  cout<<"/***************MeasurementMCL***************/"<<endl;
    double dSum=0, dWt=0, dMax=0, dMin = 1;
//    m_weight_zero = m_weight;
    m_weight.clear();
    double w_avg = 0;
    //    cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl;
    //    temp_pose->x= m_pose->x;
    //    temp_pose->y= m_pose->y;
    //    temp_pose->theta= m_pose->theta;
    //    cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl;

    double sum_w = 0;
    for(int i = 0; i < m_particleNum; i++)
    {
        dWt = 1.0;
        //        cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
        for(int j = 0; j < dist.size(); j++)
        {
            if(dist[j] >= z_max) // cm
                continue;


#if ROBOTAROUND
            //            if(m_pf[i].x < temp_pose->x - 50 || m_pf[i].x > temp_pose->x + 50 || m_pf[i].y < temp_pose->y - 50 || m_pf[i].y > temp_pose->y + 50)
            //            {
            //                dWt = 0;
            //                break;
            //            }
#endif

            int x = m_pf[i].x - (dist[j])*sin(rad(theta[j]));
            int y = m_pf[i].y - (dist[j])*cos(rad(theta[j]));

            if(x < 0 || y < 0 || x > m_sizeX-1 || y > m_sizeY-1)
                dWt *= 0;
            else
            {
                //                cout<<"x: "<<x<<"  y: "<<y<<endl;
                dWt *= likelihood.mapLikelihood[y][x];
            }
        }
        //        cout<<"###########################################################"<<endl;

        m_weight.push_back(dWt);
        //        cout<<"dWt[ "<<i<<"]: "<<dWt<<endl;

        //        std::cout<<"m_weight"<<i<<" ==    "<<m_weight[i]<<endl;

        if(m_weight[i] >= dMax&&m_weight[i]!=0)
        {
            drawline=true;
            dMax = m_weight[i];
                        std::cout<<"-----   dMax ====     "<<dMax<<endl;
                        std::cout<<"-----   dMax ====     "<<dMax<<endl;
                       std::cout<<"-----   dMax ====     "<<dMax<<endl;
            m_pose->x = m_pf[i].x;
            m_pose->y = m_pf[i].y;
            m_pose->theta = m_pf[i].theta;
        }

        if(m_weight[i] <= dMin)
        {
            dMin = m_weight[i];

            //            std::cout<<"-----   dMin ====     "<<dMin<<endl;
            //            std::cout<<"-----   dMin ====     "<<dMin<<endl;
        }
        //        sum_w +=m_weight[i];
        //        cout<<"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"<<endl;
        dSum += m_weight[i];
    }
  //  cout<<"ratio ====     "<<dMax/dMin<<endl;
  //  std::cout<<"-----   dMax ====     "<<dMax<<endl;
  //  std::cout<<"-----   dMin ====     "<<dMin<<endl;
    //    cout<<"summ ====     "<<sum_w<<endl;
    //    cout<<"summ ====     "<<sum_w<<endl;
    //    cout<<"summ ====     "<<sum_w<<endl;
    for(int i = 0; i < m_particleNum; i++)
    {
#if AUGMENTED
        w_avg += m_weight[i]/m_particleNum;
#endif
        if(dSum == 0)
            continue;
        m_weight[i] /= dSum;//normalize
    }
    m_dMax = dMax;
#if AUGMENTED
    w_slow += alpha_slow*(w_avg - w_slow);
    w_fast += alpha_fast*(w_avg - w_fast);
#endif

//    if((dMax/dMin == 1) || (dMin == 0)) m_weight = m_weight_zero ;
//    m_weight_zero.clear();
}

void robit_mcl::resamplingMCL(vector<RobotPose> tempPose)
{
    double reset_prob = std::max(0.0, 1-(w_fast/w_slow));
    for(int i = 1; i <m_weight.size(); i++)
        m_weight[i] += m_weight[i-1];
    int index = int(_rnSampler.Generate() * m_particleNum); //0 ~ m_particleNum
    double beta = 0.0;
    vector<RobotPose> pfOld(m_pf);
    m_pf.clear();


    for(int i = 0; i < m_particleNum; i++)
    {
        beta = _rnSampler.Generate();
        if(beta < reset_prob)
        {
            RobotPose randomParticle(0,0,0);
            while(1)
            {
                int random_index = _rnSampler.Generate()*tempPose.size();
                randomParticle.x =tempPose[random_index].x + _rnSampler.Generate()*5;
                randomParticle.y =tempPose[random_index].y + _rnSampler.Generate()*5;

                if(randomParticle.x < 0 || randomParticle.y < 0 || randomParticle.x > m_sizeX || randomParticle.y > m_sizeY)
                    continue;
                else
                    break;
            }
            m_pf.push_back(randomParticle);
        }
        else
        {
            for(unsigned int j=0; j < m_weight.size(); j++)
            {
                if(beta < m_weight[j])
                {
                    index = j;
                    break;
                }
            }
            m_pf.push_back(pfOld[index]);
        }
    }

}

void robit_mcl::resamplingMCL()
{
    double reset_prob = std::max(0.0, 1-(w_fast/w_slow));
    for(int i = 1; i <m_weight.size(); i++)
        m_weight[i] += m_weight[i-1];
    int index = int(_rnSampler.Generate() * m_particleNum); //0 ~ m_particleNum
    double beta = 0.0;
    vector<RobotPose> pfOld(m_pf);
    m_pf.clear();

    //    for(int i = 0; i < tempPose.size(); i++)
    //    {

    //    }

    for(int i = 0; i < m_particleNum; i++)
    {
        //        beta = _rnSampler.Generate()*m_dMax*2;
        beta = _rnSampler.Generate();
        //        cout<<"reset_prob: "<<reset_prob<<endl;
#if AUGMENTED
        if(beta < reset_prob)
        {
            RobotPose randomParticle(0,0,0);
            while(1)
            {
                randomParticle.x = m_pose->x + _rnSampler.Generate()*5;
                randomParticle.y = m_pose->y + _rnSampler.Generate()*5;
                randomParticle.theta = pfOld[i].theta;
#if ROBOTAROUND
                double dist = sqrtf((m_pose->x-randomParticle.x)*(m_pose->x-randomParticle.x) + (m_pose->y-randomParticle.y)*(m_pose->y-randomParticle.y));
                if(randomParticle.x < 0 || randomParticle.y < 0 || randomParticle.x > m_sizeX || randomParticle.y > m_sizeY)
                    continue;
                else
                    break;

#elif !ROBOTAROUND
                if(randomParticle.x < 0 || randomParticle.y < 0 || randomParticle.x > m_sizeX || randomParticle.y > m_sizeY)
                    continue;
                else
                    break;
#endif
            }
            m_pf.push_back(randomParticle);
            //            cout<<"reset particle !!!!!!!!!"<<endl;
        }
        else
        {
            for(unsigned int i=0; i < m_weight.size(); i++)
            {
                if(beta < m_weight[i])
                {
                    index = i;
                    break;
                }
            }
            m_pf.push_back(pfOld[index]);
        }
#elif !AUGMENTED

        for(unsigned int i=0; i < m_weight.size(); i++)
        {
            if(beta < m_weight[i])
            {
                index = i;
                break;
            }
        }
        m_pf.push_back(pfOld[index]);

#endif
    }
}

