/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/localization_v2/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace std;
int cnt = 0;
namespace localization_v2 {
/*****************************************************************************
** Implementation
*****************************************************************************/
double yaw;
double ball_panAngle = 0.0;
double ball_tiltAngle = 0.0;
double panAngle = 0.0;
double tiltAngle = 0.0;
double saved_panAngle;
double saved_tiltAngle;
bool flag_surved = false;
bool flag_Lcross = false;
bool flag_feature = false;
bool flag_PC = false;
bool vision_ik_loop_check = false;
int rb1_x=0;
int rb1_y=0;
double rb1_yaw=0;
int rb2_x=0;
int rb2_y=0;
double rb2_yaw=0;
int rb3_x=0;
int rb3_y=0;
double rb3_yaw=0;
int rb4_x=0;
int rb4_y=0;
double rb4_yaw=0;
ros::Publisher localPub;
ros::Publisher localPub_v2;

msg_generate::robocupcontroller gameMsg;

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{}

QNode::~QNode() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"localization_v2");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.

#if MSG

    imuSub = n.subscribe("imu", 100, &QNode::imuCallback, this);
    coordinateSub = n.subscribe("ikcoordinate", 100, &QNode::coordinateCallback, this);
    ptSub = n.subscribe("pantilt", 100, &QNode::pantiltCallback, this);
    localSub = n.subscribe("robocup2019_local", 100, &QNode::localCallback, this);
    gameSub = n.subscribe("gamecontroller", 100, &QNode::gameCallback, this);
    localPub = n.advertise<msg_generate::position_msg>("position", 100);
    localPub_v2=n.advertise<msg_generate::localv2_msg>("localxy",100);
    udp_RB1_sub=n.subscribe("udp_RB1",100,&QNode::udp1Callback,this);
    udp_RB2_sub=n.subscribe("udp_RB2",100,&QNode::udp2Callback,this);
    udp_RB3_sub=n.subscribe("udp_RB3",100,&QNode::udp3Callback,this);
    udp_RB4_sub=n.subscribe("udp_RB4",100,&QNode::udp4Callback,this);
#endif
    start();
    return true;
}


void QNode::run() {
    ros::Rate loop_rate(33);
    while ( ros::ok() ) {

        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

#if MSG

void QNode::udp1Callback(const msg_generate::localv2_msg::ConstPtr &msg)
{

        rb1_x=msg->local_x;
        rb1_y=msg->local_y;
        rb1_yaw=msg->local_yaw;
        Q_EMIT udp1();



}
void QNode::udp2Callback(const msg_generate::localv2_msg::ConstPtr &msg)
{
        rb2_x=msg->local_x;
        rb2_y=msg->local_y;
        rb2_yaw=msg->local_yaw;
        Q_EMIT udp2();


}
void QNode::udp3Callback(const msg_generate::localv2_msg::ConstPtr &msg)
{

        rb3_x=msg->local_x;
        rb3_y=msg->local_y;
        rb3_yaw=msg->local_yaw;
        Q_EMIT udp3();



}
void QNode::udp4Callback(const msg_generate::localv2_msg::ConstPtr &msg)
{

        rb4_x=msg->local_x;
        rb4_y=msg->local_y;
        rb4_yaw=msg->local_yaw;
        Q_EMIT udp4();



}
void QNode::imuCallback(const msg_generate::imu_msg::ConstPtr &msg)
{
    yaw = msg->yaw;/*modify_yaw;*/
    if(yaw < 0) yaw = 360 + yaw;
    //    raw_yaw = yaw;

    Q_EMIT recvImu();
}

void QNode::coordinateCallback(const msg_generate::ikcoordinate_msg::ConstPtr &msg)
{
    cout<<"coordinateCallback"<<endl;
    Xmoved = (double)msg->X*0.13;//0.37/*0.25*//**0.62*/;
    Ymoved = (double)msg->Y*0.02;
    Q_EMIT step();
}

void QNode::pantiltCallback(const msg_generate::pan_tilt_msg::ConstPtr &msg)
{
    static int pantilt_cnt=0;
    if(isSeeBall == true)
    {

        ball_panAngle = msg->Angle_Yaw;
        ball_tiltAngle = msg->Angle_Pitch;
    }
    if(deeplearning_flag == true)
    {
        if(pantilt_cnt==0)
        {
            pantilt_cnt++;
        }

        if(pantilt_cnt==1)
        {
            panAngle = msg->Angle_Yaw;
            tiltAngle = msg->Angle_Pitch;
        }
    }
    else
    {
        pantilt_cnt=0;
    }

    if(gameMsg.state == 0 && gameMsg.state ==2)
    {
        isSeeBall = false;
        deeplearning_flag = false;
    }
}

void QNode::localCallback(const msg_generate::localization_msg::ConstPtr &msg)
{

    m_localMsg.ballDist       = msg->ballDist;
    m_localMsg.ballTheta      = msg->ballTheta;

    m_localMsg.xcrossDist     = msg->xcrossDist;
    m_localMsg.xcrossTheta    = msg->xcrossTheta;

    m_localMsg.goalpostDist   = msg->goalpostDist;
    m_localMsg.goalpostTheta  = msg->goalpostTheta;

    m_localMsg.pointDist      = msg->pointDist;
    m_localMsg.pointTheta     = msg->pointTheta;

    m_localMsg.penaltyCircleDist = msg->penaltyCircleDist;
    m_localMsg.penaltyCircleTheta = msg->penaltyCircleTheta;

    m_localMsg.oneline = msg->oneline;

    if( m_localMsg.xcrossDist.size()>0&&deeplearning_flag == false)
    {
//        cout<<"x cross through 1st door" <<endl;
        if( m_localMsg.xcrossDist[0]>10 && deeplearning_flag == false /*&& m_localMsg.xcrossDist[0]<900*/)
        {
            localMsg = m_localMsg;
            deeplearning_flag = true;
//            cout<<"x cross through 2st door" <<endl;
        }

    }
    if(m_localMsg.penaltyCircleDist.size()>0&&deeplearning_flag == false){

        if(m_localMsg.penaltyCircleDist[0]>10 && deeplearning_flag == false){
            localMsg = m_localMsg;
            deeplearning_flag = true;
        }
    }
    if(m_localMsg.oneline==0&&m_localMsg.penaltyCircleDist.size()==0&&m_localMsg.xcrossDist.size()==0&& deeplearning_flag == false)
    {
        localMsg = m_localMsg;
        deeplearning_flag = true;
    }
    if(m_localMsg.ballDist > 10 )
    {
        localMsg.ballDist = m_localMsg.ballDist;
        localMsg.ballTheta = m_localMsg.ballTheta;
        isSeeBall = true;
    }
    Q_EMIT localUpdate();

}
void QNode::gameCallback(const msg_generate::robocupcontroller::ConstPtr &msg)
{
    gameMsg.firstHalf       = msg->firstHalf;
    gameMsg.firstside       = msg->firstside;
    gameMsg.secondInfo      =msg->secondInfo;
    gameMsg.secondState     =msg->secondState;
    gameMsg.state           = msg->state;
    gameMsg.robotNum        = msg->robotNum;
    gameMsg.kickoffTeam     = msg->kickoffTeam;
    gameMsg.penalty         = msg->penalty;
    gameMsg.readyTime       = msg->readyTime;

    Q_EMIT game();
}
#endif


}  // namespace localization_v2
