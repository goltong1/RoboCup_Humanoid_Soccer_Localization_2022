/**
 * @file /include/localization_v2/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef localization_v2_QNODE_HPP_
#define localization_v2_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#define MSG 1

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>

#if MSG
#include "/home/robit/catkin_ws/devel/include/msg_generate/ikcoordinate_msg.h"
#include "/home/robit/catkin_ws/devel/include/msg_generate/imu_msg.h"
#include "/home/robit/catkin_ws/devel/include/msg_generate/pan_tilt_msg.h"
#include "/home/robit/catkin_ws/devel/include/msg_generate/localization_msg.h"
#include "/home/robit/catkin_ws/devel/include/msg_generate/position_msg.h"
#include "/home/robit/catkin_ws/devel/include/msg_generate/robocupcontroller.h"
#include "/home/robit/catkin_ws/devel/include/msg_generate/localv2_msg.h"

#endif

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace localization_v2 {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    void run();
    double Xmoved;
    double Ymoved;
    bool IsRun = false;
    bool deeplearning_flag = false;
    bool isSeeBall = false;
#if MSG
    msg_generate::localization_msg localMsg;
    msg_generate::localization_msg m_localMsg;


#endif


Q_SIGNALS:
    void rosShutdown();

    void recvImu();
    void step();
    void localUpdate();
    void game();
    void udp1();
    void udp2();
    void udp3();
    void udp4();
private:
    int init_argc;
    char** init_argv;
#if MSG
    ros::Subscriber coordinateSub;
    ros::Subscriber imuSub;
    ros::Subscriber ptSub;
    ros::Subscriber localSub;
    ros::Subscriber gameSub;
    ros::Subscriber udp_RB1_sub;
    ros::Subscriber udp_RB2_sub;
    ros::Subscriber udp_RB3_sub;
    ros::Subscriber udp_RB4_sub;
    void imuCallback(const msg_generate::imu_msg::ConstPtr&);
    void coordinateCallback(const msg_generate::ikcoordinate_msg::ConstPtr&);
    void pantiltCallback(const msg_generate::pan_tilt_msg::ConstPtr&);
    void localCallback(const msg_generate::localization_msg::ConstPtr&);
    void gameCallback(const msg_generate::robocupcontroller::ConstPtr&);
    void udp1Callback(const msg_generate::localv2_msg::ConstPtr&);
    void udp2Callback(const msg_generate::localv2_msg::ConstPtr&);
    void udp3Callback(const msg_generate::localv2_msg::ConstPtr&);
    void udp4Callback(const msg_generate::localv2_msg::ConstPtr&);
#endif

};

}  // namespace localization_v2

#endif /* localization_v2_QNODE_HPP_ */
