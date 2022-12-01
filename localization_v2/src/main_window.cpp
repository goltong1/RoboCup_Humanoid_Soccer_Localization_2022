/**
     * @file /src/main_window.cpp
     *
     * @brief Implementation for the qt gui.
     *
     * @date February 2011
     **/
/*****************************************************************************
    ** Includes
    *****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/localization_v2/main_window.hpp"

/*****************************************************************************
    ** Namespaces
    *****************************************************************************/
#define ROBIT 21
#define ROBIT_TEAM 1
#define ADD 1
#define deg(r)  (180.0/M_PI) * r // radian to degree
#define rad(d)  (M_PI/180.0) * d // degree to radian
//#define ROBIT               37      //37

#define ROBOT_WIDTH     80
#define ROBOT_HEIGHT    80

#define ZMAX            200 // cm
#define TEMPSIZE        10
bool ikcoordinate = 0;
bool robot1_alive=false;
bool robot2_alive=false;
bool robot3_alive=false;
bool robot4_alive=false;
double tempPose2[10][3]={{0,},};
int coount = 0;
bool L_cross = false;
bool PCircle = false;
extern bool drawline;
static bool check_kickoff = false;

namespace localization_v2 {
using namespace Qt;
int vision_callback_cnt=0;
KalmanFilterC kalman(Point(0,0));
extern double ball_panAngle;
extern double ball_tiltAngle;
extern double panAngle;
extern double tiltAngle;
extern double yaw;
extern int rb1_x;
extern int rb1_y;
extern double rb1_yaw;
extern int rb2_x;
extern int rb2_y;
extern double rb2_yaw;
extern int rb3_x;
extern int rb3_y;
extern double rb3_yaw;
extern int rb4_x;
extern int rb4_y;
extern double rb4_yaw;
extern ros::Publisher localPub;
extern msg_generate::robocupcontroller gameMsg;
extern ros::Publisher localPub_v2;
bool isLean(const Vec4i& _l1, const Vec4i& _l2)
{
    Vec4i l1(_l1), l2(_l2);

    Point AP1(l1[0],l1[1]);
    Point AP2(l1[2],l1[3]);

    Point BP1(l2[0],l2[1]);
    Point BP2(l2[2],l2[3]);

    float length1 = sqrtf((l1[2] - l1[0])*(l1[2] - l1[0]) + (l1[3] - l1[1])*(l1[3] - l1[1])); // 첫번째 라인의 길이
    float length2 = sqrtf((l2[2] - l2[0])*(l2[2] - l2[0]) + (l2[3] - l2[1])*(l2[3] - l2[1])); // 두번째 라인의 길이

    float product = (l1[2] - l1[0])*(l2[2] - l2[0]) + (l1[3] - l1[1])*(l2[3] - l2[1]);

    if (fabs(product / (length1 * length2)) > cos((CV_PI / 180) * 70))
    {
        //        cout<<"fabs(product / (length1 * length2)): "<<fabs(product / (length1 * length2))<<endl;
        return true;
    }
    else
    {
        /*
            double t;
            double s;
            double under = (BP2.y-BP1.y)*(AP2.x-AP1.x)-(BP2.x-BP1.x)*(AP2.y-AP1.y);
            if(under==0) return false;

            double _t = (BP2.x-BP1.x)*(AP1.y-BP1.y) - (BP2.y-BP1.y)*(AP1.x-BP1.x);
            double _s = (AP2.x-AP1.x)*(AP1.y-BP1.y) - (AP2.y-AP1.y)*(AP1.x-BP1.x);

            t = _t/under;
            s = _s/under;

            if(t<0.0 || t>1.0 || s<0.0 || s>1.0) return false;
            if(_t==0 && _s==0) return false;

            int x = AP1.x + t * (double)(AP2.x-AP1.x);
            int y = AP1.y + t * (double)(AP2.y-AP1.y);


            if(l1[0] <l1[2])
            {
                if((l1[0] < x + 5) && (l1[2] > x - 5) )
                {
                    return true;

                }
                else
                {
                    return false;
                }
            }
            else
            {
                if((l1[0] > x - 5) && (l1[2] < x + 5) )
                {
                    return true;

                }
                else
                {
                    return false;
                }
            }*/
        return false;
    }
}

/*****************************************************************************
    ** Implementation [MainWindow]
    *****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    qnode.init();
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    drawField();

    robot_MCL = new RobotPose(550,400,90);
    print("initMCL");
    // robot img
    item_robot = new MyItem();

    QObject::connect(&qnode, SIGNAL(recvImu()), this, SLOT(updateYaw()));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(step()), this, SLOT(updateOdometry()));
    QObject::connect(&qnode, SIGNAL(localUpdate()), this, SLOT(visionCallback()));
    QObject::connect(&qnode, SIGNAL(game()), this, SLOT(gameDataUpdate()));
    QObject::connect(&qnode, SIGNAL(udp1()), this, SLOT(updateUdp1()));
    QObject::connect(&qnode, SIGNAL(udp2()), this, SLOT(updateUdp2()));
    QObject::connect(&qnode, SIGNAL(udp3()), this, SLOT(updateUdp3()));
    QObject::connect(&qnode, SIGNAL(udp4()), this, SLOT(updateUdp4()));
}

MainWindow::~MainWindow() {}

void MainWindow::on_pb_init_clicked()
{
    if(!isSet)
    {

        double particleNum = ui.le_particle->text().toDouble();
        kalman.init(Point2d(robot_MCL->x, robot_MCL->y));
        MCL = new robit_mcl();
        MCL->initMCL(robot_MCL->x, robot_MCL->y, robot_MCL->theta, particleNum, m_width, m_height);
        robot_kalman = robot_MCL;//계속 작동중임..
        MCL->z_max = ZMAX;

        item_particle.assign(particleNum,new QGraphicsEllipseItem);

        for(int i = 0; i < particleNum; i++)
        {
            if(i <501)
                item_particle[i] = scene.addEllipse(100, 100, 1, 1, QPen(Qt::red));  //이거 주석처리하면 계속 particle안그려짐.0305
            else if(i <701)
                item_particle[i] = scene.addEllipse(100, 100, 1, 1, QPen(Qt::yellow));  //이거 주석처리하면 계속 particle안그려짐.0305
            else if(i >700)
                item_particle[i] = scene.addEllipse(100, 100, 1, 1, QPen(Qt::blue));  //이거 주석처리하면 계속 particle안그려짐.0305
        }
        m_updateTimer = new QTimer(this);
        updateField();
        isSet = true;

        msg_generate::localv2_msg xymsg;

        xymsg.local_x=robot_MCL->x;
        xymsg.local_y=robot_MCL->y;
        xymsg.local_yaw=currentYaw;

        localPub_v2.publish(xymsg);

        QString fileName = "/home/robit/catkin_ws/src/localization_v2/data/2022_new_test";//"/home/robit/catkin_ws/src/localization_v2/data/211130_likelihoodData_(Zhit_0.1,Zrand_0.01,Zmax_0.15,Sigmahit_0.15)";//"/home/robit/catkin_ws/src/localization_v2/data/210824_likelihoodData_(Zhit_0.3,Zrand_0.01,Zmax_1.4,Sigmahit_0.15)";
        QString fileName2 = "/home/robit/catkin_ws/src/localization_v2/data/2022_new_testPC";
        QString fileName3 = "/home/robit/catkin_ws/src/localization_v2/data/2022_new_test_L";
        ifstream fin_data(fileName.toLatin1());
        ifstream fin_data2(fileName2.toLatin1());
        ifstream fin_data3(fileName3.toLatin1());
        Mat A(m_height, m_width, CV_16UC1);
        if(fin_data.is_open())
        {
            print("success!!!!");
            for(int y = 0; y < m_height; y++)
            {
                for(int x = 0; x < m_width; x++)
                {
                    string data;
                    getline(fin_data,data);
                    likelihood->mapLikelihood[y][x] = atof(data.c_str());
                    double w =  likelihood->mapLikelihood[y][x]*255;
                    A.data[A.cols*y+x] = w > 255 ? 255: w;
                }
            }
            fin_data.close();

            QImage qimageField((const unsigned char*)(A.data), A.cols, A.rows, A.cols,QImage::Format_Indexed8);
            QGraphicsPixmapItem *likelihoodField = new QGraphicsPixmapItem(QPixmap::fromImage(qimageField));
            //        scene.addItem(likelihoodField);
            scene_likelihood.addItem(likelihoodField);
        }
        else
        {
            print("X_CORSS fail!!!!");
        }

        if(fin_data2.is_open())
        {
            print("success!!!!");
            for(int y = 0; y < m_height; y++)
            {
                for(int x = 0; x < m_width; x++)
                {
                    string data;
                    getline(fin_data2,data);
                    likelihoodPC->mapLikelihood[y][x] = atof(data.c_str());
                    double w =  likelihoodPC->mapLikelihood[y][x]*255;
                    A.data[A.cols*y+x] = w > 255 ? 255: w;
                }
            }
            fin_data2.close();
        }
        else
        {
            print("PC fail!!!!");
        }

        if(fin_data3.is_open())
        {
            print("success!!!!");
            for(int y = 0; y < m_height; y++)
            {
                for(int x = 0; x < m_width; x++)
                {
                    string data;
                    getline(fin_data3,data);
                    likelihood_L->mapLikelihood[y][x] = atof(data.c_str());
                    double w =  likelihood_L->mapLikelihood[y][x]*255;
                    A.data[A.cols*y+x] = w > 255 ? 255: w;
                }
            }
            fin_data3.close();
        }
        else
        {
            print("PC fail!!!!");
        }


        /************************** START 0824 modified by dohyeong ***********************/
        /************************** This part is a likelihood field of full field ***********************/
        //    QString fileName_L = fileName;
        //    //    fileName = fileName+"PC";
        //    fin_data.open(fileName.toLatin1());
        //    cout<<"fileName.toLatin1 about PC ===    "<<fin_data.is_open()<<endl;
        //    if(fin_data.is_open())
        //    {
        //        print("success!!!!PC");
        //        for(int y = 0; y < m_height; y++)
        //        {
        //            for(int x = 0; x < m_width; x++)
        //            {
        //                string data;
        //                getline(fin_data,data);
        //                likelihoodPC->mapLikelihood[y][x] = atof(data.c_str());
        //                double w =  likelihoodPC->mapLikelihood[y][x]*255;
        //                A.data[A.cols*y+x] = w > 255 ? 255: w;
        //            }
        //        }
        //        fin_data.close();

        //        QImage qimageField((const unsigned char*)(A.data), A.cols, A.rows,A.cols, QImage::Format_Indexed8);
        //        QGraphicsPixmapItem *likelihoodField = new QGraphicsPixmapItem(QPixmap::fromImage(qimageField));
        //        //                scene.addItem(likelihoodField);
        //        //        scene_likelihood.addItem(likelihoodField);
        //    }
        //    else
        //    {
        //        print("fail!!!!PC");
        //    }

        //    //dohyeong edit 0303
        //    //    fileName = fileName_L+"_L";
        //    fin_data.open(fileName.toLatin1());
        //    cout<<"fileName.toLatin1 about L ===    "<<fin_data.is_open()<<endl;
        //    if(fin_data.is_open())
        //    {
        //        print("success!!!! L");
        //        for(int y = 0; y < m_height; y++)
        //        {
        //            for(int x = 0; x < m_width; x++)
        //            {
        //                string data;
        //                getline(fin_data,data);
        //                likelihood_L->mapLikelihood[y][x] = atof(data.c_str());
        //                double w =  likelihood_L->mapLikelihood[y][x]*255;
        //                A.data[A.cols*y+x] = w > 255 ? 255: w;
        //            }
        //        }
        //        fin_data.close();

        //        QImage qimageField((const unsigned char*)(A.data), A.cols, A.rows,A.cols, QImage::Format_Indexed8);
        //        QGraphicsPixmapItem *likelihoodField = new QGraphicsPixmapItem(QPixmap::fromImage(qimageField));
        //        //                scene.addItem(likelihoodField);
        //        //        scene_likelihood.addItem(likelihoodField);
        //    }
        //    else
        //    {
        //        print("fail!!!! L");
        //    }
        /************************** END 0824 modified by dohyeong ***********************/

    }
    if(!isYaw)
    {
        isYaw = true;
        datumYaw = yaw;
    }
}

void MainWindow::on_pb_reset_clicked()
{
    if(isSet)
    {

        delete MCL;
        delete robot_MCL;
        delete item_robot;
        tempPose.clear();
        for(int i = 0; i < item_particle.size(); i++)
            scene.removeItem(item_particle[i]);
        item_particle.clear();
        disconnect(m_updateTimer,SIGNAL(timeout()),this, SLOT(timerCallback()));


        //        drawField();  //211130 dh modified
        robot_MCL = new RobotPose(0,0,0);
        print("initMCL");
        item_robot = new MyItem();

        isSet = false;
        if(isYaw)   isYaw = false;
    }
}

void MainWindow::mousePressEvent(QMouseEvent *event)
{
    if(!isSet)
    {
        // draw robot
        scene.addItem(item_robot);
        QPoint remapped = ui.graphicsView->mapFromParent(event->pos());
        if(ui.graphicsView->rect().contains(remapped))
        {
            QPointF mousePoint = ui.graphicsView->mapToScene(remapped);

            robot_MCL->x = mousePoint.x();
            robot_MCL->y = mousePoint.y();

            if(robot_MCL->x <= 0)    robot_MCL->x = 1;
            else if(robot_MCL->x >= m_width) robot_MCL->x = m_width - 1;

            if(robot_MCL->y <= 0)    robot_MCL->y = 1;
            else if(robot_MCL->y >= m_height) robot_MCL->y = m_height - 1;
            robot_MCL->theta = 0;

            item_robot->setPos(robot_MCL->x, robot_MCL->y);
            item_robot->setRotation(-robot_MCL->theta);

            ui.lb_x->setText(QString::number(robot_MCL->x));
            ui.lb_y->setText(QString::number(robot_MCL->y));
        }
    }

}
void MainWindow::draw_line_average(){

    /***********************************    real top   ********************************************/

    for(int i = 50; i <150; i++)
        scene.addEllipse(i, 100, 1, 1, QPen(Qt::red),QBrush(Qt::red));
    for(int height = 100; height <150; height++)
        scene.addEllipse(150, height, 1, 1, QPen(Qt::red),QBrush(Qt::red));
    for(int i = 150; i <400; i++)
        scene.addEllipse(i, 150, 1, 1, QPen(Qt::red),QBrush(Qt::red));
    for(int height = 100; height <150; height++)
        scene.addEllipse(400, height, 1, 1, QPen(Qt::red),QBrush(Qt::red));
    for(int i = 400; i <500; i++)
        scene.addEllipse(i, 100, 1, 1, QPen(Qt::red),QBrush(Qt::red));
    for(int height = 100; height <150; height++)
        scene.addEllipse(500, height, 1, 1, QPen(Qt::red),QBrush(Qt::red));
    for(int i = 500; i <750; i++)
        scene.addEllipse(i, 150, 1, 1, QPen(Qt::red),QBrush(Qt::red));
    for(int height = 100; height <150; height++)
        scene.addEllipse(750, height, 1, 1, QPen(Qt::red),QBrush(Qt::red));
    for(int i = 750; i <850; i++)
        scene.addEllipse(i, 100, 1, 1, QPen(Qt::red),QBrush(Qt::red));

    /***********************************    real bottom   ********************************************/

    for(int i = 50; i <150; i++)
        scene.addEllipse(i, 517, 1, 1, QPen(Qt::red),QBrush(Qt::red));
    for(int height = 460; height <517; height++)
        scene.addEllipse(150, height, 1, 1, QPen(Qt::red),QBrush(Qt::red));
    for(int i = 150; i <400; i++)
        scene.addEllipse(i, 460, 1, 1, QPen(Qt::red),QBrush(Qt::red));
    for(int height = 460; height <517; height++)
        scene.addEllipse(400, height, 1, 1, QPen(Qt::red),QBrush(Qt::red));
    for(int i = 400; i <500; i++)
        scene.addEllipse(i, 517, 1, 1, QPen(Qt::red),QBrush(Qt::red));
    for(int height = 460; height <517; height++)
        scene.addEllipse(500, height, 1, 1, QPen(Qt::red),QBrush(Qt::red));
    for(int i = 500; i <750; i++)
        scene.addEllipse(i, 460, 1, 1, QPen(Qt::red),QBrush(Qt::red));
    for(int height = 460; height <517; height++)
        scene.addEllipse(750, height, 1, 1, QPen(Qt::red),QBrush(Qt::red));
    for(int i = 750; i <850; i++)
        scene.addEllipse(i, 517, 1, 1, QPen(Qt::red),QBrush(Qt::red));

    /***********************************    real left side   ********************************************/

    for(int j  = 50; j<120; j++){
        scene.addEllipse(100, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int height = 100; height<121; height++){
        scene.addEllipse(height,120 , 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int j  = 120; j<492; j++){
        scene.addEllipse(121, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int height = 100; height<121; height++){
        scene.addEllipse(height,492 , 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int j  = 492; j<562; j++){
        scene.addEllipse(100, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    /***********************************   real right side   ********************************************/
    for(int j  = 50; j<120; j++){
        scene.addEllipse(800, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int height = 779; height<800; height++){
        scene.addEllipse(height,120 , 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int j  = 120; j<492; j++){
        scene.addEllipse(779, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int height = 779; height<800; height++){
        scene.addEllipse(height,492 , 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int j  = 492; j<562; j++){
        scene.addEllipse(800, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    /***********************************   real left middle side    ********************************************/

    for(int j  = 80; j<266; j++){
        scene.addEllipse(270, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int height = 121; height<270; height++){
        scene.addEllipse(height,80 , 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int j  = 266; j<346; j++){
        scene.addEllipse(195, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int height = 195; height<270; height++){
        scene.addEllipse(height,266, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int height = 195; height<270; height++){
        scene.addEllipse(height,346 , 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int j  = 80; j<533; j++){
        scene.addEllipse(121, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int height = 121; height<270; height++){
        scene.addEllipse(height,533 , 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int j  = 346; j<533; j++){
        scene.addEllipse(270, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    /***********************************   real right middle side    ********************************************/
    for(int j  = 80; j<533; j++){
        scene.addEllipse(779, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int j  = 80; j<266; j++){
        scene.addEllipse(630, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int j  = 346; j<533; j++){
        scene.addEllipse(630, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int j  = 266; j<346; j++){
        scene.addEllipse(705, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int height = 630; height<779; height++){
        scene.addEllipse(height,80 , 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int height = 630; height<779; height++){
        scene.addEllipse(height,533 , 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int height = 630; height<705; height++){
        scene.addEllipse(height,266 , 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int height = 630; height<705; height++){
        scene.addEllipse(height,266 , 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int height = 630; height<705; height++){
        scene.addEllipse(height,346 , 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int height = 630; height<705; height++){
        scene.addEllipse(height,346 , 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    /***********************************   real left double side_1    ********************************************/
    for(int width = 50; width<230; width++){
        scene.addEllipse(width,80 , 1, 1, QPen(Qt::red),QBrush(Qt::red));
    }
    for(int width = 50; width<230; width++){
        scene.addEllipse(width,170 , 1, 1, QPen(Qt::red),QBrush(Qt::red));
    }
    for(int height = 80; height<170; height++){
        scene.addEllipse(230,height , 1, 1, QPen(Qt::red),QBrush(Qt::red));
    }
    /***********************************  real left double side_12   ********************************************/
    for(int width = 50; width<230; width++){
        scene.addEllipse(width,446 , 1, 1, QPen(Qt::red),QBrush(Qt::red));
    }
    for(int width = 50; width<230; width++){
        scene.addEllipse(width,536 , 1, 1, QPen(Qt::red),QBrush(Qt::red));
    }
    for(int height = 446; height<536; height++){
        scene.addEllipse(230,height , 1, 1, QPen(Qt::red),QBrush(Qt::red));
    }
    /***********************************   real right double side_1    ********************************************/
    for(int width = 670; width<850; width++){
        scene.addEllipse(width,80 , 1, 1, QPen(Qt::red),QBrush(Qt::red));
    }
    for(int width = 670; width<850; width++){
        scene.addEllipse(width,170 , 1, 1, QPen(Qt::red),QBrush(Qt::red));
    }
    for(int height = 80; height<170; height++){
        scene.addEllipse(670,height , 1, 1, QPen(Qt::red),QBrush(Qt::red));
    }
    /***********************************  real right double side_2   ********************************************/
    for(int width = 670; width<850; width++){
        scene.addEllipse(width,446 , 1, 1, QPen(Qt::red),QBrush(Qt::red));
    }
    for(int width = 670; width<850; width++){
        scene.addEllipse(width,536 , 1, 1, QPen(Qt::red),QBrush(Qt::red));
    }
    for(int height = 446; height<536; height++){
        scene.addEllipse(670,height , 1, 1, QPen(Qt::red),QBrush(Qt::red));
    }
    /***********************************   real middle    ********************************************/
    for(int j  = 40; j<70; j++){//middle
        scene.addEllipse(415, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int j  = 40; j<70; j++){//middle
        scene.addEllipse(482, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int j  = 70; j<230; j++){//middle
        scene.addEllipse(532, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int j  = 70; j<230; j++){//middle
        scene.addEllipse(365, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int j  = 230; j<380; j++){//middle
        scene.addEllipse(415, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int j  = 230; j<380; j++){//middle
        scene.addEllipse(482, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int j  = 380; j<540; j++){//middle
        scene.addEllipse(365, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int j  = 380; j<540; j++){//middle
        scene.addEllipse(532, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int j  = 540; j<570; j++){//middle
        scene.addEllipse(415, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int j  = 540; j<570; j++){//middle
        scene.addEllipse(482, j, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int width  = 365; width < 415; width++){//middle
        scene.addEllipse(width, 70, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int width  = 482; width < 532; width++){//middle
        scene.addEllipse(width, 70, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int width  = 365; width < 415; width++){//middle
        scene.addEllipse(width, 230, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int width  = 482; width < 532; width++){//middle
        scene.addEllipse(width, 230, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int width  = 365; width < 415; width++){//middle
        scene.addEllipse(width, 380, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int width  = 482; width < 532; width++){//middle
        scene.addEllipse(width, 380, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int width  = 365; width < 415; width++){//middle
        scene.addEllipse(width, 540, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
    for(int width  = 482; width < 532; width++){//middle
        scene.addEllipse(width, 540, 1, 1, QPen(Qt::blue),QBrush(Qt::blue));
    }
}

void MainWindow::timerCallback()
{

}

void MainWindow::updateYaw()
{
    static int yaw_cnt=0;
    //    currentYaw = yaw - datumYaw;

    //    if(currentYaw > 180) currentYaw -= 360;
    //    else if(currentYaw <= -180) currentYaw += 360;

    if(isSet)
    {
        if(qnode.deeplearning_flag == true)
        {
            if(yaw_cnt==0)
            {
                yaw_cnt++;
            }

            if(yaw_cnt==1)
            {
                currentYaw = yaw;
            }
        }
        else
        {
            yaw_cnt=0;
        }
        robot_MCL->theta = yaw;
        item_robot->setRotation(-robot_MCL->theta);
        msg_generate::position_msg posMsg;

        posMsg.robot_area =0;
        posMsg.ball_area = 0;
        posMsg.yaw = currentYaw;
        localPub.publish(posMsg);
    }
}


void MainWindow::updateOdometry()
{
    //    cout<<"start updateodometry and qnode.deeplearning_flag is "<<qnode.deeplearning_flag<<endl;
    //    cout<<"start updateodometry and qnode.deeplearning_flag is "<<qnode.deeplearning_flag<<endl;
    //    //    cout<<"Is Run  ===   "<<qnode.IsRun<<endl;
    //#if ADD

    //    if(gameMsg.firstHalf)
    //        GOALPOST = gameMsg.firstside ? RIGHTSIDE : LEFTSIDE; // FIRSTHALF_GOAL;
    //    else
    //        GOALPOST = gameMsg.firstside ? LEFTSIDE : RIGHTSIDE; // SECONDHALF_GOAL;
    
    
    
    //    //    qnode.IsRun = true;

    //    // If Game State == KICKOFF && ROBOT == 2  => Poisition change
    //    // 211130 Modified start
    //    //로봇을 위치를 보정해주기 위해 Reset과정을 통해 로봇을 지워준다.(Reset함수변형한것임)--Start
    //    //reset
    //    if(gameMsg.kickoffTeam == ROBIT && gameMsg.robotNum == 2 && gameMsg.state ==1) check_kickoff = false;
    //    if(gameMsg.penalty == 34) {
    //        check_kickoff = true;
    //        cout<<"now penalty so, check kickoff false "<<check_kickoff<<endl;
    //    }
    //    if(gameMsg.kickoffTeam == ROBIT && gameMsg.robotNum == 2 && gameMsg.state == 3 && check_kickoff == false){
    //        qnode.deeplearning_flag = true;
    //        cout<<"com kick offfffffffffff";
    //        cout<<"com kick offfffffffffff";
    //        cout<<"com kick offfffffffffff";
    //        cout<<"com kick offfffffffffff";
    //        delete MCL;
    //        delete robot_MCL;
    //        delete item_robot;

    //        for(int i = 0; i < item_particle.size(); i++)
    //            scene.removeItem(item_particle[i]);
    //        item_particle.clear();
    //        disconnect(m_updateTimer,SIGNAL(timeout()),this, SLOT(timerCallback()));
    //        robot_MCL = new RobotPose(450,280,90);
    //        print("initMCL");
    //        item_robot = new MyItem();

    //        //Reset end
    //        //Mouse event start
    //        //        // draw robot
    //        int new_robot_x ;

    //        int new_robot_y = 250+45; //   250 -> 205
    //        if(GOALPOST == LEFTSIDE) new_robot_x = 415+43 ;// 415-> 372
    //        else new_robot_x = 435+45;


    //        scene.addItem(item_robot);
    //        QPoint remapped(new_robot_x,new_robot_y);
    //        if(ui.graphicsView->rect().contains(remapped))
    //        {
    //            QPointF mousePoint = ui.graphicsView->mapToScene(remapped);

    //            robot_MCL->x = mousePoint.x() - 50;
    //            robot_MCL->y = mousePoint.y() - 50;

    //            if(robot_MCL->x <= 0)    robot_MCL->x = 1;
    //            else if(robot_MCL->x >= m_width) robot_MCL->x = m_width - 1;

    //            if(robot_MCL->y <= 0)    robot_MCL->y = 1;
    //            else if(robot_MCL->y >= m_height) robot_MCL->y = m_height - 1;
    //            robot_MCL->theta = 0;

    //            item_robot->setPos(robot_MCL->x + 50, robot_MCL->y + 50);
    //            item_robot->setRotation(-robot_MCL->theta);

    //            ui.lb_x->setText(QString::number(robot_MCL->x));
    //            ui.lb_y->setText(QString::number(robot_MCL->y));
    //        }


    //        //로봇을 위치를 보정해주기 위해 Reset과정을 통해 로봇을 지워준다.(Reset함수변형한것임)--End
    //        //위치를 보정한 값을 다시 로컬상에 그려준다.(Set함수변형한것임)--Start

    //        double particleNum = ui.le_particle->text().toDouble();
    //        kalman.init(Point2d(robot_MCL->x, robot_MCL->y));
    //        MCL = new robit_mcl();
    //        MCL->initMCL(robot_MCL->x, robot_MCL->y, robot_MCL->theta, particleNum, m_width, m_height);
    //        robot_kalman = robot_MCL;//계속 작동중임..
    //        MCL->z_max = ZMAX;

    //        item_particle.assign(particleNum,new QGraphicsEllipseItem);
    //        for(int i = 0; i < particleNum; i++)
    //        {
    //            if(i <501)
    //                item_particle[i] = scene.addEllipse(100, 100, 1, 1, QPen(Qt::red));  //이거 주석처리하면 계속 particle안그려짐.0305
    //            else if(i <701)
    //                item_particle[i] = scene.addEllipse(100, 100, 1, 1, QPen(Qt::yellow));  //이거 주석처리하면 계속 particle안그려짐.0305
    //            else if(i >700)
    //                item_particle[i] = scene.addEllipse(100, 100, 1, 1, QPen(Qt::blue));  //이거 주석처리하면 계속 particle안그려짐.0305
    //        }
    //        m_updateTimer = new QTimer(this);
    //        updateField();
    //        check_kickoff = true;
    //}
    //        //위치를 보정한 값을 다시 로컬상에 그려준다.(Set함수변형한것임)--End
    //    // 211130 Modified End

    //    for(int i = 0; i < item_visualPoint.size(); i++)
    //        scene.removeItem(item_visualPoint[i]);
    //    item_visualPoint.clear();

    //    if(qnode.deeplearning_flag==true){
    //        qnode.deeplearning_flag = false;
    //        vector<Point2d> Points;
    //        bool only_one = true;
    //        if(isSet == true && isvision == false){

    //            isvision = true;

    //            vector<Point2d> visualPoints;
    //            vector<Point2d> visualBall;
    //            vector<Point2d> visualCircle;

    //            cout<<qnode.localMsg.penaltyCircleDist.size()<<endl;

    //            if(qnode.localMsg.penaltyCircleDist.size()!=0)
    //            {

    //                cout<<"Penalty circle ----------- 1108"<<endl;
    //                //PenaltyCircle을 로컬상 좌표로 변환 해주는 과정--Start

    //                PCircle = true;
    //                double dist;
    //                double theta;


    //                dist = qnode.localMsg.penaltyCircleDist[0]/10;
    //                theta = qnode.localMsg.penaltyCircleTheta[0] + currentYaw + panAngle;
    //                fmod(theta, 360);

    //                Point2d PC_point;
    //                PC_point.x = robot_MCL->x - (dist)*sin(rad(theta));
    //                PC_point.y = robot_MCL->y - (dist)*cos(rad(theta));

    //                //PenaltyCircle을 로컬상 좌표로 변환 해주는 과정--End
    //                //로봇을 위치를 보정해주기 위해 Reset과정을 통해 로봇을 지워준다.(Reset함수변형한것임)--Start
    //                //reset
    //                int save_robot_x = robot_MCL->x; //지우기 전에 현재 로봇의 위치를 저장한다.
    //                int save_robot_y = robot_MCL->y; //지우기 전에 현재 로봇의 위치를 저장한다.
    //                double G_PC_x;
    //                double G_PC_y;
    //                bool continute_flag = true;
    //                if(save_robot_x >535)//415,475,535//
    //                {
    //                    G_PC_x = 850;
    //                    G_PC_y = 400;//306;
    //                }
    //                else if(save_robot_x < 415){
    //                    G_PC_x = 250;//227;
    //                    G_PC_y = 400;//306;
    //                }
    //                else
    //                    continute_flag = false;
    //                if(continute_flag == true){
    //                    delete MCL;
    //                    delete robot_MCL;
    //                    delete item_robot;

    //                    cout<<"item size b ==  "<<item_particle.size()<<endl;
    //                    for(int i = 0; i < item_particle.size(); i++)
    //                        scene.removeItem(item_particle[i]);
    //                    cout<<"item size ==  "<<item_particle.size()<<endl;
    //                    item_particle.clear();
    //                    disconnect(m_updateTimer,SIGNAL(timeout()),this, SLOT(timerCallback()));
    //                    // drawField();
    //                    robot_MCL = new RobotPose(550,400,90);
    //                    print("initMCL");
    //                    item_robot = new MyItem();

    //                    //Reset end
    //                    //Mouse event start
    //                    //        // draw robot
    //                    int new_robot_x;
    //                    int new_robot_y;
    //                    if(PC_point.x >= G_PC_x && PC_point.y >=G_PC_y){
    //                        new_robot_x = save_robot_x - PC_point.x + G_PC_x;
    //                        new_robot_y = save_robot_y - PC_point.y + G_PC_y;
    //                    }
    //                    else if(PC_point.x <= G_PC_x && PC_point.y <=G_PC_y){
    //                        new_robot_x = save_robot_x + G_PC_x - PC_point.x;
    //                        new_robot_y = save_robot_y + G_PC_y - PC_point.y;
    //                    }
    //                    else if(PC_point.x >= G_PC_x && PC_point.y <=G_PC_y){
    //                        new_robot_x = save_robot_x - PC_point.x + G_PC_x;
    //                        new_robot_y = save_robot_y + G_PC_y - PC_point.y;
    //                    }
    //                    else if(PC_point.x <= G_PC_x && PC_point.y >=G_PC_y){
    //                        new_robot_x = save_robot_x + G_PC_x - PC_point.x;
    //                        new_robot_y = save_robot_y - PC_point.y + G_PC_y;
    //                    }

    //                    scene.addItem(item_robot);
    //                    QPoint remapped(new_robot_x,new_robot_y);
    //                    if(ui.graphicsView->rect().contains(remapped))
    //                    {
    //                        QPointF mousePoint = ui.graphicsView->mapToScene(remapped);

    //                        robot_MCL->x = mousePoint.x() - 50;
    //                        robot_MCL->y = mousePoint.y() - 50;

    //                        if(robot_MCL->x <= 0)    robot_MCL->x = 1;
    //                        else if(robot_MCL->x >= m_width) robot_MCL->x = m_width - 1;

    //                        if(robot_MCL->y <= 0)    robot_MCL->y = 1;
    //                        else if(robot_MCL->y >= m_height) robot_MCL->y = m_height - 1;
    //                        robot_MCL->theta = 0;

    //                        item_robot->setPos(robot_MCL->x + 50, robot_MCL->y + 50);
    //                        item_robot->setRotation(-robot_MCL->theta);

    //                        ui.lb_x->setText(QString::number(robot_MCL->x));
    //                        ui.lb_y->setText(QString::number(robot_MCL->y));
    //                    }


    //                    //로봇을 위치를 보정해주기 위해 Reset과정을 통해 로봇을 지워준다.(Reset함수변형한것임)--End
    //                    //위치를 보정한 값을 다시 로컬상에 그려준다.(Set함수변형한것임)--Start

    //                    double particleNum = ui.le_particle->text().toDouble();
    //                    kalman.init(Point2d(robot_MCL->x, robot_MCL->y));
    //                    MCL = new robit_mcl();
    //                    MCL->initMCL(robot_MCL->x, robot_MCL->y, robot_MCL->theta, particleNum, m_width, m_height);

    //                    robot_kalman = robot_MCL;//계속 작동중임..

    //                    MCL->z_max = ZMAX;

    //                    item_particle.assign(particleNum,new QGraphicsEllipseItem);

    //                    for(int i = 0; i < particleNum; i++)
    //                    {
    //                        if(i <501)
    //                            item_particle[i] = scene.addEllipse(100, 100, 1, 1, QPen(Qt::red));  //이거 주석처리하면 계속 particle안그려짐.0305
    //                        else if(i <701)
    //                            item_particle[i] = scene.addEllipse(100, 100, 1, 1, QPen(Qt::yellow));  //이거 주석처리하면 계속 particle안그려짐.0305
    //                        else if(i >700)
    //                            item_particle[i] = scene.addEllipse(100, 100, 1, 1, QPen(Qt::blue));  //이거 주석처리하면 계속 particle안그려짐.0305
    //                    }

    //                    m_updateTimer = new QTimer(this);
    //                    updateField();
    //                    //위치를 보정한 값을 다시 로컬상에 그려준다.(Set함수변형한것임)--End

    //                }
    //            }
    //            if(qnode.localMsg.xcrossDist.size()>0)
    //            {
    //            if(qnode.localMsg.xcrossDist[0] <1100 && qnode.localMsg.xcrossDist[0] > 0)
    //            {
    //                cout<<"--------------X CROSS START--- 1108---------------"<<endl;
    //                cout<<"--------------X CROSS START--- 1108---------------"<<endl;
    //                cout<<"--------------X CROSS START--- 1108---------------"<<endl;
    //                L_cross = true;

    //                vector<double> dist;
    //                vector<double> theta;
    //                for(int i = 0; i < qnode.localMsg.pointDist.size(); i++)
    //                {
    //                    if(qnode.localMsg.pointDist[i] == 99999)
    //                        continue;
    //                    else
    //                    {
    //                        dist.push_back(qnode.localMsg.pointDist[i]/10);
    //                        double th = qnode.localMsg.pointTheta[i] + currentYaw + panAngle;
    //                        fmod(th, 360);
    //                        theta.push_back(th);
    //                    }
    //                }

    //                for(int i = 40; i < theta.size(); i+=30)
    //                {
    //                    //                    MCL->measurementMCL(dist, theta, *likelihood_L);
    //                    MCL->measurementMCL(dist, theta, *likelihood);
    //                    cout<<" ---- Xcross MeasurementMCL ------  "<<endl;
    //                    //                cout<<" ---- Lcross MeasurementMCL ------  "<<endl;
    //                    //                cout<<" ---- Lcross MeasurementMCL ------  "<<endl;
    //                    MCL->resamplingMCL();

    //                    RobotPose robotMCL = MCL->getRobotPose();
    //                    robot_MCL->x        = robotMCL.x;
    //                    robot_MCL->y        = robotMCL.y;
    //                    robot_MCL->theta    = robotMCL.theta;
    //                    //                for(int i = 0; i < dist.size(); i++)
    //                    //                {
    //                    //                    if(dist[i] > ZMAX)     continue;   // cm
    //                    //                    Point2d visualPoint;
    //                    //                    visualPoint.x = robot_MCL->x - (dist[i])*sin(rad(theta[i]));
    //                    //                    visualPoint.y = robot_MCL->y - (dist[i])*cos(rad(theta[i]));
    //                    //                    visualCircle.push_back(visualPoint);
    //                    //                }
    //                    break;
    //                }
    //            }
    //            }
    //            if(qnode.localMsg.pointDist.size())
    //            {
    //                vector<double> dist;
    //                vector<double> theta;
    //                for(int i = 0; i < qnode.localMsg.pointDist.size(); i++)
    //                {
    //                    if(qnode.localMsg.pointDist[i] == 99999)
    //                        continue;
    //                    else
    //                    {
    //                        dist.push_back(qnode.localMsg.pointDist[i]/10);
    //                        double th = qnode.localMsg.pointTheta[i] + currentYaw + panAngle;
    //                        fmod(th, 360);
    //                        theta.push_back(th);
    //                    }
    //                }
    //                for(int i = 40; i < theta.size(); i+=30)
    //                {


    //                    for(int i = 0; i < dist.size(); i++)
    //                    {
    //                        if(dist[i]>ZMAX) continue;   // cm
    //                        Point2d point;
    //                        point.x = robot_MCL->x - (dist[i])*sin(rad(theta[i]));
    //                        point.y = robot_MCL->y - (dist[i])*cos(rad(theta[i]));
    //                        Points.push_back(point);
    //                    }

    //                    //                if(qnode.localMsg.penaltyCircleDist.size()!=0) only_one = false;
    //                    //                if(qnode.localMsg.xcrossDist[0]  < 1100 && qnode.localMsg.xcrossDist[0]  > 0) only_one = false;
    //                    //                if(only_one == false){
    //                    MCL->measurementMCL(dist, theta, *likelihood);//pointdist를 적용하는 부분
    //                    //              re  }
    //                    for(int i =0;i<10;i++) MCL->resamplingMCL();
    //                    RobotPose robotMCL = MCL->getRobotPose();
    //                    robot_MCL->x        = robotMCL.x;
    //                    robot_MCL->y        = robotMCL.y;
    //                    robot_MCL->theta    = robotMCL.theta;
    //                    for(int i = 0; i < dist.size(); i++)
    //                    {
    //                        if(dist[i]>ZMAX) continue;   // cm
    //                        Point2d visualPoint;
    //                        visualPoint.x = robot_MCL->x - (dist[i])*sin(rad(theta[i]));
    //                        visualPoint.y = robot_MCL->y - (dist[i])*cos(rad(theta[i]));
    //                        visualPoints.push_back(visualPoint);
    //                    }
    //                    break;

    //                }
    //            }
    static int savePoseCnt = 0;

    if((savePoseCnt++)%1 == 0)
    {
        if(tempPose.size() < TEMPSIZE)
        {
            tempPose.push_back(*robot_MCL);
        }
        else
        {
            // Circular Queue
            m_Queue_cnt = (m_Queue_cnt+1)%TEMPSIZE;
            tempPose[m_Queue_cnt] = *robot_MCL;
        }
    }

    //            /************************ball 좌표 넣는 부분***************************/
    //            bool IsBall = false;
    //            double dist;
    //            double theta;
    //            if(qnode.localMsg.ballDist > 0 /*&& qnode.localMsg.ballDist < 1200*/ ){ //210824_dohyeong_modified
    //                IsBall = true;
    //                dist = qnode.localMsg.ballDist/10;
    //                theta = qnode.localMsg.ballTheta + currentYaw + panAngle;
    //                fmod(theta, 360);

    //                Point2d Ball_point;
    //                Ball_point.x = 400;//robot_MCL->x - (dist)*sin(rad(theta));
    //                Ball_point.y = 334;//robot_MCL->y - (dist)*cos(rad(theta));
    //                visualBall.push_back(Ball_point);
    //            }
    //            /************************ball 좌표 넣는 부분 End ***************************/

    //            //drawing Points of contours
    //            //            for(int i = 0; i < item_visualPoint.size(); i++)
    //            //                scene.removeItem(item_visualPoint[i]);


    //            //            item_visualPoint.clear();
    //            item_visualPoint.assign(visualPoints.size()+1+visualBall.size(),new QGraphicsEllipseItem); //크기를선언하고 출력하지않으면, "QGraphicsScene::removeItem: item 0x7f930c006410's scene (0x0) is different from this scene (0x7fffd49fc3f0)" 경고문구가 출력된다.

    //            if(L_cross == true){
    //                if(IsBall == true){
    //                    for(int i = 0; i < visualBall.size(); i++)
    //                        item_visualPoint[i] = scene.addEllipse(visualBall[i].x+50, visualBall[i].y+50, 15, 15, QPen(Qt::cyan),QBrush(Qt::cyan));
    //                }
    //                for(int i = 1; i < visualPoints.size(); i++)
    //                    item_visualPoint[i] = scene.addEllipse(visualPoints[i].x+50, visualPoints[i].y+50, 5, 5, QPen(Qt::yellow),QBrush(Qt::yellow));
    //            }
    //            else if(PCircle == true){
    //                if(IsBall == true){
    //                    for(int i = 0; i < visualBall.size(); i++)
    //                        item_visualPoint[i] = scene.addEllipse(visualBall[i].x+50, visualBall[i].y+50, 15, 15, QPen(Qt::cyan),QBrush(Qt::cyan));
    //                }
    //                for(int i = 1; i < visualPoints.size(); i++)
    //                    item_visualPoint[i] = scene.addEllipse(visualPoints[i].x+50, visualPoints[i].y+50, 5, 5, QPen(Qt::red),QBrush(Qt::red));

    //            }
    //            else if(only_one == false){
    //                if(IsBall == true){
    //                    for(int i = 0; i < visualBall.size(); i++)
    //                        item_visualPoint[i] = scene.addEllipse(visualBall[i].x+50, visualBall[i].y+50, 15, 15, QPen(Qt::cyan),QBrush(Qt::cyan));
    //                }
    //                for(int i = 1; i < visualPoints.size(); i++)
    //                    item_visualPoint[i] = scene.addEllipse(visualPoints[i].x+50, visualPoints[i].y+50, 5, 5, QPen(Qt::blue),QBrush(Qt::blue));

    //            }
    //            else{
    //                if(IsBall == true){
    //                    for(int i = 0; i < visualBall.size(); i++)
    //                        item_visualPoint[i] = scene.addEllipse(visualBall[i].x+50, visualBall[i].y+50, 15, 15, QPen(Qt::cyan),QBrush(Qt::cyan));
    //                }
    //                for(int i = 1; i < visualPoints.size(); i++)
    //                    item_visualPoint[i] = scene.addEllipse(visualPoints[i].x+50, visualPoints[i].y+50, 5, 5, QPen(Qt::white),QBrush(Qt::white));
    //            }


    //            item_visualPoint[visualPoints.size()+visualBall.size()] = scene.addEllipse(robot_kalman->x+50, robot_kalman->y+50, 10, 10, QPen(Qt::black),QBrush(Qt::black));
    //            isvision = false;
    //            L_cross = false;
    //            PCircle = false;
    //        }
    //    }
    //    else if(qnode.isSeeBall){
    //        vector<Point2d> visualBall;
    //        double dist;
    //        double theta;
    //        if(qnode.localMsg.ballDist > 0 /*&& qnode.localMsg.ballDist < 1200*/ ){ //210824_dohyeong_modified
    //            dist = qnode.localMsg.ballDist/10;
    //            theta = qnode.localMsg.ballTheta + currentYaw + ball_panAngle;
    //            fmod(theta, 360);

    //            Point2d Ball_point;
    //            Ball_point.x = robot_MCL->x - (dist)*sin(rad(theta));
    //            Ball_point.y = robot_MCL->y - (dist)*cos(rad(theta));
    //            visualBall.push_back(Ball_point);


    //            item_visualPoint.assign(visualBall.size(),new QGraphicsEllipseItem); //크기를선언하고 출력하지않으면, "QGraphicsScene::removeItem: item 0x7f930c006410's scene (0x0) is different from this scene (0x7fffd49fc3f0)" 경고문구가 출력된다.
    //            for(int i = 0; i < visualBall.size(); i++)
    //                item_visualPoint[i] = scene.addEllipse(visualBall[i].x+50, visualBall[i].y+50, 15, 15, QPen(Qt::cyan),QBrush(Qt::cyan));
    //        }
    //        qnode.isSeeBall = false;
    //    }
    //#endif
    //updateOdometry();

    cout<<"Now Robot X ===   "<<robot_MCL->x<<endl;
    cout<<"Now Robot Y ===   "<<robot_MCL->y<<endl;

    msg_generate::localv2_msg xymsg;

    xymsg.local_x=robot_MCL->x;
    xymsg.local_y=robot_MCL->y;
    xymsg.local_yaw=currentYaw;
    //    xymsg.ballDist = robot_MCL->

//    if(qnode.localMsg.ballDist > 0 /*&& qnode.localMsg.ballDist < 1200*/ )
//    { //210824_dohyeong_modified
//        //        IsBall = true;
//        float dist = qnode.localMsg.ballDist/10;
//        float theta = qnode.localMsg.ballTheta + currentYaw + panAngle;
//        fmod(theta, 360);

//        xymsg.ball_x = robot_MCL->x - (dist)*sin(rad(theta));
//        xymsg.ball_y = robot_MCL->y - (dist)*cos(rad(theta));
//    }
    if(isSet)
    {
        localPub_v2.publish(xymsg);
    }
    //    double particleNum = ui.le_particle->text().toDouble();
    //    kalman.init(Point2d(robot_MCL->x, robot_MCL->y));
    //    MCL = new robit_mcl();
    //    MCL->initMCL(robot_MCL->x, robot_MCL->y, robot_MCL->theta, particleNum, m_width, m_height);


    //    for(int i = 0; i < particleNum; i++)
    //    {
    //       scene.addEllipse(100, 100, 1, 1, QPen(Qt::red));  //이거 주석처리하면 계속 particle안그려짐.0305
    //    }

    ikcoordinate = 1;
    if(isSet)
    {
        //        if(yesterday == false){
        double x = 0, y = 0;
        x += qnode.Xmoved * cos(rad(yaw)) - qnode.Ymoved * sin(rad(yaw));
        y += qnode.Xmoved * sin(rad(yaw)) + qnode.Ymoved * cos(rad(yaw));

        kalman.GetPrediction();
        cv::Point odometry(kalman.Update(Point2d(x,y)));

        transform(odometry.x, odometry.y);

        RobotPose robotMCL = MCL->getRobotPose();
        robot_MCL->x = robotMCL.x;
        robot_MCL->y = robotMCL.y;
        robot_MCL->theta = robotMCL.theta;



        robot_MCL->x += odometry.x;
        robot_MCL->y += odometry.y;



        if (robot_MCL->x>1000)
        {
            robot_MCL->x=1000;

        }
        else if (robot_MCL->x<100)
        {
            robot_MCL->x=100;

        }
        else {
            robot_kalman->x += odometry.x;
        }
        if (robot_MCL->y>730)
        {
            robot_MCL->y=730;

        }
        else if (robot_MCL->y<100)
        {
            robot_MCL->y=100;

        }
        else {
            robot_kalman->y += odometry.y;
        }
        y_robot_MCL = robot_MCL;
        y_robot_kalman = robot_kalman;




        for(int i =0;i<tempPose.size();i++)
        {
            tempPose2[i][0] = tempPose[i].x;
            tempPose2[i][1] = tempPose[i].y;
            tempPose2[i][2] = tempPose[i].theta;
        }


        //        }



        //        else{



        //            if(yesterday == true)
        //            {
        //                robot_MCL    = y_robot_MCL;
        //                robot_kalman = y_robot_kalman;


        //                for(int i =0;i<10;i++)
        //                {
        //                    tempPose[i].x     = tempPose2[i][0];
        //                    tempPose[i].y     = tempPose2[i][1];
        //                    tempPose[i].theta = tempPose2[i][2];
        //                }
        //                yesterday = false;

        //            }
        //        }

        RobotPose lastPose(robot_MCL->x,robot_MCL->y,robot_MCL->theta);
        MCL-> predictionMCL(robot_MCL->x, robot_MCL->y, robot_MCL->theta, lastPose, tempPose);
        updateField();
    }

}

void MainWindow::visionCallback()
{
    if(vision_callback_cnt==30)
    {
        if(isSet)
        {
            cout<<"start updateodometry and qnode.deeplearning_flag is "<<qnode.deeplearning_flag<<endl;
            cout<<"start updateodometry and qnode.deeplearning_flag is "<<qnode.deeplearning_flag<<endl;
            //    cout<<"Is Run  ===   "<<qnode.IsRun<<endl;
            if(gameMsg.firstHalf)
                GOALPOST = gameMsg.firstside ? RIGHTSIDE : LEFTSIDE; // FIRSTHALF_GOAL;
            else
                GOALPOST = gameMsg.firstside ? LEFTSIDE : RIGHTSIDE; // SECONDHALF_GOAL;

            qnode.IsRun = true;

            //If Game State == KICKOFF && ROBOT == 2  => Poisition change
            //211130 Modified start
            //로봇을 위치를 보정해주기 위해 Reset과정을 통해 로봇을 지워준다.(Reset함수변형한것임)--Start
            //reset
            //    if(gameMsg.kickoffTeam == ROBIT && gameMsg.robotNum == 2 && gameMsg.state ==1) check_kickoff = false;
            //    if(gameMsg.penalty == 34) {
            //        check_kickoff = true;
            //        cout<<"now penalty so, check kickoff false "<<check_kickoff<<endl;
            //    }
            //    if(gameMsg.kickoffTeam == ROBIT && gameMsg.robotNum == 2 && gameMsg.state == 3 && check_kickoff == false){
            //        qnode.deeplearning_flag = true;
            //        cout<<"com kick offfffffffffff";
            //        cout<<"com kick offfffffffffff";
            //        cout<<"com kick offfffffffffff";
            //        cout<<"com kick offfffffffffff";
            //        delete MCL;
            //        delete robot_MCL;
            //        delete item_robot;

            //        for(int i = 0; i < item_particle.size(); i++)
            //            scene.removeItem(item_particle[i]);
            //        item_particle.clear();
            //        disconnect(m_updateTimer,SIGNAL(timeout()),this, SLOT(timerCallback()));
            //        robot_MCL = new RobotPose(550,400,90);
            //        print("initMCL");
            //        item_robot = new MyItem();

            //        //Reset end
            //        //Mouse event start
            //        //        // draw robot
            //        int new_robot_x ;

            //        int new_robot_y = 250+45; //   250 -> 205
            //        if(GOALPOST == LEFTSIDE) new_robot_x = 415+43 ;// 415-> 372
            //        else new_robot_x = 435+45;


            //        scene.addItem(item_robot);
            //        QPoint remapped(new_robot_x,new_robot_y);
            //        if(ui.graphicsView->rect().contains(remapped))
            //        {
            //            QPointF mousePoint = ui.graphicsView->mapToScene(remapped);

            //            robot_MCL->x = mousePoint.x();
            //            robot_MCL->y = mousePoint.y();

            //            if(robot_MCL->x <= 0)    robot_MCL->x = 1;
            //            else if(robot_MCL->x >= m_width) robot_MCL->x = m_width - 1;

            //            if(robot_MCL->y <= 0)    robot_MCL->y = 1;
            //            else if(robot_MCL->y >= m_height) robot_MCL->y = m_height - 1;
            //            robot_MCL->theta = 0;

            //            item_robot->setPos(robot_MCL->x, robot_MCL->y);
            //            item_robot->setRotation(-robot_MCL->theta);

            //            ui.lb_x->setText(QString::number(robot_MCL->x));
            //            ui.lb_y->setText(QString::number(robot_MCL->y));
            //        }


            //        //로봇을 위치를 보정해주기 위해 Reset과정을 통해 로봇을 지워준다.(Reset함수변형한것임)--End
            //        //위치를 보정한 값을 다시 로컬상에 그려준다.(Set함수변형한것임)--Start

            //        double particleNum = ui.le_particle->text().toDouble();
            //        kalman.init(Point2d(robot_MCL->x, robot_MCL->y));
            //        MCL = new robit_mcl();
            //        MCL->initMCL(robot_MCL->x, robot_MCL->y, robot_MCL->theta, particleNum, m_width, m_height);
            //        robot_kalman = robot_MCL;//계속 작동중임..
            //        MCL->z_max = ZMAX;

            //        item_particle.assign(particleNum,new QGraphicsEllipseItem);
            //        for(int i = 0; i < particleNum; i++)
            //        {
            //            if(i <501)
            //                item_particle[i] = scene.addEllipse(100, 100, 1, 1, QPen(Qt::red));  //이거 주석처리하면 계속 particle안그려짐.0305
            //            else if(i <701)
            //                item_particle[i] = scene.addEllipse(100, 100, 1, 1, QPen(Qt::yellow));  //이거 주석처리하면 계속 particle안그려짐.0305
            //            else if(i >700)
            //                item_particle[i] = scene.addEllipse(100, 100, 1, 1, QPen(Qt::blue));  //이거 주석처리하면 계속 particle안그려짐.0305
            //        }
            //        m_updateTimer = new QTimer(this);
            //        updateField();
            //        check_kickoff = true;
            //}


            //위치를 보정한 값을 다시 로컬상에 그려준다.(Set함수변형한것임)--End
            // 211130 Modified End
            for(int i = 0; i < item_visualPoint.size(); i++)
                scene.removeItem(item_visualPoint[i]);
            item_visualPoint.clear();
            msg_generate::localv2_msg xymsg;

            xymsg.local_x=robot_MCL->x;
            xymsg.local_y=robot_MCL->y;
            xymsg.local_yaw=currentYaw;
            //    xymsg.ballDist = robot_MCL->

            if(qnode.localMsg.ballDist > 0 /*&& qnode.localMsg.ballDist < 1200*/ )
            { //210824_dohyeong_modified
                //        IsBall = true;
                float dist = qnode.localMsg.ballDist/10;
                float theta = qnode.localMsg.ballTheta + robot_MCL->theta + ball_panAngle;


                fmod(theta, 360);

                cout<<"@@@@@@@@@@@theta@@@@@@@@@@@@:"<<theta<<endl;
                xymsg.ball_x = robot_MCL->x - (dist)*sin(rad(theta));
                xymsg.ball_y = robot_MCL->y - (dist)*cos(rad(theta));
            }
            if(isSet)
            {
                localPub_v2.publish(xymsg);
            }
            //    qnode.deeplearning_flag=true;
            if(qnode.deeplearning_flag==true){

                qnode.deeplearning_flag = false;
                vector<Point2d> Points;
                bool only_one = true;
                if(isSet == true && isvision == false){

                    isvision = true;

                    vector<Point2d> visualPoints;
                    vector<Point2d> visualBall;
                    vector<Point2d> visualCircle;

                    cout<<qnode.localMsg.penaltyCircleDist.size()<<endl;

                    if(qnode.localMsg.penaltyCircleDist.size()!=0&&qnode.localMsg.pointDist.size()>0)
                    {

                        cout<<"Penalty circle ----------- 1108"<<endl;
                        //PenaltyCircle을 로컬상 좌표로 변환 해주는 과정--Start

                        PCircle = true;
                        //                double dist;
                        //                double theta;


                        //                dist = qnode.localMsg.penaltyCircleDist[0]/10;
                        //                theta = qnode.localMsg.penaltyCircleTheta[0] + currentYaw + panAngle;
                        //                fmod(theta, 360);

                        //                Point2d PC_point;
                        //                PC_point.x = robot_MCL->x - (dist)*sin(rad(theta));
                        //                PC_point.y = robot_MCL->y - (dist)*cos(rad(theta));

                        //                //PenaltyCircle을 로컬상 좌표로 변환 해주는 과정--End
                        //                //로봇을 위치를 보정해주기 위해 Reset과정을 통해 로봇을 지워준다.(Reset함수변형한것임)--Start
                        //                //reset
                        //                int save_robot_x = robot_MCL->x; //지우기 전에 현재 로봇의 위치를 저장한다.
                        //                int save_robot_y = robot_MCL->y; //지우기 전에 현재 로봇의 위치를 저장한다.
                        //                double G_PC_x;
                        //                double G_PC_y;
                        //                bool continute_flag = true;
                        //                if(save_robot_x >535)//415,475,535//
                        //                {
                        //                    G_PC_x = 850;
                        //                    G_PC_y = 400;//306;
                        //                }
                        //                else if(save_robot_x < 415){
                        //                    G_PC_x = 250;//227;
                        //                    G_PC_y = 400;//306;
                        //                }
                        //                else
                        //                    continute_flag = false;
                        //                if(continute_flag == true){
                        //                    delete MCL;
                        //                    delete robot_MCL;
                        //                    delete item_robot;

                        //                    cout<<"item size b ==  "<<item_particle.size()<<endl;
                        //                    for(int i = 0; i < item_particle.size(); i++)
                        //                        scene.removeItem(item_particle[i]);
                        //                    cout<<"item size ==  "<<item_particle.size()<<endl;
                        //                    item_particle.clear();
                        //                    disconnect(m_updateTimer,SIGNAL(timeout()),this, SLOT(timerCallback()));
                        //                    // drawField();
                        //                    robot_MCL = new RobotPose(550,400,90);
                        //                    print("initMCL");
                        //                    item_robot = new MyItem();

                        //                    //Reset end
                        //                    //Mouse event start
                        //                    //        // draw robot
                        //                    int new_robot_x;
                        //                    int new_robot_y;
                        //                    if(PC_point.x >= G_PC_x && PC_point.y >=G_PC_y){
                        //                        new_robot_x = save_robot_x - PC_point.x + G_PC_x;
                        //                        new_robot_y = save_robot_y - PC_point.y + G_PC_y;
                        //                    }
                        //                    else if(PC_point.x <= G_PC_x && PC_point.y <=G_PC_y){
                        //                        new_robot_x = save_robot_x + G_PC_x - PC_point.x;
                        //                        new_robot_y = save_robot_y + G_PC_y - PC_point.y;
                        //                    }
                        //                    else if(PC_point.x >= G_PC_x && PC_point.y <=G_PC_y){
                        //                        new_robot_x = save_robot_x - PC_point.x + G_PC_x;
                        //                        new_robot_y = save_robot_y + G_PC_y - PC_point.y;
                        //                    }
                        //                    else if(PC_point.x <= G_PC_x && PC_point.y >=G_PC_y){
                        //                        new_robot_x = save_robot_x + G_PC_x - PC_point.x;
                        //                        new_robot_y = save_robot_y - PC_point.y + G_PC_y;
                        //                    }

                        //                    scene.addItem(item_robot);
                        //                    QPoint remapped(new_robot_x,new_robot_y);
                        //                    if(ui.graphicsView->rect().contains(remapped))
                        //                    {
                        //                        QPointF mousePoint = ui.graphicsView->mapToScene(remapped);

                        //                        robot_MCL->x = mousePoint.x() - 50;
                        //                        robot_MCL->y = mousePoint.y() - 50;

                        //                        if(robot_MCL->x <= 0)    robot_MCL->x = 1;
                        //                        else if(robot_MCL->x >= m_width) robot_MCL->x = m_width - 1;

                        //                        if(robot_MCL->y <= 0)    robot_MCL->y = 1;
                        //                        else if(robot_MCL->y >= m_height) robot_MCL->y = m_height - 1;
                        //                        robot_MCL->theta = 0;

                        //                        item_robot->setPos(robot_MCL->x + 50, robot_MCL->y + 50);
                        //                        item_robot->setRotation(-robot_MCL->theta);

                        //                        ui.lb_x->setText(QString::number(robot_MCL->x));
                        //                        ui.lb_y->setText(QString::number(robot_MCL->y));
                        //                    }


                        //                    //로봇을 위치를 보정해주기 위해 Reset과정을 통해 로봇을 지워준다.(Reset함수변형한것임)--End
                        //                    //위치를 보정한 값을 다시 로컬상에 그려준다.(Set함수변형한것임)--Start

                        //                    double particleNum = ui.le_particle->text().toDouble();
                        //                    kalman.init(Point2d(robot_MCL->x, robot_MCL->y));
                        //                    MCL = new robit_mcl();
                        //                    MCL->initMCL(robot_MCL->x, robot_MCL->y, robot_MCL->theta, particleNum, m_width, m_height);

                        //                    robot_kalman = robot_MCL;//계속 작동중임..

                        //                    MCL->z_max = ZMAX;

                        //                    item_particle.assign(particleNum,new QGraphicsEllipseItem);

                        //                    for(int i = 0; i < particleNum; i++)
                        //                    {
                        //                        if(i <501)
                        //                            item_particle[i] = scene.addEllipse(100, 100, 1, 1, QPen(Qt::red));  //이거 주석처리하면 계속 particle안그려짐.0305
                        //                        else if(i <701)
                        //                            item_particle[i] = scene.addEllipse(100, 100, 1, 1, QPen(Qt::yellow));  //이거 주석처리하면 계속 particle안그려짐.0305
                        //                        else if(i >700)
                        //                            item_particle[i] = scene.addEllipse(100, 100, 1, 1, QPen(Qt::blue));  //이거 주석처리하면 계속 particle안그려짐.0305
                        //                    }

                        //                    m_updateTimer = new QTimer(this);
                        //                    updateField();
                        //                    //위치를 보정한 값을 다시 로컬상에 그려준다.(Set함수변형한것임)--End

                        //                }

                        vector<double> dist;
                        vector<double> theta;
                        for(int i = 0; i < qnode.localMsg.pointDist.size(); i++)
                        {
                            if(qnode.localMsg.pointDist[i] == 99999)
                                continue;
                            else
                            {
                                dist.push_back(qnode.localMsg.pointDist[i]/10);
                                double th = qnode.localMsg.pointTheta[i] + currentYaw + panAngle;
                                fmod(th, 360);
                                theta.push_back(th);
                            }
                        }

                        //                for(int i = 40; i < theta.size(); i+=30)
                        //                {
                        //                    MCL->measurementMCL(dist, theta, *likelihood_L);
                        MCL->measurementMCL(dist, theta, *likelihoodPC);
                        //                cout<<" ---- Lcross MeasurementMCL ------  "<<endl;
                        //                cout<<" ---- Lcross MeasurementMCL ------  "<<endl;
                        MCL->resamplingMCL();

                        RobotPose robotMCL = MCL->getRobotPose();
                        robot_MCL->x        = robotMCL.x;
                        robot_MCL->y        = robotMCL.y;
                        robot_MCL->theta    = robotMCL.theta;
                        for(int i = 0; i < dist.size(); i++)
                        {
                            if(dist[i] > ZMAX)     continue;   // cm
                            Point2d visualPoint;
                            visualPoint.x = robot_MCL->x - (dist[i])*sin(rad(theta[i]));
                            visualPoint.y = robot_MCL->y - (dist[i])*cos(rad(theta[i]));
                            visualPoints.push_back(visualPoint);
                        }

                        //                    break;
                        //                }
                    }
                    else if(qnode.localMsg.xcrossDist.size()>0&&qnode.localMsg.pointDist.size()>0)
                    {
                        if(qnode.localMsg.xcrossDist[0] <1900 && qnode.localMsg.xcrossDist[0] > 0)
                        {
                            cout<<"--------------X CROSS START--- 1108---------------"<<endl;
                            cout<<"--------------X CROSS START--- 1108---------------"<<endl;
                            cout<<"--------------X CROSS START--- 1108---------------"<<endl;
                            L_cross = true;

                            vector<double> dist;
                            vector<double> theta;
                            for(int i = 0; i < qnode.localMsg.pointDist.size(); i++)
                            {
                                if(qnode.localMsg.pointDist[i] == 99999)
                                    continue;
                                else
                                {
                                    dist.push_back(qnode.localMsg.pointDist[i]/10);
                                    double th = qnode.localMsg.pointTheta[i] + currentYaw + panAngle;
                                    fmod(th, 360);
                                    theta.push_back(th);
                                }
                            }

                            //                for(int i = 40; i < theta.size(); i+=30)
                            //                {
                            //                    MCL->measurementMCL(dist, theta, *likelihood_L);
                            MCL->measurementMCL(dist, theta, *likelihood);
                            cout<<" ---- Xcross MeasurementMCL ------  "<<endl;
                            //                cout<<" ---- Lcross MeasurementMCL ------  "<<endl;
                            //                cout<<" ---- Lcross MeasurementMCL ------  "<<endl;
                            MCL->resamplingMCL();

                            RobotPose robotMCL = MCL->getRobotPose();
                            robot_MCL->x        = robotMCL.x;
                            robot_MCL->y        = robotMCL.y;
                            robot_MCL->theta    = robotMCL.theta;
                            for(int i = 0; i < dist.size(); i++)
                            {
                                if(dist[i] > ZMAX)     continue;   // cm
                                Point2d visualPoint;
                                visualPoint.x = robot_MCL->x - (dist[i])*sin(rad(theta[i]));
                                visualPoint.y = robot_MCL->y - (dist[i])*cos(rad(theta[i]));
                                visualPoints.push_back(visualPoint);
                            }

                            //                    break;
                            //                }
                        }
                    }
                    else if(qnode.localMsg.penaltyCircleDist.size()==0&&qnode.localMsg.xcrossDist.size()==0&&qnode.localMsg.pointDist.size()>0&&qnode.localMsg.oneline==0)
                    {
                        cout<<" ---- Lcross MeasurementMCL ------  "<<endl;
                        cout<<" ---- Lcross MeasurementMCL ------  "<<endl;
                        only_one=false;
                        vector<double> dist;
                        vector<double> theta;
                        for(int i = 0; i < qnode.localMsg.pointDist.size(); i++)
                        {
                            if(qnode.localMsg.pointDist[i] == 99999)
                                continue;
                            else
                            {
                                dist.push_back(qnode.localMsg.pointDist[i]/10);
                                double th = qnode.localMsg.pointTheta[i] + currentYaw + panAngle;
                                fmod(th, 360);
                                theta.push_back(th);
                            }
                        }
                        //                for(int i = 40; i < theta.size(); i+=30)
                        //                {


                        for(int i = 0; i < dist.size(); i++)
                        {
                            if(dist[i]>ZMAX) continue;   // cm
                            Point2d point;
                            point.x = robot_MCL->x - (dist[i])*sin(rad(theta[i]));
                            point.y = robot_MCL->y - (dist[i])*cos(rad(theta[i]));
                            Points.push_back(point);
                        }

                        //                if(qnode.localMsg.penaltyCircleDist.size()!=0) only_one = false;
                        //                if(qnode.localMsg.xcrossDist[0]  < 1100 && qnode.localMsg.xcrossDist[0]  > 0) only_one = false;
                        //                if(only_one == false){
                        if(Points.size()>0)
                        {
                            MCL->measurementMCL(dist, theta, *likelihood_L);//pointdist를 적용하는 부분
                            //              re  }
                            //for(int i =0;i<10;i++)
                            MCL->resamplingMCL();
                            RobotPose robotMCL = MCL->getRobotPose();
                            robot_MCL->x        = robotMCL.x;
                            robot_MCL->y        = robotMCL.y;
                            robot_MCL->theta    = robotMCL.theta;
                            for(int i = 0; i < dist.size(); i++)
                            {
                                if(dist[i]>ZMAX) continue;   // cm
                                Point2d visualPoint;
                                visualPoint.x = robot_MCL->x - (dist[i])*sin(rad(theta[i]));
                                visualPoint.y = robot_MCL->y - (dist[i])*cos(rad(theta[i]));
                                visualPoints.push_back(visualPoint);
                            }
                        }
                        //                    break;

                        //                }
                    }
                    static int savePoseCnt = 0;

                    if((savePoseCnt++)%1 == 0)
                    {
                        if(tempPose.size() < TEMPSIZE)
                        {
                            tempPose.push_back(*robot_MCL);
                        }
                        else
                        {
                            // Circular Queue
                            m_Queue_cnt = (m_Queue_cnt+1)%TEMPSIZE;
                            tempPose[m_Queue_cnt] = *robot_MCL;
                        }
                    }

                    /************************ball 좌표 넣는 부분***************************/
                    bool IsBall = false;
                    double dist;
                    double theta;
                    if(qnode.localMsg.ballDist > 0){ //210824_dohyeong_modified
                        IsBall = true;
                        dist = qnode.localMsg.ballDist/10;
                        theta = qnode.localMsg.ballTheta + currentYaw + panAngle;
                        fmod(theta, 360);

                        Point2d Ball_point;
                        Ball_point.x = robot_MCL->x - (dist)*sin(rad(theta));
                        Ball_point.y = robot_MCL->y - (dist)*cos(rad(theta));
                        visualBall.push_back(Ball_point);
                    }
                    /************************ball 좌표 넣는 부분 End ***************************/

                    //drawing Points of contours
                    //            for(int i = 0; i < item_visualPoint.size(); i++)
                    //                scene.removeItem(item_visualPoint[i]);


                    //            item_visualPoint.clear();
                    item_visualPoint.assign(visualPoints.size()+1+visualBall.size(),new QGraphicsEllipseItem); //크기를선언하고 출력하지않으면, "QGraphicsScene::removeItem: item 0x7f930c006410's scene (0x0) is different from this scene (0x7fffd49fc3f0)" 경고문구가 출력된다.

                    if(L_cross==true){
                        if(IsBall == true){
                            for(int i = 0; i < visualBall.size(); i++)
                                item_visualPoint[i] = scene.addEllipse(visualBall[i].x, visualBall[i].y, 15, 15, QPen(Qt::cyan),QBrush(Qt::cyan));
                        }
                        for(int i = 1; i < visualPoints.size(); i++)
                            item_visualPoint[i] = scene.addEllipse(visualPoints[i].x, visualPoints[i].y, 5, 5, QPen(Qt::yellow),QBrush(Qt::yellow));
                    }
                    else if(PCircle == true){
                        if(IsBall == true){
                            for(int i = 0; i < visualBall.size(); i++)
                                item_visualPoint[i] = scene.addEllipse(visualBall[i].x, visualBall[i].y, 15, 15, QPen(Qt::cyan),QBrush(Qt::cyan));
                        }
                        for(int i = 1; i < visualPoints.size(); i++)
                            item_visualPoint[i] = scene.addEllipse(visualPoints[i].x, visualPoints[i].y, 5, 5, QPen(Qt::red),QBrush(Qt::red));

                    }
                    else if(only_one == false&&true){
                        if(IsBall == true){
                            for(int i = 0; i < visualBall.size(); i++)
                                item_visualPoint[i] = scene.addEllipse(visualBall[i].x, visualBall[i].y, 15, 15, QPen(Qt::cyan),QBrush(Qt::cyan));
                        }
                        for(int i = 1; i < visualPoints.size(); i++)
                            item_visualPoint[i] = scene.addEllipse(visualPoints[i].x, visualPoints[i].y, 5, 5, QPen(Qt::blue),QBrush(Qt::blue));

                    }
                    else{
                        if(IsBall == true){
                            for(int i = 0; i < visualBall.size(); i++)
                                item_visualPoint[i] = scene.addEllipse(visualBall[i].x, visualBall[i].y, 15, 15, QPen(Qt::cyan),QBrush(Qt::cyan));
                        }
                        if(true)
                        {
                            for(int i = 1; i < visualPoints.size(); i++)
                                item_visualPoint[i] = scene.addEllipse(visualPoints[i].x, visualPoints[i].y, 5, 5, QPen(Qt::white),QBrush(Qt::white));
                        }
                    }
                    drawline=false;

                    item_visualPoint[visualPoints.size()+visualBall.size()] = scene.addEllipse(robot_kalman->x, robot_kalman->y, 10, 10, QPen(Qt::black),QBrush(Qt::black));
                    isvision = false;
                    L_cross = false;
                    PCircle = false;
                    only_one=true;
                }

                RobotPose lastPose(robot_MCL->x,robot_MCL->y,robot_MCL->theta);
                MCL-> predictionMCL(robot_MCL->x, robot_MCL->y, robot_MCL->theta, lastPose, tempPose);

            }
            else if(qnode.isSeeBall){
                vector<Point2d> visualBall;
                double dist;
                double theta;
                if(qnode.localMsg.ballDist > 0 /*&& qnode.localMsg.ballDist < 1200*/ ){ //210824_dohyeong_modified
                    dist = qnode.localMsg.ballDist/10;
                    theta = qnode.localMsg.ballTheta + robot_MCL->theta + ball_panAngle;
                    fmod(theta, 360);

                    Point2d Ball_point;
                    Ball_point.x = robot_MCL->x - (dist)*sin(rad(theta));
                    Ball_point.y = robot_MCL->y - (dist)*cos(rad(theta));
                    visualBall.push_back(Ball_point);


                    item_visualPoint.assign(visualBall.size(),new QGraphicsEllipseItem); //크기를선언하고 출력하지않으면, "QGraphicsScene::removeItem: item 0x7f930c006410's scene (0x0) is different from this scene (0x7fffd49fc3f0)" 경고문구가 출력된다.
                    for(int i = 0; i < visualBall.size(); i++)
                        item_visualPoint[i] = scene.addEllipse(visualBall[i].x, visualBall[i].y, 15, 15, QPen(Qt::cyan),QBrush(Qt::cyan));
                }
                qnode.isSeeBall = false;
            }


            m_updateTimer = new QTimer(this);
            updateField();

        }
        vision_callback_cnt=0;
    }
    else {
        vision_callback_cnt++;
    }
}

void MainWindow::updateUdp1()
{
    if(!robot1_alive)
    {
        robot1 =new MyItem;
        robot1->color1=QBrush(Qt::yellow);
        robot1->color2=QBrush(Qt::red);
        robot1_alive=true;
        scene.addItem(robot1);
    }

    cout<<"signal1 check"<<endl;
    //    rb1_x=550;
    //    rb1_y=400;
    //    rb1_yaw=0;
    if(!(rb1_x==0&&rb1_y==0))
    {

        robot1->setPos(rb1_x,rb1_y);
        robot1->setRotation(-rb1_yaw);


        //        robot1.assign(2,new QGraphicsEllipseItem);
        //        robot1[0]=scene.addEllipse(rb1_x-10, rb1_y-10, 20, 20, QPen(Qt::gray),QBrush(Qt::gray));
        //        robot1[1]=scene.addEllipse(rb1_x-5+10*cos(rb1_yaw), rb1_y-5+10*sin(rb1_yaw), 10, 10, QPen(Qt::red),QBrush(Qt::red));
    }
    else
    {
        scene.removeItem(robot1);
        robot1_alive=false;
    }
}
void MainWindow::updateUdp2()
{

    if(!robot2_alive)
    {
        robot2 =new MyItem;
        robot2->color1=QBrush(Qt::black);
        robot2->color2=QBrush(Qt::red);
        robot2_alive=true;
        scene.addItem(robot2);
    }

    cout<<"signal2 check"<<endl;
    if(!(rb2_x==0&&rb2_y==0))
    {

        robot2->setPos(rb2_x,rb2_y);
        robot2->setRotation(-rb2_yaw);


    }
    else
    {
        scene.removeItem(robot2);
        robot2_alive=false;
    }
}
void MainWindow::updateUdp3()
{
    if(!robot3_alive)
    {
        robot3 =new MyItem;
        robot3->color1=QBrush(Qt::magenta);
        robot3->color2=QBrush(Qt::red);
        robot3_alive=true;
        scene.addItem(robot3);
    }

    cout<<"signal3 check"<<endl;
    if(!(rb3_x==0&&rb3_y==0))
    {

        robot3->setPos(rb3_x,rb3_y);
        robot3->setRotation(-rb3_yaw);


    }
    else
    {
        scene.removeItem(robot3);
        robot3_alive=false;
    }
}
void MainWindow::updateUdp4()
{
    if(!robot4_alive)
    {
        robot4 =new MyItem;
        robot4->color1=QBrush(Qt::white);
        robot4->color2=QBrush(Qt::red);
        robot4_alive=true;
        scene.addItem(robot4);
    }

    cout<<"signal4 check"<<endl;
    if(!(rb4_x==0&&rb4_y==0))
    {

        robot4->setPos(rb4_x,rb4_y);
        robot4->setRotation(-rb4_yaw);


    }
    else
    {
        scene.removeItem(robot4);
        robot4_alive=false;
    }
}
void MainWindow::gameDataUpdate()
{
        if(gameMsg.firstHalf)
            GOALPOST = gameMsg.firstside ? RIGHTSIDE : LEFTSIDE; // FIRSTHALF_GOAL;
        else
            GOALPOST = gameMsg.firstside ? LEFTSIDE : RIGHTSIDE; // SECONDHALF_GOAL;
    /** for kickoff robot begin */
    if(gameMsg.robotNum==2&&gameMsg.state == STATE_SET && gameMsg.kickoffTeam == ROBIT)
    {
        if(gameMsg.firstHalf)
        {

            robot_MCL->x=  (FIELD_WIDTH / 2) + 20;
            robot_MCL->y=  (FIELD_HEIGHT / 2);
        }
        else
        {
            robot_MCL->x=  (FIELD_WIDTH / 2) - 20;
            robot_MCL->y=  (FIELD_HEIGHT / 2);
        }
        if(isSet)
        {
            RobotPose lastPose(robot_MCL->x,robot_MCL->y,robot_MCL->theta);
            MCL-> predictionMCL(robot_MCL->x, robot_MCL->y, robot_MCL->theta, lastPose, tempPose);
            updateField();
        }
    }
    //        if(nowX <= 0)    nowX = 1;
    //        else if(nowX >= fieldW) nowX = fieldW - 1;

    //        if(nowY <= 0)    nowY = 1;
    //        else if(nowY >= fieldH) nowY = fieldH - 1;

    //        pastX = nowX; pastY = nowY;

    //        if(!robot->isActive())
    //            scene->addItem(robot);supercom@dohyeongssu
    //        robot->setPos(nowX - fieldW / 2, nowY - fieldH / 2);
    //        robot->setRotation(-currentYaw);
    //        ui.textEdit_area->setText(QString::number(Where_is_Robot(nowX, nowY)));

    //        if(isSet)
    //        {
    //            on_pushButton_reset_clicked();
    //            isYaw = true;
    //        }
    //    }
    else if(gameMsg.robotNum==2&&stateBefore == STATE_SET && gameMsg.state == STATE_PLAYING && gameMsg.kickoffTeam == ROBIT)
    {
        if(gameMsg.firstHalf)
        {
            robot_MCL->x=  (FIELD_WIDTH / 2) + 20;
            robot_MCL->y=  (FIELD_HEIGHT / 2);
        }
        else
        {
            robot_MCL->x=  (FIELD_WIDTH / 2) - 20;
            robot_MCL->y=  (FIELD_HEIGHT / 2);
        }
        if(isSet)
        {
            RobotPose lastPose(robot_MCL->x,robot_MCL->y,robot_MCL->theta);
            MCL-> predictionMCL(robot_MCL->x, robot_MCL->y, robot_MCL->theta, lastPose, tempPose);
            updateField();
        }
    }
    //        if(nowX <= 0)    nowX = 1;
    //        else if(nowX >= fieldW) nowX = fieldW - 1;

    //        if(nowY <= 0)    nowY = 1;
    //        else if(nowY >= fieldH) nowY = fieldH - 1;

    //        if(!robot->isActive())
    //            scene->addItem(robot);
    //        robot->setPos(nowX - fieldW / 2, nowY - fieldH / 2);
    //        robot->setRotation(-currentYaw);
    //        ui.textEdit_area->setText(QString::number(Where_is_Robot(nowX, nowY)));

    //        if(!isSet)
    //        {
    //            isYaw = true;
    //            on_pushButton_set_clicked();
    //        }
    //    }
    //    /** for kickoff robot end */

    //cout<<gameMsg.penalty<<endl;

    if(gameMsg.state==0)
    {
        switch(gameMsg.robotNum)
        {
        case 1:
        {
            if(GOALPOST == LEFTSIDE)
            {
                robot_MCL->x = 370;
                robot_MCL->y = 705;
            }
            else {
                robot_MCL->x = 730;
                robot_MCL->y = 705;
            }
            break;
        }
        case 2:
        {
            if(GOALPOST == LEFTSIDE)
            {
                robot_MCL->x = 440;
                robot_MCL->y = 705;
            }
            else {
                robot_MCL->x = 660;
                robot_MCL->y = 705;
            }
            break;

        }
        case 3:
        {
            if(GOALPOST == LEFTSIDE)
            {
                robot_MCL->x = 300;
                robot_MCL->y = 705;
            }
            else {
                robot_MCL->x = 800;
                robot_MCL->y = 705;
            }
            break;


        }
        default:
        {

            break;
        }
        }
        if(isSet)
        {
            RobotPose lastPose(robot_MCL->x,robot_MCL->y,robot_MCL->theta);
            MCL-> predictionMCL(robot_MCL->x, robot_MCL->y, robot_MCL->theta, lastPose, tempPose);
            updateField();
        }
    }
    if(gameMsg.penalty!=0)
    {
        if(gameMsg.firstHalf)
        {

            robot_MCL->x = 800;
            robot_MCL->y = 710;

        }
        else
        {

            robot_MCL->x = 300;
            robot_MCL->y = 710;

        }
        if(isSet)
        {
            RobotPose lastPose(robot_MCL->x,robot_MCL->y,robot_MCL->theta);
            MCL-> predictionMCL(robot_MCL->x, robot_MCL->y, robot_MCL->theta, lastPose, tempPose);
            updateField();
        }
    }
    //6Penaly 9 Throw in
    else if(gameMsg.state==3&&gameMsg.secondState==6&&gameMsg.secondInfo[0]==ROBIT)
    {
        //예외****

        if(gameMsg.robotNum==2)
        {
            if(GOALPOST == LEFTSIDE)
            {
                robot_MCL->x = 815;
                robot_MCL->y = 400;
            }
            else {
                robot_MCL->x = 285;
                robot_MCL->y = 400;
            }
        }
        if(gameMsg.robotNum==1&&((rb1_x==0&&rb1_y==0)||((rb1_x==300||rb1_x==800)&&rb1_y==710)))
        {
            if(GOALPOST == LEFTSIDE)
            {
                robot_MCL->x = 815;
                robot_MCL->y = 400;
            }
            else {
                robot_MCL->x = 285;
                robot_MCL->y = 400;
            }
        }
        else if(gameMsg.robotNum==1&&!((rb1_x==0&&rb1_y==0)||((rb1_x==300||rb1_x==800)&&rb1_y==710)))
        {
            if(GOALPOST == LEFTSIDE)
            {
                robot_MCL->x = 800;
                robot_MCL->y = 650;
            }
            else {
                robot_MCL->x = 300;
                robot_MCL->y = 650;
            }
        }
        if(gameMsg.robotNum==3&&((rb1_x==0&&rb1_y==0)||((rb1_x==300||rb1_x==800)&&rb1_y==710))&&((rb2_x==0&&rb2_y==0)||((rb2_x==300||rb2_x==800)&&rb2_y==710)))
        {
            if(GOALPOST == LEFTSIDE)
            {
                robot_MCL->x = 815;
                robot_MCL->y = 400;
            }
            else {
                robot_MCL->x = 285;
                robot_MCL->y = 400;
            }
        }

        if(isSet)
        {
            RobotPose lastPose(robot_MCL->x,robot_MCL->y,robot_MCL->theta);
            MCL-> predictionMCL(robot_MCL->x, robot_MCL->y, robot_MCL->theta, lastPose, tempPose);
            updateField();
        }

    }
    else if(gameMsg.state==3&&gameMsg.secondState==6&&gameMsg.secondInfo[0]!=ROBIT)
    {
        if(gameMsg.robotNum==2)
        {
            if(GOALPOST == LEFTSIDE)
            {
                robot_MCL->x = 550;
                robot_MCL->y = 400;
            }
            else {
                robot_MCL->x = 550;
                robot_MCL->y = 400;
            }
        }
        if(gameMsg.robotNum==1)
        {
            if(GOALPOST == LEFTSIDE)
            {
                robot_MCL->x = 550;
                robot_MCL->y = 150;
            }
            else {
                robot_MCL->x = 550;
                robot_MCL->y = 150;
            }
        }
        if(gameMsg.robotNum==3)
        {
            if(GOALPOST == LEFTSIDE)
            {
                robot_MCL->x = 300;
                robot_MCL->y = 650;
            }
            else {
                robot_MCL->x = 800;
                robot_MCL->y = 650;
            }
        }
        if(isSet)
        {
            RobotPose lastPose(robot_MCL->x,robot_MCL->y,robot_MCL->theta);
            MCL-> predictionMCL(robot_MCL->x, robot_MCL->y, robot_MCL->theta, lastPose, tempPose);
            updateField();
        }

    }
    else if(gameMsg.state==3&&gameMsg.secondState==9&&gameMsg.secondInfo[0]==ROBIT)
    {
        //예외****
//        if(gameMsg.robotNum==3)
//        {
//            if(GOALPOST == LEFTSIDE)
//            {
//                robot_MCL->x = 850;
//                robot_MCL->y = 400;
//            }
//            else {
//                robot_MCL->x = 250;
//                robot_MCL->y = 400;
//            }
//        }
        if(isSet)
        {
            RobotPose lastPose(robot_MCL->x,robot_MCL->y,robot_MCL->theta);
            MCL-> predictionMCL(robot_MCL->x, robot_MCL->y, robot_MCL->theta, lastPose, tempPose);
            updateField();
        }

    }
    //freekick,penalty kick
    //        if(gameMsg.state == STATE_PLAYING&&gameMsg.secondState==STATE2_DIRECT_FREEKICK)
    //        {
    //            if(gameMsg.firstHalf)
    //            {
    //                robot_MCL->x = 300;
    //                robot_MCL->y = 710;
    //            }
    //            else
    //            {
    //                robot_MCL->x = 800;
    //                robot_MCL->y = 710;
    //            }
    //        }
    //        if(nowX <= 0)    nowX = 1;
    //        else if(nowX >= fieldW) nowX = fieldW - 1;

    //        if(nowY <= 0)    nowY = 1;
    //        else if(nowY >= fieldH) nowY = fieldH - 1;

    //        pastX = nowX; pastY = nowY;

    //        if(!robot->isActive())
    //            scene->addItem(robot);
    //        robot->setPos(nowX - fieldW / 2, nowY - fieldH / 2);
    //        robot->setRotation(-currentYaw);
    //        ui.textEdit_area->setText(QString::number(Where_is_Robot(nowX, nowY)));

    //        if(isSet)
    //        {
    //            isPenalty = true;
    //            on_pushButton_reset_clicked();
    //            isYaw = true;
    //        }
    //    }

    //        if(nowX <= 0)    nowX = 1;
    //        else if(nowX >= fieldW) nowX = fieldW - 1;

    //        if(nowY <= 0)    nowY = 1;
    //        else if(nowY >= fieldH) nowY = fieldH - 1;

    //        pastX = nowX; pastY = nowY;

    //        if(!robot->isActive())
    //            scene->addItem(robot);
    //        robot->setPos(nowX - fieldW / 2, nowY - fieldH / 2);
    //        robot->setRotation(-currentYaw);
    //        ui.textEdit_area->setText(QString::number(Where_is_Robot(nowX, nowY)));

    //        if(!isSet)
    //        {
    //            isPenalty = false;
    //            isYaw = true;
    //            on_pushButton_set_clicked();
    //        }
    //    }




    stateBefore = gameMsg.state;
}


void MainWindow::updateField()
{
    RobotPose robotMCL = MCL-> getRobotPose();
    //    if(fuckyous == false || fuckyous_num != 0){
    robot_MCL->x = robotMCL.x;
    robot_MCL->y = robotMCL.y;
    robot_MCL->theta = robotMCL.theta;
    //}
    //    else{
    //    robot_MCL->x =100;
    //    robot_MCL->y = 100;
    //    robot_MCL->theta = robotMCL.theta;
    //        fuckyous_num++;
    //    }
    if(robot_MCL->x <= 0)    robot_MCL->x = 1;
    else if(robot_MCL->x >= m_width) robot_MCL->x = m_width - 1;

    if(robot_MCL->y <= 0)    robot_MCL->y = 1;
    else if(robot_MCL->y >= m_height) robot_MCL->y = m_height - 1;
    if(MCL->check_particle == true){
        MCL->check_particle = true;
    }


    vector<RobotPose> pfMCL =  MCL-> getParticlePose();


    //        item_robot->setPos(dX+50,dY+50);
    print("robot_MCL->x: ", robot_MCL->x);
    item_robot->setPos(robot_MCL->x, robot_MCL->y);
    print("theta: ", robot_MCL->theta);

    for(int i = 0; i <pfMCL.size(); i++)
    {

        int x = pfMCL[i].x-100;
        int y = pfMCL[i].y-100;
        item_particle[i]->setPos(x,y);
    }

    MCL->check_particle = false;
}



void MainWindow::drawField()
{

#if ROBIT_TEAM
    m_width = 1100;//700;
    m_height = 800;
    nRadius =75; // center circle radius
    int Goal_area_length = 100;//92;
    int Goal_area_width = 300;//362//412;
#else
    m_width = 900;
    m_height = 600;
    int nRadius =75; // center circle radius
    int Goal_area_length = 100;
    int Goal_area_width = 500;
#endif

    m_field = new Mat(m_height, m_width, CV_8UC1);
    m_penaltyCircle = new Mat(m_height, m_width, CV_8UC1);
    m_Lcross = new Mat(m_height, m_width, CV_8UC1);
    int i = 0;
    image = imread("/home/robit/catkin_ws/src/localization_v2/img/2022_RoboCup_Field.png");
    cv::resize( image, image, cv::Size( (m_width), (m_height)), 0, 0, CV_INTER_NN );

    vector<vector<cv::Point>> pointAll;
    vector<cv::Point> point;
    /************************** START 0824 modified by dohyeong ***********************/
    /************************** This part is a likelihood field of full field ***********************/
    //    for(i = 0; i < 51; i++) // Top Horizontal_left
    //    {
    //        m_field->data[m_field->cols*0 + i] = 255;
    //        point.push_back(cv::Point(i,0));
    //    }

    //    for(i = 373; i < 476; i++) // Top Horizontal_middle
    //    {
    //        m_field->data[m_field->cols*0 + i] = 255;
    //        point.push_back(cv::Point(i,0));
    //    }

    //    for(i = 798; i < m_width; i++) // Top Horizontal_right
    //    {
    //        m_field->data[m_field->cols*0 + i] = 255;
    //        point.push_back(cv::Point(i,0));
    //    }

    //    for(i = 0; i < 51; i++) // Bottom Horizontal_left
    //    {
    //        m_field->data[m_field->cols*(m_field->rows-1) + i] = 255;
    //        point.push_back(cv::Point(i,m_field->rows-1));
    //    }

    //    for(i = 373; i < 476; i++) // Bottom Horizontal_middle
    //    {
    //        m_field->data[m_field->cols*(m_field->rows-1) + i] = 255;
    //        point.push_back(cv::Point(i,m_field->rows-1));
    //    }

    //    for(i = 798; i < m_width; i++) // Bottom Horizontal_right
    //    {
    //        m_field->data[m_field->cols*(m_field->rows-1) + i] = 255;
    //        point.push_back(cv::Point(i,m_field->rows-1));
    //    }

    //    for(i = 1; i < 52; i++)//center vertical_top
    //    {
    //        m_field->data[m_field->cols*i + m_field->cols/2] = 255;
    //        point.push_back(cv::Point( m_field->cols/2,     i));
    //    }
    /************************** END 0824 modified by dohyeong ***********************/

    //    for(i = 100; i < 700; i++)//center vertical_middle
    //    {
    //        m_field->data[m_field->cols*i + m_field->cols/2] = 255;
    //        point.push_back(cv::Point( m_field->cols/2,     i));
    //    }

    /************************** START 0824 modified by dohyeong ***********************/
    /************************** This part is a likelihood field of full field ***********************/
    //    for(i = 448; i < m_height-1; i++)//center vertical_bottom
    //    {
    //        m_field->data[m_field->cols*i + m_field->cols/2] = 255;
    //        point.push_back(cv::Point( m_field->cols/2,     i));
    //    }


    //    for(i = 3; i < 128; i++) // Left Vertical_top
    //    {
    //        m_field->data[m_field->cols*i +1   ] = 255;
    //        point.push_back(cv::Point(0, i));
    //    }

    //    for(i = 374; i < m_height-3; i++) // Left Vertical_bottom
    //    {
    //        m_field->data[m_field->cols*i +1   ] = 255;
    //        point.push_back(cv::Point(0, i));
    //    }

    //    for(i = 3; i < 128; i++) // Right Vertical_top
    //    {
    //        m_field->data[m_field->cols*i + m_field->cols - 1] = 255;
    //        point.push_back(cv::Point(m_field->cols - 1, i));
    //    }
    //    for(i = 374; i < m_height-3; i++) // Right Vertical_bottom
    //    {
    //        m_field->data[m_field->cols*i + m_field->cols - 1] = 255;
    //        point.push_back(cv::Point(m_field->cols - 1, i));
    //    }

    /************************** END 0824 modified by dohyeong ***********************/

    //    // LEFT keeper area
    //    for(i = 3; i<Goal_area_length; i++)
    //    {
    //        m_field->data[m_field->cols*76/*52*/ + i] = 255;
    //        point.push_back(cv::Point(i, 76/*52*/));
    //    }
    //    for(i = 3; i<Goal_area_length; i++)
    //    {
    //        m_field->data[m_field->cols*m_field->rows - m_field->cols*76/*52*/ + i] = 255;
    //        point.push_back(cv::Point(i, m_field->rows - 76/*52*/));
    //    }
    //    for(i = 1; i<45; i++)//너비임!
    //    {
    //        m_field->data[Goal_area_length-1 + m_field->cols*76/*52*/ + m_field->cols*i] = 255;
    //        point.push_back(cv::Point(Goal_area_length-1, 76/*52*/ + i));
    //    }
    //    for(i = 320; i<Goal_area_width; i++)//너비임!
    //    {
    //        m_field->data[Goal_area_length-1 + m_field->cols*76/*52*/ + m_field->cols*i] = 255;
    //        point.push_back(cv::Point(Goal_area_length-1, 76/*52*/ + i));
    //    }
    //    // RIGHT keeper area515
    //    for(i = 3; i<Goal_area_length; i++)
    //    {
    //        m_field->data[m_field->cols*76/*52*/ + m_field->cols-i] = 255;
    //        point.push_back(cv::Point(m_field->cols-i, 76/*52*/));
    //    }

    //    for(i = 3; i<Goal_area_length; i++)
    //    {
    //        m_field->data[m_field->cols*m_field->rows - m_field->cols*76/*52*/ + m_field->cols - i] = 255;
    //        point.push_back(cv::Point(m_field->cols - i, m_field->rows - 76/*52*/));
    //    }

    //    for(i = 1; i<45; i++)
    //    {
    //        m_Lcross->data[m_Lcross->cols*75/*52*/ + m_Lcross->cols - 142/*91*/ + m_Lcross->cols*i] = 255;
    //        point.push_back(cv::Point(m_Lcross->cols - 142/*91*/, 75/*52*/ + i));
    //    }

    //    for(i = 320; i<Goal_area_width+3; i++)
    //    {
    //        m_Lcross->data[m_Lcross->cols*75/*52*/ + m_Lcross->cols - 142/*91*/ + m_Lcross->cols*i] = 255;
    //        point.push_back(cv::Point(m_Lcross->cols - 142/*91*/, 75/*52*/ + i));
    //    }


    /************************** START 0824 modified by dohyeong ***********************/
    /************************** This part is a likelihood field of full field ***********************/

    //    // left keeper area
    //    for(i = 3; i<Goal_area_length; i++)
    //    {
    //        m_field->data[m_field->cols*76/*52*/ + i] = 255;
    //        point.push_back(cv::Point(i, 76/*52*/));
    //    }
    //    for(i = 3; i<Goal_area_length; i++)
    //    {
    //        m_field->data[m_field->cols*m_field->rows - m_field->cols*76/*52*/ + i] = 255;
    //        point.push_back(cv::Point(i, m_field->rows - 76/*52*/));
    //    }
    //    for(i = 1; i<Goal_area_width; i++)//너비임!
    //    {
    //        m_field->data[Goal_area_length-1 + m_field->cols*76/*52*/ + m_field->cols*i] = 255;
    //        point.push_back(cv::Point(Goal_area_length-1, 76/*52*/ + i));
    //    }

    //    // right keeper area
    //    for(i = 3; i<Goal_area_length; i++)
    //    {
    //        m_field->data[m_field->cols*76/*52*/ + m_field->cols-i] = 255;
    //        point.push_back(cv::Point(m_field->cols-i, 76/*52*/));
    //    }

    //    for(i = 3; i<Goal_area_length; i++)
    //    {
    //        m_field->data[m_field->cols*m_field->rows - m_field->cols*76/*52*/ + m_field->cols - i] = 255;
    //        point.push_back(cv::Point(m_field->cols - i, m_field->rows - 76/*52*/));
    //    }

    //    for(i = 1; i<Goal_area_width; i++)
    //    {
    //        m_field->data[m_field->cols*75/*52*/ + m_field->cols - 142/*91*/ + m_field->cols*i] = 255;
    //        point.push_back(cv::Point(m_field->cols - 142/*91*/, 75/*52*/ + i));
    //    }

    // center circle
    int nCx, nCy;
    nCx = m_field->cols/2;
    nCy = m_field->rows/2;
    int dR2 = nRadius*nRadius;
    for(int y = nCy - nRadius; y <= nCy + nRadius; y++)
    {
        for(int x = nCx - nRadius; x <= nCx + nRadius; x++)
        {
            int A = (y-nCy)*(y-nCy) + (x-nCx)*(x-nCx);
            if(50 >= abs(A-dR2))
            {
                if(x == nCx)
                    continue;
                m_field->data[y*m_field->cols + x] = 255;
                point.push_back(cv::Point(x, y));
            }
        }
    }
    for(i = 250; i<800-250; i++)
    {
        m_field->data[m_field->cols*i + 550] = 255;
        point.push_back(cv::Point(550, i));
    }
//    for(i = 0; i<=20; i++)
//    {
//        m_field->data[m_field->cols*400 + 545+i] = 255;
//        point.push_back(cv::Point(540+i, 400));
//    }

    /************************** START 0824 modified by dohyeong ***********************/
    /************************** This part is a likelihood field of full field ***********************/
    //    // left penalty circle
    //    int LnCx, LnCy;
    //    LnCx = 180;/*110;*/
    //    LnCy = m_field->rows/2;
    //    cv::circle(*m_field,Point(LnCx,LnCy),5,255,1);


    //    for(int y = LnCy - 5; y <= LnCy + 5; y++)
    //    {
    //        for(int x = LnCx - 5; x <= LnCx + 5; x++)
    //        {
    //            if(m_field->data[y*m_field->cols + x] == 255)
    //                point.push_back(cv::Point(x, y));
    //        }
    //    }

    //    int RnCx, RnCy;
    //    RnCx = m_field->cols - 180/*110*/;
    //    RnCy = m_field->rows/2;
    //    cv::circle(*m_field,Point(RnCx,RnCy),5,255,1);


    //    for(int y = RnCy - 5; y <= RnCy + 5; y++)
    //    {
    //        for(int x = RnCx - 5; x <= RnCx + 5; x++)
    //        {
    //            if(m_field->data[y*m_field->cols + x] == 255)
    //                point.push_back(cv::Point(x, y));
    //        }
    //    }

    /************************** END 0824 modified by dohyeong ***********************/
    /************************** This part is a likelihood field of full field ***********************/

    pointAll.push_back(point);
    point.clear();

    likelihood = new Likelihood(*m_field, 1, ui.Zrand->value(), 1, 5, 1);
    likelihood->_linePlot.clear();
    likelihood->_linePlot = pointAll;
    pointAll.clear();


    // ----------------------------------- penalty circle -----------------------------------
    //----------------------이 부분 지우면 pc에 저장 안됌  start ----------------------//
    // left penalty circle
    //    int LnCx2, LnCy2;
    //    LnCx2 = 250/*110*/;
    //    LnCy2 = m_penaltyCircle->rows/2;
    //    cv::circle(*m_penaltyCircle,Point(LnCx2,LnCy2),5,255,1);

    for(i =300; i<=320; i++)
    {
        m_penaltyCircle->data[m_penaltyCircle->cols*400 + i] = 255;
        point.push_back(cv::Point(i,400));
    }

    for(i =390; i<=410; i++)
    {
        m_penaltyCircle->data[m_penaltyCircle->cols*i + 310] = 255;
        point.push_back(cv::Point(310,i));
    }
    for(i =300; i<=320; i++)
    {
        m_penaltyCircle->data[m_penaltyCircle->cols*400 +m_penaltyCircle->cols-i] = 255;
        point.push_back(cv::Point(m_penaltyCircle->cols-i,400));
    }

    for(i =390; i<=410; i++)
    {
        m_penaltyCircle->data[m_penaltyCircle->cols*i +m_penaltyCircle->cols-310] = 255;
        point.push_back(cv::Point(m_penaltyCircle->cols-310,i));
    }
//    for(i =250; i<=550; i++)
//    {
//        m_penaltyCircle->data[m_penaltyCircle->cols*i +200] = 255;
//        point.push_back(cv::Point(200,i));
//    }
    for(i =250; i<=550; i++)
    {
        m_penaltyCircle->data[m_penaltyCircle->cols*i +350] = 255;
        point.push_back(cv::Point(350,i));
    }
//    for(i =250; i<=550; i++)
//    {
//        m_penaltyCircle->data[m_penaltyCircle->cols*i +900] = 255;
//        point.push_back(cv::Point(900,i));
//    }
    for(i =250; i<=550; i++)
    {
        m_penaltyCircle->data[m_penaltyCircle->cols*i +750] = 255;
        point.push_back(cv::Point(750,i));
    }
    //    for(int y = LnCy2 - 10; y <= LnCy2 + 10; y++)
    //    {
    //        for(int x = LnCx2 - 10; x <= LnCx2 + 10; x++)
    //        {
    //            if(m_penaltyCircle->data[y*m_penaltyCircle->cols + x] == 255)
    //                point.push_back(cv::Point(x, y));
    //        }
    //    }
    //    // left penalty circle
    //    int RnCx2, RnCy2;
    //    RnCx2 = m_penaltyCircle->cols - 250/*110*/;
    //    RnCy2 = m_penaltyCircle->rows/2;
    //    cv::circle(*m_penaltyCircle,Point(RnCx2,RnCy2),5,255,1);


    //    for(int y = RnCy2 - 10; y <= RnCy2 + 10; y++)
    //    {
    //        for(int x = RnCx2 - 10; x <= RnCx2 + 10; x++)
    //        {
    //            if(m_penaltyCircle->data[y*m_penaltyCircle->cols + x] == 255)
    //                point.push_back(cv::Point(x, y));
    //        }
    //    }


    //----------------------이 부분 지우면 pc에 저장 안됌  end ----------------------//
    pointAll.push_back(point);
    point.clear();
    likelihoodPC = new Likelihood(*m_penaltyCircle, 1, ui.Zrand->value(), 1, 5, 1);
    likelihoodPC->_linePlot.clear();
    likelihoodPC->_linePlot = pointAll;
    pointAll.clear();


    // ----------------------------------- L cross -----------------------------------//
    // LEFT keeper area
    //    for(i = 50; i<Goal_area_length; i++)
    //    {
    //        m_Lcross->data[m_Lcross->cols*250/*52*/ + i+100] = 255;
    //        point.push_back(cv::Point(i+100, 250/*52*/));
    //    }
    //    for(i = 50; i<Goal_area_length; i++)
    //    {
    //        m_Lcross->data[m_Lcross->cols*m_Lcross->rows - m_Lcross->cols*250/*52*/ + i+100] = 255;
    //        point.push_back(cv::Point(i+100, m_field->rows - 250/*52*/));
    //    }
    //    for(i = 1; i<50; i++)//너비임!
    //    {
    //        m_Lcross->data[Goal_area_length-1 + m_Lcross->cols*250/*52*/ + m_Lcross->cols*i+100] = 255;
    //        point.push_back(cv::Point(Goal_area_length-1+100, 250/*52*/ + i));
    //    }
    //    for(i = 250; i<Goal_area_width; i++)//너비임!
    //    {
    //        m_field->data[Goal_area_length-1 + m_Lcross->cols*250/*52*/ + m_Lcross->cols*i+100] = 255;
    //        point.push_back(cv::Point(Goal_area_length-1+100, 250/*52*/ + i));
    //    }
    //    // RIGHT keeper area
    //    for(i = 50; i<Goal_area_length; i++)
    //    {
    //        m_Lcross->data[m_Lcross->cols*250/*52*/ + m_Lcross->cols-i-100] = 255;
    //        point.push_back(cv::Point(m_Lcross->cols-i-100, 250/*52*/));
    //    }

    //    for(i = 50; i<Goal_area_length; i++)
    //    {
    //        m_Lcross->data[m_Lcross->cols*m_Lcross->rows - m_Lcross->cols*250/*52*/ + m_Lcross->cols - i-100] = 255;
    //        point.push_back(cv::Point(m_Lcross->cols - i-100, m_Lcross->rows - 250/*52*/));
    //    }

    //    for(i = 1; i<50; i++)
    //    {
    //        m_Lcross->data[m_Lcross->cols*250/*52*/ + m_Lcross->cols - 200/*91*/ + m_Lcross->cols*i] = 255;
    //        point.push_back(cv::Point(m_Lcross->cols - 200/*91*/, 250/*52*/ + i));
    //    }

    //    for(i = 250; i<Goal_area_width; i++)
    //    {
    //        m_Lcross->data[m_Lcross->cols*250/*52*/ + m_Lcross->cols - 200/*91*/ + m_Lcross->cols*i] = 255;
    //        point.push_back(cv::Point(m_Lcross->cols - 200/*91*/, 250/*52*/ + i));
    //    }
    /*side lines*/
    for(i = 400; i<700; i++)
    {
        m_Lcross->data[m_Lcross->cols*100 + i] = 255;
        point.push_back(cv::Point(i, 100));
    }

    for(i = 400; i<700; i++)
    {
        m_Lcross->data[m_Lcross->cols*(m_Lcross->rows-100) + i] = 255;
        point.push_back(cv::Point(i,m_Lcross->rows-100 ));
    }

    for(i = 100; i<200; i++)
    {
        m_Lcross->data[m_Lcross->cols*i + 550] = 255;
        point.push_back(cv::Point(550,i));
    }

    for(i = 600; i<700; i++)
    {
        m_Lcross->data[m_Lcross->cols*i + 550] = 255;
        point.push_back(cv::Point(550,i));
    }

    //side 2lines
    //    for(i = 0; i<200; i++)
    //    {
    //        m_Lcross->data[m_Lcross->cols*100 + 100+i] = 255;
    //        point.push_back(cv::Point(100+i,100));
    //    }
    //    for(i = 0; i<200; i++)
    //    {
    //        m_Lcross->data[m_Lcross->cols*700 + 100+i] = 255;
    //        point.push_back(cv::Point(100+i,700));
    //    }
    //    for(i = 0; i<200; i++)
    //    {
    //        m_Lcross->data[m_Lcross->cols*150 + 100+i] = 255;
    //        point.push_back(cv::Point(100+i,150));
    //    }
    //    for(i = 0; i<200; i++)
    //    {
    //        m_Lcross->data[m_Lcross->cols*650 + 100+i] = 255;
    //        point.push_back(cv::Point(100+i,650));
    //    }

    //    for(i = 0; i<200; i++)
    //    {
    //        m_Lcross->data[m_Lcross->cols*100 + 800+i] = 255;
    //        point.push_back(cv::Point(800+i,100));
    //    }
    //    for(i = 0; i<200; i++)
    //    {
    //        m_Lcross->data[m_Lcross->cols*700 + 800+i] = 255;
    //        point.push_back(cv::Point(800+i,700));
    //    }
    //    for(i = 0; i<200; i++)
    //    {
    //        m_Lcross->data[m_Lcross->cols*150 + 800+i] = 255;
    //        point.push_back(cv::Point(800+i,150));
    //    }
    //    for(i = 0; i<200; i++)
    //    {
    //        m_Lcross->data[m_Lcross->cols*650 + 800+i] = 255;
    //        point.push_back(cv::Point(800+i,650));
    //    }
    nCx = m_Lcross->cols/2;
    nCy = m_Lcross->rows/2;
    dR2 = nRadius*nRadius;
    for(int y = nCy - nRadius; y <= nCy + nRadius; y++)
    {
        for(int x = nCx - nRadius; x <= nCx + nRadius; x++)
        {
            int A = (y-nCy)*(y-nCy) + (x-nCx)*(x-nCx);
            if(50 >= abs(A-dR2))
            {
                if(x == nCx)
                    continue;
                m_Lcross->data[y*m_Lcross->cols + x] = 255;
                point.push_back(cv::Point(x, y));
            }
        }
    }

    for(i = 250; i<550; i++)
    {
        m_Lcross->data[m_Lcross->cols*i + 550] = 255;
        point.push_back(cv::Point(550, i));
    }
//    for(i = 0; i<=20; i++)
//    {
//        m_Lcross->data[m_Lcross->cols*400 + 545+i] = 255;
//        point.push_back(cv::Point(540+i, 400));
//    }
    pointAll.push_back(point);
    point.clear();
    likelihood_L = new Likelihood(*m_Lcross, 1, ui.Zrand->value(), 1, 5, 1);
    likelihood_L->_linePlot.clear();
    likelihood_L->_linePlot = pointAll;
    pointAll.clear();

    //    // ----------------------------------- draw field -----------------------------------
    //    QImage qimageField((const unsigned char*)(m_field->data), m_field->cols, m_field->rows, QImage::Format_Indexed8);
    //    item_field = new QGraphicsPixmapItem(QPixmap::fromImage(qimageField));
    //    scene.addItem(item_field);
    //    ui.graphicsView->setScene(&scene);

    // ----------------------------------- draw field -----------------------------------
    QImage qimageField((const unsigned char*)(image.data), image.cols, image.rows,image.cols*3, QImage::Format_RGB888);
    item_field = new QGraphicsPixmapItem(QPixmap::fromImage(qimageField));
    scene.addItem(item_field);
    ui.graphicsView->setScene(&scene);
    ui.graphicsView_2->setScene(&scene_likelihood);

}

void MainWindow::on_pb_likelihood_clicked()
{
    print("on_pb_likelihood_clicked");

    likelihood->Set(ui.Zhit->value(), ui.Zrand->value(), ui.sigmahit->value(), ui.Zmax->value(), 0.01);
    likelihood->likelihoodField();


    likelihood_L->Set(ui.Zhit->value(), ui.Zrand->value(), ui.sigmahit->value(), ui.Zmax->value(), 0.01);
    likelihood_L->likelihoodField();

    Mat A(m_height, m_width, CV_16UC1);
    for(int y = 0; y < m_height; y++)
    {
        for(int x = 0; x < m_width; x++)
        {
            m_field->data[y*m_field->cols + x] = likelihood->mapLikelihood[y][x];
            if(likelihood->mapLikelihood[y][x]!=0)
            {
                double w =  likelihood->mapLikelihood[y][x]*255;
                A.data[A.cols*y+x] = w > 255 ? 255: w;
            }
            else
            {
                A.data[A.cols*y+x] = 0;
            }
            cout<<"likelihood->mapLikelihood[y][x]: "<<likelihood->mapLikelihood[y][x]<<endl;
        }
    }


    // ----------------------------------- penalty circle -----------------------------------

    likelihoodPC->Set(ui.Zhit->value(), ui.Zrand->value(), ui.sigmahit->value(), ui.Zmax->value(), 0.01);
    likelihoodPC->likelihoodField();

    Mat penaltyCircle(m_height, m_width, CV_16UC1);
    for(int y = 0; y < m_height; y++)
    {
        for(int x = 0; x < m_width; x++)
        {
            m_penaltyCircle->data[y*m_penaltyCircle->cols + x] = likelihoodPC->mapLikelihood[y][x];
            if(likelihoodPC->mapLikelihood[y][x]!=0)
            {
                double w =  likelihoodPC->mapLikelihood[y][x]*255;
                penaltyCircle.data[penaltyCircle.cols*y+x] = w > 255 ? 255: w;
            }
            else
            {
                penaltyCircle.data[penaltyCircle.cols*y+x] = 0;
            }
            cout<<"likelihoodPC->mapLikelihood[y][x]: "<<likelihoodPC->mapLikelihood[y][x]<<endl;
        }
    }

    QImage qimageField((const unsigned char*)(A.data), A.cols, A.rows,  A.cols,QImage::Format_Indexed8);
    QGraphicsPixmapItem *likelihoodField = new QGraphicsPixmapItem(QPixmap::fromImage(qimageField));
    scene_likelihood.addItem(likelihoodField);

    //    QImage qimageField((const unsigned char*)(penaltyCircle.data), penaltyCircle.cols, penaltyCircle.rows, QImage::Format_Indexed8);
    //    QGraphicsPixmapItem *likelihoodField = new QGraphicsPixmapItem(QPixmap::fromImage(qimageField));
    //    scene.addItem(likelihoodField);

}

void MainWindow::on_pb_likelihoodSave_clicked()
{
    print("on_pb_likelihoodSave_clicked");
    print("home path: " + QDir::homePath());

    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Open file"), QDir::homePath() + "/catkin_ws/src/localization_v2/data");
    ofstream fout_data(fileName.toLatin1());
    Mat A(m_height, m_width, CV_16UC1);

    if(fout_data.is_open())
    {
        print("success!!!!");
        for(int y = 0; y < m_height; y++)
        {
            for(int x = 0; x < m_width; x++)
            {
                fout_data << likelihood->mapLikelihood[y][x] <<endl;
                double w =  likelihood->mapLikelihood[y][x]*255;
                A.data[A.cols*y+x] = w > 255 ? 255: w;
            }
        }
        fout_data.close();
        cv::imwrite(QDir::homePath().toStdString() + "/catkin_ws/src/localization_v2/data/likelihoodImg.bmp", A);
    }
    else
    {
        print("fail!!!!");
    }
    QString fileName_L = fileName;
    fileName = fileName+"PC";
    fout_data.open(fileName.toLatin1());

    if(fout_data.is_open())
    {
        print("success!!!!");
        for(int y = 0; y < m_height; y++)
        {
            for(int x = 0; x < m_width; x++)
            {
                fout_data << likelihoodPC->mapLikelihood[y][x] <<endl;
                double w =  likelihoodPC->mapLikelihood[y][x]*255;
                A.data[A.cols*y+x] = w > 255 ? 255: w;
            }
        }
        fout_data.close();
        cv::imwrite(QDir::homePath().toStdString() + "/catkin_ws/src/localization_v2/data/likelihoodImg.bmp", A);
    }
    else
    {
        print("fail!!!!");
    }

    //dohyeong edit0303

    fileName = fileName_L+"_L";
    fout_data.open(fileName.toLatin1());

    if(fout_data.is_open())
    {
        print("success!!!!");
        for(int y = 0; y < m_height; y++)
        {
            for(int x = 0; x < m_width; x++)
            {
                fout_data << likelihood_L->mapLikelihood[y][x] <<endl;
                double w =  likelihood_L->mapLikelihood[y][x]*255;
                A.data[A.cols*y+x] = w > 255 ? 255: w;
            }
        }
        fout_data.close();
        cv::imwrite(QDir::homePath().toStdString() + "/catkin_ws/src/localization_v2/data/likelihoodImg.bmp", A);
    }
    else
    {
        print("fail!!!!");
    }
}
void MainWindow::on_pb_likelihoodRead_clicked()
{
    print("on_pb_likelihoodRead_clicked");
    print("home path: " + QDir::homePath());
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Open file"), QDir::homePath() + "/catkin_ws/src/localization_v2/data");
    //std::cout<<fileName<<std::endl;
    ifstream fin_data(fileName.toLatin1());
    Mat A(m_height, m_width, CV_16UC1);
    if(fin_data.is_open())
    {
        print("success!!!!");
        for(int y = 0; y < m_height; y++)
        {
            for(int x = 0; x < m_width; x++)
            {
                string data;
                getline(fin_data,data);
                likelihood->mapLikelihood[y][x] = atof(data.c_str());
                double w =  likelihood->mapLikelihood[y][x]*255;
                A.data[A.cols*y+x] = w > 255 ? 255: w;
            }
        }
        fin_data.close();

        QImage qimageField((const unsigned char*)(A.data), A.cols, A.rows, A.cols,QImage::Format_Indexed8);
        QGraphicsPixmapItem *likelihoodField = new QGraphicsPixmapItem(QPixmap::fromImage(qimageField));
        //        scene.addItem(likelihoodField);
        scene_likelihood.addItem(likelihoodField);
    }
    else
    {
        print("fail!!!!");
    }



    /************************** START 0824 modified by dohyeong ***********************/
    /************************** This part is a likelihood field of full field ***********************/
    //    QString fileName_L = fileName;
    //    //    fileName = fileName+"PC";
    //    fin_data.open(fileName.toLatin1());
    //    cout<<"fileName.toLatin1 about PC ===    "<<fin_data.is_open()<<endl;
    //    if(fin_data.is_open())
    //    {
    //        print("success!!!!PC");
    //        for(int y = 0; y < m_height; y++)
    //        {
    //            for(int x = 0; x < m_width; x++)
    //            {
    //                string data;
    //                getline(fin_data,data);
    //                likelihoodPC->mapLikelihood[y][x] = atof(data.c_str());
    //                double w =  likelihoodPC->mapLikelihood[y][x]*255;
    //                A.data[A.cols*y+x] = w > 255 ? 255: w;
    //            }
    //        }
    //        fin_data.close();

    //        QImage qimageField((const unsigned char*)(A.data), A.cols, A.rows,A.cols, QImage::Format_Indexed8);
    //        QGraphicsPixmapItem *likelihoodField = new QGraphicsPixmapItem(QPixmap::fromImage(qimageField));
    //        //                scene.addItem(likelihoodField);
    //        //        scene_likelihood.addItem(likelihoodField);
    //    }
    //    else
    //    {
    //        print("fail!!!!PC");
    //    }

    //    //dohyeong edit 0303
    //    //    fileName = fileName_L+"_L";
    //    fin_data.open(fileName.toLatin1());
    //    cout<<"fileName.toLatin1 about L ===    "<<fin_data.is_open()<<endl;
    //    if(fin_data.is_open())
    //    {
    //        print("success!!!! L");
    //        for(int y = 0; y < m_height; y++)
    //        {
    //            for(int x = 0; x < m_width; x++)
    //            {
    //                string data;
    //                getline(fin_data,data);
    //                likelihood_L->mapLikelihood[y][x] = atof(data.c_str());
    //                double w =  likelihood_L->mapLikelihood[y][x]*255;
    //                A.data[A.cols*y+x] = w > 255 ? 255: w;
    //            }
    //        }
    //        fin_data.close();

    //        QImage qimageField((const unsigned char*)(A.data), A.cols, A.rows,A.cols, QImage::Format_Indexed8);
    //        QGraphicsPixmapItem *likelihoodField = new QGraphicsPixmapItem(QPixmap::fromImage(qimageField));
    //        //                scene.addItem(likelihoodField);
    //        //        scene_likelihood.addItem(likelihoodField);
    //    }
    //    else
    //    {
    //        print("fail!!!! L");
    //    }
    /************************** END 0824 modified by dohyeong ***********************/
}

void MainWindow::on_pushButton_move_clicked()
{

    RobotPose lastPose(robot_MCL->x,robot_MCL->y,robot_MCL->theta);

    robot_MCL->theta += m_theta;
    print("before: ",robot_MCL->theta);
    robot_MCL->theta = fmod(robot_MCL->theta,360.0);
    print("after: ",robot_MCL->theta);

    robot_MCL->x += m_dist*cos(rad(robot_MCL->theta));
    robot_MCL->y += m_dist*sin(rad(robot_MCL->theta));

    robot_MCL->x = fabs(fmod(robot_MCL->x,m_width));
    robot_MCL->y = fabs(fmod(robot_MCL->y,m_height));

    print("robot_MCL->x: ",robot_MCL->x);
    print("robot_MCL->y: ",robot_MCL->y);

    MCL-> predictionMCL(robot_MCL->x, robot_MCL->y, robot_MCL->theta, lastPose, tempPose);

    vector<MeasurementData> dataAll;
    MeasurementData data;

    data.featureX = m_width/2;
    data.featureY =m_height/2+nRadius;
    data.measurement = sqrt((data.featureX-robot_MCL->x)*(data.featureX-robot_MCL->x) + (data.featureY-robot_MCL->y)*(data.featureY-robot_MCL->y));
    dataAll.push_back(data);

    data.featureX = m_width/2;
    data.featureY =m_height/2-nRadius;
    data.measurement = sqrt((data.featureX-robot_MCL->x)*(data.featureX-robot_MCL->x) + (data.featureY-robot_MCL->y)*(data.featureY-robot_MCL->y));
    dataAll.push_back(data);

    data.featureX = m_width-92;
    data.featureY =m_height-52;
    data.measurement = sqrt((data.featureX-robot_MCL->x)*(data.featureX-robot_MCL->x) + (data.featureY-robot_MCL->y)*(data.featureY-robot_MCL->y));
    dataAll.push_back(data);

    MCL-> measurementMCL(dataAll,m_noise);
    //    MCL-> resamplingMCL(tempPose);
    MCL-> resamplingMCL();


    updateField();
}

void MainWindow::on_pb_run_clicked()
{
    m_timer = new QTimer(this);
    connect(m_timer,SIGNAL(timeout()),this, SLOT(callback()));
    m_timer->start(100);


}
void MainWindow::on_pb_stop_clicked()
{
    disconnect(m_timer,SIGNAL(timeout()),this, SLOT(callback()));
}

void MainWindow::callback()
{
    on_pushButton_move_clicked();
}

void MainWindow::on_horizontalSlider_dist_valueChanged(int value)
{
    m_dist = value;
    ui.lb_dist->setText(QString::number(m_dist));
}

void MainWindow::on_horizontalSlider_theta_valueChanged(int value)
{
    m_theta = value;
    ui.label_theta->setText(QString::number(m_theta));
}
void MainWindow::on_horizontalSlider_noise_valueChanged(int value)
{
    m_noise = value;
    ui.label_noise->setText(QString::number(m_noise));
}

void MainWindow::transform(int &x, int &y)
{
    int temp = x;
    x = y * (-1);
    y = temp * (-1);
}


}  // namespace localization_v2









//void localization_v2::MainWindow::on_pushButton_clicked()
//{
////    msg_generate::localv2_msg xymsg;

////    xymsg.local_x=robot_MCL->x;
////    xymsg.local_y=robot_MCL->y;
////    xymsg.local_yaw=currentYaw;

////    localPub_v2.publish(xymsg);
//    cout<<"ITEM VISUALLPoint size ==  "<<item_visualPoint.size()<<endl;
//    for(int i = 0; i < item_visualPoint.size(); i++)
//        scene.removeItem(item_visualPoint[i]);
//}
