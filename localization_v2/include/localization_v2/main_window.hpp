/**
 * @file /include/localization_v2/main_window.hpp
 *
 * @brief Qt based gui for localization_v2.
 *
 * @date November 2010
 **/
#ifndef localization_v2_MAIN_WINDOW_H
#define localization_v2_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
//#include <QGraphicsSvgItem>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>


#include "kfc.h"
#include "robit_mcl.h"
#include "likelihood.h"
#include "robitkalman.h"
#include "myitem.h"
#include <fstream>

#define GRID_SIZE 20
#define RAD_SCAN 6
#define NMAX_Particle 250
#define NMIN_Particle 20
#define NDEF_Particle 100

#define FIELD_WIDTH 1100
#define FIELD_HEIGHT 800
#define BORDER 100
#define MATWIDTH (FIELD_WIDTH+(2*BORDER))
#define MATHEIGHT (FIELD_HEIGHT+(2*BORDER))
#define CENTERX ((FIELD_WIDTH/2) + BORDER)
#define CENTERY ((FIELD_HEIGHT/2) + BORDER)

#define PENALTY_WIDTH 100
#define PENALTY_HEIGHT 500
#define PENALTY_X1 (-FIELD_WIDTH/2)
#define PENALTY_Y1 (-FIELD_HEIGHT/2+50)
#define PENALTY_X2 (FIELD_WIDTH/2-PENALTY_WIDTH)
#define PENALTY_Y2 (PENALTY_Y1)
#define PENALTY_POINT 210

#define GOAL_WIDTH 60
#define GOAL_HEIGHT 260
#define GOAL_X1 (-FIELD_WIDTH/2 - GOAL_WIDTH)
#define GOAL_Y1 (-FIELD_HEIGHT/2 + 170)
#define GOAL_X2 (FIELD_WIDTH/2)
#define GOAL_Y2 (-FIELD_HEIGHT/2 + 170)

#define LINE_X1 0
#define LINE_X2 0
#define LINE_Y1 (-FIELD_HEIGHT/2)
#define LINE_Y2 (FIELD_HEIGHT/2)

#define LEFTSIDE                        -90
#define RIGHTSIDE                        90

#define GAME_ROUNDROBIN             0
#define GAME_PLAYOFF                1
#define GAME_DROPIN                 2

#define STATE_INITIAL               0
#define STATE_READY                 1
#define STATE_SET                   2
#define STATE_PLAYING               3
#define STATE_FINISHED              4

#define STATE2_NORMAL               0
#define STATE2_PENALTYSHOOT         1
#define STATE2_OVERTIME             2
#define STATE2_TIMEOUT              3
#define STATE2_DIRECT_FREEKICK      4
#define STATE2_INDIRECT_FREEKICK    5
#define STATE2_PENALTYKICK          6



/*****************************************************************************
** Namespace
*****************************************************************************/

namespace localization_v2 {

using namespace cv;
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

    inline void print(const int str){ui.textEdit->append(QString::number(str));}
    inline void print(const char str[]){ui.textEdit->append(QString(str));}
    inline void print(const string str){ui.textEdit->append(QString(str.c_str()));}
    inline void print(const QString str){ui.textEdit->append(str);}

    inline void print(const char str[], double num){ui.textEdit->append(QString(str) + QString::number(num));}
    inline void print(const string str, double num){ui.textEdit->append(QString(str.c_str()) + QString::number(num));}
    inline void print(const QString str, double num){ui.textEdit->append(str + QString::number(num));}
    bool only_one_line(vector<Point2d> point);
    bool only_one_line_v2(vector<Point2d> point);
    int raw_yaw;
    int GOALPOST = 0;
    //QGraphicsItemGroup a;
protected:
    void mousePressEvent(QMouseEvent *event);


public Q_SLOTS:
    void on_pushButton_move_clicked();
    void on_horizontalSlider_dist_valueChanged(int value);
    void on_horizontalSlider_theta_valueChanged(int value);
    void on_horizontalSlider_noise_valueChanged(int value);
    void on_pb_likelihood_clicked();
    void on_pb_likelihoodSave_clicked();
    void on_pb_likelihoodRead_clicked();
    void on_pb_init_clicked();
    void on_pb_run_clicked();
    void on_pb_stop_clicked();
    void on_pb_reset_clicked();
    void draw_line_average();
    void callback();
    void timerCallback();

    void updateYaw();
    void updateOdometry();
    void visionCallback();
    void gameDataUpdate();
    void updateUdp1();
    void updateUdp2();
    void updateUdp3();
    void updateUdp4();
//    void on_pushButton_clicked();

private:
    void drawField();
    void updateField();
    void transform(int &x, int &y);



private:
    Ui::MainWindowDesign ui;
    QNode qnode;

    QGraphicsScene scene;
    QGraphicsScene scene_likelihood;
    QGraphicsPixmapItem *item_field;
    MyItem *item_robot;
    MyItem* robot1;
    MyItem* robot2;
    MyItem* robot3;
    MyItem* robot4;
    vector<QGraphicsEllipseItem*> item_particle;
    vector<QGraphicsEllipseItem*> item_visualPoint;
    vector<RobotPose> tempPose;
    //    vector<QGraphicsLineItem*> item_visualLine;

    robit_mcl *MCL;
    Likelihood *likelihood;
    Likelihood *likelihoodPC;
    Likelihood *likelihood_L;
    Mat image;
    Mat *m_field;
    Mat *m_penaltyCircle;
    Mat *m_Lcross;
    RobotPose *robot_MCL;
    RobotPose *y_robot_MCL;
    RobotPose *robot_kalman;
    RobotPose *y_robot_kalman;

    int datumYaw;
    int currentYaw = 0;
    int stateBefore;

    // field data
    int m_width;
    int m_height;
    int nRadius;


    int m_theta;
    double m_dist;
    int m_noise;

    QTimer *m_timer;
    QTimer *m_updateTimer;


    bool isSet = false;
    bool isYaw;
    bool isvision = false;

    double dX=0;
    double dY=0;

    int m_Queue_cnt = 0;
};

}  // namespace localization_v2

#endif // localization_v2_MAIN_WINDOW_H
