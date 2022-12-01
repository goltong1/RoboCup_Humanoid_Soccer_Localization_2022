#include "../include/localization_v2/myitem.h"


//extern double X;
//extern double Y;
//extern double Yaw;
//extern int IK_flag;
double Yaw_pos = 0.0;

//*****************************//

//extern int time1;        //To distinguish First and Second_half
//extern int startpos;     //To set robot position
//extern int shootinfo;

int Y_time = 0;
double X_coordinate = 0;
double Y_coordinate = 0;



MyItem::MyItem()   //****set start pos****
{
    int StartX = 0;
    int StartY = 0;
    setRotation(0);

    setPos(mapToParent(StartX, StartY));
}

QRectF MyItem::boundingRect() const   //****make rec****
{
    return QRect(-13,-9,26,18);
}

QRectF MyItem::bounding_smallRect() const
{
    return QRect(-2,-15,4,13);
}

void MyItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)    //****paint rec****
{
    QRectF rec = boundingRect();
    QBrush Brush = color1;
    QRectF rec_small = bounding_smallRect();
    QBrush Brush_small = color2;

    painter->fillRect(rec, Brush);
    painter->drawRect(rec);
    painter->fillRect(rec_small, Brush_small);
    painter->drawRect(rec_small);

}

void MyItem::advance(int phase)   //****move rec****
{
//    if(IK_flag == 1)
//    {

//        if(!phase) return;
//        std::cout<<"X :"<<X<<std::endl;
//        double X_pos = 0.0;
//        double Y_pos = 0.0;


//        X_pos+=X;
//        Yaw_pos+=Yaw;

//        setRotation(-90+Yaw_pos);

//        //Limit coordinate
//    //    if(pos().ry()>310 || pos().ry()<-300)
//    //    {
//    //        X = 0;
//    //        Y = 0;
//    //        Yaw = 0;
//    //    }
//    //    else if(pos().rx()>450)
//    //    {

//    //        X = 0;
//    //        Y = 0;
//    //        Yaw = 90;
//    //    }
//    //    else if(pos().rx()<-450)
//    //    {
//    //        std::cout<<"reset"<<std::endl;
//    //        X = 0;
//    //        Y = 0;
//    //        Yaw = -90;
//    //    }

//    //    if(Yaw > 360)
//    //        Yaw = Yaw - 360;
//    //    else if(Yaw < -360)
//    //        Yaw = Yaw + 360;

//    //    if(time1 == 0)
//    //    {
//    //        if(startpos == 0)
//    //        {
//    //             setRotation(Yaw-90);
//    //             //cout << "Yaw : " << Yaw << endl;
//    //             if((Yaw-90)>0 && (Yaw-90)<180 || (Yaw-90)<-180 && (Yaw-90)>-360)
//    //             {
//    //                 shootinfo = 1;
//    //                 //cout << "Don't shoot!!!!!!!" << endl;
//    //             }
//    //             else
//    //                 shootinfo = 0;
//    //        }
//    //        else if(startpos == 1)
//    //        {
//    //            setRotation(Yaw);
//    //            //cout << "Yaw : " << Yaw << endl;
//    //            if(Yaw>0 && Yaw<180 || Yaw<-180 && Yaw>-360)
//    //            {
//    //                shootinfo = 1;
//    //                //cout << "Don't shoot!!!!!!!" << endl;
//    //            }
//    //            else
//    //                shootinfo = 0;
//    //        }

//    //    }
//    //    else if(time1 == 1)
//    //    {
//    //        if(startpos == 0)
//    //        {
//    //             setRotation(Yaw+90);
//    //             //cout << "Yaw : " << Yaw << endl;
//    //             if((Yaw+90)<0 && (Yaw+90)>-180 || (Yaw+90)>180 && (Yaw+90)<360)width
//    //             {
//    //                 shootinfo = 1;
//    //                 //cout << "Don't shoot!!!!!!!";
//    //             }
//    //             else
//    //                 shootinfo = 0;
//    //        }
//    //        else if(startpos == 1)
//    //        {
//    //             setRotation(Yaw);
//    //             //cout << "Yaw : " << Yaw << endl;
//    //             if(Yaw<0 && Yaw>-180 || Yaw>180 && Yaw<360)
//    //             {
//    //                 shootinfo = 1;
//    //                 //cout << "Don't shoot!!!!!!!";
//    //             }
//    //             else
//    //                 shootinfo = 0;
//    //        }
//    //    }
//        setPos(mapToParent(Y_pos,-X_pos));

//        X_coordinate = pos().rx();
//        Y_coordinate = pos().ry();

//        cout << "X : " << X << endl;
//        cout << "Y : " << Y_pos << endl;
//    //    cout << "Yaw : " << Yaw << endl;
//        cout << "X_coordinate: " << pos().rx() <<endl;
//        cout << "Y_coordinate: " << pos().ry() << "\n\n";

//    }


}

MyBall::MyBall()
{
    int StartX = 0;
    int StartY = 0;
    setRotation(0);

    setPos(mapToParent(StartX, StartY));
}

QRectF MyBall::boundingRect() const
{
    return QRect(-10, -10, 20, 20);
}

QPainterPath MyBall::shape() const
{
    QPainterPath path;
    path.addEllipse(this->boundingRect());
    return path;
}

void MyBall::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    QRectF rec = boundingRect();
    QPainterPath p = shape();
    QBrush Brush = QBrush(Qt::black);

    painter->fillPath(p, Brush);
}

