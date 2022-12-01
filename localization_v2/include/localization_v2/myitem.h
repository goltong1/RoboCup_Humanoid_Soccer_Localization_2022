#ifndef MYITEM_H
#define MYITEM_H

#include <QPainter>
#include <QGraphicsScene>
#include <QGraphicsItem>

#include <iostream>

using namespace std;

class MyItem : public QGraphicsItem
{
public:
    MyItem();
    QRectF bounding_smallRect() const;
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    void advance(int phase);
    QBrush color1=QBrush(Qt::blue);
    QBrush color2=QBrush(Qt::red);
protected:

private:
    qreal angle;
    qreal speed;
    void DoCollision();
};

class MyBall : public QGraphicsEllipseItem
{
public:
    MyBall();
    QRectF boundingRect() const;
    QPainterPath shape() const;

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
};

#endif // MYITEM_H
