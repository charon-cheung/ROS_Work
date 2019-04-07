#include <QDebug>
#include "../include/viewer/viewer.h"
#include "rviz/render_panel.h"
#include "rviz/selection/selection_handler.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/view_controller.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/visualization_manager.h"
#include "rviz/load_resource.h"
#include "rviz/properties/bool_property.h"


namespace topic_viewer
{
Viewer::Viewer(QWidget* parent)
    :rviz::Panel(parent)
{
    InitUI();
    batSub = nh.subscribe("BMS",50,  &Viewer::batteryCb,this);
    deviceSub = nh.subscribe("deviceStatus",50,  &Viewer::deviceCb,this);
}
Viewer::~Viewer()
{
}
void Viewer::InitUI()
{
        this->resize(330, 550);
        this->setMaximumSize(QSize(1550, 888));
        this->setStyleSheet("background-color: rgb(114, 159, 207);");
        label = new QLabel(this);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(80, 110, 81, 31));
        vel = new QLCDNumber(this);
        vel->setObjectName(QStringLiteral("vel"));
        vel->setGeometry(QRect(190, 160, 111, 41));
        label_3 = new QLabel(this);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(200, 290, 48, 48));

        label_5 = new QLabel(this);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(80, 300, 81, 31));
        label_6 = new QLabel(this);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(80, 380, 72, 18));
        velLabel = new QLabel(this);
        velLabel->setObjectName(QStringLiteral("velLabel"));
        velLabel->setGeometry(QRect(80, 160, 81, 41));
        velLabel->setText("导航速度");
        label_2 = new QLabel(this);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(350, 0, 32, 32));

        charger = new QLabel(this);
        charger->setObjectName(QStringLiteral("charger"));
        charger->setGeometry(QRect(310, 0, 32, 32));

        emergency = new QLabel(this);
        emergency->setObjectName(QStringLiteral("emergency"));
        emergency->setGeometry(QRect(10, 10, 32, 32));
}

void Viewer::batteryCb(const riki_msgs::Battery& vel)
{
    QString iconPath  = "/home/hlhp/R50_v100/workspace/src/topic_viewer/icon/";
    if(vel.Rsoc>78)
        label_2->setPixmap(QPixmap(iconPath+"battery_4.png"));
    else if (vel.Rsoc<78 && vel.Rsoc>57)
        label_2->setPixmap(QPixmap(iconPath+"battery_3.png"));
    else if (vel.Rsoc<57 && vel.Rsoc>36)
        label_2->setPixmap(QPixmap(iconPath+"battery_2.png"));
    else if (vel.Rsoc<36 && vel.Rsoc>15)
        label_2->setPixmap(QPixmap(iconPath+"battery_1.png"));
}

void Viewer::deviceCb(const riki_msgs::DeviceStatus& msg)
{
    QString iconPath  = "/home/hlhp/R50_v100/workspace/src/topic_viewer/icon/";
    if(msg.Emergency)
        emergency->setPixmap(QPixmap(iconPath+"emergency.png"));
    else
        emergency->setPixmap(QPixmap(QString("")));
    if(msg.charger)
        charger->setPixmap(QPixmap(iconPath+"red.png"));
    else
        charger->setPixmap(QPixmap(iconPath+"green.png"));

    vel->display(msg.speed);

}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(topic_viewer::Viewer,rviz::Panel)