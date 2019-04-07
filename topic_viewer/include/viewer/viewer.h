
#ifndef MY_PLUGIN_H
#define MY_PLUGIN_H

#include <ros/ros.h>
#include <ros/console.h>
#include "riki_msgs/Battery.h"
#include <riki_msgs/DeviceStatus.h>
#include <riki_msgs/HealthStatus.h>
#include <riki_msgs/NavStatus.h>
#include <riki_msgs/StatusDataMsg.h>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <rviz/panel.h>
#include <QComboBox>
#include <QDialog>
#include <QTextEdit>
#include <QGridLayout>
#include <QVBoxLayout>


namespace topic_viewer
{
class Viewer: public rviz::Panel
{
    Q_OBJECT
public:
    Viewer(QWidget* parent=0);
    ~Viewer();
    void InitUI();
    void batteryCb(const riki_msgs::Battery& vel);
    void deviceCb(const riki_msgs::DeviceStatus& msg);

public:
    QLabel *label;
    QLCDNumber *vel;
    QLabel *label_3;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *velLabel;
    QLabel *label_2;
    QLabel *charger;
    QLabel *emergency;
    ros::Subscriber    batSub,deviceSub,taskSub,healthSub;
    ros::NodeHandle nh;
};
}
#endif
