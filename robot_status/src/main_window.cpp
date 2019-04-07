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
#include <QTimer>
#include <iostream>
#include "../include/robot_status/main_window.hpp"

namespace robot_status {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)

{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    InitUI();
    hasROS = true;
    QObject::connect(ui.quit_button,SIGNAL(clicked()),this,SLOT(quit()) );
    QObject::connect(&qnode, SIGNAL(exitSpin()),this,SLOT(quit()) );
    QObject::connect(&qnode, SIGNAL(batSig(QString,QString)),this,SLOT(freshUI(QString,QString)) );
    QObject::connect(&qnode, SIGNAL(curSig(QString,QString)),this,SLOT(freshUI(QString,QString)) );
    connect(&qnode, SIGNAL(waveStateSig(QString,QString)), this,SLOT(freshUI(QString,QString)) );
    connect(&qnode, SIGNAL(liftSig(QString,QString)), this,SLOT(freshUI(QString,QString)) );

    qRegisterMetaType<QNode::Device>("QNode::Device");
    qRegisterMetaType<QNode::Health>("QNode::Health");
    qRegisterMetaType<QNode::Task>("QNode::Task");
    qRegisterMetaType<QNode::Task>("QNode::Wave");
    qRegisterMetaType<QNode::Task>("QNode::LiftSensor");
    qRegisterMetaType<QNode::Task>("QNode::Driver");
    qRegisterMetaType<QNode::Task>("QNode::Red");
    connect(&qnode, SIGNAL(deviceSig(QNode::Device)),this,SLOT(freshDevice(QNode::Device)) );
    connect(&qnode, SIGNAL(healthSig(QNode::Health)),this,SLOT(freshHealth(QNode::Health)) );
    connect(&qnode, SIGNAL(taskSig(QNode::Task)),this,SLOT(freshTask(QNode::Task)) );
    connect(&qnode, SIGNAL(waveSig(QNode::Wave)),this,SLOT(freshWave(QNode::Wave)) );
    connect(&qnode, SIGNAL(liftSensorSig(QNode::LiftSensor)), this,SLOT(freshLiftSensor(QNode::LiftSensor)) );
    connect(&qnode, SIGNAL(driverSig(QNode::Driver)), this,SLOT(freshDriver(QNode::Driver)) );
    connect(&qnode, SIGNAL(redSig(QNode::Red)), this,SLOT(freshRed(QNode::Red)) );

    if (!qnode.init())
    {
        showNoMasterMessage();
        hasROS = false;
    }
    timer = new QTimer;
    connect(timer,SIGNAL(timeout()), this, SLOT(checkROS()));
    timer->start(3000);
}

MainWindow::~MainWindow()
{
    ros::shutdown();
    ros::waitForShutdown();
}

void MainWindow::InitUI()
{
    ui.quit_button->setFont(QFont("Inconsolata",22));
    ui.title->setFont(QFont("Inconsolata",26));
    ui.name->setFont(QFont("Inconsolata",22));
    ui.statusMsg->setFont(QFont("Inconsolata",22));
    ui.label_8->setFont(QFont("Inconsolata",16));
    ui.label_9->setFont(QFont("Inconsolata",16));
    ui.label_10->setFont(QFont("Inconsolata",16));
    ui.label_11->setFont(QFont("Inconsolata",16));
    ui.label_12->setFont(QFont("Inconsolata",16));
    ui.label_13->setFont(QFont("Inconsolata",16));
    ui.statusbar->showMessage("ROS通信正常");
    this->setStyleSheet("background-color: rgb(222,222,222);");
    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    this->setWindowState(Qt::WindowMaximized);
}

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("没有网络连接,请检查是否启动roscore !");
    msgBox.exec();
    //QThread::sleep(3);
    close();
}

bool MainWindow::hasRoscore()
{
    return hasROS;
}

void MainWindow::quit()
{
    //qnode.quit();
    ros::shutdown();
    ros::waitForShutdown();
    qApp->quit();
}

void MainWindow::freshUI(QString value,QString type)
{
    QMetaMethod metaMethod = sender()->metaObject()->method(senderSignalIndex());
    qDebug() << metaMethod.name()<<qrand()%100;
    if(type=="battery")
    {
        int Rsoc = value.toInt();
        ui.rsocNum->setText(value+"%");
        if(Rsoc>=78)
            ui.battery->setPixmap(QPixmap(":/images/battery_4.png"));
        else if (Rsoc<78 && Rsoc>=57)
            ui.battery->setPixmap(QPixmap(":/images/battery_3.png"));
        else if (Rsoc<57 && Rsoc>=36)
            ui.battery->setPixmap(QPixmap(":/images/battery_2.png"));
        else if (Rsoc<36 && Rsoc>=15)
            ui.battery->setPixmap(QPixmap(":/images/battery_1.png"));
    }
    else if(type=="cur")
    {
        ui.cur->display(value.toFloat());
    }
    else if(type=="waveState")
    {
        if(value.toInt())
            ui.wave->setPixmap(QPixmap(":/images/wave.png"));
        else
            ui.wave->setPixmap(QPixmap(""));
    }
    else if(type=="lift")
    {
        if(!value.toInt())
            ui.lift->setPixmap(QPixmap(":/images/lift.png"));
        else
            ui.lift->setPixmap(QPixmap(""));
    }
}

void MainWindow::freshDevice(QNode::Device value)
{
    QMessageBox *msg = new QMessageBox(this);
    msg->setAttribute(Qt::WA_DeleteOnClose);
    msg->setWindowTitle("注意");
    if(value.emergency)
    {
        ui.emergency->setPixmap(QPixmap(":/images/emergency.png"));
        //msg->setText("机器人进入急停状态！");
        //msg->show();
        //QTimer::singleShot(3000, msg, SLOT(close()) );
    }
    else
    {
        ui.emergency->setPixmap(QPixmap(""));
        //msg->setText("机器人恢复正常状态！");
        //msg->show();
        //QTimer::singleShot(3000, msg, SLOT(close()) );
    }
    if(value.charger)
        ui.charger->setPixmap(QPixmap(":/images/light-bulb.png"));
    else
        ui.charger->setPixmap(QPixmap(":/images/bulb.png"));
    if(value.speed)
        ui.speed->display(value.speed);
}

void MainWindow::freshHealth(QNode::Health value)
{
    if(value.mcu)
        ui.mcu->setPixmap(QPixmap(":/images/green.png"));
    else
        ui.mcu->setPixmap(QPixmap(":/images/red.png"));
    if(value.imu)
        ui.imu->setPixmap(QPixmap(":/images/green.png"));
    else
        ui.imu->setPixmap(QPixmap(":/images/red.png"));
    if(value.laser)
        ui.laserCon->setPixmap(QPixmap(":/images/green.png"));
    else
        ui.laserCon->setPixmap(QPixmap(":/images/red.png"));
    if(value.laserData)
        ui.laserData->setPixmap(QPixmap(":/images/green.png"));
    else
        ui.laserData->setPixmap(QPixmap(":/images/red.png"));
    if(value.odom)
        ui.odom->setPixmap(QPixmap(":/images/green.png"));
    else
        ui.odom->setPixmap(QPixmap(":/images/red.png"));
    if(value.router)
        ui.router->setPixmap(QPixmap(":/images/green.png"));
    else
        ui.router->setPixmap(QPixmap(":/images/red.png"));
}

void MainWindow::freshTask(QNode::Task value)
{
    ui.statusMsg->setText(value.statusMsg);
    ui.name->setText(value.name);
}

void MainWindow::freshWave(QNode::Wave value)
{
    QString text;
    if(value.s1)
        text = "超声波传感器1正常\n";
    else
        text = "超声波传感器1不正常\n";
    if(value.s2)
        text += "超声波传感器2正常\n";
    else
        text += "超声波传感器2不正常\n";
    if(value.s3)
        text += "超声波传感器3正常\n";
    else
        text += "超声波传感器3不正常\n";
    if(value.s4)
        text += "超声波传感器4正常\n";
    else
        text += "超声波传感器4不正常\n";
    if(value.s5)
        text += "超声波传感器5正常\n";
    else
        text += "超声波传感器5不正常\n";
    if(value.s6)
        text += "超声波传感器6正常\n";
    else
        text += "超声波传感器6不正常\n";

    QMessageBox *msg = new QMessageBox(this);
    msg->setAttribute(Qt::WA_DeleteOnClose);
    msg->setWindowTitle("超声波传感器检查完毕");
    msg->setText(text);
    msg->show();
    QTimer::singleShot(3000, msg, SLOT(close()) );
    disconnect(&qnode, SIGNAL(waveSig(QNode::Wave)),this,SLOT(freshWave(QNode::Wave)) );
}

void MainWindow::freshDriver(QNode::Driver value)
{
    QString text;
    if(value.d1)
        text = "驱动器1正常\n";
    else
        text = "驱动器1不正常\n";
    if(value.d2)
        text += "驱动器2正常\n";
    else
        text += "驱动器2不正常\n";
    QMessageBox *msg = new QMessageBox(this);
    msg->setAttribute(Qt::WA_DeleteOnClose);
    msg->setWindowTitle("电机驱动器检查完毕");
    msg->setText(text);
    msg->show();
    QTimer::singleShot(3000, msg, SLOT(close()) );
    disconnect(&qnode, SIGNAL(driverSig(QNode::Driver)), this,SLOT(freshDriver(QNode::Driver)) );
}

void MainWindow::freshLiftSensor(QNode::LiftSensor value)
{
    QMessageBox *msg = new QMessageBox(this);
    msg->setAttribute(Qt::WA_DeleteOnClose);
    msg->setWindowTitle("升降装置传感器信息");

    QString text = "氢气浓度:"+QString::number(value.h2)+"\n";
    text += "噪声:"+QString::number(value.noise)+"\n";
    text += "湿度:"+QString::number(value.rh)+"\n";
    text += "温度:"+QString::number(value.temp)+"\n";
    msg->setText(text);
    msg->show();
    QTimer::singleShot(3000, msg, SLOT(close()) );
    disconnect(&qnode, SIGNAL(liftSensorSig(QNode::LiftSensor)), this,SLOT(freshLiftSensor(QNode::LiftSensor)) );
}

void MainWindow::freshRed(QNode::Red value)
{

}

void MainWindow::checkROS()
{
    if(!ros::master::check())
    {
        qDebug()<<"no ros";
        ui.statusbar->showMessage("ROS通信失败");
    }
    else
    {
        qDebug()<<"yes ros";
        ui.statusbar->showMessage("ROS通信正常");
    }
}

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "robot_status");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "robot_status");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}

}  // namespace robot_status

