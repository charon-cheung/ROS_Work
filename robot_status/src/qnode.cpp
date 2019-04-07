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
#include "../include/robot_status/qnode.hpp"

namespace robot_status {

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode()
{
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
    //没有roscore时，检查的时间太长
    if ( ! ros::master::check() )
    {
		return false;
	}
    ros::start();
    start();
	return true;
}

void QNode::run() {
    qDebug()<<"thread   start"<<qrand()%100;
    ros::NodeHandle n;
    batSub = n.subscribe("BMS",50,  &QNode::batteryCb, this);
    deviceSub = n.subscribe("deviceStatus",50,  &QNode::deviceCb,this);
    taskSub = n.subscribe("taskStatus",50,  &QNode::taskCb,this);
    healthSub = n.subscribe("healthStatus",50,  &QNode::healthCb,this);
    waveSub = n.subscribe("Wave",50,  &QNode::waveCb, this);
    //waveStateSub = n.subscribe("Wave_State",50,  &QNode::waveStateCb, this);
    liftSub = n.subscribe("LiftStates",50,  &QNode::liftCb, this);
    liftSensorSub = n.subscribe("LiftSensor",50,  &QNode::liftSensorCb, this);
    driverSub = n.subscribe("Driver",50,  &QNode::driverCb, this);
    redSub = n.subscribe("Red",50,  &QNode::redCb, this);
    /*spin()放在其他地方会卡死UI线程,在这里卡住子线程 */
    ros::spin();
    emit exitSpin();
}

void QNode::batteryCb(const riki_msgs::Battery &vel)
{
    emit batSig(QString::number(vel.Rsoc),"battery");
    emit curSig(QString::number(vel.cur),"cur");
}

void QNode::deviceCb(const riki_msgs::DeviceStatus &msg)
{
    Device status;
    status.emergency = msg.Emergency;
    status.charger = msg.charger;
    status.speed = msg.speed;
    emit deviceSig(status);
}

void QNode::healthCb(const riki_msgs::HealthStatus &msg)
{
    Health status;
    status.mcu = msg.MCUConnection;
    status.imu = msg.ImuStatus;
    status.laser = msg.laserConnection;
    status.laserData = msg.laserData;
    status.odom = msg.odomData;
    status.router = msg.routerConnection;
    status.sonic = msg.ultrasonicX;
    emit healthSig(status);
}

void QNode::taskCb(const riki_msgs::NavStatus &msg)
{
    Task status;
    status.statusMsg = QString::fromStdString(msg.statusMsg);
    status.name = QString::fromStdString(msg.statusData.name);
    qDebug()<<"--------:"<<status.statusMsg;
    qDebug()<<"--------:"<<status.name;
    emit taskSig(status);
}

void QNode::waveCb(const riki_msgs::WaveMsg &msg)
{
    Wave status;
    status.s1 = msg.s1;
    status.s2 = msg.s2;
    status.s3 = msg.s3;
    status.s4 = msg.s4;
    status.s5 = msg.s5;
    status.s6 = msg.s6;
    emit waveSig(status);
}

void QNode::waveStateCb(int msg)
{
    emit waveStateSig(QString::number(msg),"waveState");
}

void QNode::liftCb(const riki_msgs::LiftMsg &msg)
{
    emit liftSig(QString::number(msg.error),"lift");
}

void QNode::liftSensorCb(const riki_msgs::LiftSensor &msg)
{
    LiftSensor status;
    status.h2 = msg.H2_contain;
    status.noise = msg.noise;
    status.rh = msg.RH;
    status.temp = msg.temperature;
    emit liftSensorSig(status);
}

void QNode::driverCb(const riki_msgs::Driver &msg)
{
    Driver status;
    status.d1 = msg.d1;
    status.d2 = msg.d2;
    emit driverSig(status);
}

void QNode::redCb(const riki_msgs::RedMsg &msg)
{
    Red status;
    status.leftBehind = msg.left_behind;
    status.leftFront = msg.left_front;
    status.rightBehind = msg.right_behind;
    status.rightFront = msg.right_front;
    emit redSig(status);
}

}  // namespace robot_status
