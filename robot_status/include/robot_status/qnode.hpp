/**
 * @file /include/robot_status/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robot_status_QNODE_HPP_
#define robot_status_QNODE_HPP_


#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QDebug>
#include "riki_msgs/Battery.h"
#include <riki_msgs/DeviceStatus.h>
#include <riki_msgs/HealthStatus.h>
#include <riki_msgs/NavStatus.h>
#include <riki_msgs/StatusDataMsg.h>
#include <riki_msgs/Driver.h>
#include <riki_msgs/LiftMsg.h>
#include <riki_msgs/LiftSensor.h>
#include <riki_msgs/WaveMsg.h>
#include <riki_msgs/RedMsg.h>

namespace robot_status {

class QNode : public QThread
{
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();

	bool init();
	void run();
    void batteryCb(const riki_msgs::Battery& vel);
    void deviceCb(const riki_msgs::DeviceStatus& msg);
    void healthCb(const riki_msgs::HealthStatus& msg);
    void taskCb(const riki_msgs::NavStatus& msg);
    void waveCb(const riki_msgs::WaveMsg& msg);
    void waveStateCb(int msg);
    void liftCb(const riki_msgs::LiftMsg& msg);
    void liftSensorCb(const riki_msgs::LiftSensor& msg);
    void driverCb(const riki_msgs::Driver& msg);
    void redCb(const riki_msgs::RedMsg& msg);
public:
    struct Device{
        bool emergency;
        int charger;
        float speed;
    };
    struct Health{
        bool mcu;
        bool imu;
        bool laser;
        bool laserData;
        bool odom;
        bool router;
        int sonic;
    };
    struct Task{
        QString statusMsg;
        QString name;
    };
    struct Wave{
        int w1;
        int w2;
        int w3;
        int w4;
        int w5;
        int w6;
        int s1;
        int s2;
        int s3;
        int s4;
        int s5;
        int s6;
    };
    bool waveState;
    int liftErr;
    struct LiftSensor{
        float temp;
        float rh;
        float noise;
        float h2;
    };
    struct Driver{
        int d1;
        int d2;
    };
    struct Red{
        int leftFront;
        int rightFront;
        int leftBehind;
        int rightBehind;
    };

Q_SIGNALS:
    void rosShutdown();
    void batSig(QString Rsoc,QString type);
    void curSig(QString cur,QString type);
    void deviceSig(QNode::Device value);
    void healthSig(QNode::Health value);
    void taskSig(QNode::Task value);
    void waveSig(QNode::Wave value);
    void waveStateSig(QString state,QString type);
    void liftSig(QString liftErr,QString type);
    void liftSensorSig(QNode::LiftSensor value);
    void driverSig(QNode::Driver value);
    void redSig(QNode::Red value);

    void exitSpin();
private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    ros::Subscriber batSub,deviceSub,taskSub,healthSub,waveSub,waveStateSub,liftSub,liftSensorSub,driverSub,redSub;

};

}  // namespace robot_status

#endif /* robot_status_QNODE_HPP_ */
