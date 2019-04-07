#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <riki_msgs/DeviceStatus.h>
#include <riki_msgs/Battery.h>
#include <riki_msgs/Velocities.h>
#include <std_msgs/Bool.h>
#include "../../robot_api/include/api.h"

riki_msgs::DeviceStatus msg;
int getNavLevel(ros::NodeHandle nh);
void batteryCb(const riki_msgs::Battery::ConstPtr& msg);
void velCb(const riki_msgs::Velocities::ConstPtr& msg);
void stopCb(const std_msgs::Bool::ConstPtr& msg);

int main(int argc, char** argv)
{
    cout<<"Content-type:text/html\r\n\r\n";
    setupLog("./config/setting.ini", "/devStatus.conf");
    log4cpp::Category& root = log4cpp::Category::getRoot();

    ros::init(argc,argv,"deviceStatus");
    ros::NodeHandle nh;
    root.info("subscribe 3 topics");
    //电池话题和当前速度话题
    ros::Subscriber batSub = nh.subscribe("BMS", 1000, batteryCb);
    ros::Subscriber velSub = nh.subscribe("raw_vel", 1000, velCb);
    ros::Subscriber stopSub = nh.subscribe("Stop", 1000, stopCb);
    // 发布DeviceStatus
    ros::Publisher pub = nh.advertise<riki_msgs::DeviceStatus>("deviceStatus", 1000);
    ros::Rate loop_rate(1);

    // 导航速度等级
    int navLevel = getNavLevel(nh);
    if(navLevel==-1)
        root.warn("vel level is wrong in setting.ini !");
    ROS_INFO("nav level: %f",navLevel);
    msg.NavSpeedLevel = navLevel;
    while(ros::ok())
    {
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void batteryCb(const riki_msgs::Battery::ConstPtr& bat)
{
    msg.battery = bat->Ah;
    msg.batteryV = bat->vol;
    msg.charger = (bat->cur>0)?1:0 ;  //charge:1   discharge:0
    msg.chargerCurrent = bat->cur;
    msg.chargerV = 0;   // unavailable
}

void velCb(const riki_msgs::Velocities::ConstPtr& vel)
{
    msg.speed = vel->linear_x;
    msg.currentTime = ros::Time::now();
}

void stopCb(const std_msgs::Bool::ConstPtr& status)
{
    msg.Emergency = status->data;
}
int getNavLevel(ros::NodeHandle nh)
{
    float maxVel;
    nh.getParam("/move_base/TrajectoryPlannerROS/max_vel_x",maxVel);
    ROS_INFO("max vel: %f",maxVel);
    float lowVel = getIniValue("./config/setting.ini","info","lowVel").toFloat();
    float midVel = getIniValue("./config/setting.ini","info","midVel").toFloat();
    float highVel = getIniValue("./config/setting.ini","info","highVel").toFloat();
    ROS_INFO("lowVell: %f",lowVel);
    ROS_INFO("midVel: %f",midVel);
    ROS_INFO("highVel: %f",highVel);
//    maxVel = 0.15;
    if(maxVel==lowVel)
        return 0;
    else if(maxVel==midVel)
        return 1;
    else if(maxVel==highVel)
        return 2;
    else return -1;
}