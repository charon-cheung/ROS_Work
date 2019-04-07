
#include <ros/ros.h>
#include <riki_msgs/DriverTest.h>
#include <riki_msgs/Battery.h>
//引入日志库log4cxx的头文件
#include <log4cxx/logger.h>
#include <log4cxx/propertyconfigurator.h>
#include<log4cxx/basicconfigurator.h>
#include<log4cxx/helpers/exception.h>
using namespace log4cxx;
using namespace log4cxx::helpers;

void Callback1(const riki_msgs::DriverTest& vel)
{
    ROS_INFO("motor_1 request_rpm: %d",vel.req_rpm);
    ROS_INFO("motor_1 current_rpm: %d",vel.cur_rpm);
    ROS_INFO("motor_1 error: %x",vel.error);
}
void Callback2(const riki_msgs::DriverTest& vel)
{
    ROS_INFO("motor_2 request_rpm: %d",vel.req_rpm);
    ROS_INFO("motor_2 current_rpm: %d",vel.cur_rpm);
    ROS_INFO("motor_2 error: %x",vel.error);
}
void Callback3(const riki_msgs::DriverTest& vel)
{
    ROS_INFO("motor_3 request_rpm: %d",vel.req_rpm);
    ROS_INFO("motor_3 current_rpm: %d",vel.cur_rpm);
    ROS_INFO("motor_3 error: %x",vel.error);
}
void Callback4(const riki_msgs::DriverTest& vel)
{
    ROS_INFO("motor_4 request_rpm: %d",vel.req_rpm);
    ROS_INFO("motor_4 current_rpm: %d",vel.cur_rpm);
    ROS_INFO("motor_4 error: %x",vel.error);
}
void BMS_Callback(const riki_msgs::Battery& vel)
{
    ROS_INFO("battery volt: %4.2f V",vel.vol);
    ROS_INFO("battery current: %4.3f A",vel.cur);
    ROS_INFO("battery remain_power: %5.3f Ah",vel.Ah);
    ROS_INFO("battery remain_percent: %d",vel.Rsoc);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "log_test");
    //设置日志的运行级别
    //log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
    //刷新一下，如果在修改日志运行级别之前没有生成日志也可以不调用，例如此处可以不调用
    //ros::console::notifyLoggerLevelsChanged();
#if 0
log4cxx::LoggerPtr logger(log4cxx::Logger::getLogger("lib"));
LOG4CXX_INFO(logger, "this is log4cxx info");
#endif
    ros::NodeHandle nh;
    printf("日志格式: 时间－驱动器序号－参数值");
    printf("参数有三个，request rpm是发送的每个车轮转速，current rpm是当前车轮的实际转速，error是驱动器的错误状态码");
    ros::Subscriber sub1 = nh.subscribe("DriverTest1",50,Callback1);
    ros::Subscriber sub2 = nh.subscribe("DriverTest2",50,Callback2);
    ros::Subscriber sub3 = nh.subscribe("DriverTest3",50,Callback3);
    ros::Subscriber sub4 = nh.subscribe("DriverTest4",50,Callback4);
    ros::Subscriber BMS_sub = nh.subscribe("BMS",50,BMS_Callback);
    ros::spin();
}

