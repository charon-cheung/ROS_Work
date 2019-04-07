#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <riki_msgs/AprilTagDetectionArray.h>
#include <unistd.h>
#include <tf/transform_listener.h>
#include <signal.h>

bool P1Reached=false;
bool P2Reached=false;

ros::Publisher *pub_ptr;
int num=0;
float x1;

void mySigintHandler(int sig)
{
    ROS_INFO("exiting from node tagNav");
    ros::shutdown();
    ros::waitForShutdown();
}

void subCallback(const riki_msgs::AprilTagDetectionArrayPtr& msg);
int main(int argc, char** argv)
{
    ros::init(argc,argv,"tagNav",ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);
    ros::Subscriber sub = nh.subscribe("tag_detections", 50, subCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 50);
    pub_ptr = &pub;
    ros::spin();
    return 0;
}

void subCallback(const riki_msgs::AprilTagDetectionArrayPtr& msg)
{
    double roll, pitch, yaw;
    int id=100;
    if(!msg->detections.empty())
        id = (msg->detections)[0].id[0];
    else
        ROS_WARN("No tag detected !");

    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;

    float z1,z2;
    float x;
    tf::Quaternion quat0,quat9;
#if 0
    if(!(msg->detections.empty()) && id==0)
    {
        z1 = (msg->detections)[0].pose.pose.pose.position.z;
        tf::quaternionMsgToTF((msg->detections)[0].pose.pose.pose.orientation, quat0);
        tf::Matrix3x3(quat0).getRPY(roll, pitch, yaw);
        ROS_INFO("tag0  roll:%f",roll);
        ROS_INFO("tag0  pitch:%f",pitch);
        ROS_INFO("tag0  yaw:%f\n",yaw);
    }
    if(!P1Reached && id==0)  //第一阶段
    {
        ROS_INFO("stage   1");
        ROS_INFO("distance to tag 0: %f\n",z1);
        if(pitch<0.08 && pitch>=-0.08 )
        {
            ROS_INFO("angular position 1 done !");
            twist.angular.z =  0;
        }
        else if(pitch<-0.08)  //push J
        {
            ROS_INFO("---adjusting tag0  angle, push L ---");
            twist.angular.z = 0.02;
            pub_ptr->publish(twist);
            //ros::spinOnce();
            return;
        }
        else if(pitch>0.08)   //push L
        {
            ROS_INFO("---adjusting tag0  angle, push J ---");
            twist.angular.z =  -0.02;
            pub_ptr->publish(twist);
            //ros::spinOnce();
            return;
        }
        ROS_INFO("start move to position 1");
        if(z1>1)
        {
            twist.linear.x = 0.04;
            pub_ptr->publish(twist);
            //ros::spinOnce();
        }
        else
        {
            twist.linear.x = 0;
            ROS_INFO("position 1 reached !");
            P1Reached = true;
        }
    }
    if(P1Reached && !P2Reached && id!=9)   //第二阶段
    {
        ROS_INFO("stage   2");
        twist.angular.z =  -0.04;
        pub_ptr->publish(twist);
        //ros::spinOnce();
    }
#endif
    if(id==9 && !msg->detections.empty())  //第三阶段
    {
        ROS_INFO("stage   3");
        P2Reached = true;
        x = (msg->detections)[0].pose.pose.pose.position.x;
        z2 = (msg->detections)[0].pose.pose.pose.position.z;
        num++;
        if(num==1)
            x1 = x;
        ROS_INFO("x1:%f",x1);
        num = 5;
        tf::quaternionMsgToTF((msg->detections)[0].pose.pose.pose.orientation, quat9);
        tf::Matrix3x3(quat9).getRPY(roll, pitch, yaw);

        ROS_INFO("tag9  x:%f",x);
        //ROS_INFO("tag9  roll:%f",roll);
        ROS_INFO("tag9  pitch:%f\n",pitch);
        //ROS_INFO("tag9  yaw:%f\n",yaw);

        if(x<0.02 && x>-0.02)
        {
            ROS_INFO("---Pos 1 Done ---");
        }
        else if(x<-0.02)
        {
            ROS_INFO("---adjusting tag9  angle, push L ---");
            twist.angular.z = -0.03;
            pub_ptr->publish(twist);
            return;
        }
        else if(x>0.02)
        {
            ROS_INFO("---adjusting tag9  angle, push J ---");
            twist.angular.z = 0.03;
            pub_ptr->publish(twist);
            return;
        }

        ROS_INFO("start walk");
        twist.linear.x = 0.08;
        pub_ptr->publish(twist);
        if(x1>0)
        {
            if(x<-x1)
            {
                twist.linear.x = 0;
                twist.angular.z = -0.03;
                pub_ptr->publish(twist);
                if(x>0)
                {
                    twist.linear.x = 0;
                    twist.angular.z = 0;
                    pub_ptr->publish(twist);
                }
            }
        }
        else if(x1<0)
        {
            if(x>-x1)
            {
                twist.linear.x = 0;
                twist.angular.z = 0.03;
                pub_ptr->publish(twist);
                if(x<0)
                {
                    twist.linear.x = 0;
                    twist.angular.z = 0;
                    pub_ptr->publish(twist);
                }
            }
        }
#if 0
        if(pitch<0.08 && pitch>=-0.08 )
        {
            ROS_INFO("angular position 2 done !");
            twist.angular.z =  0;
        }
        else if(pitch<-0.08)  //push J
        {
            ROS_INFO("---adjusting tag9  angle, push L ---");
            twist.angular.z = 0.03;
            pub_ptr->publish(twist);
            return;
        }
        else if(pitch>0.08)   //push L
        {
            ROS_INFO("---adjusting tag9  angle, push J---");
            twist.angular.z =  -0.03;
            pub_ptr->publish(twist);
            //ros::spinOnce();
            return;
        }
        ROS_INFO("starting move to final position");
        ROS_INFO("distance to tag 9: %f\n",z2);
        if(z2>0.5)
        {
            twist.linear.x = 0.08;
            pub_ptr->publish(twist);
        }
        else
        {
            twist.linear.x = 0;
            P1Reached = false;
            P2Reached = false;
            ROS_INFO("Final position reached !");
        }
#endif
    }
}
