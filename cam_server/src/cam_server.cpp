#include <ros/ros.h>
#include <riki_msgs/ctrl.h>

bool control(riki_msgs::ctrl::Request &req, riki_msgs::ctrl::Response &res)
{
    if(req.cmd==0)
    {
        ROS_INFO("shutting down camera");
        system("rosnode kill /usb_cam");
        ROS_INFO("close camera done");
    }
    else
    {
        ROS_INFO("starting camera");
        system("roslaunch usb_cam usb_cam.launch & ");
        ROS_INFO("camera is up");
    }
}

int main(int argc, char** argv)
{ 
    ros::init(argc,argv,"camServer");
    ros::NodeHandle nh;
    ros::ServiceServer server = nh.advertiseService("control_cam",control);
    ROS_INFO("------ waiting for client's request ------");
    ros::spin();
    return 0;
}
