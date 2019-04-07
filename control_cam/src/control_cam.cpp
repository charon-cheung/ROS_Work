#include <ros/ros.h>
#include <riki_msgs/ctrl.h>
#include <cstdlib>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controlCam");
    if(argc!=2)
    {
        ROS_INFO("client need command");
        return 1;
    }
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<riki_msgs::ctrl>("control_cam");
    riki_msgs::ctrl srv;
    srv.request.cmd = atoll(argv[1]);
    if(client.exists())
    {
        ROS_INFO("service control_cam is up");
        //ROS_INFO("service name:%s",client.getService().c_str());
    }
    else
    {
        ROS_ERROR("service control_cam is not available");
        return 1;
    }
    if(!client.call(srv))
    {
        ROS_INFO("client calling srv !");
        if(srv.request.cmd>0)
            ROS_INFO("Sending command start!");
        else
            ROS_INFO("Sending command stop!");
    }
    else
    {
        ROS_ERROR("client calls srv failed !");
        return 1;
    }
    return 0;
}
