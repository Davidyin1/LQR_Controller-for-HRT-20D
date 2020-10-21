

#ifndef SEND_CIRCLE_SEND_8_H
#define SEND_CIRCLE_SEND_8_H

#define PI 3.14159265

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include "autoware_msgs/lane.h"

#include <cmath>

class send_8{
public:
    send_8(ros::NodeHandle &nh);
    ~send_8();
    void Init_Param();
    void Publish();
    void callbackfromecu(const std_msgs::Float64MultiArray &msg);
    bool Resgo();

private:
    bool Res_flag;
    ros::Publisher pub;
    ros::Subscriber sub;
    double line;
    double r;
    double pointnum1;
    double pointnum2;
    autoware_msgs::waypoint waypoint;
    autoware_msgs::lane info;

};


#endif //SEND_CIRCLE_SEND_8_H
