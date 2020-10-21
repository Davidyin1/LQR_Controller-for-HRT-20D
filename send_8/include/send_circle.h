//
// Created by sheffieldwang on 19-9-16.
//

#ifndef SEND_TRAJECTORY_SEND_CIRCLE_H
#define SEND_TRAJECTORY_SEND_CIRCLE_H

#define PI 3.14159265

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "orca_msgs/lane.h"

#include <cmath>


class send_circle{
public:
    send_circle(ros::NodeHandle &nh);
    ~send_circle();
    void Init_Param();
    void Publish();

private:
    ros::Publisher pub;
    double line;
    double r;
    double pointnum1;
    double pointnum2;
    geometry_msgs::PoseStamped waypoint;
    orca_msgs::lane info;

};

#endif //SEND_TRAJECTORY_SEND_CIRCLE_H
