//
// Created by sheffieldwang on 19-9-16.
//

#include "send_circle.h"


send_circle::send_circle(ros::NodeHandle& nh)
{
    Init_Param();
    pub = nh.advertise<orca_msgs::lane>("/lane",5);
    Publish();

}

send_circle::~send_circle()
{}

void send_circle::Init_Param()
{
    line = 9.0;
    r = 9.0;
    //直线点数
    pointnum1 = 10.0;
    //园点数
    pointnum2 = 20.0;
};

void send_circle::Publish()
{
    info.header.stamp = ros::Time::now();
    double x[300],y[300];

    double theta;
    double length;
    int current_point = 0;

    for(length = -3; length <= line; current_point++)
    {
        length=length+line/pointnum1;
        x[current_point]=0;
        y[current_point]=length;

    }

    for(theta = 0; theta <= 180.0; current_point++)
    {
        theta=theta + 180.0/pointnum2;
        x[current_point] = r - r * cos(theta/180.0*PI);
        y[current_point] = line + r * sin(theta/180.0*PI);
        // std::cout<<"x[i] "<<y[current_point]<<std::endl;

    }

    for(length = 0; length < 2*line;current_point++)
    {
        length = length + line/pointnum1;
        x[current_point] = 2 * r;
        y[current_point] = line - length;
    }

    for(theta = 0; theta <= 180; current_point++)
    {
        theta = theta + 180/pointnum2;
        x[current_point] = r + r*cos(theta/180.0*PI);
        y[current_point] = -(line + r*sin(theta/180.0*PI));
    }

    for(length = 0;length < line; current_point++)
    {
        length = length + line/pointnum1;
        x[current_point] = 0;
        y[current_point] = -(line - length);
    }

    for(length = 0;length < line;current_point++)
    {
        length =length + line/pointnum1;
        x[current_point] = 0;
        y[current_point] = length;
    }


    for(int i = 0;i < current_point;i++)
    {   int count = 0;
        waypoint.pose.position.x = x[i];
        waypoint.pose.position.y = y[i];
        waypoint.pose.position.z = 0;
        info.poses.push_back(waypoint);
        if(i%4 == 0)
	{
	 pub.publish(info);
	 info.poses.empty();
         ros::Duration(0.1).sleep();
	}    
    }


}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"send_circle");
    ros::NodeHandle nh;
    send_circle sc(nh);

    return 0;
}
