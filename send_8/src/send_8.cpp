#include "send_8.h"
using namespace std;
send_8::send_8(ros::NodeHandle& nh)
{
    Init_Param();
    pub = nh.advertise<autoware_msgs::lane>("/final_waypoints",50);
    sub = nh.subscribe("/ecu_msg",10,&send_8::callbackfromecu,this);
    
}

send_8::~send_8()
{}

void send_8::Init_Param()
{
    line=15;
    r=9.125;
    pointnum1=3;
    pointnum2=36.0;
    Res_flag = false;
}

void send_8::Publish()
{
    info.header.stamp = ros::Time::now();

    double x[500], y[500];
    double theta;
    double length;
    int current_point = 0;

    for (length = 0; length < line; length += line / pointnum1, ++current_point) {
        x[current_point] = 0;
        y[current_point] = length;
    }

    //right circle
    for (theta = 180.0; theta > -180.0; theta -= 360.0 / pointnum2, ++current_point) {
        x[current_point] = r * cos(theta / 180 * PI) + r;
        y[current_point] = r * sin(theta / 180 * PI) + line;
    }
    //right circle
    for (theta = 180.0; theta > -180.0; theta -= 360.0 / pointnum2, ++current_point) {
        x[current_point] = r * cos(theta / 180 * PI) + r;
        y[current_point] = r * sin(theta / 180 * PI) + line;
    }
    //left circle
    for (theta = 0.0; theta < 360.0; theta += 360.0 / pointnum2, ++current_point) {
        x[current_point] = r * cos(theta / 180 * PI) - r;
        y[current_point] = r * sin(theta / 180 * PI) + line;
    }
    //left circle
    for (theta = 0.0; theta < 360.0; theta += 360.0 / pointnum2, ++current_point) {
        x[current_point] = r * cos(theta / 180 * PI) - r;
        y[current_point] = r * sin(theta / 180 * PI) + line;
    }



    //line
    for (length = 0.0; length < line; length += line / pointnum1, ++current_point) {
        x[current_point] = 0;
        y[current_point] = length + line;
    }
    //final
    x[current_point] = 0.0;
    y[current_point] = y[current_point - 1] + line / pointnum1;
    current_point += 1;
    cout<<"The number of the current points is:"<<current_point;
    for(int i = 0;i < current_point;++i)
    {   
        waypoint.pose.pose.position.x = x[i];
        waypoint.pose.pose.position.y = y[i];
        waypoint.pose.pose.position.z = 0;
        info.waypoints.push_back(waypoint);
        if(i%15 == 0&&i!=0)
	{
        pub.publish(info);
        info.waypoints.clear();
        waypoint.pose.pose.position.x = x[i];
        waypoint.pose.pose.position.y = y[i];
        waypoint.pose.pose.position.z = 0;
        info.waypoints.push_back(waypoint);
        //ros::Duration(0.5).sleep();
	}    
    }


}

void send_8::callbackfromecu(const std_msgs::Float64MultiArray &msg){
       if (msg.data[2] == 1.0){
            Res_flag = true;
       }
       else
       {
           Res_flag = false;
       }
       
}

bool send_8::Resgo(){
    return Res_flag;
}

int main(int argc,char** argv)
{
     
    
    ros::init(argc,argv,"send_8");
    ros::NodeHandle nh;
    send_8 s8(nh);
    while (ros::ok()){
        // ROS_INFO("%d",s8.Resgo());
          if (s8.Resgo()){
              s8.Publish();
              break;
           }
            else{
                // ROS_INFO("RES WRONG");
            }
         ros::spinOnce();
    }
    

    return 0;
}