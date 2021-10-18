#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include <sstream>
#include <iostream>
#include <vector>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
//#include <opencv2/core.hpp>
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/features2d.hpp>
//

#include "tunnel_moving/LiDAR_1017.h"

double arr[360];
const int size = 360; //  1440;
chan::darkroom room(arr, size);
pair<double, double> room_pair;

geometry_msgs::Twist cmd_vel;

void SubLiDAR(const sensor_msgs::LaserScan::ConstPtr& ranges)
{
    for (int j = 1; j < size; j++)
    {
        double temp_doub = ranges->ranges.at(j);
        arr[j] = min(3.5 / 2, temp_doub);
    }

    room.calmap();
    room_pair = room.print();

    cmd_vel.linear.x = room_pair.first;
    cmd_vel.angular.z = room_pair.second;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_tunnel");
    ros::NodeHandle n_pub;
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/scan", 1000, SubLiDAR);

    ros::Publisher pub = n_pub.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        pub.publish(cmd_vel);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Tunnel mode is Cleared");
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    pub.publish(cmd_vel);

    // delete[] arr;
    return 0;
}
