#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

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
int tunnel_mission_on = 0;

void SubLiDAR(const sensor_msgs::LaserScan::ConstPtr& ranges) {
    if (tunnel_mission_on == 1) {
        for (int j = 1; j < size; j++) {
            double temp_doub = ranges->ranges.at(j);
            arr[j] = min(3.5 / 2, temp_doub);
        }

        room.calmap();
        room_pair = room.print();

        cmd_vel.linear.x = room_pair.first;
        cmd_vel.angular.z = room_pair.second;
    }
}

void SubMission(const std_msgs::String::ConstPtr& mission_msg) {
    if (mission_msg->data == "tunnel") {
        tunnel_mission_on = 1;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "control_tunnel");
    ros::NodeHandle n_pub;
    ros::NodeHandle n_lidar;
    ros::NodeHandle n_mission;
    
    ros::Subscriber sub_lidar = n_lidar.subscribe("/scan", 1000, SubLiDAR);
    ros::Subscriber sub_mission = n_mission.subscribe("/detect/mission", 1000, SubMission);

    ros::Publisher pub = n_pub.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        if (tunnel_mission_on == 1) {
            pub.publish(cmd_vel);
        }

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
