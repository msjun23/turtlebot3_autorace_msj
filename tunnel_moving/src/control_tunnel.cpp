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

#include "tunnel_moving/LiDAR.h"

using namespace chan;

double arr[360];
const int size = 360; //  1440;
darkroom room(arr, size);
pair<double, double> room_pair;

geometry_msgs::Twist cmd_vel;
int tunnel_mission_on = 1;

void SubLiDAR(const sensor_msgs::LaserScan::ConstPtr& ranges) {
    if (tunnel_mission_on == 1) {
		// 보간 수식으로 채우는 방법
        double* tmp = new double[size];
        for (int j = 0; j < size; j++) {
            arr[j] = ranges->ranges.at(j);
            arr[j] = min(3.5 / 2, arr[j]);
        }

		// for (int j = 0; j < size; j++) {
		// 	if (tmp[j] != 0) {
		// 		arr[j] = tmp[j];
		// 		continue;
		// 	}
		// 	int ii = (j + size - 1) % size;	// a
		// 	int jj = (j + 1) % size;		// b

		// 	while (tmp[ii] == 0) {
		// 		ii = (ii + size - 1) % size;
		// 	}
		// 	while (tmp[jj] == 0) {
		// 		jj = (jj + 1) % size;
		// 	}
		// 	// cos 제 1법칙으로 
		// 	double C = ((jj - ii + size) % size) * deg_to_rad;		// c
		// 	double cosC = cos(C);
		// 	double c = cos_2nd_law(tmp[ii], tmp[jj], cosC);
		// 	double cosB = (tmp[ii] - tmp[jj] * cosC)/c;

		// 	// sin's law
		// 	cosC = cos(((j - ii + size) % size) * deg_to_rad);
		// 	double B = acos(cosB);
		// 	double A = PI - (B + acos(cosC));
		// 	double r2 = tmp[ii] / sin(A);
		// 	double b = r2 * sin(B);

		// 	arr[j] = b;
		// }

        ROS_INFO("Sibal");

        //room.calmap();
        room_pair = room.print();

        cmd_vel.linear.x = room_pair.first;
        cmd_vel.angular.z = room_pair.second;

        delete[] tmp;
    }
}

void SubMission(const std_msgs::String::ConstPtr& mission_msg) {
    if (!tunnel_mission_on && mission_msg->data == "tunnel") {
        ROS_INFO("tunnel mode on!!!!!!!!");
        tunnel_mission_on = 1;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "control_tunnel");
    ros::NodeHandle n_pub;
    ros::NodeHandle n_lidar;
    ros::NodeHandle n_mission;
    
    ros::Subscriber sub_lidar = n_lidar.subscribe("/scan", 1000, SubLiDAR);
    //ros::Subscriber sub_mission = n_mission.subscribe("/detect/mission", 1000, SubMission);

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
