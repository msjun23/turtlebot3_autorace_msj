#pragma once
// #ifndef paramiters
// #define paramiters

#include <iostream>
#include <string>
#include <iomanip>
#include <cassert>
#include <stdio.h>
#include <chrono>
#include <math.h>
//#include <opencv2\opencv.hpp>

#pragma warning(disable : 4996)

//using namespace cv;

namespace chan {
#define  endl '\n'
#define  NC = "\033[39m";
#define  RED = "\033[31m";
	const double PI = 3.14159265;
	const double deg_to_rad = PI / 180.;
	const double rad_to_deg = 180. / PI;

	const double angle_to_vel = 12;
	const double const_vel = 0.1;
	const double gain_gap_vel = 1;
	const double gain_win_vel = 0.015;
}
// #endif 
