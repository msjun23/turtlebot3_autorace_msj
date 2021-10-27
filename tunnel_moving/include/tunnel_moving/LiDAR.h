
#pragma once
// #ifndef LiDAR
// #define LiDAR

#include "paramiters.h"


using namespace std;
using namespace cv;

namespace chan {
	double cos_2nd_law(double a, double b, double cos_) {
		if (a == 0 || b == 0)
			return 0;
		return sqrt(a * a + b * b - 2 * a * b * cos_);
	}

	// �ͳ� �ⱸ ��ġ�� �˷��ִ� class
	// -180 ~ 180
	class global_path {
		double goal_x, x, goal_y, y, len_parameter; // map size (%)
	public:
		global_path(Size map_size);
		void move(double x, double y);
		void move_reset();
		double toward(double angle);
	};

	global_path::global_path(Size map_size) {
		this->goal_x = map_size.width;
		this->goal_y = map_size.height;

		//assert(x > 0 && y > 0);

		// scaling
		this->len_parameter = max(x, y) / 100;
		x /= len_parameter;
		y /= len_parameter;
	}

	void global_path::move_reset() {
		x = 0, y = 0;
	}

	void global_path::move(double _x, double _y) {
		this->x += _x / len_parameter;
		this->y += _y / len_parameter;
	}

	// input : radian angle
	// output : degree
	double global_path::toward(double robot_angle) {
		double toward = atan2(goal_y - this->y, goal_x - this->x) - robot_angle;
		toward *= 180 / PI;

		if (toward > 180) {
			toward -= 180;
		}else if (toward < -180) {
			toward += 180;
		}
		
		double ang_vel = toward; // -180 ~ 180 [deg/sec]
		

		return -1;
	}



	// �� class�� �Ҹ�� �Է¹迭�� �Ҹ��Ŵ
	// �ⱸ�� ã�� ���� ��� ���
	// (0,0)���� ���Ⱑ 1�� �������� �̵�
	// -180 ~ 180
	class darkroom
	{
		// fector
		int size = 0;					// 360�� ��� ������ �� �ִ���. ���ش�
		double da = 0.5;				// LiDAR�� ������ angle����, radian
		double* arr = nullptr;			// LiDAR�� ���� �����ϴ� �迭
										// �ܺ��� �迭�� ���
		double range = 500.0/(3.5*2);	// LiDAR�� �Ÿ� 1�� ȯ�� �Ÿ�
		const int hFoV = 70;


		// control
		//int** map = nullptr;		// ��ġ�� ���� map�� �����ϴ� �迭, �ڽ��� ��ġ�� �߽�
		Mat map;					// 0		: �� ����
									// 1 or 255 : ��

		// const int cmsize = 51;	// const int_map size
		// int center = 25;			// (int_map size-1)/2
		
		const int cmsize = 500;		// const int_map size
		int center = 249;			// (int_map size-2)/2

		int prow, pcol;				// window

		double gain_dis = 0.4;		// gap gain
		double gain_wth = 0.6;		// 1 - dis


	public:
		darkroom();
		darkroom(double* arr, const double size);
		~darkroom();
		void release();
		void calmap();
		void obs(int row, int col);
		pair<double, double> window();
		pair<double, double> print();
		pair<double, double> follow_the_gap();

	private:

	};




	pair<double, double> darkroom::window() {
		double area, deg, deg_sum1 = 0, deg_sum2 = 0;
		int cnt1 = 0, cnt2 = 0;

		for (int i = 0; i < hFoV; i++) {
			if (isinf(arr[i]) || arr[i] == 0) {
				continue;
			}
			deg_sum1 += arr[i] * da * i;
			cnt1++;
		}
		for (int i = 360 - hFoV - 1; i < 360; i++) {
			if (isinf(arr[i]) || arr[i] == 0) {
				continue;
			}
			deg_sum2 += arr[i] * da * ( i - 360 );
			cnt2++;
		}

		deg = (deg_sum1 / cnt1 + deg_sum2 / cnt2 ) / 2;
		area = deg_sum1 + (-deg_sum2);

		double vel_ = area * gain_win_vel,
			vel_angle = deg;
		
		return make_pair(vel_, vel_angle);
	}
	

	pair<double, double> darkroom::follow_the_gap() {
		double cos_ = cos(da);
		double* width = new double[hFoV * 2 + 1]{}; // -70 ~ 70
		double* distance_ = new double[hFoV * 2 + 1]{}; // -70 ~ 70
		double min_w = 100'000'000, max_w = -1;
		double min_d = 100'000'000, max_d = -1;
		int i = 360 - hFoV;
		double  prev = arr[i];
		int cnt = 0;
		while (prev == 0) {
			width[cnt] = 0;
			distance_[cnt] = 0;
			cnt++;
			i++;
			prev = arr[i];
		}

		for (i++; i < 360; i++) {
			if (isinf(arr[i]) || arr[i] == 0) {
				width[cnt] = 0;
				distance_[cnt] = 0;
				cnt++;
			}
			else {
				// ���� �Ұ� ��(0)�� ���� ���� ���� �ش� ������ ���� ������ ���
				width[cnt] = cos_2nd_law(arr[i], prev, cos_);
				distance_[cnt] = (arr[i] + prev) / 2.0;

				min_w = min(min_w, width[cnt]);
				max_w = max(max_w, width[cnt]);
				min_d = min(min_d, distance_[cnt]);
				max_d = max(max_d, distance_[cnt]);

				// cout << "!" << arr[i] << " // " << prev << " // " << width[cnt] << " " << distance_[cnt] << endl;
				prev = arr[i];
				cnt++;
			}
		}
		// cout << min_w << " " << max_w << " " << min_d << " " << max_d << endl;
		for (i = 0; i < hFoV + 1; i++) {
			if (isinf(arr[i]) || arr[i] == 0) {
				width[cnt] = 0;
				distance_[cnt] = 0;
				cnt++;
			}
			else {
				// ���� �Ұ� ��(0)�� ���� ���� ���� �ش� ������ ���� ������ ���
				width[cnt] = cos_2nd_law(arr[i], prev, cos_);
				distance_[cnt] = (arr[i] + prev) / 2.0;

				min_w = min(min_w, width[cnt]);
				max_w = max(max_w, width[cnt]);
				min_d = min(min_d, distance_[cnt]);
				max_d = max(max_d, distance_[cnt]);

				// cout << "!" << arr[i] << " // " << prev << " // " << width[cnt] << " " << distance_[cnt] << endl;
				prev = arr[i];
				cnt++;
			}
		}
		// cout << min_w << " " << max_w << " " << min_d << " " << max_d << endl;
		
		// nomalization
		double range_w = max_w - min_w;
		double range_d = max_d - min_d;
		int gap_index = 0;
		double* gap = new double[hFoV * 2 + 1];

		gap[0] = distance_[gap_index] * gain_dis + width[gap_index] * gain_wth;
		for (i = 1; i < hFoV * 2 + 1; i++) {
			if (distance_[i] == 0 || width[i] == 0)
				continue;
			//cout << i << " : " << distance_[i] << " / " << width[i] << " / " << endl;
			width[i] = (width[i] - min_w) / range_w;
			distance_[i] = (distance_[i] - min_d) / range_d;
			gap[i] = distance_[i] * gain_dis + width[i] * gain_wth;
			if (gap[i] > gap[gap_index]) {
				gap_index = i;
			}
			// cout << i << " : " << distance_[i] << " / " << width[i] << " / " << new_gap << endl;
		}
		int len = 2, flag = -1;
		double threshold = gap[gap_index] * 2/3;
		while (len != hFoV) {
			double tample1 = gap[(gap_index - len + size) % size];
			double tample2 = gap[(gap_index + len) % size];
			if (threshold < tample1) {
				flag = 0;
				break;
			}
			else if(threshold < tample2){
				flag = 1;
				break;
			}
			len++;
		}
		if (len >= hFoV) { // or flag == -1
			len = 0;
		}
		cout << gap[gap_index] << " " << gap[gap_index + (flag ? len : -len)] << " " << gap_index << " " << ( flag? len:-len ) << "��� \n";

		delete[] width, distance_, gap;
		return make_pair( gap[gap_index] * gain_gap_vel, (( ( gap_index + (flag ? len : -len)/2 - hFoV ) * deg_to_rad) - (da / 2)) );
	}


	pair<double, double> darkroom::print() {
		//assert(!map.empty());
		//dilate(this->map, this->map, cv::Mat(), cv::Point(-1, -1), 5);
		//erode(this->map, this->map, cv::Mat(), cv::Point(-1, -1), 5);
		pair<double, double> w = window();
		pair<double, double> g = follow_the_gap();

		double wrow = sin(w.second) * w.first * 100 + center;
		double wcol = cos(w.second) * w.first * 100 + center;

		double grow = sin(g.second) * g.first * 100 + center;
		double gcol = cos(g.second) * g.first * 100 + center;

		int deg_w = (int(w.second * rad_to_deg) + size) % size;
		int deg_g = (int(g.second * rad_to_deg) + size) % size;
		cout << deg_w << " " << deg_g << endl;
		double angle, vel;
		if (arr[deg_w] > arr[deg_g]) {
			angle = w.second;
			vel = w.first;
		}
		else {
			angle = g.second;
			vel = g.first;
		}

		double rdian_hfov = hFoV / 180.0 * PI;
		double row_upper = sin(rdian_hfov) * 100.0 + center;
		double col_upper = cos(rdian_hfov) * 100.0 + center;

		double row_lower = sin(-rdian_hfov) * 100.0 + center;
		double col_lower = cos(-rdian_hfov) * 100.0 + center;

		double row = sin(angle) * vel * 100 + center;
		double col = cos(angle) * vel * 100 + center;

		/*cv::line(map, Point(center, center), Point(center + 100, center), Scalar(0, 100, 0), 1);
		cv::line(map, Point(center, center), Point((int)col_lower, (int)row_lower), Scalar(0, 100, 0), 1);
		cv::line(map, Point(center, center), Point((int)col_upper, (int)row_upper), Scalar(0, 100, 0), 1);

		cv::line(map, Point(center, center), Point((int)gcol, (int)grow), Scalar(0, 0, 255), 1);	// gap ����
		cv::line(map, Point(center, center), Point((int)wcol, (int)wrow), Scalar(255, 0, 0), 1);	// window �Ķ�
		cv::line(map, Point(center, center), Point((int)col, (int)row), Scalar(255, 0, 255), 2);
		*/
		// imshow("LiDAR Map", this->map);

		return make_pair(vel, angle * angle_to_vel);
		/*
		assert(map != nullptr);
		for (int i = cmsize - 1; i > 0; i--) {
			for (int j = 0; j < cmsize; j++) {
				if (i == center && j == center) {
					cout << RED "��" NC;
					continue;
				}
				// 0 : �� ����
				// 1 : ��
				if (map[i][j]) {
					cout << char(128) << ' ';
					// cout << "��";
				}
				else {
					cout << "  ";
					// cout << "��";
				}
			}cout << '\n';
		}
		*/
	}


	// arr�� ����� map�� ���
	void darkroom::calmap() {
		if (!map.empty()) {
			map.release();
			
			// map.deallocate();

			/*for (int i = 0; i < cmsize; i++) {
				for (int j = 0; j < cmsize; j++) {
					map.at<char>(i, j) = 0;
				}
			}*/
		}
		map = Mat(this->cmsize, this->cmsize, CV_8UC3, Scalar::all(0));
		prow = -1, pcol = -1;

		// LiDAR�� ���� �̿��� ��ֹ��� ��ġ
		for (int i = 0; i < size; i++) {
			if (isinf(arr[i]) || arr[i] == 0) {
				// continue;
			}
			double row = arr[i] * sin(da * i) * range + center;
			double col = arr[i] * cos(da * i) * range + center;
			obs(row, col);
		}
		
		for (int i = 0; true; i++){
			if (!isinf(arr[i]) && arr[i] != 0) {
				double row = arr[i] * sin(da * i) * range + center;
				double col = arr[i] * cos(da * i) * range + center;
				obs(row, col);
				break;
			}
		}
	}

	void darkroom::obs(int row, int col) {
		if (prow < cmsize && pcol < cmsize && prow >= 0 && pcol >= 0){
			cv::line(map, Point(pcol, prow), Point((int)col, (int)row), Scalar(255, 255, 255), 1);
			map.at<Vec3b>((int)prow, (int)pcol) = Vec3b(0, 0, 255);
		}

		if (row < cmsize && col < cmsize && row >= 0 && col >= 0){
			map.at<Vec3b>((int)row, (int)col) = Vec3b(0, 0, 255);
			pcol = col, prow = row;
		}
		
		/*map.at<char>((int)row + 1, (int)col) = 255;
		map.at<char>((int)row - 1, (int)col) = 255;
		map.at<char>((int)row, (int)col + 1) = 255;
		map.at<char>((int)row, (int)col - 1) = 255;

		map.at<char>((int)row + 1, (int)col + 1) = 255;
		map.at<char>((int)row + 1, (int)col - 1) = 255;
		map.at<char>((int)row - 1, (int)col + 1) = 255;
		map.at<char>((int)row - 1, (int)col - 1) = 255;*/
	}


	void darkroom::release() {
		/*if (arr != nullptr) {
			delete[] arr;
		}
		if (map != nullptr) {
			for (int i = 0; i < cmsize; i++) {
				delete[] map[i];
			}; delete[] map;
		}*/
	}

	darkroom::darkroom(double* arr, const double size) {
		assert(arr != nullptr);
		this->arr = arr;
		this->size = size;
		this->da = 2.0 * PI / size;

		/*int** mat = new int* [cmsize];
		for (int i = 0; i < cmsize; i++) {
			mat[i] = new int[cmsize] {};
		}
		this->map = mat;*/
	}

	darkroom::~darkroom() {
		release();
	}
}

/*
int main()
{
	ios_base::sync_with_stdio(false);	cin.tie(nullptr); cout.tie(nullptr);
	FILE* dummy;
	dummy = freopen("C:\\input_LiDAR.txt", "r", stdin);

	std::chrono::system_clock::time_point Start = std::chrono::system_clock::now();

	const int size = 1440;
	double* arr = new double[size] {};
	for (int i = 0; i < size; i++) {
		string str;
		cin >> str;
		double num = stod(str);
		arr[i] = num;
		// cout << num << endl;
	}

	fclose(dummy);

	chan::darkroom room(arr, size);
	// room.print();

	std::chrono::system_clock::time_point End = std::chrono::system_clock::now();
	std::chrono::milliseconds mill = std::chrono::duration_cast<std::chrono::milliseconds>(End - Start);
	std::cout << "Test() �Լ��� �����ϴ� �ɸ� �ð�(��) : " << mill.count() << "[ms]" << std::endl;
}
*/

// #endif 