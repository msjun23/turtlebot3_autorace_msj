
#pragma once
// #ifndef LiDAR
// #define LiDAR

#define PI 3.14159265


using namespace std;
using namespace cv;

namespace chan {

	// �� class�� �Ҹ�� �Է¹迭�� �Ҹ��Ŵ
	// �ⱸ�� ã�� ���� ��� ���
	// (0,0)���� ���Ⱑ 1�� �������� �̵�
	class darkroom
	{
		// fector
		int size = 0;				// 360�� ��� ������ �� �ִ���. ���ش�
		double da = 0.5;			// LiDAR�� ������ angle����
		double* arr = nullptr;		// LiDAR�� ���� �����ϴ� �迭
									// �ܺ��� �迭�� ���
		double range = 50.0/3.5;	// LiDAR�� �Ÿ� 1�� ȯ�� �Ÿ�

		// control
		//int** map = nullptr;		// ��ġ�� ���� map�� �����ϴ� �迭, �ڽ��� ��ġ�� �߽�
		Mat map;					// 0		: �� ����
									// 1 or 255 : ��

		// const int cmsize = 51;	// const int_map size
		// int center = 25;			// (int_map size-1)/2
		
		const int cmsize = 50;		// const int_map size
		int center = 24;			// (int_map size-2)/2

		int prow, pcol;

	public:
		darkroom();
		darkroom(double* arr, const double size);
		~darkroom();
		void release();
		void calmap();
		pair<double, double> print();
		void obs(int row, int col);
		double window();

	private:

	};


	double darkroom::window() {
		double deg_sum = 0;
		int cnt = 0;

		for (int i = 0; i < 70; i++) {
			if (isinf(arr[i])) {
				continue;
			}
			deg_sum += arr[i] * da * i;
			cnt++;
		}
		for (int i = 360-70-1; i < 360; i++) {
			if (isinf(arr[i])) {
				continue;
			}
			deg_sum += arr[i] * da * (i - 360);
			cnt++;
		}

		deg_sum /= cnt;

		double row = sin(deg_sum) * 10 + center;
		double col = cos(deg_sum) * 10 + center;

		if (row < cmsize && col < cmsize && row >= 0 && col >= 0) {
			cv::line(map, Point(center, center), Point((int)col, (int)row), Scalar(255, 0, 0), 2);
		}

		return deg_sum;
	}


	pair<double, double> darkroom::print() {
		assert(!map.empty());
		//dilate(this->map, this->map, cv::Mat(), cv::Point(-1, -1), 5);
		//erode(this->map, this->map, cv::Mat(), cv::Point(-1, -1), 5);

		double deg = window();
		double vel = 0.1;
		resize(this->map, this->map, Size(500, 500), 0, 0, INTER_LINEAR);
		
		imshow("LiDAR Map", this->map);

		return make_pair(vel, deg);
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
			if (isinf(arr[i])) {
				continue;
			}
			double row = arr[i] * sin(da * i) * range + center;
			double col = arr[i] * cos(da * i) * range + center;
			row, col, obs(row, col);
			//if (row < cmsize - 1 && col < cmsize - 1 && row >= 1 && col >= 0) {
				//map[(int)row][(int)col] = 255; // 1 or 255
				
				//system("cls");
				//darkroom::print();
			//}
			//else {
				//continue;
			//}

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