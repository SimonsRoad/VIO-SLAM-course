
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <iomanip>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <eigen3/Eigen/Dense>
#include "System.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const int nDelayTimes = 0;
string sConfig_path = "../config/";
string sData_path = "/home/ubuntu/coding/VIO-SLAM-course/projects/07_vins/vins_sys_code/data/";
string feature_file = "/keyframe/"; //文件夹下有每一帧cam提取的pose
string imu_data = "imu_pose.txt";   // imu_raw data
// string imu_data = "imu_pose_noise.txt";

std::shared_ptr<System> pSystem;

//将IMU数据载入system
void PubImuData()
{
	string sImu_data_file = sData_path + imu_data;
	cout << "1 PubImuData start sImu_data_filea: " << sImu_data_file << endl;
	ifstream fsImu;
	fsImu.open(sImu_data_file.c_str());
	if (!fsImu.is_open())
	{
		cerr << "Failed to open imu file! " << sImu_data_file << endl;
		return;
	}

	std::string sImu_line;
	double dStampNSec = 0.0;
	Vector4d q;
	Vector3d t;
	Vector3d vAcc;
	Vector3d vGyr;
	while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data　every line
	{
		std::istringstream ssImuData(sImu_line);
		ssImuData >> dStampNSec >> q.w() >> q.x() >> q.y() >> q.z() >> t.x() >> t.y() >> t.z() >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
		// cout << "Imu t: " << fixed << dStampNSec << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << endl;
		pSystem->PubImuData(dStampNSec, vGyr, vAcc); //带时间戳的IMU数据载入系统，
		usleep(5000 * nDelayTimes);
	}
	fsImu.close();
}

//将image载入system
void PubImageData()
{
	string sImage_file = sData_path + "feature_files_list.txt";

	cout << "1 PubImageData start sImage_file: " << sImage_file << endl;

	ifstream fsImage;
	fsImage.open(sImage_file.c_str()); //所有图像特征的文件名列表　600个
	if (!fsImage.is_open())
	{
		cerr << "Failed to open image file! " << sImage_file << endl;
		return;
	}

	std::string sImage_line;
	double dStampNSec;
	string sImgFileName;

	while (std::getline(fsImage, sImage_line) && !sImage_line.empty())// 读取每一行，每一行都是一副图片所提的所有特征点
		{
			std::istringstream ssImuData(sImage_line);
			ssImuData >> dStampNSec >> sImgFileName;
			// cout << "Image t : " << fixed << dStampNSec << " Name: " << sImgFileName << endl;

			string imagePath = sData_path + sImgFileName;
			//读取每个camera提取的特征点

			ifstream featuresImage;
			featuresImage.open(imagePath.c_str());
			if (!featuresImage.is_open())
			{
				cerr << "Failed to open features file! " << imagePath << endl;
				return;
			}
			std::string featuresImage_line;
			std::vector<int> feature_id;
			int ids = 0;
			std::vector<Vector3d> landmark;
			std::vector<Vector2d> featurePoint;
			std::vector<Vector2d> featureVelocity;
			static double lastTime;
			static std::vector<Vector2d> lastfeaturePoint(50);
			while (std::getline(featuresImage, featuresImage_line) && !featuresImage_line.empty()) // 读取一副图像的所有体征的，每一行就是一个特征点
			{
				Vector3d current_landmark;
				Vector2d current_featurePoint;
				Vector2d current_featureVelocity;

				double temp;
				std::istringstream ssfeatureData(featuresImage_line);
				ssfeatureData >> current_landmark.x() >> current_landmark.y() >> current_landmark.z() >> temp >> current_featurePoint.x() >> current_featurePoint.y();
				landmark.push_back(current_landmark);
				featurePoint.push_back(current_featurePoint);
				feature_id.push_back(ids);
				current_featureVelocity.x() =  (current_featurePoint.x() - lastfeaturePoint[ids].x())/(dStampNSec-lastTime);
				current_featureVelocity.y() =  (current_featurePoint.y() - lastfeaturePoint[ids].y())/(dStampNSec-lastTime);
				featureVelocity.push_back(current_featureVelocity);

				ids++;
			}
			featuresImage.close();
			lastTime = dStampNSec;
			lastfeaturePoint = featurePoint;
			pSystem->PubFeatureData(dStampNSec, feature_id, featurePoint, landmark, featureVelocity); //带时间戳的feature point数据载入系统，
			usleep(50000 * nDelayTimes);
		}
}

int main(int argc, char **argv)
{
	// if (argc != 3)
	// {
	// 	cerr << "./run_euroc PATH_TO_FOLDER/MH-05/mav0 PATH_TO_CONFIG/config \n"
	// 		 << "For example: ./run_euroc /home/ubuntu/dataset/EuRoc/MH-05/ ../config/" << endl;
	// 	return -1;
	// }
	// sData_path = argv[1];
	// sConfig_path = argv[2];

	pSystem.reset(new System(sConfig_path));

	//启动多线程
	std::thread thd_BackEnd(&System::ProcessBackEnd, pSystem);

	// sleep(5);
	std::thread thd_PubImuData(PubImuData); //imu数据的预处理－＞imu buf

	std::thread thd_PubImageData(PubImageData); //image数据预处理-> image buf

	//std::thread thd_Draw(&System::Draw, pSystem);//轨迹实时可视化的线程

	thd_PubImuData.join();
	thd_PubImageData.join();

	// thd_BackEnd.join();
	// thd_Draw.join();

	cout << "main end... see you ..." << endl;
	return 0;
}
