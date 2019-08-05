//
// Created by hyj on 17-6-22.
//

#include <fstream>
#include <iostream>
#include "../src/imu.h"
#include "../src/utilities.h"

/* vector有两个参数，后面的参数一般是默认的，这里用适合数据格式来对齐方式来初始化容器*/
// 从文件中读取 points
// 存入 vector points
// 将同一行的两个点存入 lines vector
std::vector<std::pair<Eigen::Vector4d, Eigen::Vector4d>>
CreatePointsLines(std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &points)
{
    std::vector<std::pair<Eigen::Vector4d, Eigen::Vector4d>> lines;

    char file_path[] = "house_model/house.txt";
    std::ifstream f;
    f.open(file_path);
    try
    {
        if (f.fail())
        {
            throw file_path;
        }
    }
    catch (char *s)
    {
        std::cout << "open file:" << s << " failed !" << std::endl;
        std::cout << "Please check the file path, or open in the bin dir" << std::endl;
        exit(1);
    }
    while (!f.eof())
    {
        std::string s;
        std::getline(f, s);
        if (!s.empty())
        {
            //每一行齐次坐标 line的两个端点
            std::stringstream ss;
            ss << s;
            double x, y, z;
            ss >> x;
            ss >> y;
            ss >> z;
            Eigen::Vector4d pt0(x, y, z, 1);
            ss >> x;
            ss >> y;
            ss >> z;
            Eigen::Vector4d pt1(x, y, z, 1);

            bool isHistoryPoint = false;
            for (int i = 0; i < points.size(); ++i)
            {
                Eigen::Vector4d pt = points[i];
                if (pt == pt0)
                {
                    isHistoryPoint = true;
                }
            }
            if (!isHistoryPoint)
                points.push_back(pt0); //把house的所有顶点都加进去

            isHistoryPoint = false;
            for (int i = 0; i < points.size(); ++i)
            {
                Eigen::Vector4d pt = points[i];
                if (pt == pt1)
                {
                    isHistoryPoint = true;
                }
            }
            if (!isHistoryPoint)
                points.push_back(pt1);

            // pt0 = Twl * pt0;
            // pt1 = Twl * pt1;
            lines.push_back(std::make_pair(pt0, pt1)); // lines
        }
    }

    f.close();

    // create more 3d points, you can comment this code
    int n = points.size();
    for (int j = 0; j < n; ++j)
    {
        Eigen::Vector4d p = points[j] + Eigen::Vector4d(0.5, 0.5, -0.5, 0);
        points.push_back(p);
    }

    // save points
    std::stringstream filename;
    filename << "all_points.txt";
    save_points(filename.str(), points);
    return lines;
}

int main()
{

    //    Eigen::Quaterniond Qwb;
    //    Qwb.setIdentity();
    //    Eigen::Vector3d omega (0,0,M_PI/10);
    //    double dt_tmp = 0.005;
    //    for (double i = 0; i < 20.; i += dt_tmp) {
    //        Eigen::Quaterniond dq;
    //        Eigen::Vector3d dtheta_half =  omega * dt_tmp /2.0;
    //        dq.w() = 1;
    //        dq.x() = dtheta_half.x();
    //        dq.y() = dtheta_half.y();
    //        dq.z() = dtheta_half.z();
    //
    //        Qwb = Qwb * dq;
    //    }
    //
    //    std::cout << Qwb.coeffs().transpose() <<"\n"<<Qwb.toRotationMatrix() << std::endl;

    // 生成3d points, 房子模型的3D点
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points;
    std::vector<std::pair<Eigen::Vector4d, Eigen::Vector4d>> lines;
    lines = CreatePointsLines(points);
    std::cout << "creat " << points.size() << "points and" << lines.size() << "lines" << std::endl;

    // IMU model
    Param params;
    IMU imuGen(params);

    // create imu data
    // imu pose gyro acc
    std::vector<MotionData> imudata;
    std::vector<MotionData> imudata_noise;
    for (float t = params.t_start; t < params.t_end;)
    {
        MotionData data = imuGen.MotionModel(t); //每一个时间戳t,对应一帧数据包括 设定的位姿以及模拟产生的IMU数据(真值，不带噪声)
        imudata.push_back(data);

        // add imu noise
        MotionData data_noise = data;
        imuGen.addIMUnoise(data_noise); // 给模拟的IMU数据添加噪声 ==> noise and bias
        imudata_noise.push_back(data_noise);

        t += 1.0 / params.imu_frequency;
    }
    imuGen.init_velocity_ = imudata[0].imu_velocity;
    imuGen.init_twb_ = imudata.at(0).twb;
    imuGen.init_Rwb_ = imudata.at(0).Rwb;
    save_Pose("imu_pose.txt", imudata);             // 真值数据
    save_Pose("imu_pose_noise.txt", imudata_noise); // 带噪声数据

    // 利用积分，通过IMU raw data计算位姿
    imuGen.testImu("imu_pose.txt", "imu_int_pose.txt"); // test the imu data, integrate the imu data to generate the imu trajecotry
    imuGen.testImu("imu_pose_noise.txt", "imu_int_pose_noise.txt");

    imuGen.testImu_eulerIntegration("imu_pose.txt", "imu_int_pose_eulerIntegraton.txt");
    imuGen.testImu_midPointIntegration("imu_pose.txt", "imu_int_pose_midPointIntegration.txt");

    std::cout << "gentrate IMU data" << std::endl;

    // cam pose
    std::vector<MotionData> camdata;
    for (float t = params.t_start; t < params.t_end;)
    {

        MotionData imu = imuGen.MotionModel(t); // imu body frame to world frame motion
        MotionData cam;

        cam.timestamp = imu.timestamp;
        cam.Rwb = imu.Rwb * params.R_bc;           // cam frame in world frame
        cam.twb = imu.twb + imu.Rwb * params.t_bc; //  Tcw = Twb * Tbc ,  t = Rwb * tbc + twb

        camdata.push_back(cam);
        t += 1.0 / params.cam_frequency;
    }
    save_Pose("cam_pose.txt", camdata); // timestamp q_wxyz  t_xyz  gyro_xyz acc_xyz
    save_Pose_asTUM("cam_pose_tum.txt", camdata);
    std::cout << "gentrate cam data" << std::endl; //timestamp t_xyz q_xyzw


    // points obs in image
    std::string file_list = "feature_files_list.txt"; //保存这些文件的列表
    std::ofstream save_files_list(file_list, std::fstream::out);
    for (int n = 0; n < camdata.size(); ++n) //每一帧图像基本都能看见所有的点
    {
        MotionData data = camdata[n];
        Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
        Twc.block(0, 0, 3, 3) = data.Rwb;
        Twc.block(0, 3, 3, 1) = data.twb;

        // 遍历所有的特征点，看哪些特征点在视野里
        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points_cam;   // ３维点在当前cam视野里
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> features_cam; // 对应的２维图像坐标
        for (int i = 0; i < points.size(); ++i)
        {
            Eigen::Vector4d pw = points[i];           // 最后一位存着feature id
            pw[3] = 1;                                //改成齐次坐标最后一位
            Eigen::Vector4d pc1 = Twc.inverse() * pw; // T_wc.inverse() * Pw  -- > point in cam frame

            if (pc1(2) < 0)
                continue; // z必须大于０,在摄像机坐标系前方

            Eigen::Vector2d obs(pc1(0) / pc1(2), pc1(1) / pc1(2)); //归一化相机平面坐标
                                                                   //            if( (obs(0)*460 + 255) < params.image_h && ( obs(0) * 460 + 255) > 0 &&
                                                                   //                    (obs(1)*460 + 255) > 0 && ( obs(1)* 460 + 255) < params.image_w )
            {
                points_cam.push_back(points[i]); //真实的３d点　
                features_cam.push_back(obs);     //投影到相机平面下的观测点
            }
        }

        // save points
        std::stringstream filename1;
        filename1 << "keyframe/all_points_" << n << ".txt"; //每一帧图像对应的特征点


        //save_features(filename1.str(), points_cam, features_cam); //
       // save_files_list.open(file_list);
        if (!save_files_list.is_open())
            std::cout << "can not open the save_points file" << std::endl;
            save_files_list << camdata[n].timestamp << "\t \t \t" << filename1.str() << std::endl;
     
        std::cout << "generate cam " << n << "feature points" << std::endl;
    }
       save_files_list.close();

    // lines obs in image　线特征，还没有用到
    for (int n = 0; n < camdata.size(); ++n)
    {
        MotionData data = camdata[n];
        Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
        Twc.block(0, 0, 3, 3) = data.Rwb;
        Twc.block(0, 3, 3, 1) = data.twb;

        // 遍历所有的特征点，看哪些特征点在视野里
        //        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points_cam;    // ３维点在当前cam视野里
        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> features_cam; // 对应的２维图像坐标
        for (int i = 0; i < lines.size(); ++i)
        {
            std::pair<Eigen::Vector4d, Eigen::Vector4d> linept = lines[i];

            Eigen::Vector4d pc1 = Twc.inverse() * linept.first;  // T_wc.inverse() * Pw  -- > point in cam frame
            Eigen::Vector4d pc2 = Twc.inverse() * linept.second; // T_wc.inverse() * Pw  -- > point in cam frame

            if (pc1(2) < 0 || pc2(2) < 0)
                continue; // z必须大于０,在摄像机坐标系前方

            Eigen::Vector4d obs(pc1(0) / pc1(2), pc1(1) / pc1(2),
                                pc2(0) / pc2(2), pc2(1) / pc2(2));
            //if(obs(0) < params.image_h && obs(0) > 0 && obs(1)> 0 && obs(1) < params.image_w)
            {
                features_cam.push_back(obs);
            }
        }

        // save lines
        // std::stringstream filename1;
        // filename1 << "keyframe/all_lines_" << n << ".txt";
        // save_lines(filename1.str(), features_cam);
    }

    return 0;
}
