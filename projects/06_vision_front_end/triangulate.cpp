//
// Created by hyj on 18-11-11.
//
#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <Eigen/Dense>

struct Pose
{
    Pose(Eigen::Matrix3d R, Eigen::Vector3d t) : Rwc(R), qwc(R), twc(t){};
    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond qwc;
    Eigen::Vector3d twc;

    Eigen::Vector2d uv; // 这帧图像观测到的特征坐标
};

int main()
{
    int poseNums = 10;
    double radius = 8;
    double fx = 1.;
    double fy = 1.;
    std::vector<Pose> camera_pose;
    for (int n = 0; n < poseNums; ++n)
    {
        double theta = n * 2 * M_PI / (poseNums * 4); // 1/4 圆弧
        // 绕 z轴 旋转
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()); //
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        camera_pose.push_back(Pose(R, t));
    }

    // 随机数生成 1 个 三维特征点
    std::default_random_engine generator;
    std::uniform_real_distribution<double> xy_rand(-4, 4.0);
    std::uniform_real_distribution<double> z_rand(8., 10.);
    double tx = xy_rand(generator);
    double ty = xy_rand(generator);
    double tz = z_rand(generator);

    Eigen::Vector3d Pw(tx, ty, tz);
    // 这个特征从第三帧相机开始被观测，i=3
    int start_frame_id = 3;
    int end_frame_id = poseNums;
    for (int i = start_frame_id; i < end_frame_id; ++i)
    {
        Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
        Eigen::Vector3d Pc = Rcw * (Pw - camera_pose[i].twc); //转换到每一帧图像坐标系下的坐标

        double x = Pc.x();
        double y = Pc.y();
        double z = Pc.z();

        camera_pose[i].uv = Eigen::Vector2d(x / z, y / z); //归一化相机平面坐标
    }

    /// TODO::homework; 请完成三角化估计深度的代码
    // 遍历所有的观测数据，并三角化
    Eigen::Vector3d P_est; // 结果保存到这个变量
    P_est.setZero();

    /* your code begin */
    int common_view_poseNUms = end_frame_id - start_frame_id;
    Eigen::Matrix<double, 14, 4> D_matrix;

    std::vector<Eigen::Matrix<double, 3, 4>> projectionMatrix;
    for (int i = start_frame_id, j = 0; i < end_frame_id; i++)
    {
        //由R t　得到变换矩阵　
        Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
        Twc.block(0, 0, 3, 3) = camera_pose[i].Rwc;
        Twc.block(0, 3, 3, 1) = camera_pose[i].twc;
        Twc.block(3, 0, 1, 3) = Eigen::Matrix<double, 1, 3>::Zero();

        // std::cout << "Rwc: \r\n" << camera_pose[i].Rwc << std::endl;
        // std::cout << "twc: " << camera_pose[i].twc.transpose() << std::endl;
        // std::cout << "Twc: \r\n" << Twc << std::endl;

        Eigen::Matrix4d Tcw = Twc.inverse();
        // std::cout << "Tcw: \r\n" << Tcw << std::endl;

        Eigen::Matrix<double, 3, 4> projection_k;
        projection_k.block(0, 0, 3, 3) = Tcw.block(0, 0, 3, 3);
        projection_k.block(0, 3, 3, 1) = Tcw.block(0, 3, 3, 1);
        // projectionMatrix.push_back(projection_k);


       D_matrix.block(j++, 0, 1, 4) = camera_pose[i].uv.x() * projection_k.block(2, 0, 1, 4) - projection_k.block(0, 0, 1, 4);
       D_matrix.block(j++, 0, 1, 4) = camera_pose[i].uv.y() * projection_k.block(2, 0, 1, 4) - projection_k.block(1, 0, 1, 4);
    }

    
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(D_matrix.transpose()*D_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV );
    std::cout << "svd_singularValues\r\n" << svd.singularValues() << std::endl;

    std::cout << "svd matrixU \r\n"  << svd.matrixU() << std::endl;
    std::cout << "svd matrixV \r\n"  << svd.matrixV() << std::endl;

    std::cout << "output \r\n" << svd.matrixV()*Eigen::Vector4d(0,0,0,1)<<std::endl;

    Eigen::Vector4d temp = svd.matrixV()*Eigen::Vector4d(0,0,0,1);
    temp = temp/temp.w();
    std::cout << "output \r\n" << temp <<std::endl;


    // Eigen::Vector4d rfh(0,0,0,1);
    // std::cout << "the solved: \r\n" << svd.solve(rfh) << std::endl;
    

    /* your code end */

    std::cout << "ground truth: \n"
              << Pw.transpose() << std::endl;
    std::cout << "your result: \n"
              << P_est.transpose() << std::endl;
    return 0;
}
