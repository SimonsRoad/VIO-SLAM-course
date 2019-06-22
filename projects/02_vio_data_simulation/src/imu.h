//
// Created by hyj on 18-1-19.
//

#ifndef IMUSIMWITHPOINTLINE_IMU_H
#define IMUSIMWITHPOINTLINE_IMU_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <vector>

#include "param.h"

struct MotionData
{
    // 这个宏在new一个对象时会总是返回一个对齐的指针
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp;
    Eigen::Matrix3d Rwb;
    Eigen::Vector3d twb;
    Eigen::Vector3d imu_acc;
    Eigen::Vector3d imu_gyro;

    Eigen::Vector3d imu_gyro_bias;
    Eigen::Vector3d imu_acc_bias;

    Eigen::Vector3d imu_velocity;
};

// euler2Rotation:   body frame to interitail frame
Eigen::Matrix3d euler2Rotation( Eigen::Vector3d  eulerAngles);
Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles);


class IMU
{
public:
    IMU(Param p);
    Param param_;  // IMU模型参数设定
    Eigen::Vector3d gyro_bias_;
    Eigen::Vector3d acc_bias_;

    Eigen::Vector3d init_velocity_;
    Eigen::Vector3d init_twb_;
    Eigen::Matrix3d init_Rwb_;

    MotionData MotionModel(double t); // IMU一帧数据

    void addIMUnoise(MotionData& data);
    void testImu(std::string src, std::string dist);        // imu数据进行积分，用来看imu轨迹

    void testImu_eulerIntegration(std::string src, std::string dist);           // imu数据进行欧拉法积分，用来看imu轨迹
    void testImu_midPointIntegration(std::string src, std::string dist);        // imu数据进行中值法积分，用来看imu轨迹

    void eulerIntegration(double _dt, 
                      const Eigen::Vector3d &acc_k,    const Eigen::Vector3d &gyro_k,     // 第k帧 IMU data
                      const Eigen::Vector3d &acc_bias, const Eigen::Vector3d &gyro_bias,  // IMU 偏置项，这里假定为常数
                      Eigen::Vector3d &delta_p, Eigen::Quaterniond &delta_q, Eigen::Vector3d &delta_v //当前帧积分result
                      );

    void midPointIntegration(double _dt, 
                            const Eigen::Vector3d &acc_0,    const Eigen::Vector3d &gyro_0,
                            const Eigen::Vector3d &acc_1,    const Eigen::Vector3d &gyro_1,
                            const Eigen::Vector3d &acc_bias, const Eigen::Vector3d &gyro_bias,
                            Eigen::Vector3d &delta_p, Eigen::Quaterniond &delta_q, Eigen::Vector3d &delta_v
                            );

};

#endif //IMUSIMWITHPOINTLINE_IMU_H
