//
// Created by hyj on 18-1-19.
//

#include "imu.h"
#include "utilities.h"

// euler2Rotation:   body frame to interitail frame
Eigen::Matrix3d euler2Rotation(Eigen::Vector3d eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);
    double yaw = eulerAngles(2);

    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);

    Eigen::Matrix3d RIb;
    RIb << cy * cp, cy * sp * sr - sy * cr, sy * sr + cy * cr * sp,
        sy * cp, cy * cr + sy * sr * sp, sp * sy * cr - cy * sr,
        -sp, cp * sr, cp * cr;
    return RIb;
}

Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);

    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);

    Eigen::Matrix3d R;
    R << 1, 0, -sp,
        0, cr, sr * cp,
        0, -sr, cr * cp;

    return R;
}

IMU::IMU(Param p) : param_(p)
{
    gyro_bias_ = Eigen::Vector3d::Zero();
    acc_bias_ = Eigen::Vector3d::Zero();
}

//
void IMU::addIMUnoise(MotionData &data)
{
    // 使用随机数产生噪声  均值为0 方差为1的高斯白噪声
    std::random_device rd;
    std::default_random_engine generator_(rd());
    std::normal_distribution<double> noise(0.0, 1.0);

    Eigen::Vector3d noise_gyro(noise(generator_), noise(generator_), noise(generator_));
    Eigen::Matrix3d gyro_sqrt_cov = param_.gyro_noise_sigma * Eigen::Matrix3d::Identity(); //
    data.imu_gyro = data.imu_gyro + gyro_sqrt_cov * noise_gyro / sqrt(param_.imu_timestep) + gyro_bias_;

    Eigen::Vector3d noise_acc(noise(generator_), noise(generator_), noise(generator_));
    Eigen::Matrix3d acc_sqrt_cov = param_.acc_noise_sigma * Eigen::Matrix3d::Identity(); //
    data.imu_acc = data.imu_acc + acc_sqrt_cov * noise_acc / sqrt(param_.imu_timestep) + acc_bias_;

    // gyro_bias update
    Eigen::Vector3d noise_gyro_bias(noise(generator_), noise(generator_), noise(generator_));
    gyro_bias_ += param_.gyro_bias_sigma * sqrt(param_.imu_timestep) * noise_gyro_bias;
    data.imu_gyro_bias = gyro_bias_;

    // acc_bias update
    Eigen::Vector3d noise_acc_bias(noise(generator_), noise(generator_), noise(generator_));
    acc_bias_ += param_.acc_bias_sigma * sqrt(param_.imu_timestep) * noise_acc_bias;
    data.imu_acc_bias = acc_bias_;
}

// 通过构造运动方差，产生IMU数据
MotionData IMU::MotionModel(double t)
{

    MotionData data;
    // param
    float ellipse_x = 15;
    float ellipse_y = 20;
    float z = 1;         // z轴做sin运动
    float K1 = 10;       // z轴的正弦频率是x，y的k1倍
    float K = M_PI / 10; // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周

    // translation
    // twb:  body frame in world frame
    Eigen::Vector3d position(ellipse_x * cos(K * t) + 5, ellipse_y * sin(K * t) + 5, z * sin(K1 * K * t) + 5);
    Eigen::Vector3d dp(-K * ellipse_x * sin(K * t), K * ellipse_y * cos(K * t), z * K1 * K * cos(K1 * K * t)); // position导数　in world frame
    double K2 = K * K;
    Eigen::Vector3d ddp(-K2 * ellipse_x * cos(K * t), -K2 * ellipse_y * sin(K * t), -z * K1 * K1 * K2 * sin(K1 * K * t)); // position二阶导数

    // Rotation
    double k_roll = 0.1;
    double k_pitch = 0.2;
    Eigen::Vector3d eulerAngles(k_roll * cos(t), k_pitch * sin(t), K * t);   // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
    Eigen::Vector3d eulerAnglesRates(-k_roll * sin(t), k_pitch * cos(t), K); // euler angles 的导数

    //    Eigen::Vector3d eulerAngles(0.0,0.0, K*t );   // roll ~ 0, pitch ~ 0, yaw ~ [0,2pi]
    //    Eigen::Vector3d eulerAnglesRates(0.,0. , K);      // euler angles 的导数

    Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles);                               // body frame to world frame 欧拉角到旋转矩阵的变换
    Eigen::Vector3d imu_gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates; //  euler rates trans to body gyro 欧拉角速度到 IMU body角速度

    Eigen::Vector3d gn(0, 0, -9.81);                        //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
    Eigen::Vector3d imu_acc = Rwb.transpose() * (ddp - gn); //  Rbw * Rwn * gn = gs

    // 模拟的IMUacc gyro 真值 以及对应的ground truth position and oriention
    data.imu_gyro = imu_gyro;
    data.imu_acc = imu_acc;
    data.Rwb = Rwb;
    data.twb = position;
    data.imu_velocity = dp;
    data.timestamp = t;
    return data;
}

//读取生成的imu数据并用imu动力学模型对数据进行计算，最后保存imu积分以后的轨迹，
//用来验证数据以及模型的有效性。
void IMU::testImu(std::string src, std::string dist)
{
    std::vector<MotionData> imudata;
    LoadPose(src, imudata);

    std::ofstream save_points;
    save_points.open(dist);

    double dt = param_.imu_timestep;
    Eigen::Vector3d Pwb = init_twb_;     // position :    from  imu measurements
    Eigen::Quaterniond Qwb(init_Rwb_);   // quaterniond:  from imu measurements
    Eigen::Vector3d Vw = init_velocity_; // velocity  :   from imu measurements
    Eigen::Vector3d gw(0, 0, -9.81);     // ENU frame

    for (int i = 1; i < imudata.size(); ++i)
    {

        MotionData imupose = imudata[i];

        //delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
        Eigen::Quaterniond dq;
        Eigen::Vector3d dtheta_half = imupose.imu_gyro * dt / 2.0;
        dq.w() = 1;
        dq.x() = dtheta_half.x();
        dq.y() = dtheta_half.y();
        dq.z() = dtheta_half.z();

        /// imu 动力学模型 欧拉积分
        Eigen::Vector3d acc_w = Qwb * (imupose.imu_acc) + gw; // aw = Rwb * ( acc_body - acc_bias ) + gw
        Qwb = Qwb * dq;
        Vw = Vw + acc_w * dt;
        Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;

        /// 中值积分

        //　按着imu postion, imu quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以imu存了两次
        save_points << imupose.timestamp << " "
                    << Qwb.w() << " "
                    << Qwb.x() << " "
                    << Qwb.y() << " "
                    << Qwb.z() << " "
                    << Pwb(0) << " "
                    << Pwb(1) << " "
                    << Pwb(2) << " "
                    << Qwb.w() << " "
                    << Qwb.x() << " "
                    << Qwb.y() << " "
                    << Qwb.z() << " "
                    << Pwb(0) << " "
                    << Pwb(1) << " "
                    << Pwb(2) << " "
                    << std::endl;
    }
    save_points.close();
    std::cout << "test　end" << std::endl;
}

void IMU::testImu_eulerIntegration(std::string src, std::string dist)
{
    std::vector<MotionData> imudata;
    LoadPose(src, imudata);

    std::ofstream save_points;
    save_points.open(dist);

    double dt = param_.imu_timestep;

    // Eigen::Vector3d Pwb = init_twb_;     // position :    from  imu measurements
    // Eigen::Quaterniond Qwb(init_Rwb_);   // quaterniond:  from imu measurements
    // Eigen::Vector3d Vw = init_velocity_; // velocity  :   from imu measurements

    Eigen::Vector3d Pwb ;     // position :    from  imu measurements
    Eigen::Quaterniond Qwb;   // quaterniond:  from imu measurements
    Eigen::Vector3d Vw ; // velocity  :   from imu measurements

    // std::cout << "init: " << Pwb.transpose() << " "<<  " "<< Vw.transpose() << std::endl;
    for (int i = 0; i < imudata.size(); ++i)
    {
        MotionData imupose = imudata[i];
        eulerIntegration(dt,
                         imupose.imu_acc, imupose.imu_gyro,
                         imupose.imu_acc_bias, imupose.imu_gyro_bias,
                         Pwb, Qwb, Vw // current of updated
        );
        // std::cout << "bias: " <<  imupose.imu_acc_bias.transpose() << " "<< " "<< imupose.imu_gyro_bias.transpose() << std::endl;
        //　按着imu postion, imu quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以imu存了两次
        save_points << imupose.timestamp << " "
                    << Qwb.w() << " "
                    << Qwb.x() << " "
                    << Qwb.y() << " "
                    << Qwb.z() << " "
                    << Pwb(0) << " "
                    << Pwb(1) << " "
                    << Pwb(2) << " "
                    << Qwb.w() << " "
                    << Qwb.x() << " "
                    << Qwb.y() << " "
                    << Qwb.z() << " "
                    << Pwb(0) << " "
                    << Pwb(1) << " "
                    << Pwb(2) << " "
                    << std::endl;
    }
    std::cout << "test　end of eulerIntegration" << std::endl;
    save_points.close();
}

void IMU::testImu_midPointIntegration(std::string src, std::string dist)
{
    std::vector<MotionData> imudata;
    LoadPose(src, imudata);

    std::ofstream save_points;
    save_points.open(dist);

    double dt = param_.imu_timestep;
    Eigen::Vector3d Pwb = init_twb_;     // position :    from  imu measurements
    Eigen::Quaterniond Qwb(init_Rwb_);   // quaterniond:  from imu measurements
    Eigen::Vector3d Vw = init_velocity_; // velocity  :   from imu measurements

    Eigen::Vector3d last_acc = imudata[0].imu_acc;
    Eigen::Vector3d last_gyro = imudata[0].imu_gyro;

    for (int i = 1; i < imudata.size(); ++i)
    {
        MotionData imupose = imudata[i];
        midPointIntegration(dt,
                            last_acc, last_gyro,
                            imupose.imu_acc, imupose.imu_gyro,
                            imupose.imu_acc_bias, imupose.imu_gyro_bias,
                            Pwb, Qwb, Vw // current of updated
        );

        last_acc = imupose.imu_acc;
        last_gyro = imupose.imu_gyro;
        //　按着imu postion, imu quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以imu存了两次
        save_points << imupose.timestamp << " "
                    << Qwb.w() << " "
                    << Qwb.x() << " "
                    << Qwb.y() << " "
                    << Qwb.z() << " "
                    << Pwb(0) << " "
                    << Pwb(1) << " "
                    << Pwb(2) << " "
                    << Qwb.w() << " "
                    << Qwb.x() << " "
                    << Qwb.y() << " "
                    << Qwb.z() << " "
                    << Pwb(0) << " "
                    << Pwb(1) << " "
                    << Pwb(2) << " "
                    << std::endl;
    }
    std::cout << "test　end of midPointIntegration" << std::endl;
    save_points.close();
}

void IMU::eulerIntegration(double _dt,
                           const Eigen::Vector3d &acc_k, const Eigen::Vector3d &gyro_k,                    // 第k帧 IMU data
                           const Eigen::Vector3d &acc_bias, const Eigen::Vector3d &gyro_bias,              // IMU 偏置项，这里假定为常数
                           Eigen::Vector3d &delta_p, Eigen::Quaterniond &delta_q, Eigen::Vector3d &delta_v //前一帧result,以及updated当前帧积分result
)
{
    Eigen::Vector3d un_gyro = gyro_k - gyro_bias; //  w = gyro_body - gyro_bias
    //  △q  delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
    delta_q = delta_q * Eigen::Quaterniond(1, un_gyro(0) * _dt / 2, un_gyro(1) * _dt / 2, un_gyro(2) * _dt / 2);

    Eigen::Vector3d gw(0, 0, -9.81);                                    // ENU frame
    Eigen::Vector3d un_acc = delta_q.toRotationMatrix() * (acc_k) + gw; // aw = Rwb * ( acc_body - acc_bias ) + gw

    // △v
    delta_v = delta_v + un_acc * _dt;

    // △p
    delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
}

void IMU::midPointIntegration(double _dt,
                              const Eigen::Vector3d &acc_0, const Eigen::Vector3d &gyro_0,
                              const Eigen::Vector3d &acc_1, const Eigen::Vector3d &gyro_1,
                              const Eigen::Vector3d &acc_bias, const Eigen::Vector3d &gyro_bias,
                              Eigen::Vector3d &delta_p, Eigen::Quaterniond &delta_q, Eigen::Vector3d &delta_v)
{
    Eigen::Vector3d un_gyro = 0.5 * (gyro_0 + gyro_1) - gyro_bias;
    delta_q = delta_q * Eigen::Quaterniond(1, un_gyro(0) * _dt / 2, un_gyro(1) * _dt / 2, un_gyro(2) * _dt / 2);

    Eigen::Vector3d gw(0, 0, -9.81); // ENU frame
    Eigen::Vector3d un_acc_0 = delta_q.toRotationMatrix() * (acc_0 - acc_bias) + gw;
    Eigen::Vector3d un_acc_1 = delta_q.toRotationMatrix() * (acc_1 - acc_bias) + gw;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    // △v
    delta_v = delta_v + un_acc * _dt;

    // △p
    delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
}
