#include <chrono>
#include <iostream>
#include <thread>

#include "bmi088_drv.h"

#include "ros/node_handle.h"
#include "sensor_msgs/Imu.h"

BMI088Drv init(ros::NodeHandle& nh)
{
    auto accel_dev = nh.param<std::string>("accel_dev", "/dev/spidev0.0");
    auto gyro_dev = nh.param<std::string>("gyro_dev", "/dev/spidev0.1");
    auto spi_speed = nh.param<int>("spi_speed", 5000000);

    auto gpio_chip = nh.param<std::string>("gpio_chip", "/dev/gpiochip0");
    auto accel_int = nh.param<int>("accel_int", 25);
    auto gyro_int = nh.param<int>("gyro_int", 24);
    
    auto accel_hz = nh.param<int>("accel_hz", 1600);
    auto accel_bw = nh.param<int>("accel_bw", 0);
    auto accel_range = nh.param<int>("accel_range", 3);
    auto gyro_hz = nh.param<int>("gyro_hz", 2000);
    auto gyro_range = nh.param<int>("gyro_range", 250);

    AccODR acc_odr;
    switch (accel_hz) {
    case 1600:
        acc_odr = AccODR::ODR_1600;
        break;
    case 800:
        acc_odr = AccODR::ODR_800;
        break;
    case 400:
        acc_odr = AccODR::ODR_400;
        break;
    case 200:
        acc_odr = AccODR::ODR_200;
        break;
    case 100:
        acc_odr = AccODR::ODR_100;
        break;
    case 50:
        acc_odr = AccODR::ODR_50;
        break;
    case 25:
        acc_odr = AccODR::ODR_25;
        break;
    case 13:
    case 12:
        acc_odr = AccODR::ODR_12_5;
        break;
    default:
        std::cerr << "Invalid Accel Data Rate: " << accel_hz << "Hz" << std::endl;
        throw std::invalid_argument("invalid accel data rate");
    }

    AccBandWidth acc_bw;
    switch (accel_bw) {
    case 0:
    case 1:
        acc_bw = AccBandWidth::BW_Normal;
        break;
    case 2:
        acc_bw = AccBandWidth::BW_OSR2;
        break;
    case 4:
        acc_bw = AccBandWidth::BW_OSR4;
        break;
    default:
        std::cerr << "Invalid Accel BandWidth: " << accel_hz << std::endl;
        throw std::invalid_argument("invalid accel bandwidth");
    }

    AccRange acc_range;
    switch (accel_range) {
    case 3:
        acc_range = AccRange::RANGE_3G;
        break;
    case 6:
        acc_range = AccRange::RANGE_6G;
        break;
    case 12:
        acc_range = AccRange::RANGE_12G;
        break;
    case 24:
        acc_range = AccRange::RANGE_24G;
        break;
    default:
        std::cerr << "Invalid Accel Range: +/-" << accel_range << "g" << std::endl;
        throw std::invalid_argument("invalid accel range");
    }

    GyroODR gyro_odr;
    switch (gyro_hz) {
    case 2000:
        gyro_odr = GyroODR::ODR_2000_BW_230;
        break;
    case 1000:
        gyro_odr = GyroODR::ODR_1000_BW_116;
        break;
    case 400:
        gyro_odr = GyroODR::ODR_400_BW_47;
        break;
    case 200:
        gyro_odr = GyroODR::ODR_200_BW_23;
        break;
    case 100:
        gyro_odr = GyroODR::ODR_100_BW_12;
        break;
    default:
        std::cerr << "Invalid Gyro Data Rate: " << gyro_hz << "Hz" << std::endl;
        throw std::invalid_argument("invalid gyro data rate");
    }

    GyroRange gyroRange;
    switch (gyro_range) {
    case 2000:
        gyroRange = GyroRange::RANGE_2000_DPS;
        break;
    case 1000:
        gyroRange = GyroRange::RANGE_1000_DPS;
        break;
    case 500:
        gyroRange = GyroRange::RANGE_500_DPS;
        break;
    case 250:
        gyroRange = GyroRange::RANGE_250_DPS;
        break;
    case 125:
        gyroRange = GyroRange::RANGE_125_DPS;
        break;
    default:
        std::cerr << "Invalid Gyro Data Range: " << gyro_hz << " deg/s" << std::endl;
        throw std::invalid_argument("invalid gyro range");
    }

    return BMI088Drv(
        accel_dev.c_str(),
        gyro_dev.c_str(),
        spi_speed,
        gpio_chip.c_str(),
        accel_int,
        gyro_int,
        acc_odr,
        acc_bw,
        acc_range,
        gyro_odr,
        gyroRange);
}

void publish_imu(ros::Publisher& pub, Vec3d accel, Vec3d gyro, uint64_t seq_id, std::string frame_id)
{
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.seq = seq_id;
    imu_msg.header.frame_id = frame_id;

    imu_msg.orientation_covariance[0] = -1; // no orientation

    imu_msg.linear_acceleration.x = accel.x;
    imu_msg.linear_acceleration.y = accel.y;
    imu_msg.linear_acceleration.z = accel.z;

    imu_msg.angular_velocity.x = gyro.x;
    imu_msg.angular_velocity.y = gyro.y;
    imu_msg.angular_velocity.z = gyro.z;

    pub.publish(imu_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bmi088_driver");
    ros::NodeHandle nh("~");
    ros::NodeHandle n;

    auto topic = n.advertise<sensor_msgs::Imu>("imu", 100);
    auto drv = init(nh);

    std::cout << "BMI088 Driver Running..." << std::endl;

    Vec3d accel, gyro;
    bool accel_rdy = false, gyro_rdy = false;
    int max_retry = 10;

    long seq_id = 0;
    while (ros::ok()) {
        int flag = drv.Wait();

        if (flag & 1) {
            for (int i=0; i<max_retry; i++) {
                accel_rdy = drv.ReadAccel(&accel);
                if (accel_rdy) break;
            }
        }
        if (flag & 2) {
            for (int i=0; i<max_retry; i++) {
                gyro_rdy = drv.ReadGyro(&gyro);
                if (gyro_rdy) break;
            }
        }

        if (accel_rdy && gyro_rdy) {
            accel_rdy = false;
            gyro_rdy = false;
            publish_imu(topic, accel, gyro, seq_id++, "/base_print");
        }

        ros::spinOnce();
    }
}