#pragma once

#include <memory>
#include <string>

struct Vec3d
{
    double x;
    double y;
    double z;
};

/**
 * @brief 加速度计数据输出速率
 *
 */
enum class AccODR
{
    ODR_12_5, // 12.5Hz
    ODR_25,   // 25Hz
    ODR_50,   // 50Hz
    ODR_100,  // 100Hz
    ODR_200,  // 200Hz
    ODR_400,  // 400Hz
    ODR_800,  // 800Hz
    ODR_1600, // 1600Hz
};

/**
 * @brief 加速度计带宽
 *
 */
enum class AccBandWidth
{
    BW_OSR4,   // 4倍超采样
    BW_OSR2,   // 2倍超采样
    BW_Normal, // 正常
};

/**
 * @brief 加速度计量程
 *
 */
enum class AccRange
{
    RANGE_3G,  // +-3G
    RANGE_6G,  // +-6G
    RANGE_12G, // +-12G
    RANGE_24G, // +-24G
};

/**
 * @brief 陀螺仪数据输出速率与带宽
 *
 */
enum class GyroODR
{
    ODR_2000_BW_532, // 2000Hz, 带宽532Hz
    ODR_2000_BW_230, // 2000Hz, 带宽230Hz
    ODR_1000_BW_116, // 1000Hz, 带宽116Hz
    ODR_400_BW_47,   // 400Hz, 带宽47Hz
    ODR_200_BW_23,   // 200Hz, 带宽23Hz
    ODR_100_BW_12,   // 100Hz, 带宽12Hz
    ODR_200_BW_64,   // 200Hz, 带宽64Hz
    ODR_100_BW_32,   // 100Hz, 带宽32Hz
};

/**
 * @brief 陀螺仪量程
 *
 */
enum class GyroRange
{
    RANGE_2000_DPS, // 2000Deg/s
    RANGE_1000_DPS, // 1000Deg/s
    RANGE_500_DPS,  // 500Deg/s
    RANGE_250_DPS,  // 250Deg/s
    RANGE_125_DPS,  // 125Deg/s
};

class BMI088Drv
{
    class impl;

private:
    std::unique_ptr<impl> ptr;

public:
    /**
     * @brief 新建一个BMI088驱动
     *
     * @param accel 加速度计SPI端口，一般为 /dev/spidev0.0
     * @param gyro  陀螺仪SPI端口，一般为 /dev/spidev0.1
     * @param acc_odr 加速度计数据速率
     * @param acc_bw 加速度计带宽
     * @param acc_range 加速度计量程
     * @param gyro_odr 陀螺仪数据速率
     * @param gyro_range 陀螺仪量程
     */
    BMI088Drv(const char *accel, const char *gyro, AccODR acc_odr, AccBandWidth acc_bw, AccRange acc_range, GyroODR gyro_odr, GyroRange gyro_range);
    ~BMI088Drv();

    BMI088Drv(BMI088Drv&& other);

    /**
     * @brief 阻塞读加速度计信息
     *
     * @return Vec3d
     */
    Vec3d ReadAccel();
    bool ReadAccel(Vec3d* data);

    /**
     * @brief 阻塞读陀螺仪信息
     *
     * @return Vec3d
     */
    Vec3d ReadGyro();
    bool ReadGyro(Vec3d* data);

    /**
     * @brief 打印调试信息
     *
     * @param times
     */
    void print(uint8_t times);
};
