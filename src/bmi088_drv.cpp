#include "bmi088_drv.h"

#include "bmi08x.h"

#include <chrono>
#include <functional>
#include <iostream>
#include <math.h>
#include <string>
#include <thread>
#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define GRAVITY_EARTH (9.80665)

struct spi_device
{
    int fd;
    int speed_hz;
};

/**
 * @brief 休眠指定时间长度
 *
 * @param period
 * @param intf_ptr
 */
void delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    std::this_thread::sleep_for(std::chrono::microseconds(period));
}

BMI08_INTF_RET_TYPE spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    // printf("Reading SPI %d bytes at 0x%02x\n", len, reg_addr);
    spi_device *dev = (spi_device *)intf_ptr;

    struct spi_ioc_transfer xfer = {0};

    uint8_t buff[33] = {0};

    xfer.tx_buf = (unsigned long)buff;
    xfer.rx_buf = (unsigned long)buff;
    xfer.len = len + 1;
    xfer.speed_hz = dev->speed_hz;

    buff[0] = reg_addr;

    int status = ioctl(dev->fd, SPI_IOC_MESSAGE(1), &xfer);
    if (status < 0)
    {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    memcpy(reg_data, &buff[1], len);
    // printf("response(%2d, %2d): ", len, status);
    // for (int i = 0; i < len; i++) {
    //     printf("%02x", reg_data[i]);
    // }
    // printf("\n");
    return 0; // means success
}

BMI08_INTF_RET_TYPE spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    // printf("Writing SPI %d bytes at 0x%02x\n", len, reg_addr);

    spi_device *dev = (spi_device *)intf_ptr;

    uint8_t buff[33] = {0};
    buff[0] = reg_addr;
    memcpy(&buff[1], reg_data, len);

    struct spi_ioc_transfer xfer = {0};

    xfer.tx_buf = (unsigned long)buff;
    xfer.rx_buf = 0;
    xfer.len = len + 1;
    xfer.speed_hz = dev->speed_hz;

    int status = ioctl(dev->fd, SPI_IOC_MESSAGE(1), &xfer);
    if (status < 0)
    {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }
    return 0; // means success
}

class BMI088Drv::impl
{
private:
    spi_device spi_accel, spi_gyro;
    int accel_range = 3;
    int gyro_range = 250;

    struct bmi08_dev dev = {
        .intf_ptr_accel = &spi_accel,
        .intf_ptr_gyro = &spi_gyro,
        .intf = BMI08_SPI_INTF,    // 使用SPI接口
        .variant = BMI088_VARIANT, // 使用 BMI088
        .read_write_len = 32,
        .read = &spi_read,
        .write = &spi_write,
        .delay_us = &delay_us,
    };

private:
    /**
     * @brief 打开SPIDEV接口
     *
     * @param port 路径
     * @return int FD
     */
    int setup_spi(const char *port)
    {
        int fd = open(port, O_RDONLY);
        if (fd < 0)
        {
            std::cerr << "Error when open " << port << ": " << std::strerror(errno) << std::endl;
        }
        return fd;
    }

    void print_result(const char *api_name, int8_t result)
    {
        if (result == BMI08_OK)
            return;
        std::cerr << "[ERR] " << api_name << ": " << (int)result << std::endl;
    }

    double lsb_to_mps2(int16_t val)
    {
        double half_scale = (double)(1 << (16 - 1));
        return (GRAVITY_EARTH * val * accel_range) / half_scale;
    }

    Vec3d lsb_to_mps2(bmi08_sensor_data &val)
    {
        return Vec3d{
            .x = lsb_to_mps2(val.x),
            .y = lsb_to_mps2(val.y),
            .z = lsb_to_mps2(val.z),
        };
    }

    double lsb_to_dps(int16_t val)
    {
        double half_scale = (double)(1 << (16 - 1));
        return ((double)gyro_range / (half_scale)) * (val);
    }

    Vec3d lsb_to_dps(bmi08_sensor_data &val)
    {
        return Vec3d{
            .x = lsb_to_dps(val.x),
            .y = lsb_to_dps(val.y),
            .z = lsb_to_dps(val.z),
        };
    }

    void delay_ms(int ms)
    {
        delay_us(ms * 1000, nullptr);
    }

    /**
     * @brief 初始化设备
     *
     */
    void device_init(uint8_t acc_odr, uint8_t acc_bw, uint8_t acc_range, uint8_t gyro_odr, uint8_t gyro_range)
    {
        auto result = bmi08xa_init(&dev);
        print_result("bmi08xa_init", result);
        if (result != BMI08_OK)
        {
            throw std::logic_error("accel init failed");
        }

        result = bmi08g_init(&dev);
        print_result("bmi08g_init", result);
        if (result != BMI08_OK)
        {
            throw std::logic_error("gyro init failed");
        }

        // 重置中断状态以防出现问题
        result = bmi08a_soft_reset(&dev);
        print_result("bmi08a_soft_reset", result);
        result = bmi08g_soft_reset(&dev);
        print_result("bmi08g_soft_reset", result);

        // 写配置文件，大概
        std::cout << "Uploading config file..." << std::endl;
        result = bmi08a_load_config_file(&dev);
        if (result == BMI08_OK) {
            std::cout << "Done." << std::endl;
        }
        print_result("bmi08a_load_config_file", result);

        // init accel
        dev.accel_cfg.odr = acc_odr;
        dev.accel_cfg.range = acc_range;

        dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE; /*user_accel_power_modes[user_bmi088_accel_low_power]; */
        dev.accel_cfg.bw = acc_bw;                   /* Bandwidth and OSR are same */

        result = bmi08a_set_power_mode(&dev);
        print_result("bmi08a_set_power_mode", result);

        result = bmi08xa_set_meas_conf(&dev);
        print_result("bmi08xa_set_meas_conf", result);

        // init gyro
        dev.gyro_cfg.odr = gyro_odr;
        dev.gyro_cfg.range = gyro_range;
        dev.gyro_cfg.bw = gyro_odr;
        dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

        result = bmi08g_set_power_mode(&dev);
        print_result("bmi08g_set_power_mode", result);

        result = bmi08g_set_meas_conf(&dev);
        print_result("bmi08g_set_meas_conf", result);
    }

    /**
     * @brief 启用/禁用传感器中断
     *
     * @param enable
     */
    void set_interrupt_state(bool enable)
    {
        uint8_t state = enable ? BMI08_ENABLE : BMI08_DISABLE;
        bmi08_accel_int_channel_cfg accel_cfg = {
            .int_channel = BMI08_INT_CHANNEL_1,
            .int_type = BMI08_ACCEL_INT_DATA_RDY,
            .int_pin_cfg = {
                .lvl = BMI08_INT_ACTIVE_HIGH,
                .output_mode = BMI08_INT_MODE_PUSH_PULL,
                .enable_int_pin = state,
            }};

        auto result = bmi08a_set_int_config(&accel_cfg, &dev);
        print_result("bmi08a set int", result);

        bmi08_gyro_int_channel_cfg gyro_cfg = {
            .int_channel = BMI08_INT_CHANNEL_3,
            .int_type = BMI08_GYRO_INT_DATA_RDY,
            .int_pin_cfg = {
                .lvl = BMI08_INT_ACTIVE_HIGH,
                .output_mode = BMI08_INT_MODE_PUSH_PULL,
                .enable_int_pin = state,
            }};

        result = bmi08g_set_int_config(&gyro_cfg, &dev);
        print_result("bmi08g set int", result);

        uint8_t data;
        result = bmi08g_get_regs(BMI08_REG_GYRO_INT3_INT4_IO_MAP, &data, 1, &dev);
        print_result("bmi08g_get_regs", result);
    }

public:
    impl(const char *accel, const char *gyro, AccODR acc_odr, AccBandWidth acc_bw, AccRange acc_range, GyroODR gyro_odr, GyroRange gyro_range, int speed_hz = 5000000)
    {
        spi_accel.speed_hz = speed_hz;
        spi_gyro.speed_hz = speed_hz;

        spi_accel.fd = setup_spi(accel);
        spi_gyro.fd = setup_spi(gyro);
        if (spi_accel.fd < 0 || spi_gyro.fd < 0)
        {
            throw std::invalid_argument("could not open spi device");
        }

        this->gyro_range = 125 * (1 << ((int8_t)GyroRange::RANGE_125_DPS - (int8_t)gyro_range));
        this->accel_range = 3 * (1 << (int8_t)acc_range);

        delay_ms(10);

        device_init(
            BMI08_ACCEL_ODR_12_5_HZ + (uint8_t)acc_odr,
            BMI08_ACCEL_BW_OSR4 + (uint8_t)acc_bw,
            (uint8_t)acc_range,
            (uint8_t)gyro_odr,
            (uint8_t)gyro_range);
        set_interrupt_state(true);

        delay_ms(10);
    }

    /**
     * @brief 读取加速度信息
     * 
     * @param data 数据
     * @return true 有数据
     * @return false 还没有新的测量数据
     */
    bool ReadAccel(Vec3d *data)
    {
        uint8_t status = 0;
        auto result = bmi08a_get_data_int_status(&status, &dev);
        print_result("bmi08a_get_data_int_status", result);

        if ((status & BMI08_ACCEL_DATA_READY_INT) == 0)
        {
            return false; // data not ready
        }

        struct bmi08_sensor_data bmi08_accel;
        result = bmi08a_get_data(&bmi08_accel, &dev);
        print_result("bmi08a_get_data", result);

        *data = lsb_to_mps2(bmi08_accel);
        return true;
    }

    /**
     * @brief 读加速度计信息
     *
     * @return Vec3d
     */
    Vec3d ReadAccel()
    {
        Vec3d result = {0, 0, 0};
        for (int i = 0; i < 100; i++)
        {
            bool success = ReadAccel(&result);
            if (success)
                return result;
        }
        return {0, 0, 0};
    }

    /**
     * @brief 读取陀螺仪信息
     *
     * @param data 陀螺仪数据
     * @return true 有数据。注意这个标志位只会在280-400us后自动清除，请务必等待这么久之后再来读取。
     * @return false 无新数据
     */
    bool ReadGyro(Vec3d *data)
    {
        uint8_t status = 0;
        auto result = bmi08g_get_data_int_status(&status, &dev);
        print_result("bmi08g_get_data_int_status", result);

        if ((status & BMI08_GYRO_DATA_READY_INT) == 0)
        {
            return false;
        }

        struct bmi08_sensor_data bmi08_gyro;
        result = bmi08g_get_data(&bmi08_gyro, &dev);
        print_result("bmi08g_get_data", result);

        *data = lsb_to_mps2(bmi08_gyro);
        return true;
    }

    /**
     * @brief 读陀螺仪信息
     *
     * @return Vec3d
     */
    Vec3d ReadGyro()
    {
        Vec3d result = {0, 0, 0};
        for (int i = 0; i < 100; i++)
        {
            bool success = ReadGyro(&result);
            if (success)
                return result;
        }
        return {0, 0, 0};
    }

    /**
     * @brief 打印调试信息
     *
     * @param times_to_read
     */
    void print(uint8_t times_to_read)
    {
        if (dev.accel_cfg.power == BMI08_ACCEL_PM_ACTIVE)
        {
            printf("\nACCEL DATA\n");
            printf("Accel data in m/s^2\n");
            printf("Accel data range : %dG \n\n", accel_range);

            printf("Sample_Count, Acc_ms2_X, Acc_ms2_Y, Acc_ms2_Z\n");

            for (int i = 0; i < times_to_read; i++)
            {
                auto data = ReadAccel();
                printf("%d, %4.4f, %4.4f, %4.4f\n",
                       i,
                       data.x,
                       data.y,
                       data.z);
            }
        }

        if (dev.gyro_cfg.power == BMI08_GYRO_PM_NORMAL)
        {
            printf("\n\nGYRO DATA\n");
            printf("Gyro data in degrees per second\n");
            printf("Gyro data range : %d dps\n\n", gyro_range);

            printf("Sample_Count, Gyr_DPS_X, Gyr_DPS_Y, Gyr_DPS_Z\n");

            for (int i = 0; i < times_to_read; i++)
            {
                auto data = ReadGyro();
                delay_us(400, nullptr); // gyro 数据ready不会自动清除

                printf("%d, %4.4f, %4.4f, %4.4f\n",
                       i,
                       data.x,
                       data.y,
                       data.z);
            }
        }
    }

    ~impl()
    {
        close(spi_accel.fd);
        close(spi_gyro.fd);
    }
};

BMI088Drv::BMI088Drv(const char *accel, const char *gyro, AccODR acc_odr, AccBandWidth acc_bw, AccRange acc_range, GyroODR gyro_odr, GyroRange gyro_range)
    : ptr(std::make_unique<BMI088Drv::impl>(accel, gyro, acc_odr, acc_bw, acc_range, gyro_odr, gyro_range))
{
}

BMI088Drv::~BMI088Drv()
{
}

BMI088Drv::BMI088Drv(BMI088Drv&& other): ptr(std::move(other.ptr))
{
}

Vec3d BMI088Drv::ReadAccel()
{
    return ptr->ReadAccel();
}

Vec3d BMI088Drv::ReadGyro()
{
    return ptr->ReadGyro();
}

bool BMI088Drv::ReadAccel(Vec3d* data)
{
    return ptr->ReadAccel(data);
}

bool BMI088Drv::ReadGyro(Vec3d* data)
{
    return ptr->ReadGyro(data);
}

void BMI088Drv::print(uint8_t times)
{
    ptr->print(times);
}

