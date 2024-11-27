#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <stdint.h>

#include "MspParser.h"
#include "SerialPort.h"

#ifdef _WIN32
#define DEFAULT_PORT "COM1"
#else
#define DEFAULT_PORT "/dev/ttyUSB0"
#endif

enum class FcCommand
{
    /**
     * @brief Invalid command.
     */
    NONE = -1,

    /**
     * @brief Arm the drone.
     */
    ARM,

    /**
     * @brief Disarm the drone.
     */
    DISARM,
};

struct Attitude
{
    /// Roll angle (deg).
    float roll{0.0f};
    /// Pitch angle (deg).
    float pitch{0.0f};
    /// Yaw angle (deg).
    float yaw{0.0f};
    /// Altitude (cm).
    float altitude{0.0f};
};

struct RawImu
{
    /// Accelerometer in x-axis (m/s^2).
    float accX{0.0f};
    /// Accelerometer in y-axis (m/s^2).
    float accY{0.0f};
    /// Accelerometer in z-axis (m/s^2).
    float accZ{0.0f};
    /// Gyroscope in x-axis (deg/s).
    float gyroX{0.0f};
    /// Gyroscope in y-axis (deg/s).
    float gyroY{0.0f};
    /// Gyroscope in z-axis (deg/s).
    float gyroZ{0.0f};
};


/**
 * @brief Fc controller class
 */
class FcController
{
public:

    /**
     * @brief Get the version of the flight controller.
     * @return std::string version in format "MAJOR.MINOR.PATCH"
     */
    static std::string getVersion();

    /**
     * @brief Constructor.
     */
    FcController();

    /**
     * @brief Destructor.
     */
    ~FcController();

    /**
     * @brief Initialize the flight controller.
     * @param std::string port the port to connect to the flight controller.
     * @param int baudrate the baudrate to connect to the flight controller.
     * @return True if the flight controller is initialized successfully.
     */
    bool init(std::string port = DEFAULT_PORT, int baudrate = 115200);

    /**
     * @brief Execute a command on the flight controller.
     * @param FcCommand command the command to execute.
     * @return True if the command is executed successfully.
     */
    bool executeCommand(FcCommand command);

    /**
     * @brief Get the attitude of the flight controller.
     * @param Attitude attitude of the flight controller.
     * @return True if the attitude is retrieved successfully.
     */
    bool getAttitude(Attitude& attitude);

    /**
     * @brief Get the raw imu data of the flight controller.
     * @param RawImu rawImu the raw imu data of the flight controller.
     * @return True if the raw imu data is retrieved successfully.
     */
    bool getRawImu(RawImu& rawImu);

    /**
     * @brief Set the rc channels of the flight controller.
     * @param std::vector<uint16_t> rcChannels the rc channels to set.
     * @return True if the rc channels are set successfully.
     */
    bool setRcChannels(std::vector<uint16_t>& rcChannels);

    /**
     * @brief Get the rc channels of the flight controller.
     * @param std::vector<uint16_t> rcChannels the rc channels of the flight controller.
     * @return True if the rc channels are retrieved successfully.
     */
    bool getRcChannels(std::vector<uint16_t>& rcChannels);

    /**
     * @brief Check if the flight controller is armed.
     * @return True if the flight controller is armed.
     */
    bool isArmed();

private:

    /// Serial port object.
    cr::clib::SerialPort m_serialPort;
    /// Msp parser object.
    MspParser m_parser;
};