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

enum Flags
{
    NONE = 0,
    NO_BLOCKING = 1,
};

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

    /**
     * @brief Internal thread interval in milliseconds.
     * If FcController is initialized with Flags::NO_BLOCKING, FcController will
     * create an internal thread to read data from the flight controller at this interval.
     * Any request of data will return the last data read from the flight controller.
     * Default value is 20 milliseconds.
     * @param int interval in milliseconds.
     */
    THREAD_INTERVAL,

    /**
     * @brief Set the rc channels of the flight controller.
     * @param std::vector<uint16_t> rcChannels the rc channels to set.
     */
    SET_RC_CHANNELS,

    /**
     * @brief Get the rc channels of the flight controller.
     * @param std::vector<uint16_t> rcChannels the rc channels of the flight controller.
     */
    GET_RC_CHANNELS,

    /**
     * @brief Get the attitude of the flight controller.
     * @param std::vector<float> Attitude of the flight controller.
     * Elements 0 - 2: roll (deg), pitch (deg), yaw (deg).
     */
    GET_ATTITUDE,

    /**
     * @brief Get the raw imu data of the flight controller.
     * @param std::vector<float> RawImu the raw imu data of the flight controller.
     * Element 0 - 2: accx (m/s^2), accy (m/s^2), accz (m/s^2),
     * Element 3 - 5: gyrox (deg/s), gyroy (deg/s), gyroz (deg/s),
     * Element 6 - 8: magx (Gauss), magy (Gauss), magz (Gauss).
     */
    GET_RAW_IMU,

    /**
     * @brief Get altitude of the flight controller.
     * @param Altitude in cm.
     */
    GET_ALTITUDE,

    /**
     * @brief Get the battery voltage of the flight controller.
     * @param Voltage in 0.1V.
     */
    GET_BATTERY_VOLTAGE,
    
    /**
     * @brief Set the position of rectangle on OSD.
     * @param posX X coordinate in grid.
     * @param posY Y coordinate in grid.
     * @remark THIS IS A CUSTOM COMMAND IT IS ONLY SUPPORTED BY CUSTOM BETAFLIGHT.
     */
    SET_RECTANGLE_POS
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
     * @brief Initialize the flight controller.
     * @param port port to connect to the flight controller.
     * @param baudrate baudrate to connect to the flight controller.
     * @param flag Not used for now.
     * @return True if the flight controller is initialized successfully.
     */
    bool init(std::string port = DEFAULT_PORT, int baudrate = 115200, int flags = 0);

    /**
     * @brief Execute a command on the flight controller.
     * @param FcCommand the command to execute.
     * @return True if the command is executed successfully.
     */
    bool executeCommand(FcCommand command);

    /**
     * @brief Execute a command on the flight controller.
     * @param FcCommand the command to execute.
     * @param data the arguments of the command.
     * @return True if the command is executed successfully.
     */
    bool executeCommand(FcCommand command, std::vector<uint16_t> &data);

    /**
     * @brief Execute a command on the flight controller.
     * @param FcCommand the command to execute.
     * @param data Response of flight controller for sent command.
     * @return True if the command is executed successfully.
     */
    bool executeCommand(FcCommand command, std::vector<float> &data);

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
    /// Modes of the flight controller.
    MspModes m_modes;
};