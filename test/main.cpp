#include <iostream>
#include "FcController.h"

int main()
{
    std::cout << "FcController v" << FcController::getVersion() << " test" << std::endl;

    FcController fcController;

#if defined(_WIN32)
    std::string port = "COM6";
#else
    std::string port = "/dev/ttyUSB0";
#endif

    int baudrate = 115200;

    int useDefaultPort = 1; 
    std::cout << "Default port: " << port << ", " << baudrate << std::endl;
    std::cout << "Use default port? (1 - yes, 0 - no): ";
    std::cin >> useDefaultPort;
    if (useDefaultPort == 0)
    {
        std::cout << "Enter port (/dev/ttyUSB0 for linux , COM1 for windows): ";
        std::cin >> port;

#if defined(_WIN32)
        port = "\\\\.\\" + port;
#endif

    }

    if (!fcController.init(port, baudrate))
    {
        std::cout << "ERROR: Could not initialize flight controller." << std::endl;
        return -1;
    }

    while (true)
    {
        int command = 0;
        std::cout << "Commands:" << std::endl;
        std::cout << "-1 - Exit" << std::endl;
        std::cout << static_cast<int>(FcCommand::GET_ALTITUDE) << " - Get altitude" << std::endl;
        std::cout << static_cast<int>(FcCommand::GET_BATTERY_VOLTAGE) << " - Get battery voltage" << std::endl;
        std::cout << static_cast<int>(FcCommand::GET_RAW_IMU) << " - Get raw imu" << std::endl;
        std::cout << static_cast<int>(FcCommand::GET_ATTITUDE) << " - Get attitude" << std::endl;
        std::cout << static_cast<int>(FcCommand::SET_RC_CHANNELS) << " - Set rc channels" << std::endl;
        std::cout << static_cast<int>(FcCommand::SET_RECTANGLE_POS) << " - Set rectangle pos" << std::endl; // This is a custom test command.

        std::cout << "Enter command : ";
        std::cin >> command;

        if (command == -1)
        {
            break;
        }
        else if (command == static_cast<int>(FcCommand::SET_RC_CHANNELS))
        {
            std::vector<uint16_t> data;
            for (int i = 0; i < 8; i++)
            {
                uint16_t rcValue = 1300;
                data.push_back(rcValue);
            }

            if (!fcController.executeCommand(static_cast<FcCommand>(command), data))
            {
                std::cout << "ERROR: Could not execute command." << std::endl;
            }
            continue;
        }
        else if (command == static_cast<int>(FcCommand::SET_RECTANGLE_POS))
        {
            int posX = 0;
            std::cout << "Enter pos x : ";
            std::cin >> posX;
            std::cout << std::endl;

            int posY = 0;
            std::cout << "Enter pos y : ";
            std::cin >> posY;
            std::cout << std::endl;

            std::vector<uint16_t> data;
            data.push_back((uint16_t)posX);
            data.push_back((uint16_t)posY);

            if (!fcController.executeCommand(static_cast<FcCommand>(command), data))
            {
                std::cout << "ERROR: Could not execute command." << std::endl;
            }
            continue;
        }

        std::vector<float> data; // Data returned by FC.
        if (!fcController.executeCommand(static_cast<FcCommand>(command), data))
        {
            std::cout << "ERROR: Could not execute command." << std::endl;
            continue;
        }

        if (command == static_cast<int>(FcCommand::GET_ALTITUDE))
        {
            std::cout << "Altitude: " << data[0] << " cm" << std::endl;
        }
        else if (command == static_cast<int>(FcCommand::GET_BATTERY_VOLTAGE))
        {
            std::cout << "Battery voltage: " << data[0] << " V" << std::endl;
        }
        else if (command == static_cast<int>(FcCommand::GET_RAW_IMU))
        {
            std::cout << "Raw imu: : " << std::endl;
            std::cout << "Acc: " << data[0] << " " << data[1] << " " << data[2] << std::endl;
            std::cout << "Gyro: " << data[3] << " " << data[4] << " " << data[5] << std::endl;
        }
        else if (command == static_cast<int>(FcCommand::GET_ATTITUDE))
        {
            std::cout << "Roll: " << data[0] << " Pitch: " << data[1] << " Yaw: " << data[2] << std::endl;
        }
        else 
        {
            std::cout << "Invalid command." << std::endl;
        }
    }

    return 0;
}