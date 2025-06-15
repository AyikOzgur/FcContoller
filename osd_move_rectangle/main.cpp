#include <iostream>
#include <thread>
#include <chrono> 
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

    const int arenaWidth = 23;
    const int arenaHeight = 11;
    int posX = 0;
    int posY = 0;
    int dx = 1;
    int dy = 1;
    int rectangleWidth = 6;
    int rectangleHeight = 6;

    std::vector<uint16_t> data(4);

    while (true) 
    {
        // Update position
        posX += dx;
        posY += dy;

        // Pack data: data[0] = X, data[1] = Y
        data[0] = static_cast<uint16_t>(posX);
        data[1] = static_cast<uint16_t>(posY);
        data[2] = static_cast<uint16_t>(rectangleWidth);
        data[3] = static_cast<uint16_t>(rectangleHeight);

        if (!fcController.executeCommand(FcCommand::SET_RECTANGLE_POS, data)) 
        {
            std::cerr << "ERROR: Could not execute command." << std::endl;
            break;
        }

        // Slow down the loop for visible movement
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (posY >= arenaHeight || posY <= 0)
        {
            rectangleHeight += dy * 2;
            dy = -dy;
        }

        if (posX >= arenaWidth || posX <= 0)
        {
            rectangleWidth += dx * 2;
            dx = -dx;
        }
    }

    return 0;
}