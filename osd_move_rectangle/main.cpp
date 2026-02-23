#include <iostream>
#include <thread>
#include <chrono> 
#include "FcController.h"

// Sub-grid precision constants (must match Betaflight osd_symbols.h)
// Each OSD character cell is 12x18 pixels (MAX7456 format).
// Custom font characters provide sub-cell positioning:
//   4 vertical line variants at pixel columns 0, 3, 6, 9  -> 4 horizontal sub-positions
//   3 horizontal line variants at pixel rows 0, 6, 12      -> 3 vertical sub-positions
static constexpr int H_SUBDIVS = 4;  // horizontal sub-positions per character cell
static constexpr int V_SUBDIVS = 3;  // vertical sub-positions per character cell

int main()
{
    std::cout << "FcController v" << FcController::getVersion() << " test" << std::endl;

    FcController fcController;

#if defined(_WIN32)
    std::string port = "COM6";
#else
    std::string port = "/dev/ttyACM0";
#endif

    int baudrate = 115200;

    int useDefaultPort = 1; 
    std::cout << "Default port: " << port << ", " << baudrate << std::endl;
    std::cout << "Use default port? (1 - yes, 0 - no): ";
    std::cin >> useDefaultPort;
    if (useDefaultPort == 0)
    {
        std::cout << "Enter port (/dev/ttyACM0 for linux , COM1 for windows): ";
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

    // OSD grid dimensions (NTSC: 30x13, PAL: 30x16)
    const int gridCols = 30;
    const int gridRows = 13;

    // Arena dimensions in sub-grid units.
    // Leaves a 1-character margin on each side to keep the rectangle visible.
    const int arenaWidth  = (gridCols - 2) * H_SUBDIVS;  // 28 * 4 = 112
    const int arenaHeight = (gridRows - 2) * V_SUBDIVS;   // 11 * 3 = 33

    // Rectangle size in sub-grid units
    int rectangleWidth  = 5 * H_SUBDIVS;  // 5 characters wide
    int rectangleHeight = 4 * V_SUBDIVS;  // 4 characters tall

    // Starting position in sub-grid units (offset by 1 character margin)
    int posX = 1 * H_SUBDIVS;
    int posY = 1 * V_SUBDIVS;
    int dx = 1;  // move 1 sub-position per step (~3px horizontal)
    int dy = 1;  // move 1 sub-position per step (~6px vertical)

    std::vector<uint16_t> data(4);

    std::cout << "Arena: " << arenaWidth << "x" << arenaHeight << " sub-grid units" << std::endl;
    std::cout << "Rectangle: " << rectangleWidth << "x" << rectangleHeight << " sub-grid units" << std::endl;
    std::cout << "Sub-grid precision: " << H_SUBDIVS << "x" << V_SUBDIVS
              << " per character cell (12x18 px)" << std::endl;

    while (true) 
    {
        // Update position
        posX += dx;
        posY += dy;

        // Bounce off arena edges
        if (posX + rectangleWidth >= (gridCols - 1) * H_SUBDIVS || posX <= 1 * H_SUBDIVS)
        {
            dx = -dx;
        }

        if (posY + rectangleHeight >= (gridRows - 1) * V_SUBDIVS || posY <= 1 * V_SUBDIVS)
        {
            dy = -dy;
        }

        // Pack sub-grid coordinates: x, y, width, height (all uint8_t on the wire)
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
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}