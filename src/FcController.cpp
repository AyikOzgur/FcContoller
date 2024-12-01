#include "FcController.h"
#include "FcControllerVersion.h"

std::string FcController::getVersion()
{
    return FC_CONTROLLER_VERSION;
}

bool FcController::init(std::string port, int baudrate, int flags)
{
    if (!m_serialPort.open(port.c_str(), baudrate, 0))
    {
        std::cout << "ERROR: Serial port : " << port << " could not open." << std::endl;
        return false;
    }

    return true;
}

bool FcController::executeCommand(FcCommand command)
{
    // Not supported yet.
    return false;
}

bool FcController::executeCommand(FcCommand command, std::vector<uint16_t> &data)
{
    if (command != FcCommand::SET_RC_CHANNELS)
    {
        return false;
    }

    uint8_t bufferWrite[1024]{0};
    size_t bufferWriteSize = 0;
    if (!m_parser.encode(bufferWrite, bufferWriteSize, MspCommand::MSP_SET_RAW_RC, data))
    {
        return false;
    }

    if (!m_serialPort.write(bufferWrite, bufferWriteSize))
    {
        return false;
    }

    return true;
}

bool FcController::executeCommand(FcCommand command, std::vector<float> &data)
{

    if (command != FcCommand::GET_ALTITUDE && command != FcCommand::GET_BATTERY_VOLTAGE 
        && command != FcCommand::GET_ATTITUDE && command != FcCommand::GET_RAW_IMU)
    {
        return false;
    }

    uint8_t bufferWrite[1024]{0};
    size_t bufferWriteSize = 0;
    MspCommand mspCommandSent;
    switch (command)
    {
    case FcCommand::GET_ALTITUDE:
    {
        mspCommandSent = MspCommand::MSP_ALTITUDE;
        m_parser.encode(bufferWrite, bufferWriteSize, MspCommand::MSP_ALTITUDE);
        break;
    }
    case FcCommand::GET_BATTERY_VOLTAGE:
    {
        mspCommandSent = MspCommand::MSP_ANALOG;
        m_parser.encode(bufferWrite, bufferWriteSize, MspCommand::MSP_ANALOG);
        break;
    }
    case FcCommand::GET_RAW_IMU:
    {
        mspCommandSent = MspCommand::MSP_RAW_IMU;
        m_parser.encode(bufferWrite, bufferWriteSize, MspCommand::MSP_RAW_IMU);
        break;
    }
    case FcCommand::GET_ATTITUDE:
    {
        mspCommandSent = MspCommand::MSP_ATTITUDE;
        m_parser.encode(bufferWrite, bufferWriteSize, MspCommand::MSP_ATTITUDE);
        break;
    }
    default:
        return false;
    }

    // Clear the input data vector.
    data.clear();

    // Read internal buffer. To clean it. This ensures that we are not gonna process old data.
    uint8_t bufferRead[1024]{0};
    int bytesRead = m_serialPort.read(bufferRead, sizeof(bufferRead));

    int retryCount = 10;
    while (retryCount-- > 0)
    {
        if (!m_serialPort.write(bufferWrite, bufferWriteSize))
        {
            std::cout << "ERROR: Could not write command to serial port." << std::endl;
            return false;
        }

        uint8_t bufferRead[1024]{0};
        int bytesRead = m_serialPort.read(bufferRead, sizeof(bufferRead));
        if (bytesRead <= 0)
        {
            continue;
        }

        MspCommand mspCommandReceived;
        for (int i = 0; i < bytesRead; i++)
        {
            if (m_parser.decode(bufferRead[i], mspCommandReceived, data))
            {
                if (mspCommandReceived == mspCommandSent)
                {
                    return true;
                }
            }
        }
    }

    return false;
}

bool FcController::isArmed()
{
    // Not supported yet.
    return false;
}