#include "FcController.h"
#include "FcControllerVersion.h"

std::string FcController::getVersion()
{
    return FC_CONTROLLER_VERSION;
}

FcController::FcController()
{
}

FcController::~FcController()
{
}

bool FcController::init(std::string port, int baudrate, int flags)
{
    return false;
}

bool FcController::executeCommand(FcCommand command , int value)
{
    return false;
}

bool FcController::getAttitude(Attitude& attitude)
{
    return false;
}

bool FcController::getRawImu(RawImu& rawImu)
{
    return false;
}

bool FcController::setRcChannels(std::vector<uint16_t>& rcChannels)
{
    return false;
}

bool FcController::getRcChannels(std::vector<uint16_t>& rcChannels)
{
    return false;
}

bool FcController::isArmed()
{
    return false;
}