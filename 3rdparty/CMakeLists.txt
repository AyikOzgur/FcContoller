cmake_minimum_required(VERSION 3.13)



################################################################################
## 3RD-PARTY
## dependencies for the project
################################################################################
project(3rdparty LANGUAGES CXX)



################################################################################
## SETTINGS
## basic 3rd-party settings before use
################################################################################
# To inherit the top-level architecture when the project is used as a submodule.
SET(PARENT ${PARENT}_3RDPARTY)
# Disable self-overwriting of parameters inside included subdirectories.
SET(${PARENT}_SUBMODULE_CACHE_OVERWRITE OFF CACHE BOOL "" FORCE)



################################################################################
## CONFIGURATION
## 3rd-party submodules configuration
################################################################################
SET(${PARENT}_SUBMODULE_SERIAL_PORT                    ON  CACHE BOOL "" FORCE)
if (${PARENT}_SUBMODULE_SERIAL_PORT)
    SET(${PARENT}_SERIAL_PORT                          ON  CACHE BOOL "" FORCE)
    SET(${PARENT}_SERIAL_PORT_TEST                     OFF CACHE BOOL "" FORCE)
endif()

SET(${PARENT}_SUBMODULE_MSP_PARSER                     ON  CACHE BOOL "" FORCE)
if (${PARENT}_SUBMODULE_MSP_PARSER)
    SET(${PARENT}_MSP_PARSER                           ON  CACHE BOOL "" FORCE)
    SET(${PARENT}_MSP_PARSER_TEST                      OFF CACHE BOOL "" FORCE)
endif()

################################################################################
## INCLUDING SUBDIRECTORIES
## Adding subdirectories according to the 3rd-party configuration
################################################################################
if (${PARENT}_SERIAL_PORT)
    add_subdirectory(SerialPort)
endif()

if (${PARENT}_MSP_PARSER)
    add_subdirectory(MspParser)
endif()