#pragma once

#include <cstdint>
#include <string>
#include <memory>

#include <asio.hpp>
#include <asio/serial_port.hpp>

namespace cr
{
namespace clib
{

/**
 * @brief Serial port class based on ASIO.
 */
class SerialPort
{
public:
    SerialPort();
    ~SerialPort();

    /**
     * @brief Open the serial port.
     * @param port The port name (e.g. "/dev/ttyUSB0" or "COM1").
     * @param baudrate The baud rate.
     * @param timeout Read timeout in milliseconds (0 = non-blocking read).
     * @return True if the port was opened successfully.
     */
    bool open(const char* port, int baudrate, int timeout);

    /**
     * @brief Close the serial port.
     */
    void close();

    /**
     * @brief Check if the serial port is open.
     * @return True if the port is open.
     */
    bool isOpen() const;

    /**
     * @brief Read data from the serial port.
     * @param buffer The buffer to store the data.
     * @param size The maximum number of bytes to read.
     * @return The number of bytes read, or -1 on error.
     */
    int read(uint8_t* buffer, uint32_t size);

    /**
     * @brief Write data to the serial port.
     * @param buffer The data to write.
     * @param size The number of bytes to write.
     * @return True if all bytes were written successfully.
     */
    bool write(const uint8_t* buffer, uint32_t size);

private:
    /// ASIO I/O context.
    asio::io_context m_ioContext;
    /// ASIO serial port.
    asio::serial_port m_serialPort;
    /// Read timeout in milliseconds.
    int m_timeout{100};
};

} // namespace clib
} // namespace cr
