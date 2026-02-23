#include "SerialPort.h"
#include <iostream>
#include <chrono>
#include <functional>

using namespace cr::clib;

SerialPort::SerialPort()
    : m_serialPort(m_ioContext)
{
}

SerialPort::~SerialPort()
{
    close();
}

bool SerialPort::open(const char* port, int baudrate, int timeout)
{
    m_timeout = (timeout > 0) ? timeout : 100;

    try
    {
        m_serialPort.open(std::string(port));

        m_serialPort.set_option(asio::serial_port_base::baud_rate(baudrate));
        m_serialPort.set_option(asio::serial_port_base::character_size(8));
        m_serialPort.set_option(asio::serial_port_base::parity(
            asio::serial_port_base::parity::none));
        m_serialPort.set_option(asio::serial_port_base::stop_bits(
            asio::serial_port_base::stop_bits::one));
        m_serialPort.set_option(asio::serial_port_base::flow_control(
            asio::serial_port_base::flow_control::none));
    }
    catch (const asio::system_error& e)
    {
        std::cerr << "SerialPort::open error: " << e.what() << std::endl;
        return false;
    }

    return true;
}

void SerialPort::close()
{
    if (m_serialPort.is_open())
    {
        try
        {
            m_serialPort.close();
        }
        catch (const asio::system_error& e)
        {
            std::cerr << "SerialPort::close error: " << e.what() << std::endl;
        }
    }
}

bool SerialPort::isOpen() const
{
    return m_serialPort.is_open();
}

int SerialPort::read(uint8_t* buffer, uint32_t size)
{
    if (!m_serialPort.is_open())
        return -1;

    // Use async_read_some with a deadline timer to implement timeout.
    asio::steady_timer timer(m_ioContext);
    int bytesRead = 0;
    bool readComplete = false;
    bool timedOut = false;

    // Reset the io_context so it can be reused.
    m_ioContext.restart();

    // Start async read.
    m_serialPort.async_read_some(
        asio::buffer(buffer, size),
        [&](const asio::error_code& ec, std::size_t length)
        {
            readComplete = true;
            timer.cancel();
            if (!ec)
                bytesRead = static_cast<int>(length);
            else
                bytesRead = -1;
        });

    // Start deadline timer.
    timer.expires_after(std::chrono::milliseconds(m_timeout));
    timer.async_wait(
        [&](const asio::error_code& ec)
        {
            if (!ec && !readComplete)
            {
                // Timer expired before read completed — cancel the read.
                timedOut = true;
                m_serialPort.cancel();
            }
        });

    // Run the io_context until both handlers have been called.
    m_ioContext.run();

    if (timedOut)
        return 0;

    return bytesRead;
}

bool SerialPort::write(const uint8_t* buffer, uint32_t size)
{
    if (!m_serialPort.is_open())
        return false;

    try
    {
        asio::write(m_serialPort, asio::buffer(buffer, size));
    }
    catch (const asio::system_error& e)
    {
        std::cerr << "SerialPort::write error: " << e.what() << std::endl;
        return false;
    }

    return true;
}
