#include <ctlaser_driver/ctlaser_driver.hpp>

CtlaserDriver::CtlaserDriver() :
    laser_ip("0.0.0.0"),
    laser_port(0),
    client_socket(-1),
    connected(false),
    cmd(CtlaserCommand::READ_TEMP_PROCESS),
    is_reading_temp(false)
{
}

void CtlaserDriver::setServerInfo(std::string ip, int port)
{
    laser_ip = ip.c_str();
    laser_port = uint16_t(port);
}

void CtlaserDriver::configureServerAddress()
{
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(laser_port);
    if (inet_pton(AF_INET, laser_ip, &server_address.sin_addr) < 1)
    {
        close(client_socket);
        throw std::runtime_error("[ ERROR] Error configuring server address.");
    }
}

void CtlaserDriver::createSocket()
{
    client_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (client_socket == -1)
        throw std::runtime_error("[ ERROR] Failed to create tcp socket.");

    struct timeval rcv_timeout;
    rcv_timeout.tv_sec = 5;
    rcv_timeout.tv_usec = 0;
    setsockopt(client_socket, SOL_SOCKET, SO_RCVTIMEO, &rcv_timeout, sizeof(rcv_timeout));

    struct timeval snd_timeout;
    snd_timeout.tv_sec = 2;
    snd_timeout.tv_usec = 0;
    setsockopt(client_socket, SOL_SOCKET, SO_SNDTIMEO, &snd_timeout, sizeof(snd_timeout));

    int opt = 1;
    if (setsockopt(client_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
        throw std::runtime_error("[ ERROR] setsockopt(SO_REUSEADDR) failed.");

    linger lin;
    lin.l_onoff = 1;
    lin.l_linger = 0;
    setsockopt(client_socket, SOL_SOCKET, SO_LINGER, (const char *)&lin, sizeof(lin));
}

bool CtlaserDriver::stablishConnection()
{
    createSocket();

    std::cout <<  "[ INFO] Trying to stablish connection to TCP server in Ip "
                << laser_ip << " port " << laser_port << std::endl;

    connection_thread = std::thread([&](void){
        while (!connected)
            if (connect(client_socket, reinterpret_cast<sockaddr*>(&server_address), sizeof (server_address)) == -1)
            {
                std::cerr <<  "[ ERROR] Connection failed. Retrying." << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
            else
            {
                connected = true;
                std::cerr << "[ INFO] Connected to TCP server." << std::endl;
            }
    });
    connection_thread.join();
    return true;
}

void CtlaserDriver::readTemperature()
{
    is_reading_temp = true;

    unsigned char command = cmd;
    unsigned char buffer[2];

    communicate(&command, sizeof(command), buffer, sizeof(buffer));
    current_temperature = calculateTemperature(buffer);

    is_reading_temp = false;
}

double CtlaserDriver::calculateTemperature(unsigned char* bytes)
{
    return (*bytes * 256 + *(bytes + 1) - 1000) / 10.0;
}

double CtlaserDriver::calculateIrValues(unsigned char *bytes)
{
    return (*bytes * 256 + *(bytes + 1)) / 1000.0;
}

void CtlaserDriver::communicate(unsigned char *in, size_t sizeof_in, unsigned char *out, size_t sizeof_out)
{
    if (connected)
    {
        if ( send(client_socket, in, sizeof_in, 0) != -1 )
        {
            memset(out, 0, sizeof_out);
            if ( recv(client_socket, out, sizeof_out, 0) == -1 )
            {
                std::cerr << "[ ERROR] Tcp server didn't respond. Trying to reconnect." << std::endl;
                connected = false;

                close(client_socket);
                stablishConnection();
            }
        }
    }
    else
        std::cerr << "[ ERROR] Client not connected to the server." << std::endl;
}

bool CtlaserDriver::setLaserState(bool on_off)
{
    unsigned char buffer;
    unsigned char msg[3];
    msg[0] = CtlaserCommand::SET_SPOT_LASER;
    msg[1] = on_off ? 1 : 0;
    msg[2] = msg[0] ^ msg[1];

    /*
     * Sending the command, where the first byte it the task
     * the second is the desired value and the third is the
     * checksum
    */
    communicate(msg, sizeof(msg), &buffer, sizeof(buffer));

    /*
     * Testing if the answer (buffer) is equal to the command
    */
    if ( buffer != msg[1] )
    {
        std::cerr << "[ ERROR] Failed to set laser state. "
                     "Pyrometer answered different result." << std::endl;
        return false;
    }
    return true;
}

bool CtlaserDriver::setIrVariables(float value, CtlaserCommand cmd)
{
    if ((cmd != CtlaserCommand::SET_EPSILON) &&
        (cmd != CtlaserCommand::SET_TRANSMISSION))
        return false;

    if (value > 1.0f || value < 0.0f)
    {
        std::cerr << "[ ERROR] Emissivity must by a value from 0.0 to 1.0." << std::endl;
        return false;
    }

    int aux_val = int(value * 1000);

    /*
     * Sending the command, where the first byte is the task
     * the second is the desired value and the third is the
     * checksum.
     * We are braking aux_val into two bytes to insert on the
     * byte array.
    */
    unsigned char msg[4];
    msg[0] = cmd;
    msg[1] = (aux_val >> 8) & 0xFF;
    msg[2] = (aux_val >> 0) & 0xFF;
    msg[3] = msg[0] ^ msg[1] ^ msg[2];

    unsigned char buffer[2];
    communicate(msg, sizeof(msg), buffer, sizeof(buffer));

    if (cmd == CtlaserCommand::SET_EPSILON)
        std::cerr << "[ INFO] Emissivity set to: ";
    else
        std::cerr << "[ INFO] Transmissivity set to: ";
    std::cout << calculateIrValues(buffer) << std::endl;

    return true;

}

CtlaserDriver::~CtlaserDriver()
{
    if (connection_thread.joinable())
        connection_thread.join();

    close(client_socket);
}
