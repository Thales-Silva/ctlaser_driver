#ifndef CTLASER_DRIVER_HPP
#define CTLASER_DRIVER_HPP

#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <thread>
#include <cstring>
#include <cmath>
#include <mutex>

#include <ctlaser_driver/ctlaser_words.hpp>

class CtlaserDriver
{

    private:
        const char* laser_ip;
        uint16_t laser_port;

        int client_socket;
        sockaddr_in server_address;

        bool connected;
        CtlaserCommand cmd;

        std::mutex mute;

        std::thread connection_thread;
        bool is_reading_temp;

        double current_temperature;

    public:
        CtlaserDriver();
        ~CtlaserDriver();

        /**
         * \brief This function is used to load the remote tcp server information to the class.
         * \param ip is the remote server ip.
         * \param port is remote server port to communicate with.
        */
        void setServerInfo(std::string ip, int port);

        /**
         * \brief Once the server information is set with setServerInfo(), this function
         * configures the address.
        */
        void configureServerAddress();

        /**
         * \brief This function mearly (re)creates a socket whenever stablish connection is
         * called.
        */
        void createSocket();

        /**
         * \brief This function is used to (re)stablish connection with the remote tcp server.
        */
        bool stablishConnection();

        /**
         * \brief This function starts asks the pyrometer for temperature readings.
        */
        void readTemperature();

        /**
         * \brief This function is used to compute the tempertaure from the incoming bytes.
         * \param bytes is a pointer to the first position of the incoming byte array.
         * \return return the temperature calculated as (byte1*256 + byte2 - 1000) / 10.
        */
        double calculateTemperature(unsigned char* bytes);

        /**
         * \brief This function is used to compute the tempertaure from the incoming bytes.
         * \param bytes is a pointer to the first position of the incoming byte array.
         * \return return the temperature calculated as (byte1*256 + byte2 - 1000) / 10.
        */
        double calculateIrValues(unsigned char* bytes);

        /**
         * \brief This function is used to communicate with the pyrometer,
         * setting parameters and reading information.
         * \param in is a pointer to the input, containing two bytes.
         * \param out is a pointer to the output containing the response with other two bytes.
        */
        void communicate(unsigned char *in, size_t sizeof_in,
                         unsigned char *out, size_t sizeof_out);

        /**
         * \brief This function is used to turn on/off pyrometer lights.
         * \param on_off is a bool setting the lights on(true) or off(false).
        */
        bool setLaserState(bool on_off);

        /**
         * \brief This function is used to set Emissitity and Transmissivity.
         * \param value is must be between 0 and 1.
         * \param cmd must be either CtlaserCommand::SET_EPSILON
         * or CtlaserCommand::SET_TRANSMISSION.
        */
        bool setIrVariables(float value, CtlaserCommand cmd);

        double getCurrentTemperature(){return current_temperature;}
};

#endif

