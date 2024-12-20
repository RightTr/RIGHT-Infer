#pragma once


#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <cstring>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>


#define COUT_RED_START      std::cout << "\033[1;31m";
#define COUT_GREEN_START    std::cout << "\033[1;32m";
#define COUT_YELLOW_START   std::cout << "\033[1;33m";
#define COUT_BLUE_START     std::cout << "\033[1;34m";
#define COUT_PURPLE_START   std::cout << "\033[1;35m";
#define COUT_CYAN_START     std::cout << "\033[1;36m";
#define COUT_WHITE_START    std::cout << "\033[1;37m";
#define COUT_COLOR_END      std::cout << "\033[0m";

#define RXHEAD1 0xFF
#define RXHEAD2 0xFE
#define RXTAIL1 0xAA
#define RXTAIL2 0xDD


namespace com{

class UART
{
    private:
        const char *dev_name = "/dev/ttyUSB0"; 
        int fd = -1; 
        int flow_ctrl = 1;      
        int databits = 8;        
        int parity = 0;           
        int stopbits = 1;        
        int baudrate = B115200;        
        bool flag = 1;
        int retry = 5;

        struct termios configs = {0};

        int baudrate_arr[7] = {
            B115200,
            B19200,
            B9600,
            B4800,
            B2400,
            B1200,
            B300,
        };
        int flow_ctrl_arr[2] = {0, 1};
        int databits_arr[4] = {5, 6, 7, 8};
        int parity_arr[4] = {0, 1, 2};
        int stopbits_arr[2] = {1, 2};

    public:
        void UART_INIT();
        void UART_OPEN();
        void UART_CLOSE();

        void UART_CONFIG();
        void UART_SET_COM_NAME(std::string in_name);
        void UART_SET_BAUDRATE(int in_baudrate);
        void UART_SET_FLOW_CTRL(int in_flow_ctrl);
        void UART_SET_DATABITS(int in_databits);
        void UART_SET_STOPBITS(int in_stopbits);
        void UART_SET_PARITY(int in_parity);

        void UART_SEND(const uint8_t* buffer_written, size_t length);

        void UART_RECEIVE(uint8_t* buffer_read, size_t length);
        UART();
        ~UART();

};
}