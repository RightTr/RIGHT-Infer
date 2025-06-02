#include "uart.hpp"

namespace com{

void UART::UART_OPEN()
{
    fd = open(dev_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1)
    {   
        flag = 0;
        COUT_RED_START
        printf("Open device %s error!\n", dev_name);  
        COUT_COLOR_END      
    }
    else
    {
        COUT_GREEN_START
        printf("Open device %s success!\n", dev_name);
        COUT_COLOR_END
    }
}

void UART::UART_CLOSE()
{
    if(fd != -1)
    {
        close(fd);
        COUT_GREEN_START
        printf("Close device %s success!\n", dev_name);
        COUT_COLOR_END
    }
}

void UART::UART_CONFIG()
{   
    if(flag == 0) return ;

    if(tcgetattr(fd, &configs))
    {   
        flag = 0;
        COUT_RED_START
        printf("Serial config error!\n");
        COUT_COLOR_END
    }

    if(flag == 0) return ;

    cfsetispeed(&configs, baudrate);
    cfsetospeed(&configs, baudrate);

    
    configs.c_cflag |= (CLOCAL | CREAD);
    configs.c_cflag &= ~CSIZE;

    //设置数据位
    switch(databits)
    {
        case 5:
            configs.c_cflag |= CS5;
            break;
        case 6:
            configs.c_cflag |= CS6;
            break;
        case 7:
            configs.c_cflag |= CS7;
            break;
        case 8:
            configs.c_cflag |= CS8;
            break;
    }


    switch (this->parity) 
    {
        case 0: 
            this->configs.c_cflag &= ~PARENB;
            this->configs.c_iflag &= ~INPCK;
            break;
        case 1: 
            this->configs.c_cflag |= (PARODD | PARENB);
            this->configs.c_iflag |= INPCK;
            break;
        case 2: 
            this->configs.c_cflag |= PARENB;
            this->configs.c_cflag &= ~PARODD;
            this->configs.c_iflag |= INPCK;
            break;
    }

    switch (this->stopbits) 
    {
        case 1: 
            this->configs.c_cflag &= ~CSTOPB;
            break;
        case 2: 
            this->configs.c_cflag |= CSTOPB;
            break;
    }
   
    switch(flow_ctrl)
    {
        case 0: 
            configs.c_cflag &= ~CRTSCTS; 
            break;
        case 1: 
            configs.c_cflag |= CRTSCTS; 
            break;
    }


    configs.c_cc[VTIME] = 1; 
    configs.c_cc[VMIN] = 1; 

    tcflush(this->fd,TCIFLUSH); 

    if(tcsetattr(fd, TCSANOW, &configs))
    {
        flag = 0;
        COUT_RED_START
        printf("Com set error!\n");
        COUT_COLOR_END
    }
    else
    {
        flag = 1;
        COUT_GREEN_START
        printf("Com set success!\n");
        COUT_COLOR_END
    }
    
    if(flag == 0) return ;
}

void UART::UART_INIT()
{
    if(flag == 1) 
    {
        COUT_GREEN_START
        printf("Serial port init success!\n");
        COUT_COLOR_END
    }
    UART_OPEN();
    UART_CONFIG();
}

void UART::UART_SET_COM_NAME(std::string in_name)
{
    dev_name = in_name.c_str();
    COUT_GREEN_START
    printf("Set com name success!\n");
    COUT_COLOR_END
}

void UART::UART_SET_BAUDRATE(int in_baudrate)
{
    bool is_valid = 0;
    for(int i = 0; i < sizeof(baudrate_arr)/sizeof(baudrate_arr[0]); i++)
    {
        if(in_baudrate == baudrate_arr[i]) is_valid = 1;
    }
    if(is_valid)
    {
        baudrate = in_baudrate;
        COUT_GREEN_START
        printf("Set baudrate success!\n");
        COUT_COLOR_END
    }
    else
    {
        COUT_RED_START
        printf("Invalid baudrate!\n");
        COUT_COLOR_END
    }
} 

void UART::UART_SET_FLOW_CTRL(int in_flow_ctrl)
{
    bool is_valid = 0;
    for(int i = 0; i < sizeof(flow_ctrl_arr)/sizeof(flow_ctrl_arr[0]); i++)
    {
        if(in_flow_ctrl == flow_ctrl_arr[i]) is_valid = 1;
    }
    if(is_valid)
    {
        flow_ctrl = in_flow_ctrl;
        COUT_GREEN_START
        printf("Set flow_ctrl success!\n");
        COUT_COLOR_END
    }
    else
    {
        COUT_RED_START
        printf("Invalid flow_ctrl!\n");
        COUT_COLOR_END
    }
}

void UART::UART_SET_DATABITS(int in_databits)
{
    bool is_valid = 0;
    for(int i = 0; i < sizeof(databits_arr)/sizeof(databits_arr[0]); i++)
    {
        if(in_databits == databits_arr[i]) is_valid = 1;
    }
    if(is_valid)
    {
        databits = in_databits;
        COUT_GREEN_START
        printf("Set databits success!\n");
        COUT_COLOR_END
    }
    else
    {
        COUT_RED_START
        printf("Invalid databits!\n");
        COUT_COLOR_END
    }
}
void UART::UART_SET_STOPBITS(int in_stopbits)
{
   bool is_valid = 0;
    for(int i = 0; i < sizeof(stopbits_arr)/sizeof(stopbits_arr[0]); i++)
    {
        if(in_stopbits == stopbits_arr[i]) is_valid = 1;
    }
    if(is_valid)
    {
        stopbits = in_stopbits;
        COUT_GREEN_START
        printf("Set stopbits success!\n");
        COUT_COLOR_END
    }
    else
    {
        COUT_RED_START
        printf("Invalid stopbits!\n");
        COUT_COLOR_END
    }
}

void UART::UART_SET_PARITY(int in_parity)
{
    bool is_valid = 0;
    for(int i = 0; i < sizeof(parity_arr)/sizeof(parity_arr[0]); i++)
    {
        if(in_parity == parity_arr[i]) is_valid = 1;
    }
    if(is_valid)
    {
        parity = in_parity;
        COUT_GREEN_START
        printf("Set parity success!\n");
        COUT_COLOR_END
    }
    else
    {
        COUT_RED_START
        printf("Invalid parity!\n");
        COUT_COLOR_END
    }
}


void UART::UART_SEND(const uint8_t* buffer_written, size_t length)
{   
    if(!send_flag)
    {
        send_flag = 1;
        usleep(5000);
        ssize_t bytes_written = write(fd, buffer_written, length);
        usleep(5000);
        send_flag = 0;
        
        if(bytes_written != -1)
        {
            COUT_BLUE_START
            printf("Send %zd bytes success!\n", bytes_written);
            COUT_COLOR_END
        }   
        else
        {
            COUT_RED_START
            printf("Send error!\n");
            COUT_COLOR_END
        }
    }
    else
    {
        return ;
    }
}

void UART::UART_SEND_CLONE(const uint8_t* buffer_written, size_t length)
{   
    if(!send_flag)
    {
        send_flag = 1;
        usleep(5000);
        ssize_t bytes_written = write(fd, buffer_written, length);
        usleep(5000);
        send_flag = 0;

        if(bytes_written != -1)
        {
            COUT_BLUE_START
            printf("Clone Send %zd bytes success!\n", bytes_written);
            COUT_COLOR_END
        }   
        else
        {
            COUT_RED_START
            printf("Clone Send error!\n");
            COUT_COLOR_END
        }
    }
    else
    {
        return ;
    }
}

void UART::UART_RECEIVE()
{
    char getdata[10]={0};
    while(1)
    {
        ssize_t bytes_read = read(fd, getdata, sizeof(getdata)-1);
        if(bytes_read > 0)
        {
            getdata[bytes_read] = '\0';
            COUT_BLUE_START
            printf("Receive %zd bytes success!\n", bytes_read);
            COUT_COLOR_END
        }   
        else if(bytes_read == 0)
        {
            COUT_RED_START
            printf("No data receive!\n");
            COUT_COLOR_END
        }
        else if(bytes_read == -1)
        {
            COUT_RED_START
            printf("Receive error!\n");
            COUT_COLOR_END
            usleep(1000);
        } 
    }
}

UART::UART()
{
    UART_SET_COM_NAME("/dev/ttyUSB0");
    UART_SET_BAUDRATE(B115200);
    UART_SET_DATABITS(8);
    UART_SET_FLOW_CTRL(0);
    UART_SET_PARITY(0);
    UART_SET_STOPBITS(1);

    UART_INIT(); 

}

UART::~UART(){}

}
