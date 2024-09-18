/********************************************************************************
 * @file    serial_port_handler.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
 * @brief   Configure and enable serial ports for simulated and real serial
 *          data.  TCP is used for SITL and UART is used for the hardware drone.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "serial_comm.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/
#ifdef USE_UART

#ifdef JETSON_B01

#define SERIAL_PORT "/dev/ttyTHS1"
#define BAUD_RATE B115200

#elif WIN32_HARD

#define SERIAL_PORT "COM3"  // Modify based on your USB port (e.g., COM3, COM4, etc.)
#define BAUD_RATE CBR_57600 // 57600 for SiK radio, CBR_115200 typically

#else

#error "Please define JETSON_B01 or WIN32_HARD."

#endif // end JETSON_B01

#elif USE_TCP

/* Sim port/address for linux or windows */
#define SIM_IP "127.0.0.1" // IP address for SITL simulation
#define SIM_PORT 5762      // Port number for SITL

#else

#error "Please define USE_UART or USE_TCP."

#endif // end USE_UART

/********************************************************************************
 * Object definitions
 ********************************************************************************/
#ifdef USE_UART

#ifdef JETSON_B01

int uart_fd; // UART file descriptor for Linux
int serial_port = 0;

#elif WIN32_HARD

HANDLE hComm; // Handle for COM port on Windows

#else

#error "Please define JETSON_B01 or WIN32_HARD."

#endif // end JETSON_B01

#elif USE_TCP

SOCKET tcp_socket_fd; // TCP socket descriptor
struct sockaddr_in sim_addr;
int sock;

#else

#error "Please define USE_UART or USE_TCP."

#endif // end USE_UART

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: SerialComm
 * Description: Class constructor
 ********************************************************************************/
SerialComm::SerialComm(void) {}

/********************************************************************************
 * Function: ~SerialComm
 * Description: Class destructor
 ********************************************************************************/
SerialComm::~SerialComm(void) {}

/********************************************************************************
 * Function: start_uart_comm
 * Description: Configure and make available the serial port for UART or TCP comms.
 ********************************************************************************/
bool SerialComm::start_uart_comm(void)
{
#ifdef USE_UART

#ifdef JETSON_B01

    // Open the UART port for Linux
    serial_port = open(SERIAL_PORT, O_RDWR);
    if (serial_port < 0)
    {
        fprintf(stderr, "Error starting serial comms\n");
        return false;
    }

    // Configure the serial port
    struct termios serial_config;
    tcgetattr(serial_port, &serial_config);
    serial_config.c_cflag = BAUD_RATE | CS8 | CLOCAL | CREAD;
    serial_config.c_iflag = IGNPAR;
    serial_config.c_oflag = 0;
    serial_config.c_lflag = 0;
    tcflush(serial_port, TCIFLUSH);
    tcsetattr(serial_port, TCSANOW, &serial_config);

    uart_fd = serial_port;

    return true;

#elif WIN32_HARD

    // Open the COM port
    hComm = CreateFile(SERIAL_PORT, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (hComm == INVALID_HANDLE_VALUE)
    {
        DWORD errCode = GetLastError();
        fprintf(stderr, "Error opening COM port. Error code: %lu\n", errCode);
        exit(EXIT_FAILURE); // Terminate the program with failure status
        return false;
    }

    // Configure the COM port
    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hComm, &dcbSerialParams))
    {
        fprintf(stderr, "Error getting COM port state\n");
        exit(EXIT_FAILURE); // Terminate the program with failure status
        return false;
    }

    dcbSerialParams.BaudRate = BAUD_RATE;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(hComm, &dcbSerialParams))
    {
        fprintf(stderr, "Error setting COM port state\n");
        return false;
    }

    // Set timeouts for the serial port
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;         // Maximum time between two bytes, in ms
    timeouts.ReadTotalTimeoutConstant = 50;    // Total timeout constant, in ms
    timeouts.ReadTotalTimeoutMultiplier = 10;  // Per-byte timeout multiplier, in ms
    timeouts.WriteTotalTimeoutConstant = 50;   // Write timeout constant, in ms
    timeouts.WriteTotalTimeoutMultiplier = 10; // Per-byte write timeout multiplier, in ms

    if (!SetCommTimeouts(hComm, &timeouts))
    {
        fprintf(stderr, "Error setting COM port timeouts\n");
        return false;
    }

    return true;

#else

#error "Please define JETSON_B01 or WIN32_HARD."

#endif // end JETSON_B01

#elif USE_TCP

#ifdef WIN32_SIM

    // Initialize WinSock (Windows only)
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        fprintf(stderr, "WSAStartup failed\n");
        return false;
    }

#endif

    // Create a socket for the TCP connection - common for windows and linux
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
        fprintf(stderr, "Error creating socket\n");
        return false;
    }

    // Configure the address for the TCP connection
    sim_addr.sin_family = AF_INET;
    sim_addr.sin_addr.s_addr = inet_addr(SIM_IP);
    sim_addr.sin_port = htons(SIM_PORT);

    // Connect to the SITL simulation
    if (connect(sock, (struct sockaddr *)&sim_addr, sizeof(sim_addr)) < 0)
    {
        fprintf(stderr, "Error connecting to SITL\n");
        return false;
    }

    // Set the socket to non-blocking mode

#ifdef WIN32_SIM

    u_long mode = 1;
    ioctlsocket(sock, FIONBIO, &mode);

#else

    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

#endif

    tcp_socket_fd = sock;                                     // Ensure the global file descriptor is set
    printf("tcp_socket_fd initialized: %d\n", tcp_socket_fd); // Debug log

    return true;

#else

#error "Please define USE_UART or USE_TCP."

#endif // end USE_UART
}

/********************************************************************************
 * Function: send_uart
 * Description: Send a buffer of bytes on the UART serial port or TCP connection.
 ********************************************************************************/
void SerialComm::write_uart(mavlink_message_t &msg)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = 0;

    offset_buffer(buffer, len, msg);

#ifdef USE_UART

#ifdef JETSON_B01

    uint16_t n = write(serial_port, buffer, len);
    if (n < 0)
    {
        fprintf(stderr, "Error writing to serial port\n");
    }

#elif WIN32_HARD

    DWORD bytesWritten;
    if (!WriteFile(hComm, buffer, len, &bytesWritten, NULL))
    {
        fprintf(stderr, "Error writing to COM port\n");
    }

#else

#error "Please define JETSON_B01 or WIN32_HARD."

#endif // end JETSON_B01

#elif USE_TCP

    int n = send(tcp_socket_fd, (const char *)buffer, len, 0);
    if (n == SOCKET_ERROR)
    {
        fprintf(stderr, "Error writing to TCP socket\n");
    }

#else

#error "Please define USE_UART or USE_TCP."

#endif // end USE_UART

    clear_buffer(buffer, len);
}

/********************************************************************************
 * Function: read_uart
 * Description: Read some bytes from the UART serial port or TCP connection.
 ********************************************************************************/
uint8_t SerialComm::read_uart(void)
{
    uint8_t byte = 0;
    int n = 0; // Declare n to avoid undeclared identifier error

#ifdef USE_UART

#ifdef JETSON_B01

    if (read(serial_port, &byte, 1) == -1)
    {
        fprintf(stderr, "Error reading from serial port\n");
    }

#elif WIN32_HARD

    if (hComm == INVALID_HANDLE_VALUE)
    {
        DWORD errCode = GetLastError();
        fprintf(stderr, "Error opening COM port. Error code: %lu\n", errCode);
        return false;
    }

    DWORD bytesRead;
    if (!ReadFile(hComm, &byte, 1, &bytesRead, NULL))
    {
        DWORD errCode = GetLastError();
        fprintf(stderr, "Error reading from COM port. Error code: %lu\n", errCode);
        exit(EXIT_FAILURE); // Terminate the program with failure status
    }

#else

#error "Please define JETSON_B01 or WIN32_HARD."

#endif // end JETSON_B01

#elif USE_TCP

    n = recv(tcp_socket_fd, (char *)&byte, sizeof(byte), 0);
    if (n < 0)
    {
        fprintf(stderr, "Error reading from TCP port\n");
    }

#endif // end USE_UART

    return byte;
}

/********************************************************************************
 * Function: stop_uart_comm
 * Description: Close the serial port or TCP connection.
 ********************************************************************************/
void SerialComm::stop_uart_comm(void)
{
#ifdef USE_UART

#ifdef JETSON_B01

    close(serial_port);

#elif WIN32_HARD

    CloseHandle(hComm);

#else

#error "Please define JETSON_B01 or WIN32_HARD."

#endif // end JETSON_B01

#elif USE_TCP

#ifdef WIN32_SIM

    closesocket(tcp_socket_fd);
    WSACleanup();

#else

    close(tcp_socket_fd);

#endif

#else

#error "Please define USE_UART or USE_TCP."

#endif // end USE_UART
}

/********************************************************************************
 * Function: bytes_available
 * Description: Check how many bytes are available for reading.
 ********************************************************************************/
int SerialComm::bytes_available(void)
{
    int bytes = 0;

#ifdef USE_UART

#ifdef JETSON_B01

    if (ioctl(uart_fd, FIONREAD, &bytes) == -1)
    {
        perror("ioctl");
        return -1;
    }

#elif WIN32_HARD

    COMSTAT status;
    DWORD errors;
    ClearCommError(hComm, &errors, &status);
    bytes = status.cbInQue;

#else

#error "Please define JETSON_B01 or WIN32_HARD."

#endif // end JETSON_B01

#elif USE_TCP

#ifdef WIN32_SIM

    u_long availableBytes = 0;
    ioctlsocket(tcp_socket_fd, FIONREAD, &availableBytes);
    bytes = (int)availableBytes;

#else

    if (ioctl(tcp_socket_fd, FIONREAD, &bytes) == -1)
    {
        perror("ioctl");
        return -1;
    }

#endif

#else

#error "Please define USE_UART or USE_TCP."

#endif // end USE_UART

    return bytes;
}

/********************************************************************************
 * Function: offset_buffer
 * Description: Offset the buffer by its length to avoid overlapping buffers.
 ********************************************************************************/
void SerialComm::offset_buffer(uint8_t *buffer, uint16_t &len, mavlink_message_t &msg)
{
    len = len + mavlink_msg_to_send_buffer(&buffer[len], &msg);
}

/********************************************************************************
 * Function: clear_buffer
 * Description: Clear the buffer.
 ********************************************************************************/
void SerialComm::clear_buffer(uint8_t *buffer, uint16_t len)
{
    memset(buffer, 0, len);
}
