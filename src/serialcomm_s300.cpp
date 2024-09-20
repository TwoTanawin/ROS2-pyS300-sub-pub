#include "s300_ros2/serialcomm_s300.h"
// #include "serialcomm_s300.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <errno.h>
#include <sys/ioctl.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

SerialCommS300::SerialCommS300() : m_rxCount(0), m_ranges(nullptr), m_fd(-1) {}

SerialCommS300::~SerialCommS300() {
  if (m_ranges) {
    delete[] m_ranges;
  }
  disconnect();
}

int SerialCommS300::connect(const std::string &deviceName, unsigned int baudRate) {
  m_fd = ::open(deviceName.c_str(), O_RDWR | O_NOCTTY);
  if (m_fd < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("SerialCommS300"), "Unable to open serial port: %s", deviceName.c_str());
    return -1;
  }
  setFlags();
  if (setBaudRate(baudRateToBaudCode(baudRate))) {
    return -1;
  }
  tcflush(m_fd, TCIOFLUSH);
  return 0;
}

void SerialCommS300::setFlags() {
  struct termios term;
  memset(&term, 0, sizeof(term));
  tcgetattr(m_fd, &term);
  term.c_cc[VMIN] = 0;
  term.c_cc[VTIME] = 5;
  term.c_cflag = CS8 | CREAD | HUPCL | CLOCAL;
  term.c_iflag = INPCK;
  term.c_oflag = 0;
  term.c_lflag = 0;
  tcsetattr(m_fd, TCSANOW, &term);
}

int SerialCommS300::setBaudRate(int baudRate) {
  struct termios term;
  tcgetattr(m_fd, &term);
  cfsetispeed(&term, baudRate);
  cfsetospeed(&term, baudRate);
  tcsetattr(m_fd, TCSAFLUSH, &term);
  return 0;
}

int SerialCommS300::baudRateToBaudCode(int baudRate) {
  switch (baudRate) {
    case 38400:
      return B38400;
    case 115200:
      return B115200;
    case 500000:
      return B500000;
    default:
      return B115200;  // Default baud rate to 115200
  }
}

int SerialCommS300::disconnect() {
  ::close(m_fd);
  return 0;
}

// Implement the readData() function
int SerialCommS300::readData() {
  // Reading logic here
  unsigned char buffer[256];  // Example buffer to hold read data
  int bytesRead = read(m_fd, buffer, sizeof(buffer));  // Example reading from file descriptor

  if (bytesRead < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("SerialCommS300"), "Error reading from serial port");
    return -1;
  }

  // Processing logic for the read data (for example, extracting relevant data from buffer)
  // You might want to process buffer and set m_ranges or m_rxCount accordingly.

  // Return 0 if reading is successful, otherwise return an error code.
  return 0;
}
