#ifndef __SERIALCOMMS300_H__
#define __SERIALCOMMS300_H__

#include <string>
#include <rclcpp/rclcpp.hpp>

#define RX_BUFFER_SIZE 4096
#define DEFAULT_SERIAL_PORT "/dev/ttyUSB0"
#define DEFAULT_BAUD_RATE 115200

class SerialCommS300 {
public:
  SerialCommS300();
  ~SerialCommS300();

  int readData();  // Declare readData() method
  int connect(const std::string &deviceName, unsigned int baudRate = DEFAULT_BAUD_RATE);
  int disconnect();

private:
  void setFlags();
  int setBaudRate(int baudRate);
  int baudRateToBaudCode(int baudRate);

  unsigned char m_rxBuffer[RX_BUFFER_SIZE];
  int m_fd;
  size_t m_rxCount;

  float *m_ranges;
  unsigned int m_rangesCount;
};

#endif // __SERIALCOMMS300_H__
