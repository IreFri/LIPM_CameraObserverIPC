#include <chrono>
#include <thread>

#include "CppLinuxSerial/SerialPort.hpp"

using namespace std::chrono_literals;
using namespace mn::CppLinuxSerial;

int main(int argc, char *argv[])
{
  std::string dev = argv[1];

  // SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE, HardwareFlowControl::OFF, SoftwareFlowControl::OFF);
  // SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE, HardwareFlowControl::OFF, SoftwareFlowControl::ON);
  SerialPort serialPort(dev, BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE, HardwareFlowControl::OFF, SoftwareFlowControl::OFF);
  serialPort.SetTimeout(20);

	serialPort.Open();

  std::this_thread::sleep_for(100ms);

  while(true)
  {
    std::string readData;
    serialPort.Read(readData);
    std::cout << "--------------------" << std::endl;
    std::cout << "readData: " << readData << std::endl;
    // std::cout << "readData: " << std::atoi(readData.c_str()) << std::endl;
  }

  serialPort.Close();
  return 0;
}