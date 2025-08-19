/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <string>
#include <iostream>
#include <cstdio>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

void my_sleep(unsigned long milliseconds) {
#ifdef _WIN32
      Sleep(milliseconds); // 100 ms
#else
      usleep(milliseconds*1000); // 100 ms
#endif
}

void enumerate_ports()
{
	vector<serial::PortInfo> devices_found = serial::list_ports();

	vector<serial::PortInfo>::iterator iter = devices_found.begin();

	while( iter != devices_found.end() )
	{
		serial::PortInfo device = *iter++;

		printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
     device.hardware_id.c_str() );
	}
}

void print_usage()
{
	cerr << "Usage: test_serial {-e|<serial port address>} ";
    cerr << "<baudrate> [test string]" << endl;
}

int main(int argc, char **argv)
{
  enumerate_ports();

  std::string dev = argv[1];

  // port, baudrate, timeout in milliseconds
  // TODO: Change the timeout here
  serial::Serial my_serial(dev, 112500, serial::Timeout::simpleTimeout(20));
  my_serial.setFlowcontrol(serial::flowcontrol_none);
  // my_serial.setFlowcontrol(serial::flowcontrol_software);
  // my_serial.setFlowcontrol(serial::flowcontrol_hardware);

  std::cout << "Is the serial port open?" << std::endl;
  if(my_serial.isOpen())
  {
    std::cout << " Yes." << std::endl;
  }
  else
  {
    std::cout << " No." << std::endl;
    return 0;
  }

  // Test the timeout, there should be 1 second between prints
  while (true)
  {
    string result = my_serial.read(255);
    std::cout << "--------------------" << std::endl;
    std::cout << "readData: " <<  result << endl;
  }

  return 0;
}
