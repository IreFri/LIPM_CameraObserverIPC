// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <iostream>
#include <chrono>
#include <unordered_map>
#include <algorithm>
#include <vector>

int main()
{
  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  int serial_port = open("/dev/ttyACM1", O_RDWR);

  // Create new termios struct, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  // Allocate memory for read buffer, set size according to your needs
  char read_buf [255];

  auto top_start = std::chrono::high_resolution_clock::now();
  auto bot_start = std::chrono::high_resolution_clock::now();

  auto top_to_bot_start = std::chrono::high_resolution_clock::now();

  std::string concatenated_data = "";

  // https://stackoverflow.com/questions/8888748/how-to-check-if-given-c-string-or-char-contains-only-digits
  auto is_digits = [](const std::string & str)
  {
    return std::all_of(str.begin(), str.end(), ::isdigit);
  };

  auto tokenize = [](std::string s, std::string del = " ") -> std::vector<std::string>
  {
    std::vector<std::string> ret;
    int start, end = -1*del.size();
    do
    {
      start = end + del.size();
      end = s.find(del, start);
      ret.push_back(s.substr(start, end - start));
    }
    while (end != -1);
    return ret;
  };

  while(true)
  {
    // Reset data in read_buf
    memset(&read_buf, '\0', sizeof(read_buf));

    const int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

    if(num_bytes <= 0)
    {
      // nothing to do here
      continue;
    }

    for(size_t i = 0; i < num_bytes; ++i)
    {
      concatenated_data += read_buf[i];
      if(concatenated_data.back() == '\n')
      {
        // Stop the clock to measure time
        const auto end = std::chrono::high_resolution_clock::now();
        // Remove the last 2 characters
        if(!(concatenated_data.find("bot") != std::string::npos || concatenated_data.find("top") != std::string::npos))
        {
          concatenated_data = "";
          continue;
        }
        concatenated_data.erase(concatenated_data.size() - 2);
        // Tokenized
        const std::vector<std::string> tokens = tokenize(concatenated_data, ":");
        // Get which sensor and data
        const std::string & who = tokens[0];
        const std::string & data = tokens[1];
        // Measure elapsed time
        const double ms = std::chrono::duration<double, std::milli>(end - (who == "bot" ? bot_start : top_start)).count();

        std::cout << "from: " << who;
        if(is_digits(data))
        {
          const double value = std::stod(data);
          std::cout << " value: " << value;
        }
        else
        {
          std::cout << " error: " << data;
        }
        std::cout << " elapsed time: " << ms;
        std::cout << " Hz: " << 1. / (ms * 0.001);

        // We gather enough data
        if(who == "bot")
        {
          bot_start = std::chrono::high_resolution_clock::now();
          top_to_bot_start = std::chrono::high_resolution_clock::now();
        }

        if(who == "top")
        {
          const double top_to_bot_ms = std::chrono::duration<double, std::milli>(end - top_to_bot_start).count();
          std::cout << " delta ms between sensor: " << top_to_bot_ms;
          top_start = std::chrono::high_resolution_clock::now();
        }
        std::cout << std::endl;
        concatenated_data = "";
      }
    }
  }

  close(serial_port);
  return 0; // success
};