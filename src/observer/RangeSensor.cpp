#include <lipm_walking/observer/RangeSensor.h>
#include <mc_observers/ObserverMacros.h>
#include <mc_rtc/version.h>
#include <mc_control/MCController.h>
#include <mc_mujoco/devices/RangeSensor.h>
#include <chrono>

#include <boost/filesystem.hpp>

extern "C"
{
#include "usbio.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
}

namespace
{

int toBaudrate(const std::string& str)
{
  if(str == "B0")
  {
    return B0;
  }
  else if(str == "B50")
  {
    return B50;
  }
  else if(str == "B75")
  {
    return B75;
  }
  else if(str == "B110")
  {
    return B110;
  }
  else if(str == "B134")
  {
    return B134;
  }
  else if(str == "B150")
  {
    return B150;
  }
  else if(str == "B200")
  {
    return B200;
  }
  else if(str == "B300")
  {
    return B300;
  }
  else if(str == "B600")
  {
    return B600;
  }
  else if(str == "B1200")
  {
    return B1200;
  }
  else if(str == "B1800")
  {
    return B1800;
  }
  else if(str == "B2400")
  {
    return B2400;
  }
  else if(str == "B4800")
  {
    return B4800;
  }
  else if(str == "B9600")
  {
    return B9600;
  }
  else if(str == "B19200")
  {
    return B19200;
  }
  else if(str == "B38400")
  {
    return B38400;
  }
  else if(str == "B57600")
  {
    return B57600;
  }
  else if(str == "B115200")
  {
    return B115200;
  }
  else if(str == "B230400")
  {
    return B230400;
  }
  else if(str == "B460800")
  {
    return B460800;
  }

  return 0;
}

std::string fromBaudrate(int bd)
{
  if(bd == B0)
  {
    return "B0";
  }
  else if(bd == B50)
  {
    return "B50";
  }
  else if(bd == B75)
  {
    return "B75";
  }
  else if(bd == B110)
  {
    return "B110";
  }
  else if(bd == B134)
  {
    return "B134";
  }
  else if(bd == B150)
  {
    return "B150";
  }
  else if(bd == B200)
  {
    return "B200";
  }
  else if(bd == B300)
  {
    return "B300";
  }
  else if(bd == B600)
  {
    return "B600";
  }
  else if(bd == B1200)
  {
    return "B1200";
  }
  else if(bd == B1800)
  {
    return "B1800";
  }
  else if(bd == B2400)
  {
    return "B2400";
  }
  else if(bd == B4800)
  {
    return "B4800";
  }
  else if(bd == B9600)
  {
    return "B9600";
  }
  else if(bd == B19200)
  {
    return "B19200";
  }
  else if(bd == B38400)
  {
    return "B38400";
  }
  else if(bd == B57600)
  {
    return "B57600";
  }
  else if(bd == B115200)
  {
    return "B115200";
  }
  else if(bd == B230400)
  {
    return "B230400";
  }
  else if(bd == B460800)
  {
    return "B460800";
  }

  return "Undefined";
}

}

namespace lipm_walking
{

RangeSensor::RangeSensor(const std::string & type, double dt)
: mc_observers::Observer(type, dt)
{
}

void RangeSensor::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_name_ = config("robot", ctl.robot().name());

  if(config.has("serial_port"))
  {
    serial_port_name_ = static_cast<std::string>(config("serial_port"));
  }
  else
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("[RangeSensor::{}] 'serial_port' is mandatory in the configuration.", name_);
  }

  if(config.has("baudrate"))
  {
    const std::string str_baudrate = static_cast<std::string>(config("baudrate"));
    baudrate_ = toBaudrate(str_baudrate);
    if(baudrate_ == 0)
    {
      mc_rtc::log::error_and_throw<std::invalid_argument>("[RangeSensor::{}] 'baudrate' {} is not a correct value.", name_, str_baudrate);
    }
  }
  else
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("[RangeSensor::{}] 'baudrate' is mandatory in the configuration.", name_);
  }

  if(config.has("range_sensor_top"))
  {
    top_sensor_.name = static_cast<std::string>(config("range_sensor_top"));
  }
  else
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("[RangeSensor::{}] 'range_sensor_top' is mandatory in the configuration.", name_);
  }

  if(config.has("range_sensor_bot"))
  {
    bot_sensor_.name = static_cast<std::string>(config("range_sensor_bot"));
  }
  else
  {
    mc_rtc::log::warning("[RangeSensor::{}] 'range_sensor_bot' is not defined in the configuration; only top sensor {} will be used", name_, top_sensor_.name);
  }

  mc_rtc::log::info("[RangeSensor::{}] 'serial_port' is {}", name_, serial_port_name_);
  mc_rtc::log::info("[RangeSensor::{}] 'top_range_sensor' is {}", name_, top_sensor_.name);
  mc_rtc::log::info("[RangeSensor::{}] 'bot_range_sensor' is {}", name_, bot_sensor_.name);

  startReadingDevice();

  desc_ = fmt::format("{} (top_range_sensor={}, bot_range_sensor={}, serial={})", name_, top_sensor_.name, bot_sensor_.name, serial_port_name_);
}

void RangeSensor::reset(const mc_control::MCController &)
{
  // Nothing to do here
}

bool RangeSensor::run(const mc_control::MCController &)
{
  // Nothing to do here
  return true;
}

void RangeSensor::update(mc_control::MCController & ctl)
{
  t_ += ctl.solver().dt();
  if(serial_port_is_open_)
  {
    if(top_sensor_.new_data.load())
    {
      ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(top_sensor_.name).update(top_sensor_.range.load(), top_sensor_.measured_time.load() * 0.5 * 0.001);
      top_sensor_.new_data = false;
    }

    if(bot_sensor_.new_data.load())
    {
      ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(bot_sensor_.name).update(bot_sensor_.range.load(), bot_sensor_.measured_time.load() * 0.5 * 0.001);
      bot_sensor_.new_data = false;
    }
  }
}

void RangeSensor::addToLogger(const mc_control::MCController & ctl,
                                   mc_rtc::Logger & logger,
                                   const std::string & category)
{
  logger.addLogEntry(category + "_top_range", [this, &ctl]() -> double {
    return top_sensor_.range;
  });
  logger.addLogEntry(category + "_top_time", [this, &ctl]() -> double {
    return top_sensor_.measured_time;
  });
  logger.addLogEntry(category + "_top_Hz", [this, &ctl]() -> double {
    return top_sensor_.measured_Hz;
  });

  if(bot_sensor_.name != "")
  {
    logger.addLogEntry(category + "_bot_range", [this, &ctl]() -> double {
      return bot_sensor_.range;
    });
    logger.addLogEntry(category + "_bot_time", [this, &ctl]() -> double {
      return bot_sensor_.measured_time;
    });
    logger.addLogEntry(category + "_bot_Hz", [this, &ctl]() -> double {
      return bot_sensor_.measured_Hz;
    });
  }
}

void RangeSensor::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_range");
  logger.removeLogEntry(category + "_timestep");
}

void RangeSensor::addToGUI(const mc_control::MCController & ctl,
                                mc_rtc::gui::StateBuilder & gui,
                                const std::vector<std::string> & category)
{
  gui.addElement(category,
    mc_rtc::gui::Label("Serial port", [this](){ return (serial_port_is_open_ ? "Open" : "Close"); }),
    mc_rtc::gui::ComboInput("Serial port selection",
      {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3", "/dev/ttyUSB4", "/dev/ttyUSB5", "/dev/ttyUSB6"},
      [this]() -> std::string
      {
        return serial_port_name_;
      },
      [this](const std::string & s)
      {
        serial_port_name_ = s;

        mc_rtc::log::info("[RangeSensor::{}] Changed serial port to {} ", name_, serial_port_name_);

        if(serial_port_is_open_)
        {
          is_reading_ = false;
          loop_sensor_.join();
          serial_port_is_open_ = false;
        }

        startReadingDevice();
      }),
    mc_rtc::gui::ComboInput("Baudrate selection",
      {"B0", "B50", "B75", "B110", "B134", "B150", "B200", "B300", "B600", "B1200", "B1800", "B2400", "B4800", "B9600", "B19200", "B38400", "B57600", "B115200", "B230400", "B460800"},
      [this]() -> std::string
      {
        return fromBaudrate(baudrate_);
      },
      [this](const std::string & s)
      {
        baudrate_ = toBaudrate(s);
        mc_rtc::log::info("[RangeSensor::{}] Changed baudrate to {} ", name_, s);

        if(serial_port_is_open_)
        {
          is_reading_ = false;
          loop_sensor_.join();
          serial_port_is_open_ = false;
        }
        startReadingDevice();
      }),
    mc_rtc::gui::Button("Open",
      [this]()
      {
        if(!serial_port_is_open_)
        {
          startReadingDevice();
        }
        else
        {
          mc_rtc::log::warning("[RangeSensor::{}] The device is already opened", name_);
        }
      }),
    mc_rtc::gui::Button("Close",
      [this]()
      {
        if(serial_port_is_open_)
        {
          is_reading_ = false;
          loop_sensor_.join();
          serial_port_is_open_ = false;
        }
        else
        {
          mc_rtc::log::warning("[RangeSensor::{}] The device is already closed", name_);
        }
      }),
    mc_rtc::gui::Checkbox("Debug output", [this]() { return debug_.load(); }, [this]() { debug_ = !debug_.load(); })
  );

  if(bot_sensor_.name != "")
    {
      gui.addElement(category,
        mc_rtc::gui::Label("Delay between 2 sensors [ms]",
          [this]()
          {
            return measured_delay_between_sensor_.load();
          })
      );
    }

  auto top_category = category;
  top_category.push_back("top");

  gui.addElement(top_category,
    mc_rtc::gui::Label("Range [m]",
      [this]()
      {
        return (serial_port_is_open_ ? std::to_string(top_sensor_.range.load()) : "Sensor is not opened");
      }),
    mc_rtc::gui::Label("Elapsed time [ms]",
      [this]()
      {
        return top_sensor_.measured_time.load();
      }),
    mc_rtc::gui::Label("Frequency [Hz]",
      [this]()
      {
        return top_sensor_.measured_Hz.load();
      }),
    mc_rtc::gui::Label("Error",
      [this]()
      {
        return top_sensor_.error;
      }),
    mc_rtc::gui::Transform(fmt::format("X_0_{}", name_),
      [this, &ctl]()
      {
        const std::string & body_name = ctl.robot().device<mc_mujoco::RangeSensor>(top_sensor_.name).parent();
        const sva::PTransformd X_0_p = ctl.realRobot().bodyPosW(body_name);
        return ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(top_sensor_.name).X_p_s() * X_0_p;
      }),
    mc_rtc::gui::ArrayInput("Rotation [deg]", {"r", "p", "y"},
      [this, &ctl]() -> Eigen::Vector3d
      {
        return mc_rbdyn::rpyFromMat(ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(top_sensor_.name).X_p_s().rotation()).unaryExpr([](double x){return x * 180. / M_PI;});
      },
      [this, &ctl](const Eigen::Vector3d & new_rpy)
      {
        const sva::PTransformd current_X_p_s = ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(top_sensor_.name).X_p_s();
        const sva::PTransformd new_X_p_s(mc_rbdyn::rpyToMat(new_rpy.unaryExpr([](double x){return x * M_PI / 180.;})), current_X_p_s.translation());
        const_cast<mc_control::MCController &>(ctl).robot(robot_name_).device<mc_mujoco::RangeSensor>(top_sensor_.name).X_p_s(new_X_p_s);
      }),
    mc_rtc::gui::ArrayInput("Translation", {"x", "y", "z"},
      [this, &ctl]()
      {
        return ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(top_sensor_.name).X_p_s().translation();
      },
      [this, &ctl](const Eigen::Vector3d & new_translation)
      {
        const sva::PTransformd current_X_p_s = ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(top_sensor_.name).X_p_s();
        const sva::PTransformd new_X_p_s(current_X_p_s.rotation(), new_translation);
        const_cast<mc_control::MCController &>(ctl).robot(robot_name_).device<mc_mujoco::RangeSensor>(top_sensor_.name).X_p_s(new_X_p_s);
      }),
    mc_rtc::gui::Transform("X_p_s",
      [this, &ctl]()
      {
        const std::string & body_name = ctl.robot().device<mc_mujoco::RangeSensor>(top_sensor_.name).parent();
        const sva::PTransformd X_0_p = ctl.realRobot().bodyPosW(body_name);
        return ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(top_sensor_.name).X_p_s() * X_0_p;
      },
      [this, &ctl](const sva::PTransformd & X_0_s)
      {
        const std::string & body_name = ctl.robot().device<mc_mujoco::RangeSensor>(top_sensor_.name).parent();
        const sva::PTransformd X_0_p = ctl.realRobot().bodyPosW(body_name);
        const sva::PTransformd new_X_p_s = X_0_s * X_0_p.inv();
        const_cast<mc_control::MCController &>(ctl).robot(robot_name_).device<mc_mujoco::RangeSensor>(top_sensor_.name).X_p_s(new_X_p_s);
      })
  );

  auto bot_category = category;
  bot_category.push_back("bot");

  if(bot_sensor_.name != "")
  {
    gui.addElement(bot_category,
      mc_rtc::gui::Label("Range [m]",
        [this]()
        {
          return (serial_port_is_open_ ? std::to_string(bot_sensor_.range.load()) : "Sensor is not opened");
        }),
      mc_rtc::gui::Label("Elapsed time [ms]",
        [this]()
        {
          return bot_sensor_.measured_time.load();
        }),
      mc_rtc::gui::Label("Frequency [Hz]",
        [this]()
        {
          return bot_sensor_.measured_Hz.load();
        }),
      mc_rtc::gui::Label("Error",
        [this]()
        {
          return bot_sensor_.error;
        }),
        mc_rtc::gui::Transform(fmt::format("X_0_{}", name_),
        [this, &ctl]()
        {
          const std::string & body_name = ctl.robot().device<mc_mujoco::RangeSensor>(bot_sensor_.name).parent();
          const sva::PTransformd X_0_p = ctl.realRobot().bodyPosW(body_name);
          return ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(bot_sensor_.name).X_p_s() * X_0_p;
        }),
      mc_rtc::gui::ArrayInput("Rotation [deg]", {"r", "p", "y"},
        [this, &ctl]() -> Eigen::Vector3d
        {
          return mc_rbdyn::rpyFromMat(ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(bot_sensor_.name).X_p_s().rotation()).unaryExpr([](double x){return x * 180. / M_PI;});
        },
        [this, &ctl](const Eigen::Vector3d & new_rpy)
        {
          const sva::PTransformd current_X_p_s = ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(bot_sensor_.name).X_p_s();
          const sva::PTransformd new_X_p_s(mc_rbdyn::rpyToMat(new_rpy.unaryExpr([](double x){return x * M_PI / 180.;})), current_X_p_s.translation());
          const_cast<mc_control::MCController &>(ctl).robot(robot_name_).device<mc_mujoco::RangeSensor>(bot_sensor_.name).X_p_s(new_X_p_s);
        }),
      mc_rtc::gui::ArrayInput("Translation", {"x", "y", "z"},
        [this, &ctl]()
        {
          return ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(bot_sensor_.name).X_p_s().translation();
        },
        [this, &ctl](const Eigen::Vector3d & new_translation)
        {
          const sva::PTransformd current_X_p_s = ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(bot_sensor_.name).X_p_s();
          const sva::PTransformd new_X_p_s(current_X_p_s.rotation(), new_translation);
          const_cast<mc_control::MCController &>(ctl).robot(robot_name_).device<mc_mujoco::RangeSensor>(bot_sensor_.name).X_p_s(new_X_p_s);
        }),
      mc_rtc::gui::Transform("X_p_s",
        [this, &ctl]()
        {
          const std::string & body_name = ctl.robot().device<mc_mujoco::RangeSensor>(bot_sensor_.name).parent();
          const sva::PTransformd X_0_p = ctl.realRobot().bodyPosW(body_name);
          return ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(bot_sensor_.name).X_p_s() * X_0_p;
        },
        [this, &ctl](const sva::PTransformd & X_0_s)
        {
          const std::string & body_name = ctl.robot().device<mc_mujoco::RangeSensor>(bot_sensor_.name).parent();
          const sva::PTransformd X_0_p = ctl.realRobot().bodyPosW(body_name);
          const sva::PTransformd new_X_p_s = X_0_s * X_0_p.inv();
          const_cast<mc_control::MCController &>(ctl).robot(robot_name_).device<mc_mujoco::RangeSensor>(bot_sensor_.name).X_p_s(new_X_p_s);
        })
    );
  }

  gui.addPlot(
    fmt::format("RangeSensor::{}", name_),
    mc_rtc::gui::plot::X("t", [this, &ctl]() { return t_; }),
    mc_rtc::gui::plot::Y( "top_range", [this]() { return top_sensor_.range.load(); }, mc_rtc::gui::Color::Red),
    mc_rtc::gui::plot::Y( "bot_range", [this]() { return bot_sensor_.range.load(); }, mc_rtc::gui::Color::Blue)
  );
}

void RangeSensor::startReadingDevice()
{
  if(loop_sensor_.joinable())
  {
    loop_sensor_.join();
  }

  loop_sensor_ = std::thread(
    [this]()
    {
      mc_rtc::log::info("[RangeSensor::{}] Looking for {} ", name_, serial_port_name_);
      auto serial_port = std::string{};
      try
      {
        serial_port = boost::filesystem::canonical(serial_port_name_).string();
      }
      catch(...)
      {
        serial_port_is_open_ = false;
        mc_rtc::log::error("[RangeSensor::{}] Serial port {} is neither a device handle nor a symbolic link", name_, serial_port_name_);
        return;
      }

      int fd = open(serial_port.c_str(), O_RDWR);
      if(fd < 0)
      {
        serial_port_is_open_ = false;
        close(fd);
        mc_rtc::log::error("[RangeSensor::{}] No {} found", name_, serial_port_name_);
        return;
      }
      else
      {
        mc_rtc::log::info("[RangeSensor::{}] Found {}", name_, serial_port_name_);
        serial_port_is_open_ = true;
      }

      // Create new termios struct, we call it 'tty' for convention
      struct termios tty;

      // Read in existing settings, and handle any error
      if(tcgetattr(fd, &tty) != 0)
      {
        mc_rtc::log::error("[[RangeSensor::{}] Error {} from tcgetattr: {}", name_, errno, strerror(errno));
        serial_port_is_open_ = false;
        return;
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
      if (tcsetattr(fd, TCSANOW, &tty) != 0)
      {
        mc_rtc::log::error("[[RangeSensor::{}] Error {} from tcgetattr: {}", name_, errno, strerror(errno));
        serial_port_is_open_ = false;
        return;
      }

      // Allocate memory for read buffer, set size according to your needs
      char read_buf [255];

      auto top_start = std::chrono::high_resolution_clock::now();
      auto bot_start = std::chrono::high_resolution_clock::now();

      auto top_to_bot_start = std::chrono::high_resolution_clock::now();

      std::string concatenated_data = "";

      auto is_digits = [](const std::string & str)
      {
        return std::all_of(str.begin(), str.end(), ::isdigit);
      };

      auto tokenize = [](std::string s, const std::string & del = " ") -> std::vector<std::string>
      {
        if(s.find(del) == std::string::npos)
        {
          return {};
        }
        std::vector<std::string> ret;
        int start, end = -1 * del.size();
        do
        {
          start = end + del.size();
          end = s.find(del, start);
          ret.push_back(s.substr(start, end - start));
        }
        while (end != -1);
        return ret;
      };

      is_reading_ = true;
      while(is_reading_)
      {
        // Reset data in read_buf
        memset(&read_buf, '\0', sizeof(read_buf));

        const int num_bytes = read(fd, &read_buf, sizeof(read_buf));

        if(num_bytes <= 0)
        {
          continue;
        }

        if(debug_.load())
        {
          mc_rtc::log::info("[RangeSensor::{}] The buffer length is {}", name_, num_bytes);
        }

        for(size_t i = 0; i < num_bytes; ++i)
        {
          concatenated_data += read_buf[i];
          if(concatenated_data.back() == '\n')
          {
            if(debug_.load())
            {
              mc_rtc::log::info("[RangeSensor::{}] The buffer data as a string is '{}'", name_, concatenated_data);
            }
            // Stop the clock to measure time
            const auto end = std::chrono::high_resolution_clock::now();
            // Remove the last 2 characters
            if(!(concatenated_data.find("bot") != std::string::npos || concatenated_data.find("top") != std::string::npos))
            {
              concatenated_data = "";
              continue;
            }

            if(concatenated_data.size() > 2)
            {
              concatenated_data.erase(concatenated_data.size() - 2);
            }

            // Tokenized
            const std::vector<std::string> tokens = tokenize(concatenated_data, ":");
            if(tokens.size() != 2)
            {
              concatenated_data = "";
              continue;
            }
            // Get which sensor and data
            const std::string & who = tokens[0];
            const std::string & data = tokens[1];
            // Measure elapsed time
            const double ms = std::chrono::duration<double, std::milli>(end - (who == "bot" ? bot_start : top_start)).count();

            double range = -1.;
            std::string error = "";
            if(is_digits(data))
            {
              range = std::stod(data) * 0.001;
              if(debug_.load())
              {
                mc_rtc::log::info("[RangeSensor::{}] The buffer data as a double is {} for {}", name_, range, who);
              }
            }
            else
            {
              error = data;
              if(debug_.load())
              {
                mc_rtc::log::error("[RangeSensor::{}] An error happened {} for {}", name_, error, who);
              }
            }


            // We gather enough data
            if(who == "bot" && bot_sensor_.name != "")
            {
              if(range != -1.)
              {
                bot_sensor_.new_data = true;
                bot_sensor_.range = range;
              }
              bot_sensor_.error = error;
              bot_sensor_.measured_time = ms;
              bot_sensor_.measured_Hz = 1. / (ms * 0.001);

              if(debug_.load())
              {
                mc_rtc::log::info("[RangeSensor::{}] The measure elapsed time is {} ms which corresponds to {} Hz for {}", name_, bot_sensor_.measured_time.load(), bot_sensor_.measured_Hz.load(), who);
              }

              bot_start = std::chrono::high_resolution_clock::now();
              top_to_bot_start = std::chrono::high_resolution_clock::now();
            }

            if(who == "top")
            {
              if(range != -1.)
              {
                top_sensor_.new_data = true;
                top_sensor_.range = range;
              }
              top_sensor_.error = error;
              top_sensor_.measured_time = ms;
              top_sensor_.measured_Hz = 1. / (ms * 0.001);

              if(debug_.load())
              {
                mc_rtc::log::info("[RangeSensor::{}] The measure elapsed time is {} ms which corresponds to {} Hz for {}", name_, top_sensor_.measured_time.load(), top_sensor_.measured_Hz.load(), who);
              }

              if(bot_sensor_.name != "")
              {
                const double top_to_bot_ms = std::chrono::duration<double, std::milli>(end - top_to_bot_start).count();
                measured_delay_between_sensor_ = top_to_bot_ms;
              }

              if(debug_.load())
              {
                mc_rtc::log::info("[RangeSensor::{}] The between 2 sensors is {} ms", name_, measured_delay_between_sensor_.load());
              }
              top_start = std::chrono::high_resolution_clock::now();
            }
            concatenated_data = "";
          }
        }
      }

      close(fd);
      mc_rtc::log::warning("[RangeSensor::{}] Closing the loop to read data on device {}", name_, serial_port_name_);
    }
  );
}

} // namespace lipm_walking
EXPORT_OBSERVER_MODULE("RangeSensor", lipm_walking::RangeSensor)