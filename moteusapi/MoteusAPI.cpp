#include "MoteusAPI.h"

#include <errno.h>   // Error number definitions
#include <fcntl.h>   // File control definitions
#include <stdio.h>   // Standard input/output definitions
#include <string.h>  // String function definitions
#include <sys/ioctl.h>
#include <termios.h>  // POSIX terminal control definitions
#include <unistd.h>   // UNIX standard function definitions

MoteusAPI::MoteusAPI(const string dev_name, int moteus_id)
    : dev_name_(dev_name), moteus_id_(moteus_id) {
  OpenDev();
}

MoteusAPI::~MoteusAPI() { CloseDev(); }

bool MoteusAPI::SendPositionCommand(double stop_position, double velocity,
                                    double max_torque,
                                    double feedforward_torque, double kp_scale,
                                    double kd_scale, double position,
                                    double watchdog_timer) const {
  mjbots::moteus::PositionCommand p_com;
  p_com.position = position;
  p_com.velocity = velocity;
  p_com.maximum_torque = max_torque;
  p_com.stop_position = stop_position;
  p_com.kp_scale = kp_scale;
  p_com.kd_scale = kd_scale;
  p_com.feedforward_torque = feedforward_torque;
  p_com.watchdog_timeout = watchdog_timer;
  // p_com.bounds_max = 1.0;
  // p_com.bounds_min = -1.0;
  mjbots::moteus::CanFrame frame;
  mjbots::moteus::WriteCanFrame write_frame(&frame);
  // default resolutions are used modify if necessary.
  mjbots::moteus::PositionResolution pres;
  mjbots::moteus::EmitPositionCommand(&write_frame, p_com, pres);

  // Encode message to hex
  stringstream ss;
  ss << "can send 80" << std::setfill('0') << std::setw(2) << std::hex
     << moteus_id_ << " ";
  for (int ii = 0; ii < (int)frame.size; ii++) {
    ss << std::setfill('0') << std::setw(2) << std::hex << (int)frame.data[ii];
  }
  ss << '\n';

  return WriteDev(ss.str());
}

bool MoteusAPI::SendWithinCommand(double bounds_min, double bounds_max,
                                  double feedforward_torque, double kp_scale,
                                  double kd_scale, double max_torque,
                                  double stop_position, double timeout) const {
  mjbots::moteus::WithinCommand p_com;
  p_com.bounds_min = bounds_min;
  p_com.bounds_max = bounds_max;
  p_com.feedforward_torque = feedforward_torque;
  p_com.kp_scale = kp_scale;
  p_com.kd_scale = kd_scale;
  p_com.maximum_torque = max_torque;
  p_com.stop_position = stop_position;
  p_com.watchdog_timeout = timeout;
  mjbots::moteus::CanFrame frame;
  mjbots::moteus::WriteCanFrame write_frame(&frame);
  // default resolutions are used modify if necessary.
  mjbots::moteus::WithinResolution pres;
  mjbots::moteus::EmitWithinCommand(&write_frame, p_com, pres);

  // Encode message to hex
  stringstream ss;
  ss << "can send 80" << std::setfill('0') << std::setw(2) << std::hex
     << moteus_id_ << " ";
  for (int ii = 0; ii < (int)frame.size; ii++) {
    ss << std::setfill('0') << std::setw(2) << std::hex << (int)frame.data[ii];
  }
  ss << '\n';

  return WriteDev(ss.str());
}

// bool MoteusAPI::ReadState(double& position, double& velocity, double& torque,
//                           double& q_curr, double& d_curr, double&
//                           rezero_state, double& voltage, double& temperature,
//                           double& fault) const {
//   // do stuff
//   // more stuff
// }

int MoteusAPI::OpenDev() {
  struct termios toptions;
  int fd;

  fd = open(dev_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (fd == -1) {
    cerr << "MoteusAPI: Unable to open port" << endl;
    exit(EXIT_FAILURE);
  }

  if (tcgetattr(fd, &toptions) < 0) {
    cerr << "MoteusAPI: Couldn't get term attributes" << endl;
    exit(EXIT_FAILURE);
  }

  // set baud to arbitrary value, it will get ignored by dev
  speed_t brate = B115200;

  cfsetispeed(&toptions, brate);
  cfsetospeed(&toptions, brate);

  // 8N1
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;
  // no flow control
  toptions.c_cflag &= ~CRTSCTS;

  toptions.c_cflag |= CREAD | CLOCAL;
  toptions.c_iflag &= ~(IXON | IXOFF | IXANY);

  toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  toptions.c_oflag &= ~OPOST;

  toptions.c_cc[VMIN] = 0;
  toptions.c_cc[VTIME] = 0;

  tcsetattr(fd, TCSANOW, &toptions);
  if (tcsetattr(fd, TCSAFLUSH, &toptions) < 0) {
    cerr << "MoteusAPI: Couldn't set term attributes" << endl;
    exit(EXIT_FAILURE);
  }

  fd_ = fd;
}

bool MoteusAPI::WriteDev(const string& buff) const {
  int n = write(fd_, buff.c_str(), buff.size());
  if (n != buff.size()) {
    cerr << "MoteusAPI: write unsuccessful ..." << endl;
    return false;
  }
  return true;
}

int MoteusAPI::CloseDev() const { return close(fd_); }