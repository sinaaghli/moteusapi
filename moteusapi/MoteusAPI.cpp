// Copyright 2021 Sina Aghli, sinaaghli.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
  mjbots::moteus::CanFrame frame;
  mjbots::moteus::WriteCanFrame write_frame(&frame);
  mjbots::moteus::PositionResolution pres;
  mjbots::moteus::EmitPositionCommand(&write_frame, p_com, pres);

  // Encode message to hex
  stringstream ss;
  ss << "can send 80" << std::setfill('0') << std::setw(2) << std::hex
     << moteus_id_ << " ";
  for (uint ii = 0; ii < (uint)frame.size; ii++) {
    ss << std::setfill('0') << std::setw(2) << std::hex << (int)frame.data[ii];
  }
  ss << '\n';

  if (!WriteDev(ss.str()))
    throw std::runtime_error("Failiur: could not WriteDev.");

  // process response
  string resp;
  if (!((ExpectResponse("OK", resp) && ExpectResponse("rcv", resp)))) {
    return false;
  }

  return true;
}

bool MoteusAPI::SendStopCommand() {
  mjbots::moteus::CanFrame frame;
  mjbots::moteus::WriteCanFrame write_frame(&frame);
  mjbots::moteus::EmitStopCommand(&write_frame);

  // Encode message to hex
  stringstream ss;
  ss << "can send 80" << std::setfill('0') << std::setw(2) << std::hex
     << moteus_id_ << " ";
  for (uint ii = 0; ii < (uint)frame.size; ii++) {
    ss << std::setfill('0') << std::setw(2) << std::hex << (int)frame.data[ii];
  }
  ss << '\n';

  if (!WriteDev(ss.str()))
    throw std::runtime_error("Failiur: could not WriteDev.");

  // process response
  string resp;
  if (!((ExpectResponse("OK", resp) && ExpectResponse("rcv", resp)))) {
    return false;
  }

  return true;
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
  for (uint ii = 0; ii < (uint)frame.size; ii++) {
    ss << std::setfill('0') << std::setw(2) << std::hex << (int)frame.data[ii];
  }
  ss << '\n';

  if (!WriteDev(ss.str()))
    throw std::runtime_error("Failiur: could not WriteDev.");

  // process response
  string resp;
  if (!((ExpectResponse("OK", resp) && ExpectResponse("rcv", resp)))) {
    return false;
  }

  return true;
}

void MoteusAPI::ReadState(State& curr_state) const {
  mjbots::moteus::QueryCommand q_com;
  if (!curr_state.position_flag)
    q_com.position = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.velocity_flag)
    q_com.velocity = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.torque_flag)
    q_com.torque = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.q_curr_flag)
    q_com.q_current = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.d_curr_flag)
    q_com.d_current = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.rezero_state_flag)
    q_com.rezero_state = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.voltage_flag)
    q_com.voltage = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.temperature_flag)
    q_com.temperature = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.fault_flag) q_com.fault = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.mode_flag) q_com.mode = mjbots::moteus::Resolution::kIgnore;

  mjbots::moteus::CanFrame frame;
  mjbots::moteus::WriteCanFrame wcan_frame(&frame);
  mjbots::moteus::EmitQueryCommand(&wcan_frame, q_com);
  // Encode message to hex
  stringstream ss;
  ss << "can send 80" << std::setfill('0') << std::setw(2) << std::hex
     << moteus_id_ << " ";
  for (uint ii = 0; ii < (uint)frame.size; ii++) {
    ss << std::setfill('0') << std::setw(2) << std::hex << (int)frame.data[ii];
  }
  ss << '\n';

  if (!WriteDev(ss.str()))
    throw std::runtime_error("Failiur: could not WriteDev.");

  string resp;
  if (!((ExpectResponse("OK", resp) && ExpectResponse("rcv", resp)))) {
    return;
  }
  // cout << "resp is :" << resp << endl;

  /// parse response
  istringstream iss(resp);
  vector<string> words;
  copy(istream_iterator<string>(iss), istream_iterator<string>(),
       back_inserter(words));

  uint8_t decoded[readbuffsize];
  string respstr(words.at(2));
  uint loopsize = respstr.size() / 2;

  for (uint ii = 0; ii < loopsize; ii++) {
    std::stringstream stream;
    stream << respstr.substr(ii * 2, 2);
    int tmp;
    stream >> std::hex >> tmp;
    decoded[ii] = (uint8_t)tmp;
  }

  mjbots::moteus::QueryResult qr =
      mjbots::moteus::ParseQueryResult(decoded, loopsize);
  curr_state.position = qr.position;
  curr_state.velocity = qr.velocity;
  curr_state.torque = qr.torque;
  curr_state.q_curr = qr.q_current;
  curr_state.d_curr = qr.d_current;
  curr_state.voltage = qr.voltage;
  curr_state.temperature = qr.temperature;
  curr_state.fault = qr.fault;
}

bool MoteusAPI::ExpectResponse(const string& exp_string,
                               string& fullresp) const {
  char read_buff[readbuffsize];
  for (uint ii = 0; ii < readbuffsize; ii++) {
    read_buff[ii] = 0;
  }
  do {
    int res = ReadUntilDev(read_buff, '\n', readbuffsize, 1000);
    if (res) {
      cout << "Timeout: Expected response'" << exp_string
           << "' was not received" << endl;
      return false;
    }
    fullresp = string(read_buff);
  } while (fullresp.find(exp_string) == std::string::npos);
  return true;
}

int MoteusAPI::OpenDev() {
  struct termios toptions;
  int fd;

  fd = open(dev_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (fd == -1) {
    throw std::runtime_error("MoteusAPI: Unable to open port");
    exit(EXIT_FAILURE);
  }

  if (tcgetattr(fd, &toptions) < 0) {
    throw std::runtime_error("MoteusAPI: Couldn't get term attributes");
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
    throw std::runtime_error("MoteusAPI: Couldn't set term attributes");
  }

  fd_ = fd;
  return fd_;
}

bool MoteusAPI::WriteDev(const string& buff) const {
  uint n = write(fd_, buff.c_str(), buff.size());
  if (n != buff.size()) {
    return false;
  }
  return true;
}

int MoteusAPI::CloseDev() const { return close(fd_); }

int MoteusAPI::ReadUntilDev(char* buf, char until, int buf_max,
                            int timeout) const {
  char b[1];  // read expects an array, so we give it a 1-byte array
  int i = 0;
  do {
    int n = read(fd_, b, 1);  // read a char at a time
    if (n == -1) return -1;   // couldn't read
    if (n == 0) {
      usleep(1 * 1000);  // wait 1 msec try again
      timeout--;
      if (timeout == 0) return -2;
      continue;
    }
    buf[i] = b[0];
    i++;
  } while (b[0] != until && i < buf_max && timeout > 0);

  buf[i] = 0;  // null terminate the string
  return 0;
}