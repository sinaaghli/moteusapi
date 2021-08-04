#ifndef MOTEUSAPI_H__
#define MOTEUSAPI_H__

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "mjbots/moteus_protocol.h"
#include "serial-lib.h"

using namespace std;

class MoteusAPI {
 public:
  MoteusAPI(const string dev_name, int moteus_id);
  ~MoteusAPI();
  void SendPositionCommand(double stop_position, double velocity,
                           double max_torque, double feedforward_torque = 0,
                           double kp_scale = 1.0, double kd_scale = 1.0,
                           double position = NAN,
                           double watchdog_timer = NAN) const;

 private:
  int OpenDev();
  int CloseDev() const;
  int WriteDev(const string& buff) const;
  const string dev_name_;
  const int moteus_id_;
  int fd_;
};

#endif  // MOTEUSAPI_H__