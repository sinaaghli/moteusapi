// Copyright 2021 Sina Aghli, sina.aghli@gmail.com
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

struct moteus_state {
  double position;
  double velocity;
  double torque;
  double q_curr;
  double d_curr;
  double rezero_state;
  double voltage;
  double temperature;
  double fault;
  moteus_state() {
    position = NAN;
    velocity = NAN;
    torque = NAN;
    q_curr = NAN;
    d_curr = NAN;
    rezero_state = NAN;
    voltage = NAN;
    temperature = NAN;
    fault = NAN;
  }
};

class MoteusAPI {
 public:
  MoteusAPI(const string dev_name, int moteus_id);
  ~MoteusAPI();

  // see link for description
  // https://github.com/mjbots/moteus/blob/main/docs/getting_started.md#how-position-mode-works
  bool SendPositionCommand(double stop_position, double velocity,
                           double max_torque, double feedforward_torque = 0,
                           double kp_scale = 1.0, double kd_scale = 1.0,
                           double position = NAN,
                           double watchdog_timer = NAN) const;
  bool SendWithinCommand(double bounds_min, double bounds_max,
                         double feedforward_torque, double kp_scale,
                         double kd_scale, double max_torque,
                         double stop_position, double timeout) const;

  // for any state reading
  // bool ReadState() const;

 private:
  // Open /dev/dev_name_
  int OpenDev();
  int CloseDev() const;
  bool WriteDev(const string& buff) const;
  const string dev_name_;
  const int moteus_id_;
  int fd_;
};

#endif  // MOTEUSAPI_H__