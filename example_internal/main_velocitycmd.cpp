#include <moteusapi/MoteusAPI.h>

int main() {
  // replace /dev/tty.usbmodemBE6118CD1 with your own usbcan dev name
  string dev_name("/dev/tty.usbmodemBE6118CD1");
  int moteus_id = 1;
  MoteusAPI api(dev_name, moteus_id);

  // send one Velocity command
  double stop_position = NAN;
  double velocity = 0.2;
  double max_torque = 1;
  double feedforward_torque = 0;
  api.SendPositionCommand(stop_position, velocity, max_torque,
                          feedforward_torque);

  return 0;
}