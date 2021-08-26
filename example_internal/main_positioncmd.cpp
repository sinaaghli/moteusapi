#include <moteusapi/MoteusAPI.h>

int main() {
  // replace /dev/tty.usbmodemBE6118CD1 with your own usbcan dev name
  string dev_name("/dev/tty.usbmodemBE6118CD1");
  int moteus_id = 1;
  MoteusAPI api(dev_name, moteus_id);

  // send one position with speed and torque limits
  double stop_position = 0;
  double velocity = 0.05;
  double max_torque = 1;
  double feedforward_torque = 0;
  api.SendPositionCommand(stop_position, velocity, max_torque,
                          feedforward_torque);

  return 0;
}