#include <moteusapi/MoteusAPI.h>

int main() {
  // replace /dev/tty.usbmodemBE6118CD1 with your own usbcan dev name
  string dev_name("/dev/tty.usbmodemBE6118CD1");
  int moteus_id = 1;
  MoteusAPI api(dev_name, moteus_id);

  // send one within command
  double bounds_min = -1;
  double bounds_max = 1;
  double feedforward_torque = 0;
  double kp_scale = 1;
  double kd_scale = 1;
  double max_torque = 0.15;
  api.SendWithinCommand(bounds_min, bounds_max, feedforward_torque, kp_scale,
                        kd_scale, max_torque);

  return 0;
}