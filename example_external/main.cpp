#include <moteusapi/MoteusAPI.h>

int main() {
  // find your device name at >> ls /dev/
  string dev_name("/dev/tty.usbmodemBE6118CD1");
  int moteus_id = 1;
  MoteusAPI api(dev_name, moteus_id);

  // send one position with speed and torque limits
  api.SendPositionCommand(-1, 0.3, 0.3);

  return 0;
}