#include <moteusapi/MoteusAPI.h>

int main() {
  string dev_name("/dev/tty.usbmodemBE6118CD1");
  int moteus_id = 1;
  MoteusAPI api(dev_name, moteus_id);

  // send one position with speed and torque limits
  api.SendPositionCommand(0, 0, 1, 0.03, 0, 0);

  return 0;
}