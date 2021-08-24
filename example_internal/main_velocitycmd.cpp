#include <moteusapi/MoteusAPI.h>

int main() {
  string dev_name("/dev/tty.usbmodemBE6118CD1");
  int moteus_id = 1;
  MoteusAPI api(dev_name, moteus_id);

  // send one Velocity command
  api.SendPositionCommand(NAN, 0.2, 1, 0, 1, 1);

  return 0;
}