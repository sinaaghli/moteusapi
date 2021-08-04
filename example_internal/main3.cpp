#include <moteusapi/MoteusAPI.h>

int main() {
  string dev_name("/dev/tty.usbmodemBE6118CD1");
  int moteus_id = 1;
  MoteusAPI api(dev_name, moteus_id);
  api.SendPositionCommand(0.5, 0.2, 0.3);

  return 0;
}