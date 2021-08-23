#include <moteusapi/MoteusAPI.h>

int main() {
  string dev_name("/dev/tty.usbmodemBE6118CD1");
  int moteus_id = 1;
  MoteusAPI api(dev_name, moteus_id);

  // read all the states and print
  // api.ReadState(-1, 0.3, 0.3);

  // read some of the states

  return 0;
}