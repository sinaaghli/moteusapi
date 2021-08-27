#include <moteusapi/MoteusAPI.h>

int main() {
  // replace /dev/tty.usbmodemBE6118CD1 with your own usbcan dev name
  string dev_name("/dev/tty.usbmodemBE6118CD1");
  int moteus_id = 1;
  MoteusAPI api(dev_name, moteus_id);

  // define a state object
  State curr_state;

  // only read current position
  api.ReadState(curr_state.EN_Position());
  cout << "position: " << curr_state.position << endl;

  // reset the state and only read velocity and torque
  curr_state.Reset();
  api.ReadState(curr_state.EN_Velocity().EN_Torque());

  // read temperature in addition to velocity and torque
  api.ReadState(curr_state.EN_Temp());

  // print everyting
  cout << "velocity: " << curr_state.velocity << endl;
  cout << "torque: " << curr_state.torque << endl;
  cout << "temperature: " << curr_state.temperature << endl;

  return 0;
}