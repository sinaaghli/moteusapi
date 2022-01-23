#include <moteusapi/MoteusAPI.h>

#include <chrono>
#include <thread>

int main() {
  // replace /dev/tty.usbmodemBE6118CD1 with your own usbcan dev name
  string dev_name("/dev/tty.usbmodemBE6118CD1");
  int moteus_id = 1;
  MoteusAPI api(dev_name, moteus_id);

  // send one torque command
  double stop_position = 0;
  double velocity = 0;
  double max_torque = 1;
  double feedforward_torque = 0;
  double kp_scale = 0;
  double kd_scale = 0;
  // api.SendPositionCommand(stop_position, velocity, max_torque,
  //                         feedforward_torque, kp_scale, kd_scale);

  for (int ii = 0; ii < 100; ii += 1) {
    feedforward_torque = ii * 0.02;
    std::cout << ii << " = " << feedforward_torque << std::endl;
    api.SendPositionCommand(stop_position, velocity, max_torque,
                            -feedforward_torque, kp_scale, kd_scale);
    std::this_thread::sleep_for(1500ms);
    api.SendPositionCommand(stop_position, velocity, max_torque, 0, kp_scale,
                            kd_scale);
    std::this_thread::sleep_for(5000ms);
  }
  std::cout << "All Done!" << std::endl;
  return 0;
}