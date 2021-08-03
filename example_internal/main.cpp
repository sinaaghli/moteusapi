
#include <moteus_cpp/mjbots/moteus_protocol.h>
#include <moteus_cpp/serial-lib.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

using namespace std::this_thread;  // sleep_for, sleep_until
using namespace std::chrono;       // nanoseconds, system_clock, seconds

using namespace std;

int main() {
  mjbots::moteus::PositionCommand p_com;
  p_com.position = std::numeric_limits<double>::quiet_NaN();
  p_com.velocity = 0.2;
  p_com.maximum_torque = 0.3;
  p_com.stop_position = 0.8;

  mjbots::moteus::CanFrame frame;
  cout << "size was " << (int)frame.size << endl;
  mjbots::moteus::WriteCanFrame wcan_frame(&frame);
  mjbots::moteus::PositionResolution pres;
  pres.kd_scale = mjbots::moteus::Resolution::kIgnore;
  pres.kp_scale = mjbots::moteus::Resolution::kIgnore;
  pres.feedforward_torque = mjbots::moteus::Resolution::kIgnore;
  pres.maximum_torque = mjbots::moteus::Resolution::kIgnore;
  pres.stop_position = mjbots::moteus::Resolution::kIgnore;
  pres.velocity = mjbots::moteus::Resolution::kIgnore;
  pres.watchdog_timeout = mjbots::moteus::Resolution::kIgnore;

  mjbots::moteus::EmitPositionCommand(&wcan_frame, p_com, pres);
  cout << "size is " << (int)frame.size << endl;
  for (int ii = 0; ii < (int)frame.size; ii++) {
    cout << (int)frame.data[ii] << ",";
  }
  cout << endl;
  // mjbots::moteus::QueryCommand q_com;
  // mjbots::moteus::CanFrame frame;
  // cout << "size was " << (int)frame.size << endl;
  // mjbots::moteus::WriteCanFrame wcan_frame(&frame);
  // mjbots::moteus::EmitQueryCommand(&wcan_frame, q_com);
  // cout << "size is " << (int)frame.size << endl;
  // for (int ii = 0; ii < frame.size; ii++) {
  //   cout << (int)frame.data[ii] << ",";
  // }
  // cout << endl;
  const char* port = "/dev/serial/by-id/usb-mjbots_fdcanusb_BE6118CD-if00";
  seriallib serial;
  int fd = serial.serialport_init(port, 115200);
  // serial.serialport_writebytes(fd, (char*)&(frame.data), frame.size);

  // string sdata("can send 8001 420120\r");
  // string sdata("can send 8001 1010133200063\r");

  stringstream ss;
  ss << "can send 8001 ";
  for (int ii = 0; ii < (int)frame.size; ii++) {
    ss << std::setfill('0') << std::setw(2) << std::hex << (int)frame.data[ii];
  }
  ss << '\n';
  string sdata(ss.str());

  cout << "string is " << sdata << endl;
  // for (int ii = 0; ii < sdata.size(); ii++) {
  //   cout << (int)sdata[ii] << ",";
  // }
  // while (1) {
  // serial.serialport_writebytes(fd, (char*)&(frame.data), frame.size);

  int wres = serial.serialport_writebytes(fd, sdata.c_str(), sdata.size());

  char read_buff[100];
  for (int ii = 0; ii < 100; ii++) {
    read_buff[ii] = 0;
  }
  int res = serial.serialport_read_until(fd, read_buff, '\r', 100, 1000);
  cout << "res->" << res << endl;
  string resp(read_buff);
  cout << "size is : " << resp.size() << endl;
  cout << "got : " << resp << endl;

  res = serial.serialport_read_until(fd, read_buff, '\r', 100, 1000);
  cout << "res->" << res << endl;
  string resp2(read_buff);
  cout << "size is : " << resp2.size() << endl;
  cout << "got : " << resp2 << endl;
  serial.serialport_close(fd);
  //}

  // if (q_com.any_set()) {
  //   cout << "got sth" << endl;
  //   // cout << "d_current" << q_com.d_current << endl;
  //   // cout << "fault" << q_com.fault << endl;
  //   // cout << "mode" << q_com.mode << endl;
  //   // cout << "position" << q_com.position << endl;
  //   // cout << "q_current" << q_com.q_current << endl;
  //   // cout << "rezero_state" << q_com.rezero_state << endl;
  //   // cout << "temperature" << q_com.temperature << endl;
  //   // cout << "torque" << q_com.torque << endl;
  //   // cout << "velocity" << q_com.velocity << endl;
  //   // cout << "voltage" << q_com.voltage << endl;
  // } else {
  //   cout << "nothing yet ..." << endl;
  // }

  // sleep_for(seconds(1));
  // }

  return 0;
}