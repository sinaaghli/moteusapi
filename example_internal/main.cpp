
#include <moteusapi/mjbots/moteus_protocol.h>
#include <moteusapi/serial-lib.h>

#include <chrono>
#include <cmath>
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
  p_com.position = NAN;
  p_com.velocity = 0.2;
  p_com.maximum_torque = 0.9;
  p_com.stop_position = 0.0;
  p_com.kp_scale = 1;
  p_com.kd_scale = 1;
  p_com.feedforward_torque = 0.0;
  p_com.watchdog_timeout = NAN;

  mjbots::moteus::CanFrame frame;
  mjbots::moteus::WriteCanFrame wcan_frame(&frame);
  mjbots::moteus::PositionResolution pres;

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
  // const char* port = "/dev/serial/by-id/usb-mjbots_fdcanusb_BE6118CD-if00";
  const char* port = "/dev/tty.usbmodemBE6118CD1";

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

  cout << sdata << endl;

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

  /// parse response

  return 0;
}