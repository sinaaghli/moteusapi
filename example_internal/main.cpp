
#include <moteus_cpp/mjbots/moteus/moteus_protocol.h>
#include <moteus_cpp/serial-lib.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

using namespace std::this_thread;  // sleep_for, sleep_until
using namespace std::chrono;       // nanoseconds, system_clock, seconds

using namespace std;

int main() {
  mjbots::moteus::QueryCommand q_com;

  mjbots::moteus::CanFrame frame;
  cout << "size was " << (int)frame.size << endl;
  mjbots::moteus::WriteCanFrame wcan_frame(&frame);
  mjbots::moteus::EmitQueryCommand(&wcan_frame, q_com);
  cout << "size is " << (int)frame.size << endl;
  for (int ii = 0; ii < frame.size; ii++) {
    cout << (int)frame.data[ii] << ",";
  }
  cout << endl;
  const char* port = "/dev/serial/by-id/usb-mjbots_fdcanusb_BE6118CD-if00";
  seriallib serial;
  int fd = serial.serialport_init(port, 115200);
  // serial.serialport_writebytes(fd, (char*)&(frame.data), frame.size);
  cout << "fd is " << fd << endl;
  string sdata("can send 8001 420120\r");
  // string sdata("can status\r");
  cout << sdata.size() << endl;
  for (int ii = 0; ii < sdata.size(); ii++) {
    cout << (int)sdata[ii] << ",";
  }
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