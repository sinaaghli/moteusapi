
#include <moteus_cpp/mjbots/moteus_protocol.h>
#include <moteus_cpp/serial-lib.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std::this_thread;  // sleep_for, sleep_until
using namespace std::chrono;       // nanoseconds, system_clock, seconds

using namespace std;

int main() {
  mjbots::moteus::QueryCommand q_com;
  // q_com.d_current = mjbots::moteus::Resolution::kIgnore;
  // q_com.fault = mjbots::moteus::Resolution::kIgnore;
  // q_com.mode = mjbots::moteus::Resolution::kIgnore;
  // q_com.q_current = mjbots::moteus::Resolution::kIgnore;
  // q_com.rezero_state = mjbots::moteus::Resolution::kIgnore;
  // q_com.position = mjbots::moteus::Resolution::kIgnore;
  // q_com.torque = mjbots::moteus::Resolution::kIgnore;
  // q_com.temperature = mjbots::moteus::Resolution::kIgnore;
  // q_com.velocity = mjbots::moteus::Resolution::kIgnore;
  // q_com.voltage = mjbots::moteus::Resolution::kIgnore;
  mjbots::moteus::CanFrame frame;
  cout << "size was " << (int)frame.size << endl;
  mjbots::moteus::WriteCanFrame wcan_frame(&frame);
  mjbots::moteus::EmitQueryCommand(&wcan_frame, q_com);
  cout << "size is " << (int)frame.size << endl;
  for (int ii = 0; ii < frame.size; ii++) {
    cout << (int)frame.data[ii] << ",";
  }
  cout << endl;

  const char* port = "/dev/tty.usbmodemBE6118CD1";

  seriallib serial;
  int fd = serial.serialport_init(port, 115200);

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

  int buffsize = 0;
  string resp;
  do {
    int res = serial.serialport_read_until(fd, read_buff, '\r', 400, 1000);
    cout << "res->" << res << endl;
    resp = string(read_buff);
    buffsize = resp.size();
    cout << "size is : " << resp.size() << endl;
    cout << "got : " << resp << endl;
  } while (buffsize == 3);

  serial.serialport_close(fd);

  /// parse response

  istringstream iss(resp);
  vector<string> words;
  copy(istream_iterator<string>(iss), istream_iterator<string>(),
       back_inserter(words));

  uint8_t decoded[100];
  string respstr(words.at(2));
  int loopsize = respstr.size() / 2;
  cout << loopsize << endl;

  for (int ii = 0; ii < loopsize; ii++) {
    std::stringstream stream;
    stream << respstr.substr(ii * 2, 2);
    int tmp;
    stream >> std::hex >> tmp;
    decoded[ii] = (uint8_t)tmp;
  }

  mjbots::moteus::QueryResult qr =
      mjbots::moteus::ParseQueryResult(decoded, loopsize);
  cout << "vel: " << qr.velocity << endl;
  cout << "pos: " << qr.position << endl;
  cout << "torq: " << qr.torque << endl;
  cout << "q_cur: " << qr.q_current << endl;
  cout << "d_cur: " << qr.d_current << endl;
  cout << "vol: " << qr.voltage << endl;
  cout << "temp: " << qr.temperature << endl;
  cout << "fault: " << qr.fault << endl;

  return 0;
}