//
// arduino-serial-lib -- simple library for reading/writing serial ports
//
// 2006-2013, Tod E. Kurt, http://todbot.com/blog/
//
// Sebastian Giles added: _readbytes, _readbytes, _writebytes

#ifndef __ARDUINO_SERIAL_LIB_H__
#define __ARDUINO_SERIAL_LIB_H__
//#define SERIALPORTDEBUG

#include <stdint.h>  // Standard types
class seriallib {
 public:
  int serialport_init(const char* serialport, int baud);
  int serialport_close(int fd);
  int serialport_writebyte(int fd, uint8_t b);
  int serialport_write(int fd, const char* str);
  int serialport_read_until(int fd, char* buf, char until, int buf_max,
                            int timeout);
  int serialport_flush(int fd);

  int serialport_readbytes(int fd, char* buf, int len);
  int serialport_readbyte(int fd);
  int serialport_writebytes(int fd, const char* str, int len);
};

#endif
