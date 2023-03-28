#ifndef UNHUMAN_MOTORLIB_COMMUNICATION_H_
#define UNHUMAN_MOTORLIB_COMMUNICATION_H_

#include "messages.h"

class CommunicationBase {
 public:
  int receive_data(ReceiveData *const data) { return 0; }
  bool receive_ready() { return false; }

  void send_data(const SendData &data) {}
  bool send_acknowledged() { return false; }

  int receive_string(char *const string) { return 0; }
  bool send_string(const char *const string, uint16_t length) { return false; }
  bool send_string_active() const { return false; }
  void cancel_send_string() {}
};

#endif  // UNHUMAN_MOTORLIB_COMMUNICATION_H_
