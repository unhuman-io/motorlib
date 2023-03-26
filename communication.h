#ifndef UNHUMAN_MOTORLIB_COMMUNICATION_H_
#define UNHUMAN_MOTORLIB_COMMUNICATION_H_

#include "messages.h"

class CommunicationBase {
 public:
  int receive_data(ReceiveData *const data) { return 0; }
  void send_data(const SendData &data) {}
};

#endif  // UNHUMAN_MOTORLIB_COMMUNICATION_H_
