#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "messages.h"

class CommunicationBase {
 public:
    int receive_data(ReceiveData * const data) { return 0; }
    void send_data(const SendData &data) {}
};

#endif
