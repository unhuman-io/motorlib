#ifndef UNHUMAN_MOTORLIB_COMMUNICATION_H_
#define UNHUMAN_MOTORLIB_COMMUNICATION_H_

#include "messages.h"

class CommunicationBase {
 public:
    int receive_data(ReceiveData * const data) { return 0; }
    void send_data(const SendData &data) {}

    // used to wake up from sleep. Override for this feature
    bool any_new_rx_data() {
      return false;
    }

    // receive a string on ascii debug channel
    int receive_string(char* const string) {
      return 0;
    }

    // send a string on ascii debug channel
    bool send_string(const char* string, uint16_t length) {
      return true;
    }

    // is send string active
    bool send_string_active() const {
      return false;
    }

    void cancel_send_string() {
    }

    void send_one_time_api_timeout_request(uint32_t us) {
    }

    void cancel_one_time_api_timeout_request() {
    }

    // true if send_data call succeeded, used to increment
    // round robin data
    bool tx_data_ack() {
      return false;
    }
};

#endif  // UNHUMAN_MOTORLIB_COMMUNICATION_H_
