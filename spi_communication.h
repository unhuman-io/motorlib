#ifndef UNHUMAN_MOTORLIB_SPI_COMMUNICATION_H_
#define UNHUMAN_MOTORLIB_SPI_COMMUNICATION_H_

#include "communication.h"
#include "peripheral/spi_protocol.h"

class SPICommunication : public CommunicationBase {
 public:
    enum mailboxId
    {
      MAILBOX_ID_UNUSED = 0,
      MAILBOX_ID_SERIAL_TO_HOST = 1,
      MAILBOX_ID_SERIAL_FROM_HOST = 2,
      MAILBOX_ID_DATA_TO_HOST = 3,
      MAILBOX_ID_DATA_FROM_HOST = 4
    };
    SPICommunication(SpiProtocol& protocol) :
      protocol_(protocol)
    {};

    int receive_data(ReceiveData* const data) {
      return protocol_.mailboxes.read(MAILBOX_ID_DATA_FROM_HOST, (uint8_t*)data, sizeof(ReceiveData));
    }

    void send_data(const SendData& data) {
      protocol_.mailboxes.write(MAILBOX_ID_DATA_TO_HOST, (uint8_t*)&data, sizeof(SendData));
    }

    int receive_string(char* const string) {
      size_t length = protocol_.mailboxes.read(MAILBOX_ID_SERIAL_FROM_HOST, (uint8_t*)string, 64U);
      string[length] = '\x0'; // Enforce the string termination
      return length;
    }

    bool send_string(const char* const string, uint16_t length) {
      if(length > 0)
      {
        protocol_.mailboxes.write(MAILBOX_ID_SERIAL_TO_HOST, (const uint8_t*)string, length);
      }
      return true;
    }

    bool send_string_active() const {
      return false;
    }

    void cancel_send_string() {
    }

    bool new_rx_data() {
      return false;
    }

    bool any_new_rx_data() {
      return false;
    }

    bool tx_data_ack() {
      return false;
    }

 private:
    SpiProtocol& protocol_;
    friend class System;
};

#endif  // UNHUMAN_MOTORLIB_SPI_COMMUNICATION_H_
