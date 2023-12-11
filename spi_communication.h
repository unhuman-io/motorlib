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
      FIGURE_ASSERT(sizeof(ReceiveData) <= SpiMailbox::kBufferSize);
      return protocol_.mailboxes.read(MAILBOX_ID_DATA_FROM_HOST, (uint8_t*)data, sizeof(ReceiveData));
    }

    void send_data(const SendData& data) {
      FIGURE_ASSERT(sizeof(SendData) <= SpiMailbox::kBufferSize);
      protocol_.mailboxes.write(MAILBOX_ID_DATA_TO_HOST, (uint8_t*)&data, sizeof(SendData));
    }

    int receive_string(char* const string) {
      size_t length = protocol_.mailboxes.read(MAILBOX_ID_SERIAL_FROM_HOST, (uint8_t*)string, SpiMailbox::kBufferSize);
      string[length] = '\x0'; // Enforce the string termination
      return length;
    }

    bool send_string(const char* string, uint16_t length) {
      size_t chunk_length;
      // If the buffer is too long - split it onto smaller chunks.
      while(length > 0)
      {
        chunk_length = (length > SpiMailbox::kBufferSize) ? SpiMailbox::kBufferSize : length;
        protocol_.mailboxes.write(MAILBOX_ID_SERIAL_TO_HOST, (const uint8_t*)string, chunk_length);
        string += chunk_length;
        length -= chunk_length;
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
