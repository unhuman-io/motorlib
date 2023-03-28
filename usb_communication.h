#ifndef UNHUMAN_MOTORLIB_USB_COMMUNICATION_H_
#define UNHUMAN_MOTORLIB_USB_COMMUNICATION_H_

#include "communication.h"

class USBCommunication : public CommunicationBase {
 public:
  USBCommunication(USB1 &usb) : usb_(usb) {}
  int receive_data(ReceiveData *const data) {
    return usb_.receive_data(2, (uint8_t *const)data, sizeof(*data));
  }
  bool receive_ready() { return usb_.new_rx_data(2); }
  void send_data(const SendData &data) {
    usb_.send_data(2, reinterpret_cast<const uint8_t *>(&data),
                   sizeof(SendData), false);
  }
  bool send_acknowledged() { return usb_.tx_data_ack(2); }

  int receive_string(char *const string) {
    int count = usb_.receive_data(1, (uint8_t *const)string, 64);
    string[count] = 0;
    return count;
  }
  bool send_string(const char *const string, uint16_t length) {
    usb_.send_data(1, (const uint8_t *const)string,
                   std::min((uint16_t)MAX_API_DATA_SIZE, length), true);
    return true;
  }
  bool send_string_active() const { return usb_.tx_active(1); }
  void cancel_send_string() { usb_.cancel_transfer(1); }

  uint32_t get_error_count() { return usb_.get_error_count(); }

 private:
  USB1 &usb_;
  friend class System;
};

#endif  // UNHUMAN_MOTORLIB_USB_COMMUNICATION_H_
