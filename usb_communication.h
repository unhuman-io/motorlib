#ifndef USB_COMMUNICATION_H
#define USB_COMMUNICATION_H

#include "communication.h"

class USBCommunication : public CommunicationBase {
 public:
    USBCommunication(USB1 &usb) : usb_(usb) {}
    int receive_data(ReceiveData * const data) {
      return usb_.receive_data(2, (uint8_t *const) data, sizeof(*data));
    }
    void send_data(const SendData &data) {
       usb_.send_data(2, reinterpret_cast<const uint8_t *>(&data), sizeof(SendData), false);
    }
 private:
    USB1 &usb_;
};

#endif
