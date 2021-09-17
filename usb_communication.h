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
    int receive_string(char * const string) {
       int count = usb_.receive_data(1, (uint8_t * const) string, 64);
       string[count] = 0;
       return count;
    }
    bool send_string(const char * const string, uint16_t length) {
       usb_.send_data(1, (const uint8_t * const) string, 
            std::min((uint16_t) MAX_API_DATA_SIZE, length), true);
       return true;
    }
    bool new_rx_data() { return usb_.new_rx_data(1) || usb_.new_rx_data(2); }
 private:
    USB1 &usb_;
    friend class System;
};

#endif
