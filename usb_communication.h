#ifndef UNHUMAN_MOTORLIB_USB_COMMUNICATION_H_
#define UNHUMAN_MOTORLIB_USB_COMMUNICATION_H_

#include "communication.h"
#include <cstring>

class USBCommunication : public CommunicationBase {
 public:
    USBCommunication(USB1 &usb) : usb_(usb) {}
    int receive_data(ReceiveData * const data) {
      return usb_.receive_data(2, (uint8_t *const) data, sizeof(*data));
    }
    void send_data(const SendData &data) {
#ifdef USE_MOTOR_STATUS_LITE
    const uint16_t buffer_size = sizeof(MotorStatusLite);
#else
    const uint16_t buffer_size = sizeof(MotorStatus);
#endif
       usb_.send_data(2, reinterpret_cast<const uint8_t *>(&data), buffer_size, false);
    }
    int receive_string(char * const string) {
       int count = usb_.receive_data(1, (uint8_t * const) string, 64);
       string[count] = 0;
       return count;
    }
    bool send_string(const char * const string, uint16_t length) {
       // blocks until entire string has been sent
       if (string[0] == 0) {
          // binary that starts with 0, need to send as long packet
          struct {
             APIControlPacket control_packet = {0, LONG_PACKET, .long_packet = {0, 1}};
             char data[MAX_API_DATA_SIZE - sizeof(APIControlPacket)];
          } long_packet;
          long_packet.control_packet.long_packet.total_length = length;
          std::memcpy(long_packet.data, string, 
            std::min((uint16_t) (MAX_API_DATA_SIZE - sizeof(APIControlPacket)), length));
          usb_.send_data(1, (const uint8_t * const) &long_packet, 
                  std::min((uint16_t) MAX_API_DATA_SIZE, (uint16_t) (length + sizeof(APIControlPacket))), true);
       } else {
         usb_.send_data(1, (const uint8_t * const) string, 
               std::min((uint16_t) MAX_API_DATA_SIZE, length), true);
       }
       return true;
    }
    bool send_string_active() const { return usb_.tx_active(1); }
    void cancel_send_string() { usb_.cancel_transfer(1); }
    bool new_rx_data() { return usb_.new_rx_data(2); }
    bool any_new_rx_data() { return usb_.new_rx_data(2) || usb_.new_rx_data(1); }
    bool tx_data_ack() { return usb_.tx_data_ack(2); }

    void send_one_time_api_timeout_request(uint32_t us) {
       APIControlPacket timeout_request = {0, TIMEOUT_REQUEST, .timeout_request = {us}};
       usb_.send_data(1, (const uint8_t * const) &timeout_request, sizeof(timeout_request), true);
    }
    void cancel_one_time_api_timeout_request() {
       usb_.cancel_transfer(1);
    }
 private:
    USB1 &usb_;
    friend class System;
};

#endif  // UNHUMAN_MOTORLIB_USB_COMMUNICATION_H_
