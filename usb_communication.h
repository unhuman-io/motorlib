#ifndef UNHUMAN_MOTORLIB_USB_COMMUNICATION_H_
#define UNHUMAN_MOTORLIB_USB_COMMUNICATION_H_

#include "communication.h"
#include <cstring>
#include <algorithm>
#include "task.h"

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
    const uint16_t buffer_size = sizeof(MotorStatusRegular);
#endif
       usb_.send_data(2, reinterpret_cast<const uint8_t *>(&data), buffer_size, false);
    }
    int receive_string(char * const string) {
       int count = usb_.receive_data(1, (uint8_t * const) string, 64);
       string[count] = 0;
       return count;
    }
    static constexpr uint32_t MAX_PACKET_TRANSFER_SIZE = MAX_API_DATA_SIZE;

    Task<int> write_async(Scheduler& sched, uint8_t* data, uint16_t len) {
        co_return co_await usb_.send_data_async(sched, 1, data, len);
    }
    
    bool send_string_active() const { return usb_.tx_active(1); }
    void cancel_send_string() { usb_.cancel_transfer(1); }
    bool new_rx_data() { return usb_.new_rx_data(2); }
    bool any_new_rx_data() { return usb_.new_rx_data(2) || usb_.new_rx_data(1); }
    bool tx_data_ack() { return usb_.tx_data_ack(2); }

    void send_one_time_api_timeout_request(uint32_t us) {
       APIControlPacket timeout_request = {.control_packet_id = 0,
                                           .type = TIMEOUT_REQUEST,
                                           .timeout_request = {us}};
       usb_.send_data(1, (const uint8_t * const) &timeout_request, sizeof(timeout_request), true);
    }
    void cancel_one_time_api_timeout_request() {
       usb_.cancel_transfer(1);
    }
 private:
    USB1 &usb_;
    template <typename T> friend class SystemBase;
};

#endif  // UNHUMAN_MOTORLIB_USB_COMMUNICATION_H_
