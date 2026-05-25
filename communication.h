#ifndef UNHUMAN_MOTORLIB_COMMUNICATION_H_
#define UNHUMAN_MOTORLIB_COMMUNICATION_H_

#include "messages.h"
#include "task.h"
#include <cstring>

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

    // requires that derived class supports write_async
    template <typename Self>
    requires requires(Self& s, Scheduler& sched, uint8_t* data, uint16_t len) {
        { s.write_async(sched, data, len) } -> std::same_as<Task<int>>;
    }
    Task<bool> send_string_async(this Self&& self, Scheduler &sched, const char* string, uint16_t length) {
      using ActualSelf = std::decay_t<Self>;
      constexpr uint32_t MAX_SIZE = ActualSelf::MAX_PACKET_TRANSFER_SIZE;
      constexpr uint32_t MAX_PAYLOAD = MAX_SIZE - sizeof(APIControlPacket);

     // if string is binary (starts with 0? or is long, send as a series of chunks
      if (length && (string[0] == 0 || length > MAX_SIZE - 1)) {
        struct {
          APIControlPacket control_packet = {.control_packet_id = 0,
                                             .type = LONG_PACKET,
                                             .long_packet = {0, 1}};
          char data[MAX_PAYLOAD];
        } long_packet;
        long_packet.control_packet.long_packet.total_length = length;
        int32_t length_remaining = length;
        const char * str = string;
        do {
          uint16_t transfer_size = std::min((uint16_t) MAX_PAYLOAD, (uint16_t) length_remaining);
          std::memcpy(long_packet.data, str, transfer_size);
          int retval = co_await self.write_async(sched, (uint8_t * const) &long_packet, 
                  transfer_size + sizeof(APIControlPacket));
          if (retval < 0) {
            co_return false;
          }
          str += transfer_size;
          long_packet.control_packet.long_packet.packet_number++;
          length_remaining -= transfer_size;
        } while (length_remaining > 0);
      } else {
        char buf[64];
        std::memcpy(buf, string, length);
        if (length > 1) {
          buf[length++] = 0;
        }
        int retval = co_await self.write_async(sched, (uint8_t*) buf, length);
        co_return retval >= 0;
      }
      co_return true;
    }
};

#endif  // UNHUMAN_MOTORLIB_COMMUNICATION_H_
