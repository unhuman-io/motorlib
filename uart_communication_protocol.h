#pragma once
#include <cstdint>

class CommunicationProtocolBase {
  public:
    typedef void (*communication_callback)(void *, uint8_t*, uint16_t);
    // callback which is called when frame id fid is received
    void register_comms_inst(void *comms_inst) { comms_inst_ = comms_inst; }
    void register_callback(uint8_t fid, communication_callback callback) {}
    void parse(uint16_t current_index) {}
  protected:
    void * comms_inst_ = 0;
};

//int n_parses = 0;
template<uint8_t n_fids=4>
// A simple protocol that uses 2x parse period timeout to separate frames
class UARTRawProtocol : public CommunicationProtocolBase {
  public:
    // better load all the callbacks
    void register_callback(uint8_t fid, communication_callback callback) {
      callbacks_[fid-1] = callback; 
    }

    // call this at a high frequency e.g. 10 kHz
    void parse(uint16_t current_index) {
      parses_++;
      if (current_index == last_index_ &&
          current_index != tail_) {
        // no new data, time to parse
        uint8_t fid = rx_buffer_[tail_];
        if (fid && fid <= n_fids) {
          uint16_t length = current_index - tail_;
          if (current_index < tail_) {
            length = 0; // todo figure out unwrap;
          } else {
            // todo need to unwrap
            // n_parses++;
            // if (n_parses > 156) {
            //   asm("bkpt 1");
            // }
            callbacks_[fid-1](comms_inst_, (uint8_t *) &rx_buffer_[tail_+1], length);
          }
        }
        tail_ = current_index;
      }
      last_index_ = current_index;
    }
    void set_buffer(uint8_t *buf) { rx_buffer_ = buf; }
  private:
    volatile uint8_t *rx_buffer_;
    uint16_t last_index_ = 0, tail_ = 0;
    uint32_t parses_ = 0;
    communication_callback callbacks_[n_fids];
};
