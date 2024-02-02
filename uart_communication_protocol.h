#pragma once
#include <cstdint>
#include <functional>

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

class UARTRawProtocol : public CommunicationProtocolBase {
  public:
    void register_callback(uint8_t fid, communication_callback callback) {
      //asm("bkpt 2");
      callbacks_[fid-1] = callback; 
    }
    void parse(uint16_t current_index) {
      parses_++;
      if (parses_ % 1000 == 0) {
        //asm("bkpt 1");
        callbacks_[1](comms_inst_, 0, 0);
      }
    }
  private:
    volatile uint32_t parses_ = 0;
    communication_callback callbacks_[4];
};
