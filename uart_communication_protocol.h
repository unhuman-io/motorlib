#pragma once
#include <cstdint>
#include <functional>

class CommunicationProtocolBase {
  public:
    // callback which is called when frame id fid is received
    void register_callback(uint8_t fid, std::function<void(uint8_t*, uint16_t)> callback) {}
    void parse(uint16_t current_index) {}
};

class UARTRawProtocol : public CommunicationProtocolBase {
  public:
    void register_callback(uint8_t fid, std::function<void(uint8_t* a, uint16_t b)> callback) {
      funs_[fid-1] = callback; 
    }
    void parse(uint16_t current_index) {
      parses_++;
      if (parses_ % 1000 == 0) {
        funs_[1](0,0);
      }
    }
  private:
    uint32_t parses_ = 0;
    std::function<void(uint8_t* a, uint16_t b)> funs_[4];
};
