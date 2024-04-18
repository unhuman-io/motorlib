#ifndef UNHUMAN_MOTORLIB_UART_COMMUNICATION_H_
#define UNHUMAN_MOTORLIB_UART_COMMUNICATION_H_

#include <algorithm>
#include <atomic>
#include <cstring>

#include "communication.h"
#include "util.h"
#include "figure_protocol.h"

class UARTCommunication : public CommunicationBase {
 public:
  enum mailboxId {
    OBOT_CMD = 0x01,
    OBOT_STATUS = 0x02,
    OBOT_CMD_STATUS = 0x03,
    OBOT_ASCII_CMD = 0x04,
    OBOT_ASCII_RESPONSE = 0x05,
  };

  UARTCommunication(Uart& uart, figure::ProtocolParser& protocol) : uart_(uart), protocol_(protocol) {
    new_ascii_str_ = false;
    new_obot_cmd_ = false;
    status_sent_ = false;

    protocol_.registerCallback(OBOT_CMD, [this](const unsigned char* buf, uint16_t length) {
      this->callback_obot_cmd(const_cast<uint8_t*>(buf), length);  // Using const_cast if necessary
    });
    protocol_.registerCallback(OBOT_STATUS, [this](const unsigned char* buf, uint16_t length) {
      this->callback_obot_status(const_cast<uint8_t*>(buf), length);  // Using const_cast if necessary
    });
    protocol_.registerCallback(OBOT_CMD_STATUS, [this](const unsigned char* buf, uint16_t length) {
      this->callback_obot_cmd_status(const_cast<uint8_t*>(buf), length);  // Using const_cast if necessary
    });
    protocol_.registerCallback(OBOT_ASCII_CMD, [this](const unsigned char* buf, uint16_t length) {
      this->callback_obot_ascii(const_cast<uint8_t*>(buf), length);  // Using const_cast if necessary
    });

    NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
    NVIC_EnableIRQ(PendSV_IRQn);
  };

  int receive_data(ReceiveData* const data) {
    // this is called from the main loop, trigger a lower priority interrupt here
    // which will processes the packet at higher priority than main() but won't
    // affect fast_loop and main_loop period timing
    // trigger_parse();
    volatile uint32_t* icsr = (uint32_t*)0xE000ED04;
    // Pend a PendSV exception using by writing 1 to PENDSVSET at bit 28
    *icsr = 0x1 << 28;

    if (new_obot_cmd_) {
      *data = obot_cmd_;
      new_obot_cmd_ = false;
      return sizeof(obot_cmd_);
    }
    return 0;
  }

  void send_data(const SendData& data) { obot_status_ = data; }

  int receive_string(char* const string) {
    if (new_ascii_str_) {
      std::memcpy(string, ascii_str_in_, std::min(std::strlen(ascii_str_in_)+1, (size_t) OBOT_ASCII_MAX_RECEIVE_LENGTH));
      new_ascii_str_ = false;
      return std::strlen(ascii_str_in_);
    }
    *string = 0;
    return 0;
  }

  bool send_string(const char* string, uint16_t length) {
    // while(send_active());
    uint8_t packet_size;
    length = std::min(length, (uint16_t) OBOT_ASCII_MAX_SEND_LENGTH); // todo support larger packets
    uint8_t* packet = protocol_.generatePacket((const uint8_t *) string, length, (size_t) OBOT_ASCII_RESPONSE, &packet_size);
    std::memcpy(&uart_.tx_buffer_[0], &packet[0], packet_size);
    Uart::BufferDescriptor desc = {};
    desc.length = packet_size;
    desc.txBuffer = uart_.tx_buffer_;
    uart_.startTransaction(desc);
    return true;
  }

  bool send_string_active() const { return send_active(); }

  bool send_active() const { return uart_.is_tx_active(); }

  bool tx_data_ack() {
    bool status_sent = status_sent_;
    status_sent_ = false;
    return status_sent;
  }

  void parse() {
    uint16_t current_rx_index_ = uart_.get_last_rx_index();
    if (current_rx_index_ != last_rx_index_) {
      protocol_.process(uart_.get_last_rx_index());
    }
    last_rx_index_ = current_rx_index_;
  }

  void callback_obot_cmd(uint8_t* buf, uint16_t length) {
    asm("dmb");
    std::memcpy(&obot_cmd_, buf, sizeof(obot_cmd_));
    new_obot_cmd_ = true;
  }

  // todo potential collisions between obot status and ascii responses
  void callback_obot_status(uint8_t* buf, uint16_t length) {
    asm("dmb");
    // Packetize obot_status and send
    uint8_t packet_size;
    uint8_t* packet = protocol_.generatePacket(reinterpret_cast<uint8_t*>(&obot_status_), sizeof(SendData), OBOT_STATUS, &packet_size);
    std::memcpy(&uart_.tx_buffer_[0], &packet[0], packet_size);
    Uart::BufferDescriptor desc = {};
    desc.length = packet_size;
    desc.txBuffer = uart_.tx_buffer_;
    uart_.startTransaction(desc);
    status_sent_ = true;
  }

  void callback_obot_cmd_status(uint8_t* buf, uint16_t length) {
    callback_obot_cmd(buf, length);
    callback_obot_status(buf, length);
  }

  void callback_obot_ascii(uint8_t* buf, uint16_t length) {
    length = std::min(length, (uint16_t) MAX_API_DATA_SIZE);
    std::memcpy((uint8_t *) ascii_str_in_, buf, length);
    ascii_str_in_[length] = 0;
    if (length) {
      new_ascii_str_ = true;
    }
  }

 private:
  Uart& uart_;
  figure::ProtocolParser& protocol_;
  friend class System;
  std::atomic_bool new_obot_cmd_;
  ReceiveData obot_cmd_;
  SendData obot_status_;
  std::atomic_bool status_sent_;
  std::atomic_bool new_ascii_str_;
  uint16_t last_rx_index_ = 0;
  char ascii_str_in_[MAX_API_DATA_SIZE + 1];
};

#endif  // UNHUMAN_MOTORLIB_UART_COMMUNICATION_H_
