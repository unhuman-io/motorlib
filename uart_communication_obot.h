#ifndef UNHUMAN_MOTORLIB_UART_COMMUNICATION_H_
#define UNHUMAN_MOTORLIB_UART_COMMUNICATION_H_

#include "communication.h"
#include "util.h"
#include <cstring>
#include <atomic>
#include <algorithm>

volatile uint32_t status_callbacks = 0;
class UARTCommunication : public CommunicationBase {
 public:
    enum mailboxId
    {
      OBOT_CMD = 0x01,
      OBOT_STATUS = 0x02,
      OBOT_CMD_STATUS = 0x03,
      OBOT_ASCII = 0x04,
    };

    UARTCommunication(Uart &uart, ObotParser &protocol) : uart_(uart), protocol_(protocol) {
      new_ascii_str_ = false;
      new_obot_cmd_ = false;
      status_sent_ = false;

      // protocol_.set_buffer(uart.rx_buffer_);
      // protocol_.register_comms_inst(this);
      protocol_.registerCallback(OBOT_CMD, callback_obot_cmd);
      protocol_.registerCallback(OBOT_STATUS, callback_obot_status);
      protocol_.registerCallback(OBOT_CMD_STATUS, callback_obot_cmd_status);
      protocol_.registerCallback(OBOT_ASCII, callback_obot_ascii);
    

      NVIC_SetPriority(PendSV_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
      NVIC_EnableIRQ(PendSV_IRQn);
    };

    int receive_data(ReceiveData* const data) {
      // this is called from the main loop, trigger a lower priority interrupt here
      // which will processes the packet at higher priority than main() but won't 
      // affect fast_loop and main_loop period timing
      //trigger_parse();
      volatile uint32_t *icsr = (uint32_t *)0xE000ED04;
      // Pend a PendSV exception using by writing 1 to PENDSVSET at bit 28
      *icsr = 0x1 << 28;

      if (new_obot_cmd_) {
        *data = obot_cmd_;
        new_obot_cmd_ = false;
        return sizeof(obot_cmd_);
      }
      return 0;
    }

    void send_data(const SendData& data) {
      obot_status_ = data;
    }

    int receive_string(char* const string) {
      if (new_ascii_str_) {
        std::memcpy(string, ascii_str_in_, std::strlen(ascii_str_in_));
        new_ascii_str_ = false;
        return std::strlen(ascii_str_in_);
      }
      *string = 0;
      return 0;
    }

    bool send_string(const char* string, uint16_t length) {
      //while(send_active());
      // todo packetize
      std::memcpy(uart_.tx_buffer_, string, length);
      Uart::BufferDescriptor desc = {};
      desc.length = length;
      desc.txBuffer = uart_.tx_buffer_;
      uart_.startTransaction(desc);
      return true;
    }

    bool send_string_active() const {
      return send_active();
    }

    bool send_active() const {
      return uart_.is_tx_active();
    }

    bool tx_data_ack() {
      bool status_sent = status_sent_;
      status_sent_ = false;
      return status_sent;
    }

    void parse() {
      protocol_.process(uart_.get_current_rx_index());
    }

    static void callback_obot_cmd(void * inst, uint8_t *buf, uint16_t length) {
      ((UARTCommunication *) inst)->callback_obot_cmd(buf, length);
    }

    static void callback_obot_status(void * inst, uint8_t *buf, uint16_t length) {
      status_callbacks++;
      ((UARTCommunication *) inst)->callback_obot_status(buf, length);
    }

    static void callback_obot_cmd_status(void * inst, uint8_t *buf, uint16_t length) {
      ((UARTCommunication *) inst)->callback_obot_cmd_status(buf, length);
    }

    static void callback_obot_ascii(void * inst, uint8_t *buf, uint16_t length) {
      ((UARTCommunication *) inst)->callback_obot_ascii(buf, length);
    }

    void callback_obot_cmd(uint8_t *buf, uint16_t length) {
      asm("dmb");
      uart_.rx_copy((uint8_t *) &obot_cmd_, buf, std::min((size_t) length, sizeof(obot_cmd_)));
      std::memcpy(&obot_cmd_, buf, sizeof(obot_cmd_));
      new_obot_cmd_ = true;
    }

    // todo potential collisions between obot status and ascii responses
    void callback_obot_status(uint8_t *buf, uint16_t length) {
      asm("dmb");
      // todo packetize
      std::memcpy(&uart_.tx_buffer_[0], &obot_status_, sizeof(SendData));
      Uart::BufferDescriptor desc = {};
      desc.length = sizeof(SendData);
      desc.txBuffer = uart_.tx_buffer_;
      uart_.startTransaction(desc);
      status_sent_ = true;
    }

    void callback_obot_cmd_status(uint8_t *buf, uint16_t length) {
      callback_obot_cmd(buf, length);
      callback_obot_status(buf, length);
    }

    void callback_obot_ascii(uint8_t *buf, uint16_t length) {
      uart_.rx_copy((uint8_t *) ascii_str_in_, buf, std::min((int) length, MAX_API_DATA_SIZE));
      ascii_str_in_[length] = 0;
      new_ascii_str_ = true;
    }

 private:
    Uart &uart_;
    ObotParser &protocol_;
    friend class System;
    std::atomic_bool new_obot_cmd_;
    ReceiveData obot_cmd_;
    SendData obot_status_;
    std::atomic_bool status_sent_;
    std::atomic_bool new_ascii_str_;
    char ascii_str_in_[MAX_API_DATA_SIZE+1];
};


#endif  // UNHUMAN_MOTORLIB_UART_COMMUNICATION_H_
