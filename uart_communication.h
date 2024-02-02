#ifndef UNHUMAN_MOTORLIB_UART_COMMUNICATION_H_
#define UNHUMAN_MOTORLIB_UART_COMMUNICATION_H_

#include "communication.h"
#include "util.h"
#include <cstring>

volatile uint16_t uart_delay = 5;

extern "C" void PendSV_Handler(void) {
  SET_SCOPE_PIN(C,2);
  us_delay(uart_delay);
  CLEAR_SCOPE_PIN(C,2);
}

class UARTCommunication : public CommunicationBase {
 public:
    enum mailboxId
    {
      OBOT_CMD = 0x01,
      OBOT_STATUS = 0x02,
      OBOT_CMD_STATUS = 0x03,
      OBOT_ASCII = 0x04,
    };

    UARTCommunication(Uart &uart) : uart_(uart) {
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
      return 0;
    }
    int i = 0;
    void send_data(const SendData& data) {
      i++;
      if (i % 1000 == 0) {
        std::memcpy(uart_.tx_buffer_, &data, sizeof(SendData));
        Uart::BufferDescriptor desc = {};
        desc.length = sizeof(SendData);
        desc.txBuffer = uart_.tx_buffer_;
        uart_.startTransaction(desc);
      }
    }

    int receive_string(char* const string) {
      return 0;
    }

    bool send_string(const char* string, uint16_t length) {
      return true;
    }

    bool send_string_active() const {
      return false;
    }

    void cancel_send_string() {
    }

    bool new_rx_data() {
      return false;
    }

    bool any_new_rx_data() {
      return false;
    }

    bool tx_data_ack() {
      return false;
    }

 private:
    Uart &uart_;
    friend class System;
};

#endif  // UNHUMAN_MOTORLIB_UART_COMMUNICATION_H_
