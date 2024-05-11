#pragma once

#include "../communication.h"
#include <cstring>


class CANCommunication : public CommunicationBase {
 public:
    union CANID {
        struct {
            uint16_t address:7;
            uint16_t message_id:4;
        };
        uint16_t word;
    };
    enum MessageID
    {
      OBOT_CMD = 0x01,
      OBOT_CMD_STATUS = 0x02,
      OBOT_STATUS = 0x03,
      OBOT_ASCII = 0x04,
      OBOT_ASCII_RESPONSE = 0x05,
      OBOT_ENUM = 0xF,
    };

    CANCommunication(CAN &can) : can_(can) {
      CANID can_id = {.address = address_, .message_id = OBOT_CMD};
      can_.add_acceptance_filter(can_id, 0);
      can_id.message_id = OBOT_CMD_STATUS;
      can_.add_acceptance_filter(can_id, 0);
      can_id.message_id = OBOT_ASCII;
      can_.add_acceptance_filter(can_id, 1);
    };

    int receive_data(ReceiveData* const data) {
      CANID can_id = {.address = address_, .message_id = OBOT_CMD};
      int recv_len = can_.read(0, can_id.word, data);
      if (recv_len == 0) {
        can_id.message_id = OBOT_CMD_STATUS;
        recv_len = can_.read(0, can_id.word, data);
        send_data_trigger_ = true;
      }
      return recv_len;
    }

    void send_data(const SendData& data) {
      send_data_counter_++;
      if (send_data_counter_ >= send_data_default_decimation_) {
        send_data_counter_ = 0;
        send_data_trigger_ = true;
      }
      if (send_data_trigger_) {
        CANID can_id = {.address = address_, .message_id = OBOT_STATUS};
        can_.write(can_id.word, (uint8_t*)&data, sizeof(data));
        send_data_trigger_ = false;
      }
    }

    int receive_string(char* const string) {
      CANID can_id = {.address = address_, .message_id = OBOT_ASCII};
      int recv_len = can_.read(1, can_id.word, string);
      return 0;
    }

    bool send_string(const char* string, uint16_t length) {
      CANID can_id = {.address = address_, .message_id = OBOT_ASCII_RESPONSE};
      can_.write(can_id.word, (uint8_t*)&string, std::strlen(string));
      return true;
    }

 private:
    CAN &can_;
    bool send_data_trigger_ = false;
    uint16_t send_data_counter_ = 0;
    uint16_t send_data_default_decimation_ = 100;
    uint8_t address_ = 123;
};
