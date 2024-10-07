#pragma once

#include "../communication.h"
#include <cstring>

template <class CAN>
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

    CANCommunication(CAN &can, uint8_t address) : can_(can), address_(address) {
      CANID can_id = {.address = address_, .message_id = OBOT_CMD};
      can_.add_acceptance_filter(can_id.word, 0);
      can_id.message_id = OBOT_CMD_STATUS;
      can_.add_acceptance_filter(can_id.word, 0);
      can_id.message_id = OBOT_ASCII;
      can_.add_acceptance_filter(can_id.word, 1);
      can_id.message_id = OBOT_ENUM;
      can_id.address = 0;
      can_.add_acceptance_filter(can_id.word, 0);
    };

    int receive_data(ReceiveData* const data) {
      CANID can_id = {.address = address_, .message_id = OBOT_CMD};
      int recv_len = can_.read(0, can_id.word, (uint8_t*) data);

      if (recv_len < 0) {
        can_id.message_id = OBOT_CMD_STATUS;
        recv_len = can_.read(0, can_id.word, (uint8_t*) data);
        if (recv_len < 0) {
          CANID can_id_enum = {.message_id = OBOT_ENUM};
          recv_len = can_.read(0, can_id_enum.word, (uint8_t*) nullptr);
          if (recv_len >= 0) {
            //logger.log("recv enum");
            can_id.message_id = OBOT_ENUM;
            can_.write(can_id.word, nullptr, 0);
          }
        } else if (recv_len > 0) {
          send_data_trigger_ = true;
        }
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
        send_data_counter_ = 0;
      }
    }

    int receive_string(char* const string) {
      CANID can_id = {.address = address_, .message_id = OBOT_ASCII};
      int recv_len = can_.read(1, can_id.word, (uint8_t*) string);
      if (recv_len == 0) {
        string[0] = 0;
      }
      return recv_len;
    }

    bool send_string(const char* string, uint16_t length) {
      CANID can_id = {.address = address_, .message_id = OBOT_ASCII_RESPONSE};
      if (string[0] == 0 || length > MAX_CAN_DATA_SIZE) {
        struct {
          APIControlPacket control_packet = {0, LONG_PACKET, .long_packet = {0, 1}};
          char data[MAX_CAN_DATA_SIZE - sizeof(APIControlPacket)];
        } long_packet;
        long_packet.control_packet.long_packet.total_length = length;
        int32_t length_remaining = length;
        const char * str = string;
        do {
          uint16_t transfer_size = std::min((uint16_t) (MAX_CAN_DATA_SIZE - sizeof(APIControlPacket)), (uint16_t) length_remaining);
          std::memcpy(long_packet.data, str, transfer_size);
          can_.write(can_id.word, (uint8_t * const) &long_packet, 
                  transfer_size + sizeof(APIControlPacket));
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
        can_.write(can_id.word, (uint8_t*) buf, length);
      }
      return true;
    }

    void send_one_time_api_timeout_request(uint32_t us) {
       APIControlPacket timeout_request = {0, TIMEOUT_REQUEST, .timeout_request = {us}};
       CANID can_id = {.address = address_, .message_id = OBOT_ASCII_RESPONSE};
       can_.write(can_id.word, (uint8_t * const) &timeout_request, sizeof(timeout_request));
    }

 private:
    CAN &can_;
    volatile bool send_data_trigger_ = false;
    uint16_t send_data_counter_ = 0;
    uint16_t send_data_default_decimation_ = 1000;
    uint8_t address_ = 0;
};
