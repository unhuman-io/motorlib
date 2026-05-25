#pragma once

#include "../communication.h"
#include <cstring>
#include <algorithm>
#include "task.h"

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
    static constexpr uint32_t MAX_PACKET_TRANSFER_SIZE = MAX_CAN_DATA_SIZE;

    CANCommunication(CAN &can, uint8_t address) : can_(can), address_(address) {
      CANID can_id = {.address = address_, .message_id = OBOT_CMD};
      can_.add_acceptance_filter(can_id.word, 0);
      can_id.message_id = OBOT_CMD_STATUS;
      can_.add_acceptance_filter(can_id.word, 0);
      can_id.message_id = OBOT_STATUS;
      can_.add_acceptance_filter(can_id.word, 0);
      can_id.message_id = OBOT_ASCII;
      can_.add_acceptance_filter(can_id.word, 1);
      can_id.message_id = OBOT_ENUM;
      can_id.address = 0x7f;
      can_.add_acceptance_filter(can_id.word, 0);
    };

    int receive_data(ReceiveData* const data) {
        CANID can_id = {.address = address_, .message_id = OBOT_CMD};
        if (int recv_len = can_.read(0, can_id.word, (uint8_t*)data);
            recv_len > 0) {
            return recv_len;
        }

        can_id.message_id = OBOT_CMD_STATUS;
        if (int recv_len = can_.read(0, can_id.word, (uint8_t*)data);
            recv_len > 0) {
            send_data_trigger_ = true;
            return recv_len;
        }

        can_id.message_id = OBOT_STATUS;
        if (int recv_len = can_.read(0, can_id.word, (uint8_t*)nullptr);
            recv_len == 0) {
            send_data_trigger_ = true;
            return 0;
        }

        CANID can_id_enum = {.address = 0x7f, .message_id = OBOT_ENUM};
        if (int recv_len = can_.read(0, can_id_enum.word, (uint8_t*)nullptr);
            recv_len == 0) {
            can_id.message_id = OBOT_ENUM;
            can_.write(can_id.word, nullptr, 0, 2);
            return 0;
        }

        return 0;
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
      if (recv_len < 0) {
        string[0] = 0;
      }
      return recv_len;
    }

    bool send_string(const char* string, uint16_t length) {
        Scheduler sched;
        auto task = send_string_async(sched, string, length);
        while (!task.is_done()) {
            sched.poll();
        }
        return task.get_result();
    }

    Task<int> write_async(Scheduler& sched, uint8_t* data, uint16_t len) {
        CANID can_id = {.address = address_, .message_id = OBOT_ASCII_RESPONSE};
        co_return co_await can_.write_async(sched, can_id.word, data, len, 1);
    }

    void set_send_decimation(uint16_t decimation) {
      send_data_default_decimation_ = decimation;
    }

    uint16_t get_send_decimation() const {
      return send_data_default_decimation_;
    }

    void send_one_time_api_timeout_request(uint32_t us) {
       APIControlPacket timeout_request = {.control_packet_id = 0,
                                           .type = TIMEOUT_REQUEST,
                                           .timeout_request = {us}};
       CANID can_id = {.address = address_, .message_id = OBOT_ASCII_RESPONSE};
       can_.write(can_id.word, (uint8_t * const) &timeout_request, sizeof(timeout_request));
    }

 private:
    CAN &can_;
    volatile bool send_data_trigger_ = false;
    uint16_t send_data_counter_ = 0;
    uint16_t send_data_default_decimation_ = 10000;
    uint8_t address_ = 0;
};
