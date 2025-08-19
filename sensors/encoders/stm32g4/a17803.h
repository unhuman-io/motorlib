#pragma once

#include <bit>
#include "../encoder.h"

#define A17803_SET_DEBUG_API(prefix, api, encoder) \
      api.add_api_variable(prefix "a17803", new const APICallbackHex<uint32_t>([](){ \
        encoder.trigger(); return (uint32_t) encoder.read(); })); \
      api.add_api_variable(prefix "diag_str", new const APICallback([](){ \
        return A17803::get_diag_str(encoder.message_.diag); })); \

class A17803 : public EncoderBase {
  public:
    union A17803_Message {
        struct {
            uint32_t crc: 5;
            uint32_t s0: 1;
            uint32_t diag: 16;
            uint32_t s1: 1;
            uint32_t frame_count: 3;
            uint32_t prev_address: 5;
            uint32_t start_bit: 1;
        };
        struct {
            uint32_t crc: 5;
            uint32_t crc_bits: 26;
        } crc_view;
        struct {
            uint32_t crc: 5;
            uint32_t s0: 1;
            uint32_t angle: 16;
        } angle_view;
    };

    static constexpr std::string_view diag_strs[16] = {
        "spe", "sat", "spd", "ica",
        "ofe", "vcc", "por", "tde",
        "vcf", "tse", "sme", "ese",
        "eue", "bsy", "xee", "ier"
    };

    A17803(SPIDMA &spidma) : spidma_(spidma) {
    }

    void trigger() {
        spidma_.start_readwrite_isr((uint8_t *) &position_command_,
            (uint8_t *) &received_data_, sizeof(position_command_));
    }
    int32_t read() {
        spidma_.finish_readwrite_isr();
        uint32_t rev = __builtin_bswap32(received_data_);
        A17803_Message new_message = std::bit_cast<A17803_Message>(rev);
        if (new_message.crc != crc_calc(new_message.crc_view.crc_bits)) [[unlikely]] {
            crc_error_count_++;
        }
        if (((new_message.frame_count - message_.frame_count) & 0x7) != 1) [[unlikely]] {
            frame_error_count_++;
        }
        int16_t diff = new_message.angle_view.angle - message_.angle_view.angle;
        angle_ += diff;
        message_ = new_message;
        return get_value();
    }

    int32_t get_value() const {
        return angle_;
    }

    static std::string get_diag_str(uint16_t diag_bits) {
        std::string s;
        for (int i = 0; i < 16; i++) {
            if (diag_bits & (1 << i)) {
                if (!s.empty()) {
                    s += ", ";
                }
                s += diag_strs[i];
            }
        }
        if (s.empty()) {
            s = "none";
        }
        return s;
    }

    void clear_faults() {
        crc_error_count_ = 0;
        frame_error_count_ = 0;
    }

    uint32_t crc_calc(uint32_t crc_bits) {
        CRC->POL = 0x2800'0000; // CRC 0b100101, aka 0x25 left aligned
        CRC->INIT = 0x03E0'0000; // seed value 0b11111
        CRC->CR = CRC_CR_RESET;
        CRC->DR = crc_bits;
        return CRC->DR >> 27;
    }
    
    A17803_Message message_;
    int32_t angle_ = 0;
    uint32_t crc_error_count_ = 0;
    uint32_t frame_error_count_ = 0;
  private:
    SPIDMA &spidma_;
    uint32_t position_command_ = __builtin_bswap32(0x1200'001e);
    uint32_t received_data_;

};
