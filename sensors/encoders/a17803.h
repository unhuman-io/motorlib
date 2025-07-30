#pragma once

#include <bit>
#include "../encoder.h"

#define A17803_SET_DEBUG_API(prefix, api, encoder) \
      api.add_api_variable(prefix "a17803", new const APICallbackHex<uint32_t>([](){ \
        encoder.trigger(); return (uint32_t) encoder.read(); })); \
      api.add_api_variable(prefix "diag_str", new const APICallback([](){ \
        return A17803::get_diag_str(encoder.previous_message_.diag); })); \

class A17803 : public EncoderBase {
  public:
    struct A17803_Message {
        uint32_t crc: 5;
        uint32_t s0: 1;
        uint32_t diag: 16;
        uint32_t s1: 1;
        uint32_t frame_count: 3;
        uint32_t prev_address: 5;
        uint32_t start_bit: 1;
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
        previous_message_ = std::bit_cast<A17803_Message>(rev);
        return previous_message_.diag;
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
    
    A17803_Message previous_message_;
  private:
    SPIDMA &spidma_;
    uint32_t position_command_ = 0x00000000;
    uint32_t received_data_;

};
