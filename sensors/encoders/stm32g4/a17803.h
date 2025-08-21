#pragma once

#include <bit>
#include "../encoder.h"

#define A17803_SET_DEBUG_API(prefix, api, encoder) \
      api.add_api_variable(prefix "a17803", new const APICallbackHex<uint32_t>([](){ \
        encoder.trigger(); return (uint32_t) encoder.read(); })); \
      api.add_api_variable(prefix "diag_str", new const APICallback([](){ \
        return encoder.get_diag_str(); })); \
      api.add_api_variable(prefix "diag", new const APICallbackHex<uint16_t>([](){ \
        return encoder.get_diag(); })); \
      api.add_api_variable(prefix "temp", new const APICallbackFloat([](){ \
        return encoder.get_temperature(); }));\
      api.add_api_variable(prefix "ctrl", new APICallbackHex<uint16_t>([](){ \
        uint16_t ret = encoder.read_reg(0xd).data; \
        return ret; }, \
        [](uint16_t value){ encoder.write_reg(0xd, value); })); \
      api.add_api_variable(prefix "s0_s1_flag_count", new const APIUint32(&encoder.s0_s1_flag_count_)); \
      api.add_api_variable(prefix "frame_error_count", new const APIUint32(&encoder.frame_error_count_)); \
      api.add_api_variable(prefix "crc_error_count", new const APIUint32(&encoder.crc_error_count_)); \
      api.add_api_variable(prefix "pause_reads", new const APICallback([](){ encoder.spidma_.claim(); return std::string("ok"); })); \
      api.add_api_variable(prefix "resume_reads", new const APICallback([](){ encoder.spidma_.release(); return std::string("ok"); }));

class A17803 : public EncoderBase {
  public:
    union A17803_Message {
        struct {
            uint32_t crc: 5;
            uint32_t s0: 1;
            uint32_t data: 16;
            uint32_t s1: 1;
            uint32_t frame_count: 3;
            uint32_t address: 5;
            uint32_t start_bit: 1;
        };
        struct {
            uint32_t crc: 5;
            uint32_t imm1: 1;
            uint32_t data: 16;
            uint32_t imm2: 3;
            uint32_t address: 5;
            uint32_t rw: 1;         // 1 to write
            uint32_t start_bit: 1;  // set to 0
        } request_view;
        struct {
            uint32_t crc: 5;
            uint32_t crc_bits: 26;
        } crc_view;
        struct {
            uint32_t crc: 5;
            uint32_t s0: 1;
            uint32_t angle: 16;
        } angle_view;
        struct {
            uint32_t dc: 9;
            int32_t temperature: 13;
        } temperature_view;
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
        if (!spidma_.pause_.is_paused()) {
            spidma_.start_readwrite_isr((uint8_t *) &position_command_,
                (uint8_t *) &received_data_, sizeof(position_command_));
        }
    }
    int32_t read() {
        if (!spidma_.pause_.is_paused()) {
            spidma_.finish_readwrite_isr();
            uint32_t rev = __builtin_bswap32(received_data_);
            A17803_Message new_message = std::bit_cast<A17803_Message>(rev);
            if (new_message.crc != crc_calc(new_message.crc_view.crc_bits)) [[unlikely]] {
                crc_error_count_++;
            }
            if (((new_message.frame_count - message_.frame_count) & 0x7) != 1) [[unlikely]] {
                frame_error_count_++;
            }
            if (new_message.s0 || new_message.s1) [[unlikely]] {
                s0_s1_flag_count_++;
            }
            int16_t diff = new_message.angle_view.angle - message_.angle_view.angle;
            angle_ += diff;
            message_ = new_message;
        }
        return get_value();
    }

    int32_t get_value() const {
        return angle_;
    }

    A17803_Message make_reg_message(uint8_t reg) {
        // A17803_Message message {};

        // message.request_view.address = reg;

        A17803_Message message {
            .request_view = {
                .address = reg,
            }
        };
        
        message.request_view.crc = crc_calc(message.crc_view.crc_bits);
        return message;
    }

    A17803_Message read_reg(uint8_t reg) {
        uint8_t data_in[4];
        A17803_Message reg_message = make_reg_message(reg);
        uint32_t rev = __builtin_bswap32(std::bit_cast<uint32_t>(reg_message));
        spidma_.claim();
        spidma_.readwrite((uint8_t *) &rev, data_in, sizeof(data_in));
        rev = __builtin_bswap32(*reinterpret_cast<uint32_t *>(data_in));
        A17803_Message message = std::bit_cast<A17803_Message>(rev);
        //logger.log_printf("frame_count: %d", message.frame_count);
        spidma_.readwrite((uint8_t *) &position_command_, data_in, sizeof(data_in));
        
        rev = __builtin_bswap32(*reinterpret_cast<uint32_t *>(data_in));
        message = std::bit_cast<A17803_Message>(rev);
        message_.frame_count = message.frame_count; // to prevent frame count errors in read
        spidma_.release();
        //logger.log_printf("frame_count2: %d", message.frame_count);
        if (message.crc != crc_calc(message.crc_view.crc_bits)) [[unlikely]] {
            crc_error_count_++;
        } else if (message.address != reg) {
            logger.log_printf("A17803: Unexpected address 0x%02X, expected 0x%02X",
                message.address, reg);
        }
        return message;
    }

    void write_reg(uint8_t reg, uint16_t value) {
        A17803_Message message = make_reg_message(reg);
        message.request_view.data = value;
        message.request_view.rw = 1; // write
        message.request_view.crc = crc_calc(message.crc_view.crc_bits);
        uint32_t rev = __builtin_bswap32(std::bit_cast<uint32_t>(message));
        spidma_.claim();
        uint8_t data_in[4];
        spidma_.readwrite((uint8_t *) &rev, data_in, sizeof(rev));
        rev = __builtin_bswap32(*reinterpret_cast<uint32_t *>(data_in));
        message = std::bit_cast<A17803_Message>(rev);
        message_.frame_count = message.frame_count; // to prevent frame count errors in read
        spidma_.release();

    }

    uint16_t get_diag() {
        last_diag_ = read_reg(0xe).data;
        return last_diag_;
    }

    std::string get_diag_str() {
        uint16_t diag = get_diag();
        return get_diag_str(diag);
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

    float get_temperature() __attribute__((externally_visible)) {
        return (read_reg(0xf).temperature_view.temperature) * (1.0 / 13.3226) + 25;
    }

    void clear_faults() {
        crc_error_count_ = 0;
        frame_error_count_ = 0;
        s0_s1_flag_count_ = 0;
    }

    uint32_t crc_calc(uint32_t crc_bits) {
        CRC->POL = 0x2800'0000; // CRC 0b100101, aka 0x25 left aligned
        CRC->INIT = 0x03E0'0000; // seed value 0b11111
        CRC->CR = CRC_CR_RESET;
        CRC->DR = crc_bits;
        return CRC->DR >> 27;
    }
    
    A17803_Message message_;
    uint16_t last_diag_;
    int32_t angle_ = 0;
    uint32_t crc_error_count_ = 0;
    uint32_t frame_error_count_ = 0;
    uint32_t s0_s1_flag_count_ = 0;
  //private:
    SPIDMA &spidma_;
    uint32_t position_command_ = __builtin_bswap32(0x1200'001e);
    uint32_t received_data_;

};
