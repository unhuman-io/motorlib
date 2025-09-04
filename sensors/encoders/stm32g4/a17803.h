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
      api.add_api_variable(prefix "x_ehc", new const APICallbackInt16([]{\
        int16_t ret = encoder.read_reg(A17803::PrimaryAddress::X_EHC).data;\
        return ret;}));\
      api.add_api_variable(prefix "y_ehc", new const APICallbackInt16([]{\
        int16_t ret = encoder.read_reg(A17803::PrimaryAddress::Y_EHC).data;\
        return ret;}));\
      api.add_api_variable(prefix "ctrl", new APICallbackHex<uint16_t>([](){ \
        uint16_t ret = encoder.read_reg(A17803::PrimaryAddress::CTRL).data; \
        return ret; }, \
        [](uint16_t value){ encoder.write_reg(A17803::PrimaryAddress::CTRL, value); })); \
      api.add_api_variable(prefix "loopback", new APICallbackHex<uint16_t>([](){ \
        uint16_t ret = encoder.read_reg(A17803::PrimaryAddress::LOOPBACK).data; \
        return ret; }, \
        [](uint16_t value){ encoder.write_reg(A17803::PrimaryAddress::LOOPBACK, value); })); \
      api.add_api_variable(prefix "reg31", new APICallbackHex<uint32_t>([]{\
        return encoder.read_extended_reg(0x31);\
      }, [](uint32_t value){ encoder.write_extended_reg(0x31, value); }));\
      api.add_api_variable(prefix "reg32", new APICallbackHex<uint32_t>([]{\
        return encoder.read_extended_reg(0x32);\
      }, [](uint32_t value){ encoder.write_extended_reg(0x32, value); }));\
      api.add_api_variable(prefix "reg39", new APICallbackHex<uint32_t>([]{\
        return encoder.read_extended_reg(0x39);\
      }, [](uint32_t value){ encoder.write_extended_reg(0x39, value); }));\
      api.add_api_variable(prefix "reg3c", new APICallbackHex<uint32_t>([]{\
        return encoder.read_extended_reg(0x3c);\
      }, [](uint32_t value){ encoder.write_extended_reg(0x3c, value); }));\
      api.add_api_variable(prefix "reg1c", new APICallbackHex<uint32_t>([]{\
        return encoder.read_extended_reg(0x1c);\
      }, [](uint32_t value){ encoder.write_extended_reg(0x1c, value); }));\
      api.add_api_variable(prefix "stuff", new const APICallbackHex<uint32_t>([]{\
        return encoder.read_extended_reg(0x30);\
      }));\
      api.add_api_variable(prefix "speed", new const APICallbackInt16([]{\
        int16_t ret = encoder.read_reg(0x0a).data;\
        return ret;\
      }));\
      api.add_api_variable(prefix "factory", new const APICallbackHex<uint32_t>([]{\
        return encoder.read_extended_reg(0);\
      }));\
      api.add_api_variable(prefix "factory2", new const APICallbackHex<uint32_t>([]{\
        return encoder.read_extended_reg(1);\
      }));\
      api.add_api_variable(prefix "unlock", new const APICallbackHex<uint16_t>([]{\
        return encoder.unlock();\
      }));\
      api.add_api_variable(prefix "trim_sens", new const APICallbackHex<uint16_t>([]{\
        return static_cast<uint16_t>(encoder.read_extended_reg(0x51));\
      }));\
      api.add_api_variable(prefix "reg35", new APICallbackHex<uint32_t>([]{\
        return encoder.read_extended_reg(0x35);\
      }, [](uint32_t value){ encoder.write_extended_reg(0x35, value); }));\
      api.add_api_variable(prefix "y_gain", new const APICallbackHex<uint32_t>([]{\
        return encoder.read_extended_reg(0x5a);\
      }));\
      api.add_api_variable(prefix "y_offset", new const APICallbackHex<uint32_t>([]{\
        return encoder.read_extended_reg(0x5b);\
      }));\
      api.add_api_variable(prefix "x_gain", new const APICallbackHex<uint32_t>([]{\
        return encoder.read_extended_reg(0x5c);\
      }));\
      api.add_api_variable(prefix "x_offset", new const APICallbackHex<uint32_t>([]{\
        return encoder.read_extended_reg(0x5d);\
      }));\
      api.add_api_variable(prefix "x_ref_amp", new const APICallbackHex<uint32_t>([]{\
        return encoder.read_extended_reg(0x5e);\
      }));\
      api.add_api_variable(prefix "x_ref_offset", new const APICallbackHex<uint32_t>([]{\
        return encoder.read_extended_reg(0x5f);\
      }));\
      api.add_api_variable(prefix "y_ref_amp", new const APICallbackHex<uint32_t>([]{\
        return encoder.read_extended_reg(0x60);\
      }));\
      api.add_api_variable(prefix "y_ref_offset", new const APICallbackHex<uint32_t>([]{\
        return encoder.read_extended_reg(0x61);\
      }));\
      api.add_api_variable(prefix "cs3", new APICallbackHex<uint32_t>([]{\
        return encoder.read_extended_reg(0x1a);\
      }, [](uint32_t value){ encoder.write_extended_reg(0x1a, value); }));\
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
        uint8_t bytes[4];
        uint32_t word;
    };

    struct A17803_Message_Rev {
      uint32_t word;
      constexpr operator A17803_Message() const {
        return std::bit_cast<A17803_Message>(__builtin_bswap32(word));
      }
      A17803_Message_Rev() = default;
      constexpr A17803_Message_Rev(const A17803_Message& message) {
        word = __builtin_bswap32(std::bit_cast<uint32_t>(message));
      }
    };

    enum class PrimaryAddress : uint8_t {
      INDIRECT_WR_ADDRESS = 0x1,
      INDIRECT_WR_DATA_MSB = 0x2,
      INDIRECT_WR_DATA_LSB = 0x3,
      INDIRECT_WR_STATUS = 0x4,
      INDIRECT_RD_ADDRESS = 0x5,
      INDIRECT_RD_STATUS = 0x6,
      INDIRECT_RD_DATA_MSB = 0x7,
      INDIRECT_RD_DATA_LSB = 0x8,
      ANGLE = 0x9,
      SPEED = 0xa,
      CTRL = 0xd,
      ERROR = 0xe,
      TEMPERATURE = 0xf,
      X_EHC = 0x10,
      Y_EHC = 0x11,
      ACCESS = 0x1e,
      LOOPBACK = 0x1f
    };

    static constexpr std::string_view diag_strs[16] = {
        "spe", "sat", "spd", "ica",
        "ofe", "vcc", "por", "tde",
        "vcf", "tse", "sme", "ese",
        "eue", "bsy", "xee", "ier"
    };

    static consteval uint32_t crc_calc_consteval(uint32_t crc_bits) {
        uint32_t crc = 0x1F; // seed value
        for (int i = 0; i < 26; i++) {
            bool bit = ((crc_bits >> (25 - i)) & 1) ^ ((crc >> 4) & 1);
            crc = (crc << 1) & 0x1F;
            if (bit) {
                crc ^= 0x25;
            }
        }
        return crc & 0x1F;
    }
    static consteval A17803_Message make_reg_message_consteval(uint8_t reg) {
        A17803_Message message {
            .request_view = {
                .address = reg,
            }
        };
        message.request_view.crc = crc_calc_consteval((reg & 0x1f) << 20);
        return message;
    }
    static consteval A17803_Message make_reg_message_consteval(PrimaryAddress reg) {
        return make_reg_message_consteval(static_cast<uint8_t>(reg));
    }

    A17803(SPIDMA &spidma) : spidma_(spidma) {
    }

    void trigger() {
        spidma_.start_readwrite_isr((uint8_t *) &position_command_,
            (uint8_t *) &received_data_, sizeof(position_command_));
    }
    int32_t read() {
        spidma_.finish_readwrite_isr();
        if (!spidma_.pause_.is_paused()) {
          read_buf(received_data_);
        }
        return get_value();
    }

    int32_t read_buf(const A17803_Message new_message) {
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
        return get_value();
    }

    int32_t get_value() const {
        return angle_;
    }

    A17803_Message make_reg_message(uint8_t reg, bool do_crc = true) {
        // A17803_Message message {};

        // message.request_view.address = reg;

        A17803_Message message {
            .request_view = {
                .address = reg,
            }
        };
        if (do_crc) {
            spidma_.claim();
            message.request_view.crc = crc_calc(message.crc_view.crc_bits);
            spidma_.release();
        }
        return message;
    }



    A17803_Message read_reg(PrimaryAddress reg) {
        return read_reg(static_cast<uint8_t>(reg));
    }

    A17803_Message read_reg(uint8_t reg) {
        A17803_Message_Rev message_in_rev;
        A17803_Message reg_message = make_reg_message(reg);
        A17803_Message_Rev reg_message_rev(reg_message);
        spidma_.claim();
        spidma_.readwrite((uint8_t *) &reg_message_rev, (uint8_t *) &message_in_rev, sizeof(reg_message_rev));
        ns_delay(200);
        A17803_Message message_in(message_in_rev);
        //logger.log_printf("frame_count: %d", message.frame_count);
        spidma_.readwrite((uint8_t *) &position_command_, (uint8_t *) &message_in_rev, sizeof(reg_message_rev));
        ns_delay(200);
        message_in = message_in_rev;
        message_.frame_count = message_in.frame_count; // to prevent frame count errors in read
       
        //logger.log_printf("frame_count2: %d", message.frame_count);
        if (message_in.crc != crc_calc(message_in.crc_view.crc_bits)) [[unlikely]] {
            crc_error_count_++;
        } else if (message_in.address != reg) [[unlikely]] {
            // logger.log_printf("A17803: Unexpected address 0x%02X, expected 0x%02X",
            //     message.address, reg);
            // logger.log_printf("A17803: sent: %08x crc %02x", rev1, crc_calc(reg_message.crc_view.crc_bits));
        }
        spidma_.release();
        return message_in;
    }

    uint32_t read_extended_reg(uint8_t reg) {
        spidma_.claim();
        write_reg(PrimaryAddress::INDIRECT_RD_ADDRESS, reg);
        write_reg(PrimaryAddress::INDIRECT_RD_STATUS, 0x8000);
        A17803_Message message = read_reg(PrimaryAddress::INDIRECT_RD_STATUS);
        if (message.data & 0x01) [[likely]] {
          // read is ready
        } else {
          // are eeprom reads slower? Datasheet says 2 us. This doesn't trigger so far though
          logger.log_printf("A17803: read_extended_reg(0x%02X) not ready, diag: %04X", reg, message.data);
        }
        
        uint32_t value = read_reg(PrimaryAddress::INDIRECT_RD_DATA_MSB).data << 16;
        value |= read_reg(PrimaryAddress::INDIRECT_RD_DATA_LSB).data;
        spidma_.release();
        return value;
    }

    void write_extended_reg(uint8_t reg, uint32_t value) {
        spidma_.claim();
        write_reg(PrimaryAddress::INDIRECT_WR_ADDRESS, reg);
        write_reg(PrimaryAddress::INDIRECT_WR_DATA_MSB, value >> 16);
        write_reg(PrimaryAddress::INDIRECT_WR_DATA_LSB, value & 0xffff);
        write_reg(PrimaryAddress::INDIRECT_WR_STATUS, 0x8000);
        // would need to wait 6.5 ms for write to complete
        spidma_.release();
    }

    void write_reg(PrimaryAddress reg, uint16_t value) {
        write_reg(static_cast<uint8_t>(reg), value);
    }

    void write_reg(uint8_t reg, uint16_t value) {
        A17803_Message message = make_reg_message(reg, false);
        message.request_view.data = value;
        message.request_view.rw = 1; // write
        spidma_.claim();
        message.request_view.crc = crc_calc(message.crc_view.crc_bits);
        A17803_Message_Rev message_rev(message);
   
        A17803_Message_Rev message_in_rev;
        spidma_.readwrite((uint8_t *) &message_rev, (uint8_t *) &message_in_rev, sizeof(message_rev));
        ns_delay(200);
        A17803_Message message_in(message_in_rev);
        message_.frame_count = message_in.frame_count; // to prevent frame count errors in read
        spidma_.release();

    }

    uint16_t unlock() {
        logger.log_printf("A17803: unlocking %04X", read_reg(PrimaryAddress::ACCESS).data);
        write_reg(PrimaryAddress::ACCESS, 0xC418);
        write_reg(PrimaryAddress::ACCESS, 0x0e80);

        return read_reg(PrimaryAddress::ACCESS).data;
    }

    uint16_t get_diag() {
        last_diag_ = read_reg(PrimaryAddress::ERROR).data;
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
        return (read_reg(PrimaryAddress::TEMPERATURE).temperature_view.temperature) * (1.0 / 13.3226) + 25;
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
    A17803_Message_Rev position_command_ = make_reg_message_consteval(PrimaryAddress::ANGLE);
    A17803_Message_Rev received_data_;

};
