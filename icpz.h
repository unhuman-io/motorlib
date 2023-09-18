#ifndef UNHUMAN_MOTORLIB_ICPZ_H_
#define UNHUMAN_MOTORLIB_ICPZ_H_

#include "encoder.h"
#include "util.h"
#include <vector>

static uint8_t CRC_BiSS_43_30bit (uint32_t w_InputData);

class ICPZ : public EncoderBase {
 public:
    enum Disk{Default, PZ03S, PZ08S};
    ICPZ(SPIDMA &spidma, Disk disk = Default) 
      : spidma_(spidma), disk_(disk) {
      command_[0] = 0xa6; // read position
    }
    union Diag {
      struct {
        uint8_t crc6:6;
        uint8_t nWarn:1;
        uint8_t nErr:1;
      };
      uint8_t word;
    };
    enum Addr {CMD_STAT=0x76, COMMANDS=0x77};
    enum CMD {CONF_WRITE_ALL=0x41, AUTO_ADJ_ANA=0xB0, AUTO_ADJ_DIG=0xB1, AUTO_READJ_DIG=0xB2, AUTO_ADJ_ECC=0xB3};
    bool init() {
      bool success = true;
      // send reboot (currently not working probably)
      // uint8_t data_out[2] = {0x77, 0x10};
      // uint8_t data_in[2];
      // spidma_.readwrite(data_out, data_in, 2);

      //set_register(7, 9, {0});
      success = set_register(0, 0, {3}) ? success : false; // fast speed on port a, set first
      success = set_register(7, 9, {0}) ? success : false; // multiturn data length = 0
      success = set_register(7, 0xA, {0}) ? success : false; // spi_ext = 0
      success = set_register(0, 0xF, {4}) ? success : false; // 0x00 ran_fld = 0 -> never update position based on absolute track after initial, tol 4

      if (disk_ == PZ03S) {
        success = set_register(2, 2, {0, 1}) ? success : false; // fcl = 256
        success = set_register(0, 7, {8 << 4}) ? success : false; // sys_ovr = 8 (don't really need this 8 is default for 2656),
        success = set_register(1, 0xB, {9}) ? success : false; // ai_scale = 9 (1.0048),
      } else if (disk_ == PZ08S) {
        // success = set_register(2, 2, {0, 1}) ? success : false; // fcl = 446
        // success = set_register(0, 7, {8 << 4}) ? success : false; // sys_ovr = 9 (don't really need this 8 is default for 2656),
        // success = set_register(1, 0xB, {9}) ? success : false; // ai_scale = 9 (.9558),
        //FCS 216
        // ai phase -20
      }

      //  command[0] = 0xcf;
      //  command[1] = 0x40;
      //  command[2] = 0x08;     // memory bank 8
      //  spidma_.readwrite(command, data_in, 3);
      //  command[0] = 0xcf;
      //  command[1] = 0x00;
      //  command[2] = 0xbe;     // flexcode 446, 216
      //  command[3] = 0x01;
      //  command[4] = 216;
      //  command[5] = 0x00;
      //  spidma_.readwrite(command, data_in, 6);
      //  command[0] = 0xcf;
      //  command[1] = 0x40;
      //  command[2] = 0x00;     // memory bank 0
      //  spidma_.readwrite(command, data_in, 3);
      //  command[0] = 0xcf;
      //  command[1] = 0x00;
      //  command[2] = 0x03;     
      //  spidma_.readwrite(command, data_in, 3);
      //  command[0] = 0xcf;
      //  command[1] = 0x07;
      //  command[2] = 0x90;     // now 8 // sys_ovr = 9 bit
      //  spidma_.readwrite(command, data_in, 3);
      //  command[0] = 0xcf;
      //  command[1] = 0x0F;
      //  command[2] = 0x00;     // 0x00 ran_fld = 0 -> never update position based on absolute track after initial
      //  spidma_.readwrite(command, data_in, 3);
      //  command[0] = 0xcf;
      //  command[1] = 0x40;
      //  command[2] = 0x01;     // memory bank 1
      //  spidma_.readwrite(command, data_in, 3);
      //  command[0] = 0xcf;
      //  command[1] = 0x08;
      //  command[2] = 0xc0;     // ai_phase 
      //  command[3] = 0xd1;
      //  spidma_.readwrite(command, data_in, 4);
      //  command[0] = 0xcf;
      //  command[1] = 0x0a;
      //  command[2] = 0x00;     // ai_scale
      //  command[3] = 0xcd;
      //  spidma_.readwrite(command, data_in, 4);
      //  command[0] = 0xd9;
      //  command[1] = 0x41;     // write command conf write all    
      //  spidma_.readwrite(command, data_in, 2);
      //  ms_delay(10);
      //  command[0] = 0x81;
      //  command[1] = 0x76;
      //  command[2] = 0x00;     // cmd stat
      //  command[3] = 0x00;
      //  spidma_.readwrite(command, data_in, 4);
      //  if (data_in[3] == 0) { // cmd succeeded
      //    return true;
      //  }
       return success;
    }
    void trigger() {
      if (!*register_operation_) {
        ongoing_read_ = true;
        spidma_.start_readwrite(command_, data_, sizeof(command_));
      }
    }
    int32_t read() {
      if (ongoing_read_) {
        spidma_.finish_readwrite();
        uint32_t data = ((data_[1] << 16) | (data_[2] << 8) | data_[3]) << 8;
        raw_value_ = data | data_[4];
        Diag diag = {.word = data_[4]};
        uint8_t crc6_calc = ~CRC_BiSS_43_30bit(raw_value_ >> 6) & 0x3f;
        error_count_ += !diag.nErr;
        warn_count_ += !diag.nWarn;
        uint8_t crc_error = diag.crc6 == crc6_calc ? 0 : 1;
        crc_error_count_ += crc_error;
        if (!crc_error) {
          int32_t diff = (data - last_data_); // rollover summing
          pos_ += diff/256;
          //pos_ = data/256;
          last_data_ = data;
        }
        if (!diag.nErr) {
          clear_diag();
        }
        ongoing_read_ = false;
      }
      return get_value();
    }
    int32_t get_value() const {
      return pos_;
    }
    bool index_received() const { return true; }

    void set_register_operation() {
       (*register_operation_)++;
    }
    void clear_register_operation() {
       (*register_operation_)--;
    }

    std::string read_diagnosis() {
      (*register_operation_)++;
      uint8_t data_out[10] = {0x9C};
      uint8_t data_in[10];
      spidma_.readwrite(data_out, data_in, 10);
      (*register_operation_)--;
      return bytes_to_hex(data_in+2, 8);
    }

    void clear_diag() {
      (*register_operation_)++;
      uint8_t data_out = 0x20;
      uint8_t data_in;
      spidma_.readwrite(&data_out, &data_in, 1);
      (*register_operation_)--;
    }

        // non interrupt context
    std::vector<uint8_t> read_register(uint8_t address, uint8_t length) {
        (*register_operation_)++;
        std::vector<uint8_t> data_out(length+3, 0);
        data_out[0] = read_register_opcode_;
        data_out[1] = address;
        uint8_t data_in[length+3];
        
        if (type_ == PZ) {
          spidma_.readwrite(data_out.data(), data_in, length+3);
          (*register_operation_)--;
          return std::vector<uint8_t>(&data_in[3], &data_in[3+length]);
        } else {
          spidma_.readwrite(data_out.data(), data_in, 2);
          data_out[0] = 0xad;
          data_out[1] = 0;
          spidma_.readwrite(data_out.data(), data_in, length+2);
          (*register_operation_)--;
          return std::vector<uint8_t>(&data_in[2], &data_in[2+length]);
        }
    }

    // non interrupt context
    bool set_register(uint8_t bank, uint8_t address, const std::vector<uint8_t> &value) {
        (*register_operation_)++;
        uint8_t data_in[std::max(value.size(),(std::size_t) 3)];
        if (bank != bank_) {
          uint8_t data_out[] = {write_register_opcode_, 0x40, bank};
          spidma_.readwrite(data_out, data_in, 3);
          if (read_register(0x40, 1) != std::vector<uint8_t>{bank}) {
            system_log("ichaus bank " + std::to_string(read_register(0x40, 1)[0]) + " not " + std::to_string(bank));
            (*register_operation_)--;
            return false;
          }
          bank_ = bank;
        }
        std::vector<uint8_t> data_out = {write_register_opcode_, address};
        data_out.insert(data_out.end(), value.begin(), value.end());
        spidma_.readwrite(data_out.data(), data_in, data_out.size());
        bool retval = read_register(address, value.size()) == value;
        (*register_operation_)--;
        return retval;
    }

    void clear_faults() {
        error_count_ = 0;
        warn_count_ = 0;
        crc_error_count_ = 0;
    }

    std::string write_conf() {
        set_register(0, Addr::COMMANDS, {CMD::CONF_WRITE_ALL});
        wait_while_false_with_timeout_us(read_register(Addr::COMMANDS, 1)[0] == 0, 100);
        auto data = read_register(Addr::CMD_STAT, 1);
        if (data[0] == 0) {
          return "conf write success";
        } else {
          return "conf error: " + std::to_string(data[0]);
        }
    }

    void start_auto_adj_ana() {
        set_register(0, Addr::COMMANDS, {CMD::AUTO_ADJ_ANA});
    }

    void start_auto_adj_dig() {
        set_register(0, Addr::COMMANDS, {CMD::AUTO_ADJ_DIG});
    }

    void start_auto_readj_dig() {
        set_register(0, Addr::COMMANDS, {CMD::AUTO_READJ_DIG});
    }

    void start_auto_adj_ecc() {
        set_register(0, Addr::COMMANDS, {CMD::AUTO_ADJ_ECC});
    }

    std::string get_cmd_result() {
        return "command: " + std::to_string(read_register(Addr::COMMANDS, 1)[0]) + ", result: " + std::to_string(read_register(Addr::CMD_STAT, 1)[0]);
    }

 protected:
    SPIDMA &spidma_;
    Disk disk_;
    uint8_t command_[5] = {};
    uint8_t data_[5] = {};
    int32_t pos_ = 0;
    uint32_t last_data_ = 0;
    volatile int register_operation_local_ = 0;
    volatile int *register_operation_ = &register_operation_local_;
    uint8_t bank_ = 255;
    bool ongoing_read_ = false;
    uint8_t read_register_opcode_ = 0x81;
    uint8_t write_register_opcode_ = 0xcf;
    enum {PZ, MU} type_ = PZ;

    uint32_t error_count_ = 0;
    uint32_t warn_count_ = 0;
    uint32_t crc_error_count_ = 0;
    uint32_t raw_value_ = 0;
    friend void config_init();
    friend void config_maintenance();

};

uint8_t tableCRC6[64] = {
 0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09,
 0x18, 0x1B, 0x1E, 0x1D, 0x14, 0x17, 0x12, 0x11,
 0x30, 0x33, 0x36, 0x35, 0x3C, 0x3F, 0x3A, 0x39,
 0x28, 0x2B, 0x2E, 0x2D, 0x24, 0x27, 0x22, 0x21,
 0x23, 0x20, 0x25, 0x26, 0x2F, 0x2C, 0x29, 0x2A,
 0x3B, 0x38, 0x3D, 0x3E, 0x37, 0x34, 0x31, 0x32,
 0x13, 0x10, 0x15, 0x16, 0x1F, 0x1C, 0x19, 0x1A,
 0x0B, 0x08, 0x0D, 0x0E, 0x07, 0x04, 0x01, 0x02};
/*32-bit input data, right alignment, Calculation over 24 bits (mult. of 6) */
uint8_t CRC_BiSS_43_30bit (uint32_t w_InputData)
{
 uint8_t b_Index = 0;
 uint8_t b_CRC = 0;
 b_Index = (uint8_t )(((uint32_t)w_InputData >> 24u) & 0x0000003Fu);

 b_CRC = (uint8_t )(((uint32_t)w_InputData >> 18u) & 0x0000003Fu);
 b_Index = b_CRC ^ tableCRC6[b_Index];
 
 b_CRC = (uint8_t )(((uint32_t)w_InputData >> 12u) & 0x0000003Fu);
 b_Index = b_CRC ^ tableCRC6[b_Index];

 b_CRC = (uint8_t )(((uint32_t)w_InputData >> 6u) & 0x0000003Fu);
 b_Index = b_CRC ^ tableCRC6[b_Index];

 b_CRC = (uint8_t )((uint32_t)w_InputData & 0x0000003Fu);
 b_Index = b_CRC ^ tableCRC6[b_Index];

 b_CRC = tableCRC6[b_Index];

 return b_CRC;
} 

#endif  // UNHUMAN_MOTORLIB_ICPZ_H_
