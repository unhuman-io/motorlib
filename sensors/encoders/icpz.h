#ifndef UNHUMAN_MOTORLIB_ICPZ_H_
#define UNHUMAN_MOTORLIB_ICPZ_H_

#include "../../encoder.h"
#include "../../util.h"
#include <vector>
#include <map>
#include <cmath>
#include "../../logger.h"
#include "../../parameter_api.h"

static uint8_t CRC_BiSS_43_30bit (uint32_t w_InputData);

#define ICPZ_SET_DEBUG_VARIABLES(prefix, api, icpz) \
    api.add_api_variable(prefix "ai_phase", new APICallbackFloat([]{ return icpz.get_ai_phase(); },\
      [](float f){ icpz.set_ai_phase(f); }));\
    api.add_api_variable(prefix "ai_scale", new APICallbackFloat([]{ return icpz.get_ai_scale(); },\
      [](float f){ icpz.set_ai_scale(f); }));\
    api.add_api_variable(prefix "cos_off", new const APICallbackFloat([](){ return icpz.get_cos_off(); }));\
    api.add_api_variable(prefix "sin_off", new const APICallbackFloat([](){ return icpz.get_sin_off(); }));\
    api.add_api_variable(prefix "sc_gain", new const APICallbackFloat([](){ return icpz.get_sc_gain(); }));\
    api.add_api_variable(prefix "sc_phase", new const APICallbackFloat([](){ return icpz.get_sc_phase(); }));\
    api.add_api_variable(prefix "ai_phases", new const APICallbackFloat([](){ return icpz.get_ai_phases(); }));\
    api.add_api_variable(prefix "ai_scales", new const APICallbackFloat([](){ return icpz.get_ai_scales(); }));\
    api.add_api_variable(prefix "ai_sel", new APICallbackHex<uint8_t>([](){ return icpz.get_ai_sel(); }, \
        [](uint8_t u){ icpz.set_ai_sel(u); }));\
    api.add_api_variable(prefix "cos_offs", new const APICallbackFloat([](){ return icpz.get_cos_offs(); }));\
    api.add_api_variable(prefix "sin_offs", new const APICallbackFloat([](){ return icpz.get_sin_offs(); }));\
    api.add_api_variable(prefix "sc_gains", new const APICallbackFloat([](){ return icpz.get_sc_gains(); }));\
    api.add_api_variable(prefix "sc_phases", new const APICallbackFloat([](){ return icpz.get_sc_phases(); }));\
    api.add_api_variable(prefix "sc_sel", new APICallbackHex<uint16_t>([](){ return icpz.get_sc_sel(); }, \
        [](uint16_t u){ icpz.set_sc_sel(u); }));\
    api.add_api_variable(prefix "err", new APIUint32(&icpz.error_count_));\
    api.add_api_variable(prefix "warn", new APIUint32(&icpz.warn_count_));\
    api.add_api_variable(prefix "crc_cnt", new APIUint32(&icpz.crc_error_count_));\
    api.add_api_variable(prefix "raw", new APIUint32(&icpz.raw_value_));\
    api.add_api_variable(prefix "rawh", new const APICallback([](){ return u32_to_hex(icpz.raw_value_); }));\
    api.add_api_variable(prefix "value", new const APIInt32(&icpz.pos_));\
    api.add_api_variable(prefix "diag", new const APICallback([](){ return icpz.read_diagnosis(); }));\
    api.add_api_variable(prefix "conf_write", new const APICallback([](){ return icpz.write_conf(); }));\
    api.add_api_variable(prefix "conf_write_no_check", new const APICallback([](){ return icpz.write_conf_no_check(); }));\
    api.add_api_variable(prefix "auto_ana", new const APICallback([](){ icpz.start_auto_adj_ana(); return "ok"; }));\
    api.add_api_variable(prefix "auto_dig", new const APICallback([](){ icpz.start_auto_adj_dig(); return "ok"; }));\
    api.add_api_variable(prefix "readj_dig", new const APICallback([](){ icpz.start_auto_readj_dig(); return "ok"; }));\
    api.add_api_variable(prefix "auto_ecc", new const APICallback([](){ icpz.start_auto_adj_ecc(); return "ok"; }));\
    api.add_api_variable(prefix "ecc_correction", new APICallbackUint8([](){ return icpz.get_ecc_correction(); }, \
        [](uint8_t u){ icpz.set_ecc_correction(u); }));\
    api.add_api_variable(prefix "ran_tol", new APICallbackUint8([](){ return icpz.get_ran_tol(); }, \
        [](uint8_t u){ icpz.set_ran_tol(u); }));\
    api.add_api_variable(prefix "ran_fld", new APICallbackUint8([](){ return icpz.get_ran_fld(); }, \
        [](uint8_t u){ icpz.set_ran_fld(u); }));\
    api.add_api_variable(prefix "ecc_um", new APICallbackFloat([](){ return icpz.get_ecc_um(); }, \
        [](float f){ icpz.set_ecc_um(f); }));\
    api.add_api_variable(prefix "ecc_phase_deg", new APICallbackFloat([](){ return icpz.get_ecc_phase(); }, \
        [](float f){ icpz.set_ecc_phase(f); }));\
    api.add_api_variable(prefix "low", new APICallbackUint8([](){ return icpz.get_ac_eto(); }, \
        [](uint8_t u){ icpz.set_ac_eto(u); }));\
    api.add_api_variable(prefix "ac_count", new APICallbackUint8([](){ return icpz.get_ac_count(); }, \
        [](uint8_t u){ icpz.set_ac_count(u); }));\
    api.add_api_variable(prefix "led_cur", new APICallbackUint8([](){ return icpz.get_led_cur(); }, \
        [](uint8_t u){ icpz.set_led_cur(u); }));\
    api.add_api_variable(prefix "ana_sel", new APICallbackUint8([](){ return icpz.get_ac_sel(); }, \
        [](uint8_t u){ icpz.set_ac_sel(u); }));\
    api.add_api_variable(prefix "cal", new const APICallback([](){ return icpz.get_cal_string(); }));\
    api.add_api_variable(prefix "cals", new const APICallback([](){ return icpz.get_cals_string(); }));\
    api.add_api_variable(prefix "cmd_result", new const APICallback([](){ return icpz.get_cmd_result(); }));\
    api.add_api_variable(prefix "disk_um", new const APICallbackFloat([](){ return icpz.r_disk_um[icpz.disk_]; }));\
    api.add_api_variable(prefix "ipo_filt1", new APICallbackHex<uint8_t>([](){ return icpz.get_ipo_filt1(); }, \
        [](uint8_t u){ icpz.set_ipo_filt1(u); }));\
    api.add_api_variable(prefix "ipo_filt2", new APICallbackHex<uint8_t>([](){ return icpz.get_ipo_filt2(); }, \
        [](uint8_t u){ icpz.set_ipo_filt2(u); })); \
    api.add_api_variable(prefix "temp", new const APICallbackFloat([]{ return icpz.get_temperature(); })); \
    api.add_api_variable(prefix "diag_str", new const APICallback([](){ return icpz.read_diagnosis_str(); }));\
    api.add_api_variable(prefix "clear_diag", new const APICallback([](){ icpz.clear_diag(); return "ok"; }));\
    api.add_api_variable(prefix "last_error_pos", new const APIInt32(&icpz.last_error_pos_));\
    api.add_api_variable(prefix "last_warn_pos", new const APIInt32(&icpz.last_warn_pos_));\
    api.add_api_variable(prefix "i2c_bank", new APICallbackHex<uint8_t>([](){ return icpz.i2c_bank_; }, \
        [](uint8_t u){ icpz.i2c_bank_ = u; }));\
    api.add_api_variable(prefix "i2c_adr", new APICallbackHex<uint8_t>([](){ return icpz.i2c_adr_; }, \
        [](uint8_t u){ icpz.i2c_adr_ = u; }));\
    api.add_api_variable(prefix "i2c", new APICallbackHex<uint8_t>([]{ return icpz.get_i2c_data(); }, \
        [](uint8_t d){ icpz.set_i2c_data(d); }));\
    api.add_api_variable(prefix "reset", new const APICallback([](){ icpz.reset(); return "ok"; }));\

template<typename ConcreteICPZ>
class ICPZBase : public EncoderBase {
 public:
    const char *disk_names[4] = {"default", "pz03s", "pz08s", "pz16s"};
    static constexpr const char *diag_strs[32] = {"photo_amp_sat", "led_cur_low", "temp_sense_not_ready", "vddio_low",
      "interpolator_error", "5", "abz_not_ready", "uvw_not_ready",
      "alpha_overflow", "omega_overflow", "digital_photo_amp_not_ss", "prc_sync_failed",
      "analog_adjust_at_boundary", "digital_adjust_at_boundary", "temp_limit_1", "temp_limit_2",
      "multi_turn_error", "multi_turn_error_2", "multi_turn_error_3", "multi_turn_error_4",
      "multi_turn_warning", "multi_turn_pos_failed", "pin_ada_stuck_0", "pin_ada_stuck_1",
      "startup_in_progress", "eeprom_crc_error", "gpio0_in_error", "gpio1_in_error",
      "startup_aborted_timeout", "user_error_1", "user_error_2", "user_error_3"};

    enum Disk{Default, PZ03S, PZ08S, PZ16S};
    struct Encoder24 {
      union {
        struct {
          uint32_t pos:24;
          uint32_t sign:1;
        };
        struct {
          int32_t ipos:24;
        };
        uint32_t word;
      };
    };

    static constexpr float r_disk_um[4] = {1, 10700, 18600, 7200};
    ICPZBase(SPIDMA &spidma, Disk disk = Default) 
      : spidma_(spidma), disk_(disk) {
      command_[0] = Opcode::READ_POS;
    }
    union Diag {
      struct {
        uint8_t crc6:6;
        uint8_t nWarn:1;
        uint8_t nErr:1;
      };
      uint8_t word;
    };
    enum Opcode {READ_REG=0x81, WRITE_REG=0xCF, READ_POS=0xA6, WRITE_COMMAND=0xD9, READ_DIAG=0x9C, REQ_I2C=0x97, TRANSMIT_I2C=0xD2, GET_TRANS_INFO=0xAD};
    enum Addr {CMD_STAT=0x76, COMMANDS=0x77};
    enum CMD {REBOOT=0x10, SCLEAR=0x20, CONF_WRITE_ALL=0x41, AUTO_ADJ_ANA=0xB0, AUTO_ADJ_DIG=0xB1, AUTO_READJ_DIG=0xB2, AUTO_ADJ_ECC=0xB3};

    bool init() {
      bool success = true;
      logger.log_printf("icpz init start with disk: %s", disk_names[disk_]);
      
      set_register(0, 0, {3}); // fast speed on port a, set first
      reset();
      IWDG->KR = 0xAAAA; // watchdog due to slow reset at init stage

      success &= set_register(0, 0, {3}); // fast speed on port a, set first
      success &= set_register(7, 9, {0}); // multiturn data length = 0
      success &= set_register(7, 0xA, {0}); // spi_ext = 0
      success &= set_register(7, 0, {0x13, 0x07, 0, 0x11}); // disable prc error, default: {0x13, 0x0F, 0, 0x11}, 
      success &= set_register(7, 4, {0x0C, 0xC8, 0, 0x02}); // prc is a warning, default: {0x0C, 0xC0, 0, 0x02},
      success &= set_ran_tol(0);  // disables position update based on absolute track after initial, tol 0
      success &= set_ran_fld(0);
      success &= set_ai_sel(0xDD); // fast dynamic digital calibration - useful for detecting eccentricity
      success &= set_register(2, 0, {0x00, 0x0});  // no dynamic analog calibration
      success &= set_ipo_filt1(0xEA); // ipo_filt1 per datasheet

      success &= set_register(0xA, 0, {0x40 << 1}); // i2c address 0x40 for debug purposes

      if (disk_ == PZ03S) {
        success &= set_register(8, 0, {0, 1}); // fcl = 256
        success &= set_register(8, 2, {0, 0}); // fcs = 0
        success &= set_register(0, 7, {8 << 4}); // sys_ovr = 8
      } else if (disk_ == PZ08S) {
        success &= set_register(8, 0, {0xBE, 1}); // fcl = 446 (0x1BE)
        success &= set_register(8, 2, {216, 0}); // fcs = 216
        success &= set_register(0, 7, {9 << 4}); // sys_ovr = 9
        // ai phase -20
      } else if (disk_ == PZ16S) {
        success &= set_register(8, 0, {172, 0}); // fcl = 172
        success &= set_register(8, 2, {27, 0}); // fcs = 27
        success &= set_register(0, 7, {8 << 4}); // sys_ovr = 8
      }
      static_cast<ConcreteICPZ *>(this)->enable_commands_impl();
       return success;
    }
    void trigger() {
      spidma_.start_readwrite_isr(command_, data_, sizeof(command_));
    }

    uint32_t read_raw_buf(uint8_t buf[4], bool &crc_error) {
      uint32_t data = ((buf[1] << 16) | (buf[2] << 8) | buf[3]) << 8;
      raw_value_ = data >> 8;
      uint32_t word = data | buf[4];
      Diag diag = {.word = buf[4]};
      uint8_t crc6_calc = ~CRC_BiSS_43_30bit(word >> 6) & 0x3f;
      error_count_ += !diag.nErr;
      warn_count_ += !diag.nWarn;
      crc_error = diag.crc6 == crc6_calc ? false : true;
      crc_error_count_ += crc_error;
      if (!diag.nErr) {
        last_error_pos_ = raw_value_;
      }
      if (!diag.nErr) {
        if (last_diag_.nErr) {
          last_error_pos_ = pos_;
        }
      }
      if (!diag.nWarn) {
        if (last_diag_.nWarn) {
          last_warn_pos_ = pos_;
        }
      }
      last_diag_ = diag;
      return raw_value_;
    }

    int32_t read_buf(uint8_t buf[4]) {
      bool crc_error;
      Encoder24 enc = {};
      enc.word = read_raw_buf(buf, crc_error);
      if (!crc_error) {
        Encoder24 diffe = {};
        diffe.pos = enc.pos - last_enc_.pos;
        int32_t diff = diffe.ipos;
        pos_ += diff;
        last_enc_ = enc;
      }
      return get_value();
    }
    int32_t read() {
      spidma_.finish_readwrite_isr();
      read_buf(data_);
      return get_value();
    }
    int32_t get_value() const {
      return pos_;
    }
    bool index_received() const { return true; }

    std::string read_diagnosis() {
      spidma_.claim();
      uint8_t data_out[10] = {Opcode::READ_DIAG};
      uint8_t data_in[10];
      spidma_.readwrite(data_out, data_in, 10);
      clear_diag();
      spidma_.release();
      return bytes_to_hex(data_in+2, 8);
    }

    std::string read_diagnosis_str() {
      spidma_.claim();
      uint8_t data_out[10] = {Opcode::READ_DIAG};
      uint8_t data_in[10];
      spidma_.readwrite(data_out, data_in, 10);
      clear_diag();
      spidma_.release();
      uint32_t diag_err = data_in[5] << 24 | data_in[4] << 16 | data_in[3] << 8 | data_in[2];
      uint32_t diag_warn = data_in[9] << 24 | data_in[8] << 16 | data_in[7] << 8 | data_in[6];
      return "err: " + diagnosis_to_str(diag_err) + " warn: " + diagnosis_to_str(diag_warn);
    }

    static std::string diagnosis_to_str(uint32_t diag) {
      std::string s;
      for (int i=0; i<32; i++) {
        if (diag & (1 << i)) {
          s += std::string(diag_strs[i]) + " ";
        } 
      }
      return s;
    }

    void clear_diag() {
      send_command(SCLEAR);
    }

        // non interrupt context
    std::vector<uint8_t> read_register(uint8_t address, uint8_t length) {
        spidma_.claim();
        std::vector<uint8_t> data_out(length+3, 0);
        data_out[0] = Opcode::READ_REG;
        data_out[1] = address;
        uint8_t data_in[length+3];
        
        if (type_ == PZ) {
          spidma_.readwrite(data_out.data(), data_in, length+3);
          spidma_.release();
          return std::vector<uint8_t>(&data_in[3], &data_in[3+length]);
        } else {
          spidma_.readwrite(data_out.data(), data_in, 2);
          data_out[0] = Opcode::GET_TRANS_INFO;
          data_out[1] = 0;
          spidma_.readwrite(data_out.data(), data_in, length+2);
          spidma_.release();
          return std::vector<uint8_t>(&data_in[2], &data_in[2+length]);
        }
    }

    bool set_bank(uint8_t bank) {
        if (bank != bank_) {
          spidma_.claim();
          uint8_t data_in[3];
          uint8_t data_out[] = {Opcode::WRITE_REG, 0x40, bank};
          spidma_.readwrite(data_out, data_in, 3);
          if (read_register(0x40, 1) != std::vector<uint8_t>{bank}) {
            logger.log("ichaus bank " + std::to_string(read_register(0x40, 1)[0]) + " not " + std::to_string(bank));
            spidma_.release();
            return false;
          }
          bank_ = bank;
          spidma_.release();
        }
        return true;
    }

    std::vector<uint8_t> read_register(uint8_t bank, uint8_t address, uint8_t length) {
        spidma_.claim();
        std::vector<uint8_t> data_out(length+3, 0);
        data_out[0] = Opcode::READ_REG;
        data_out[1] = address;
        uint8_t data_in[length+3];
        if (!set_bank(bank)) {
          spidma_.release();
          return std::vector<uint8_t>(1,0xff);
        }
        if (type_ == PZ) {
          spidma_.readwrite(data_out.data(), data_in, length+3);
          static_cast<ConcreteICPZ*>(this)->restore_bank_impl();
          spidma_.release();
          return std::vector<uint8_t>(&data_in[3], &data_in[3+length]);
        } else {
          spidma_.readwrite(data_out.data(), data_in, 2);
          data_out[0] = Opcode::GET_TRANS_INFO;
          data_out[1] = 0;
          spidma_.readwrite(data_out.data(), data_in, length+2);
          static_cast<ConcreteICPZ*>(this)->restore_bank_impl();
          spidma_.release();
          return std::vector<uint8_t>(&data_in[2], &data_in[2+length]);
        }
    }

    // use for register addresses >= 0x40 to skip bank selection
    bool set_register(uint8_t address, const std::vector<uint8_t> &value, bool set_only = false) {
        return set_register(bank_, address, value, set_only);
    }

    // non interrupt context
    bool set_register(uint8_t bank, uint8_t address, const std::vector<uint8_t> &value, bool set_only = false) {
        spidma_.claim();
        bool retval = true;
        uint8_t data_in[std::max(value.size()+2,(std::size_t) 3)];
        if (!set_bank(bank)) {
          spidma_.release();
          return false;
        }
        std::vector<uint8_t> data_out = {Opcode::WRITE_REG, address};
        data_out.insert(data_out.end(), value.begin(), value.end());
        spidma_.readwrite(data_out.data(), data_in, data_out.size());
        if (!set_only) {
          std::vector<uint8_t> data_read = read_register(address, value.size());
          retval = data_read == value;
          static_cast<ConcreteICPZ*>(this)->restore_bank_impl();
          spidma_.release();
          if (!retval) {
            for (unsigned int i=0; i<value.size(); i++) {
              logger.log_printf("icpz register %x, set%d: %x, read%d: %x", address, i, value[i], i, data_read[i]);
            }
          }
        } else {
          static_cast<ConcreteICPZ*>(this)->restore_bank_impl();
          spidma_.release();
        }
        return retval;
    }

    void reset() {
      spidma_.claim();
      send_command(REBOOT);
      // The datasheet doesn't specify if there is any way to check for success on this command.
      ms_delay(40);
      static_cast<ConcreteICPZ*>(this)->enable_commands_impl();
      spidma_.release();
    }

    void clear_faults() {
        // todo this is called by mainloop needs to be isr safe to use clear diag here
        // clear_diag();
        error_count_ = 0;
        warn_count_ = 0;
        crc_error_count_ = 0;
    }

    std::string send_command_with_result(uint8_t command, uint32_t timeout_us) {
        send_command(command);
        // 200 ms timeout found experimentally
        uint32_t t_start = get_clock();
        bool timed_out = true;
        while (get_clock() - t_start < US_TO_CPU(timeout_us)) {
          uint8_t result = get_active_command();
          if (result == 0) {
            timed_out = false;
            break;
          } else if (result == 0xff) {
            // PZ reports 0xff when an error occurs
            return get_cmd_result();
          }
        }
        if (timed_out) {
          return "timeout error (" + std::to_string(timeout_us/1000) + " ms)";
        }
        auto data = read_register(Addr::CMD_STAT, 1);
        static_cast<ConcreteICPZ*>(this)->enable_commands_impl();
        if (data[0] == 0) {
          return "success";
        } else {
          return "conf error: " + std::to_string(data[0]);
        }
    }

    std::string write_conf() {
        return "conf write " + send_command_with_result(CMD::CONF_WRITE_ALL, 200000);
    }

    std::string write_conf_no_check() {
        send_command(CMD::CONF_WRITE_ALL);
        return "ok";
    }

    void start_auto_adj_ana() {
        send_command(CMD::AUTO_ADJ_ANA);
    }

    void start_auto_adj_dig() {
        send_command(CMD::AUTO_ADJ_DIG);
    }

    void start_auto_readj_dig() {
        send_command(CMD::AUTO_READJ_DIG);
    }

    void start_auto_adj_ecc() {
        send_command(CMD::AUTO_ADJ_ECC);
    }

    void set_ecc_correction(uint8_t on = 1) {
        set_register(2, 0xa, {on});
    }

    bool get_ecc_correction() {
        return read_register(2, 0xa, 1)[0];
    }

    bool set_ran_tol(uint8_t val) {
        uint8_t tmp = read_register(0, 0xF, 1)[0] & 0xF0;
        tmp |= val & 0xF;
        return set_register(0, 0xF, {tmp});
    }

    uint8_t get_ran_tol() {
        uint8_t ran_reg = read_register(0, 0xF, 1)[0];
        return ran_reg & 0xF;
    }

    bool set_ran_fld(uint8_t val) {
        uint8_t tmp = read_register(0, 0xF, 1)[0] & 0xF;
        tmp |= (val<<8) & 0xF;
        return set_register(0, 0xF, {tmp});
    }

    uint8_t get_ran_fld() {
        uint8_t ran_reg = read_register(0, 0xF, 1)[0];
        return ran_reg >> 7;
    }

    float get_ai_phase() {
        auto data = read_register(1, 0x8, 2);
        return get_ai_phase(data.data());
    }

    bool set_ai_phase(float f) {
        int16_t ai_phase_raw = f/180*512;
        std::vector<uint8_t> data = {(uint8_t) (ai_phase_raw << 6), (uint8_t) (ai_phase_raw >> 2)};
        return set_register(1, 0x8, data);
    }

    float get_ai_scale() {
        auto data = read_register(1, 0xa, 2);
        int16_t ai_scale_raw = ((int16_t) (data[1] << 8 | data[0])) >> 7;
        float ai_scale = (float) ai_scale_raw/1820 + 1;
        return ai_scale;
    }

    bool set_ai_scale(float f) {
        int16_t ai_scale_raw = (f-1)*1820;
        std::vector<uint8_t> data = {(uint8_t) (ai_scale_raw << 7), (uint8_t) (ai_scale_raw >> 1)};
        return set_register(1, 0xa, data);
    }

    float get_cos_off() {
        auto data = read_register(1, 0, 2);
        int16_t off_raw = ((int16_t) (data[1] << 8 | data[0])) >> 6;
        float off = off_raw * 0.235; // offset in mV
        return off;
    }

    float get_sin_off() {
        auto data = read_register(1, 2, 2);
        int16_t off_raw = ((int16_t) (data[1] << 8 | data[0])) >> 6;
        float off = off_raw * 0.235; // offset in mV
        return off;
    }

    float get_sc_gain() {
        auto data = read_register(1, 4, 2);
        int16_t gain_raw = ((int16_t) (data[1] << 8 | data[0])) >> 6;
        float gain = std::pow((float) 14.0/11, (float) gain_raw/511); 
        return gain;
    }

    float get_sc_phase() {
        auto data = read_register(1, 6, 2);
        int16_t phase_raw = ((int16_t) (data[1] << 8 | data[0])) >> 6;
        float phase =  (float) phase_raw/511 * 11.4; 
        return phase;
    }

    static float get_ai_phase(uint8_t data[2]) {
        int16_t ai_phase_raw = ((int16_t) (data[1] << 8 | data[0])) >> 6;
        float ai_phase = (float) ai_phase_raw/512*180;
        return ai_phase;
    }

    float get_ai_phases() {
        auto data = read_register(1, 0x28, 2);
        return get_ai_phase(data.data());
    }

    float get_ai_scales() {
        auto data = read_register(1, 0x2a, 2);
        int16_t ai_scale_raw = ((int16_t) (data[1] << 8 | data[0])) >> 7;
        float ai_scale = (float) ai_scale_raw/1820 + 1;
        return ai_scale;
    }

    uint8_t get_ai_sel() {
        return read_register(2, 3, 1)[0];
    }
    
    bool set_ai_sel(uint8_t sel) {
        return set_register(2, 3, {sel});
    }

    float get_cos_offs() {
        auto data = read_register(1, 0x20, 2);
        int16_t off_raw = ((int16_t) (data[1] << 8 | data[0])) >> 6;
        float off = off_raw * 0.235; // offset in mV
        return off;
    }

    float get_sin_offs() {
        auto data = read_register(1, 0x22, 2);
        int16_t off_raw = ((int16_t) (data[1] << 8 | data[0])) >> 6;
        float off = off_raw * 0.235; // offset in mV
        return off;
    }

    float get_sc_gains() {
        auto data = read_register(1, 0x24, 2);
        int16_t gain_raw = ((int16_t) (data[1] << 8 | data[0])) >> 6;
        float gain = std::pow((float) 14.0/11, (float) gain_raw/511); 
        return gain;
    }

    float get_sc_phases() {
        auto data = read_register(1, 0x26, 2);
        int16_t phase_raw = ((int16_t) (data[1] << 8 | data[0])) >> 6;
        float phase =  (float) phase_raw/511 * 11.4; 
        return phase;
    }

    uint16_t get_sc_sel() {
        return *((uint16_t *) read_register(2, 0, 2).data()) & 0xFFF;
    }

    bool set_sc_sel(uint16_t sel) {
        return set_register(2, 0, {(uint8_t) (sel & 0xFF), (uint8_t) ((sel >> 8) & 0xFF)});
    }

    uint8_t get_led_cur() {
        return (read_register(3, 0, 1)[0] & 0xE) >> 1;
    }

    bool set_led_cur(uint8_t led_cur) {
        // 0: 40 mA
        // 1: 0  mA
        // 2: 1  mA
        // 3: 2  mA
        // 4: 4  mA
        // 5: 8  mA
        // 6: 16 mA
        // 7: 32 mA
        uint8_t tmp = read_register(3, 0, 1)[0] & 0x11;
        tmp |= (led_cur << 1) & 0xE;
        return set_register(3, 0, {tmp});
    }

    std::string get_cal_string() {
        char c[200];
        std::snprintf(c, 200, "cos_off: %f, sin_off: %f, sc_gain: %f, sc_phase: %f, ai_phase: %f, ai_scale: %f, ecc_amp: %f, ecc_phase: %f", 
          get_cos_off(), get_sin_off(), get_sc_gain(), get_sc_phase(), get_ai_phase(), get_ai_scale(), get_ecc_um(), get_ecc_phase());
        return std::string(c);
    }

    std::string get_cals_string() {
        char c[200];
        std::snprintf(c, 200, "cos_off: %f, sin_off: %f, sc_gain: %f, sc_phase: %f, ai_phase: %f, ai_scale: %f", 
          get_cos_offs(), get_sin_offs(), get_sc_gains(), get_sc_phases(), get_ai_phases(), get_ai_scales());
        return std::string(c);
    }

    static float get_temperature(uint8_t buf[2]) {
        // signed value, 16 bit
        uint16_t temp_raw = (buf[1] << 8 | buf[0]);
        int16_t temp_signed = (int16_t) temp_raw;
        float temp =  (float) temp_signed/10; 
        return temp;
    }

    float get_temperature() {
          auto data = read_register(0x4e, 2);
          // signed value, 16 bit
          return get_temperature(data.data());
    }

    // set ac_eto for 10x longer timeout on calibration
    bool set_ac_eto(uint8_t on = 1) {
        auto data = read_register(0x5d, 1);
        return set_register(0, 0x5d, {(uint8_t) ((on << 7) | (data[0] & 0xF))});
    }

    bool get_ac_eto() {
        return read_register(0x5d, 1)[0] >> 7;
    }

    bool set_ac_count(uint8_t count = 8) {
        auto data = read_register(0x5d, 1);
        return set_register(0, 0x5d, {(uint8_t) ((data[0] & 0x80) | (count & 0xF))});
    }

    uint8_t get_ac_count() {
        return read_register(0x5d, 1)[0];// & 0xf;
    }

    bool set_ac_sel(uint8_t sel) {
        auto data = read_register(3, 1, 1);
        return set_register(3, 1, {(uint8_t) ((data[0] & 0x04) | (sel & 0x3))});
    }

    uint8_t get_ac_sel() {
        // sel 0 buffered signals
        // sel 2 pre conditioning signals
        // sel 3 post conditioning signals
        return read_register(3, 1, 1)[0] & 0x3;
    }

    float get_ecc_um() {
        auto data = read_register(2, 4, 4);
        uint32_t ecc_amp_raw = data[3] << 24 | data[2] << 16 | data[1] << 8 | data[0];
        float ecc_amp  = ecc_amp_raw * r_disk_um[disk_] * 1.407e-9;
        return ecc_amp;
    }

    float get_ecc_phase() {
        auto data = read_register(2, 8, 2);
        int16_t phase_raw = ((int16_t) (data[1] << 8 | data[0])) >> 2;
        float phase =  (float) phase_raw/std::pow(2, 14) * 360; 
        return phase;
    }

    bool set_ecc_um(float ecc) {
        uint32_t ecc_raw = ecc/r_disk_um[disk_] / 1.407e-9;
        return set_register(2, 4, {(uint8_t) (ecc_raw & 0xff), (uint8_t) ((ecc_raw >> 8) & 0xff), 
          (uint8_t) ((ecc_raw >> 16) & 0xff), (uint8_t) ((ecc_raw >> 24) & 0xff)});
    }

    bool set_ecc_phase(float phase_deg) {
        int16_t phase_raw = phase_deg/360 * std::pow(2, 14);
        return set_register(2, 8, {(uint8_t) ((phase_raw << 2) & 0xff), (uint8_t) ((phase_raw >> 6) & 0xff)});
    }

    uint8_t get_ipo_filt1() {
        return read_register(0, 3, 1)[0];
    }

    bool set_ipo_filt1(uint8_t u) {
        return set_register(0, 3, {u});
    }

    uint8_t get_ipo_filt2() {
        return read_register(0, 4, 1)[0];
    }

    bool set_ipo_filt2(uint8_t u) {
        return set_register(0, 4, {u});
    }

    uint8_t i2c_bank_ = 0x24;
    uint8_t i2c_adr_ = 0x0;
    uint8_t get_i2c_data() {
        spidma_.claim();
        set_bank(i2c_bank_);
        uint8_t data = get_i2c_data(i2c_adr_);
        i2c_adr_++;
        static_cast<ConcreteICPZ*>(this)->restore_bank_impl();
        spidma_.release();
        return data;
    }

    uint8_t get_i2c_data(uint8_t address) {
        uint8_t data_out[2] = {Opcode::REQ_I2C, address};
        uint8_t data_in[2];
        spidma_.claim();
        spidma_.readwrite(data_out, data_in, 2);
        uint8_t i2c_data;
        wait_while_true_with_timeout_us(get_i2c_transaction_info_and_data(&i2c_data), 20000);
        spidma_.release();
        return i2c_data;
    }

    void set_i2c_data(uint8_t data) {
        set_bank(i2c_bank_);
        uint8_t data_out[3] = {Opcode::TRANSMIT_I2C, i2c_adr_, data};
        uint8_t data_in[3];
        spidma_.claim();
        spidma_.readwrite(data_out, data_in, 3);
        static_cast<ConcreteICPZ*>(this)->restore_bank_impl();
        spidma_.release();
    }

    uint8_t get_i2c_transaction_info_and_data(uint8_t *i2c_data) {
        uint8_t data_out[3] = {Opcode::GET_TRANS_INFO};
        uint8_t data_in[3];
        spidma_.readwrite(data_out, data_in, 3);
        *i2c_data = data_in[2];
        return data_in[1];
    }
    uint8_t get_active_command() {
        return read_register(Addr::COMMANDS, 1)[0];
    }
    
    void send_command(uint8_t command) {
        static_cast<ConcreteICPZ*>(this)->disable_commands_impl();
        uint8_t data_out[2] = {Opcode::WRITE_COMMAND, command};
        uint8_t data_in[2];
        spidma_.readwrite(data_out, data_in, 2);
    }

    std::string get_cmd_result() {
        uint8_t command  = get_active_command();
        std::string s = "command: " + std::to_string(command) + ", result: " + std::to_string(read_register(Addr::CMD_STAT, 1)[0]);
        if (command == 0 || command == 0xff) {
          static_cast<ConcreteICPZ*>(this)->enable_commands_impl();
        }
        return s;
    }

    void enable_commands_impl() {
        // default nothing
    }

    void disable_commands_impl() {
        // default nothing
    }

    void restore_bank_impl() {
        // default nothing
    }


    SPIDMA &spidma_;
    Disk disk_;
    uint8_t command_[5] = {};
    uint8_t data_[5] = {};
    int32_t pos_ = 0;
    Encoder24 last_enc_ = {};
    uint8_t bank_ = 255;
    enum {PZ, MU} type_ = PZ;

    uint32_t error_count_ = 0;
    uint32_t warn_count_ = 0;
    uint32_t crc_error_count_ = 0;
    uint32_t raw_value_ = 0;
    
    Diag last_diag_ = {};
    int32_t last_error_pos_ = 0;
    int32_t last_warn_pos_ = 0;

    friend void config_init();
    friend void config_maintenance();

};

class ICPZ : public ICPZBase<ICPZ> {
  public:
    ICPZ(SPIDMA &spidma, Disk disk = Default) : ICPZBase(spidma, disk) {}
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
