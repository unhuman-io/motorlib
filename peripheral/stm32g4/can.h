#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_CAN_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_CAN_H_

#include "stm32g474xx.h"
#include <cstdint>

class CAN {
 public:
    enum CAN_INST {CAN1, CAN2, CAN3};
    enum ArbitrationBaudRate {ARB_1M, ARB_2M};
    enum DataBaudRate {DATA_1M, DATA_2M, DATA_5M, DATA_8M};
    CAN(CAN_INST inst, ArbitrationBaudRate arb = ARB_1M, DataBaudRate data = DATA_5M);
    int read(uint8_t fifo, uint16_t id, uint8_t* data);
    int write(uint16_t id, uint8_t* data, uint8_t length);
    bool add_acceptance_filter(uint16_t id, uint8_t fifo);


    void interrupt();
 private:
    struct FDCANMessageRam {
      uint32_t FLSSA[28];
      uint32_t FLESA[2][8];
      uint32_t RX_FIFO0[3][18];
      uint32_t RX_FIFO1[3][18];
      uint32_t TX_EVENT_FIOFO[3][2];
      uint32_t TX_BUFFER[3][18];
    };
    struct FLSSA {
      uint32_t sfid2:11;
      uint32_t res:5;
      uint32_t sfid1:11;
      uint32_t sfec:3;
      uint32_t sft:2;
    };

    struct RX_FIFO {
      uint32_t id_low:18;
      uint32_t id:11;
      uint32_t rtr:1;
      uint32_t xtd:1;
      uint32_t esi:1;

      uint32_t rxts:16;
      uint32_t dlc:4;
      uint32_t brs:1;
      uint32_t fdf:1;
      uint32_t res:1;
      uint32_t fidx:7;
      uint32_t anmf:1;

      uint8_t data[64];    
    };

    struct TX_BUFFER {
      union TXWord1 {
        struct {
          uint32_t id_low:18;
          uint32_t id:11;
          uint32_t rtr:1;
          uint32_t xtd:1;
          uint32_t esi:1;
        };
        uint32_t word;
      } word1;

      union TXWord2 {
        struct {
          uint32_t res:16;
          uint32_t dlc:4;
          uint32_t brs:1;
          uint32_t fdf:1;
          uint32_t res2:1;
          uint32_t efc:1;
          uint32_t mm:8;
        };
        uint32_t word;
      } word2;

      union {
        uint8_t data[64];
        uint32_t data32[16];
      };
    };

    constexpr static uint32_t FDCAN_SIZE = 0x400;
    constexpr static uint32_t FDCAN_MSG_RAM_SIZE = 0x350; //0x400;

    static uint8_t dlc_to_length(uint8_t dlc);
    static uint8_t length_to_dlc(uint8_t length);

    FDCAN_GlobalTypeDef &regs_;
    FDCANMessageRam &ram_;
    uint8_t acceptance_filter_num_ = 0;
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_CAN_H_
