#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_DRV8323S_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_DRV8323S_H_

#include "../../driver.h"
#include "../../logger.h"

extern uint16_t drv_regs_error;

class DRV8323S : public DriverBase {
 public:
  DRV8323S(SPI_TypeDef &regs, volatile int *register_operation = nullptr,
           void (*spi_reinit_callback)() = nullptr)
      : regs_(regs), spi_reinit_callback_(spi_reinit_callback) {
    if (register_operation != nullptr) {
      register_operation_ = register_operation;
    }
  }

  void drv_spi_start() {
    (*register_operation_)++;
    GPIO_SETL(A, 4, 2, 3, 5);                          // pin A4 NSS
    regs_.CR1 = 0;                                     // clear SPE
    regs_.CR2 = (15 << SPI_CR2_DS_Pos) | SPI_CR2_FRF;  // 16 bit TI mode
    // ORDER DEPENDANCE SPE set last
    regs_.CR1 =
        SPI_CR1_MSTR | (6 << SPI_CR1_BR_Pos) | SPI_CR1_SPE;  // baud = clock/64
  }

  void drv_spi_end() {
    SPI1->CR1 = 0;  // clear SPE
    // SPI1 CS-> gpio
    GPIO_SETL(A, 4, 1, 0, 0);
    GPIOA->BSRR = GPIO_ODR_OD4;

    // SPI needs reinit
    if (spi_reinit_callback_ != nullptr) {
      spi_reinit_callback_();
    }
    (*register_operation_)--;
  }

  void disable() {
    uint32_t status = get_drv_status();
    logger.log_printf("drv8323 disabled, status: %04x", status);
    logger.log("drv disable");
    GPIOC->BSRR = GPIO_BSRR_BR13;  // drv disable
    DriverBase::disable();
  }

  void enable() {
    GPIOC->BSRR = GPIO_BSRR_BS13;  // drv enable
    ms_delay(10);

    drv_spi_start();
    drv_regs_error = 0;
    for (uint8_t i = 0; i < sizeof(param->drv_regs) / sizeof(uint16_t); i++) {
      uint16_t reg_out = param->drv_regs[i];
      uint16_t reg_in = 0;
      write_reg(reg_out);
      uint8_t address = param->drv_regs[i] >> 11;
      reg_in = read_reg(address);
      if ((reg_in & 0x7FF) != (reg_out & 0x7FF)) {
        drv_regs_error |= 1 << i;
      }
      logger.log_printf("address: %d, reg_out: %x, reg_in: %x", address,
                        reg_out, reg_in);
    }

    drv_spi_end();
    DriverBase::enable();
    if (!drv_regs_error) {
      logger.log("drv8323s configure success");
    } else {
      logger.log_printf("drv8323s configure error: %02x", drv_regs_error);
    }
  }

  std::string drv_reset() {
    disable();
    ms_delay(10);
    enable();
    return "ok";
  }

  uint16_t write_reg(uint16_t reg_out) {
    regs_.DR = reg_out;
    while (!(regs_.SR & SPI_SR_RXNE))
      ;
    uint16_t reg_in = regs_.DR;
    return reg_in;
  }

  uint16_t read_reg(uint8_t address) {
    uint16_t out_value = 1 << 15 | address << 11;
    regs_.DR = out_value;
    while (!(regs_.SR & SPI_SR_RXNE))
      ;
    uint16_t value = regs_.DR;
    return value;
  }

  // return (fault status register 2 << 16) | (fault status register 1)
  uint32_t get_drv_status() {
    drv_spi_start();
    // vgs_status2 << 16 | fault status 1
    uint32_t value = read_reg(0) | read_reg(1) << 16;
    drv_spi_end();

    return value;
  }
  volatile int *register_operation_ = &register_operation_local_;

 private:
  SPI_TypeDef &regs_;
  volatile int register_operation_local_ = 0;
  void (*spi_reinit_callback_)();
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_DRV8323S_H_
