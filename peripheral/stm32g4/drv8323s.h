#pragma once

class DRV8323S {
 public:
    DRV8323S(SPI_TypeDef &regs, volatile int* register_operation = nullptr, void (*spi_reinit_callback)() = nullptr)
        : regs_(regs), spi_reinit_callback_(spi_reinit_callback) {
        if (register_operation != nullptr) {
            register_operation_ = register_operation;
        }
        drv_enable();
    }

    void drv_spi_start() {
        GPIO_SETL(A, 4, 2, 3, 5); // pin A4 NSS
        regs_.CR1 = 0; // clear SPE
        regs_.CR2 = (15 << SPI_CR2_DS_Pos) | SPI_CR2_FRF;   // 16 bit TI mode
        // ORDER DEPENDANCE SPE set last
        regs_.CR1 = SPI_CR1_MSTR | (5 << SPI_CR1_BR_Pos) | SPI_CR1_SPE;    // baud = clock/64
    }

    void drv_spi_end() {
        SPI1->CR1 = 0; // clear SPE
        // SPI1 CS-> gpio
        GPIO_SETL(A, 4, 1, 0, 0);
        GPIOA->BSRR = GPIO_ODR_OD4;

        // SPI needs reinit
        if (spi_reinit_callback_ != nullptr) {
            spi_reinit_callback_();
        }
    }

    void drv_disable() {
        GPIOC->BSRR = GPIO_BSRR_BR13; // drv disable
    }

    void drv_enable() {
        GPIOC->BSRR = GPIO_BSRR_BS13; // drv enable
        ms_delay(10);

        drv_spi_start();
        
        for (uint8_t i=0; i<sizeof(param->drv_regs)/sizeof(uint16_t); i++) {
            uint16_t reg_out = param->drv_regs[i];
            uint16_t reg_in = 0;
            write_reg(reg_out);
            reg_in = read_reg(reg_out);
            if ((reg_in & 0x7FF) != (reg_out & 0x7FF)) {
                drv_regs_error |= 1 << i;
            }
        }

        drv_spi_end();
    }

    std::string drv_reset() {
        drv_disable();
        ms_delay(10);
        drv_enable();
        return "ok";
    }

    uint16_t write_reg(uint16_t reg_out) {
        (*register_operation_)++;
        regs_.DR = reg_out;
        while(!(regs_.SR & SPI_SR_RXNE));
        uint16_t reg_in = regs_.DR;
        (*register_operation_)--;
        return reg_in;
    }

    uint16_t read_reg(uint8_t address) {
        (*register_operation_)++;
        uint16_t out_value = 1<<15 | address<<11;
        regs_.DR = out_value;
        while(!(regs_.SR & SPI_SR_RXNE));
        uint16_t value = regs_.DR;
        (*register_operation_)--;
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