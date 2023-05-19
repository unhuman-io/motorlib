#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_I2C_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_I2C_H_

#include "stm32g474xx.h"

class I2C {
 public:
    I2C(I2C_TypeDef &regs, uint16_t speed_khz = 100, float timeout_ms=.1) : regs_(regs) {
            switch (speed_khz) {
                case 1000:
                    regs_.TIMINGR = 0x00802172; // 1 Mbps at 170 MHz clock
                    break;
                case 400:
                    regs_.TIMINGR = 0x10802d9b; // 400 kHz at 170 MHz clock
                    break;
                default:
                case 100:
                    regs_.TIMINGR = 0x30a0a7fb; // 100 kHz at 170 MHz clock
                    // 0xfb << I2C_TIMINGR_SCLL_Pos | 0xa7 << I2C_TIMINGR_SCLH_Pos | 0xa << I2C_TIMINGR_SCLDEL_Pos | 3 << I2C_TIMINGR_PRESC_Pos;
                    // 42.5 MHz, 5.9us low, 3.9 us high, .26 us low stetch
                    break;
            }
            timeout_ms_ = timeout_ms;       
            regs_.CR1 |= I2C_CR1_PE;
    }
    bool write(uint8_t address, int8_t nbytes, uint8_t *data, bool stop = false) {
        regs_.ISR = I2C_ISR_TXE; // flush txdr
        regs_.TXDR = data[0];
        regs_.CR2 = 0;
        regs_.CR2 = (address << 1) | (nbytes << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;

        for (int i=1; i<nbytes; i++) {
            uint32_t t_start = get_clock();
            while_timeout_ms(!tx_ready() && !trouble(), timeout_ms_);
            if(trouble() || timed_out(timeout_ms_)) {
                //logger.log_printf("i2c write trouble or timeout isr %x, byte %d, addr %x", regs_.ISR, i, address);
                clear_isr();
                regs_.CR2 = 0;
                return false;
            }
            regs_.TXDR = data[i];
        }
        uint32_t t_start = get_clock();
        while_timeout_ms(!transfer_complete() && !trouble(), timeout_ms_);
        if(trouble()) {
            //logger.log_printf("i2c write transfer complete trouble isr %x, addr %x", regs_.ISR, address);
            clear_isr();
            regs_.CR2 = 0;
            return false;
        }
        if(timed_out(timeout_ms_)) {
            //logger.log_printf("i2c write transfer complete timeout isr %x, addr %x", regs_.ISR, address);
            clear_isr();
            regs_.CR2 = 0;
            return false;
        }
        if (stop) {
            regs_.CR2 = I2C_CR2_STOP;
        }
        return true;
    }
    bool read(uint8_t address, uint8_t nbytes, uint8_t *data) {
        //regs_.CR2 = 0;
        regs_.CR2 = (address << 1) | I2C_CR2_RD_WRN | (nbytes << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;// | I2C_CR2_AUTOEND;

        for(int i=0;i<nbytes; i++) {
            uint32_t t_start = get_clock();
            while_timeout_ms(!rx_ready() && !trouble(), timeout_ms_);
            if(trouble() || timed_out(timeout_ms_)) {
                //logger.log_printf("i2c read trouble or timeout isr %x, byte %d, addr %x", regs_.ISR, i, address);
                clear_isr();
                return false;
            }
            if (rx_ready()) {
                data[i] = regs_.RXDR;
            } else {
                //logger.log_printf("i2c read rx not ready isr %x, byte %d, addr %x", regs_.ISR, i, address);
                return false;
            }
        }
        uint32_t t_start = get_clock();
        while_timeout_ms(!transfer_complete(), timeout_ms_);
        regs_.CR2 = I2C_CR2_STOP;
        if (timed_out(timeout_ms_)) {
            //logger.log_printf("i2c read transfer complete timeout isr %x, addr %x", regs_.ISR, address);
            return false;
        }
        return true;
    }
    bool tx_ready() const {
        return regs_.ISR &  I2C_ISR_TXE;
    }
    bool rx_ready() const {
        return regs_.ISR & I2C_ISR_RXNE;
    }
    bool transfer_complete() const {
        return regs_.ISR & I2C_ISR_TC;
    }
    bool trouble() const {
        return regs_.ISR & (I2C_ISR_NACKF | I2C_ISR_ARLO | I2C_ISR_BERR);
    }
 private:
    void clear_isr() {
        regs_.ICR |= 0x3F38;
    }
    float timeout_ms_;
    I2C_TypeDef &regs_;
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_I2C_H_
