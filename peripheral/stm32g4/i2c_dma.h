#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_I2C_DMA_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_I2C_DMA_H_

#include "stm32g474xx.h"
#include "../../util.h"

// safe to call only at one time, i.e. don't call from interrupt and main simultaneously
class I2C_DMA {
 public:
    I2C_DMA(I2C_TypeDef &regs, DMA_Channel_TypeDef &tx_dma, DMA_Channel_TypeDef &rx_dma, uint16_t speed_khz = 100) : 
        regs_(regs), tx_dma_(tx_dma), rx_dma_(rx_dma) {
        init(speed_khz);
    }

    void init(uint16_t speed_khz) {
        regs_.CR1 &= ~I2C_CR1_PE;
        tx_dma_.CPAR = (uint32_t) &regs_.TXDR;
        rx_dma_.CPAR = (uint32_t) &regs_.RXDR;
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
                break;
        }            
        
        regs_.CR1 |= I2C_CR1_PE;
        regs_.CR1 |= I2C_CR1_RXDMAEN | I2C_CR1_TXDMAEN;
    }

    // return 1 for not ready, 0 for success
    int async_write(uint8_t address, int8_t nbytes, uint8_t *data, bool stop = false) {
        if (!(regs_.CR2 & I2C_CR2_AUTOEND)) {
            // this will be new or a repeat start, still need to wait for dma to be complete
            if(tx_dma_.CNDTR != 0) {
                return 1;
            }
        } else if (busy()) {
            return 1;
        }
        clear_isr();
        tx_dma_.CCR = 0;
        tx_dma_.CMAR = (uint32_t) data;
        tx_dma_.CNDTR = nbytes;
        tx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_DIR; // DIR = 1 > read from memory
        regs_.CR2 = (address << 1) | (nbytes << I2C_CR2_NBYTES_Pos) | (stop ? I2C_CR2_AUTOEND : 0) | I2C_CR2_START;
        return 0;
    }
    // return 1 for not ready, 0 for success
    int async_read(uint8_t address, uint8_t nbytes, uint8_t *data) {
        if (!(regs_.CR2 & I2C_CR2_AUTOEND)) {
            // this will be new or a repeat start, still need to wait for dma to be complete
            if(rx_dma_.CNDTR != 0) {
                return 1;
            }
        } else if (busy()) {
            return 1;
        }
        clear_isr();
        rx_dma_.CCR = 0;
        asm("nop"); // memory barrier in case optimization things nothing is using data[*]
        rx_dma_.CMAR = (uint32_t) data;
        rx_dma_.CNDTR = nbytes;
        rx_dma_.CCR = DMA_CCR_EN | DMA_CCR_MINC;
        regs_.CR2 = (address << 1) | I2C_CR2_RD_WRN | (nbytes << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | I2C_CR2_START;
        return 0;
    }

    void cancel_async_read() {
        rx_dma_.CCR = 0;
        rx_dma_.CNDTR = 0;
        regs_.CR2 |= I2C_CR2_STOP;
        ns_delay(10000);
        regs_.CR1 &= ~I2C_CR1_PE;
        regs_.CR1 |= I2C_CR1_PE;                    
        clear_isr();
    }

    void cancel_async_write() {
        tx_dma_.CCR = 0;
        tx_dma_.CNDTR = 0;
        regs_.CR2 |= I2C_CR2_STOP;
        ns_delay(10000);
        regs_.CR1 &= ~I2C_CR1_PE;
        regs_.CR1 |= I2C_CR1_PE;  
        clear_isr();
    }

    // return <= 0 for error, nbytes for success
    int write(uint8_t address, int8_t nbytes, uint8_t *data, bool stop=false, uint16_t timeout_us=1000) {
        bool error;
        uint32_t t_start = get_clock();
        bool timeout = false;
        do {
            error = async_write(address, nbytes, data, stop);
            timeout = get_clock() - t_start > CPU_FREQUENCY_HZ/1e6*timeout_us;
        } while(error && !trouble() && !timeout);
        if (error || trouble() || timeout) {
            cancel_async_write();
            return -1;
        }
        // wait for completion
        do {
            error = busy();
            timeout = get_clock() - t_start > CPU_FREQUENCY_HZ/1e6*timeout_us;
        } while(error && !trouble() && !timeout);
        if (error || trouble() || timeout) {
            //logger.log_printf("error: %d, trouble: %d, timeout: %d, isr: %x", error, trouble(), timeout, regs_.ISR);
            cancel_async_write();
            return -2;
        }

        return nbytes;
    }

    // return <= 0 for error, nbytes for success
    int read(uint8_t address, int8_t nbytes, uint8_t *data, uint16_t timeout_us=1000) {
        bool error;
        uint32_t t_start = get_clock();
        bool timeout = false;
        do {
            error = async_read(address, nbytes, data);
            timeout = get_clock() - t_start > CPU_FREQUENCY_HZ/1e6*timeout_us;
        } while(error && !trouble() && !timeout);
        if (trouble()) {
            cancel_async_read();
            return -1;
        }
        if (timeout) {
            cancel_async_read();
            return -2;
        }
        do {
            error = busy();
            timeout = get_clock() - t_start > CPU_FREQUENCY_HZ/1e6*timeout_us;
        } while(error && !trouble() && !timeout);
        if (trouble()) {
            cancel_async_read();
            return -3;
        }
        if (timeout) {
            //logger.log_printf("error: %d, trouble: %d, timeout: %d, isr: %x", error, trouble(), timeout, regs_.ISR);
            cancel_async_read();
            return -4;
        }
        asm("nop"); // todo: a nop seems necessary in order to recognize a data update (due to dma), make volatile maybe
        return nbytes;
    }
    volatile bool busy() const {
        // note start can be asserted before busy becomes active
        if (regs_.ISR & I2C_ISR_TC) {
            return false;
        }
        return (regs_.ISR & I2C_ISR_BUSY) | (regs_.CR2 & I2C_CR2_START);
    }
    volatile bool ready() const {
        return !busy();
    }
    volatile bool trouble() const {
        return regs_.ISR & (I2C_ISR_NACKF | I2C_ISR_ARLO | I2C_ISR_BERR);
    }
 private:
    void clear_isr() {
        regs_.ICR = 0x3F30;
    }
    I2C_TypeDef &regs_;
    DMA_Channel_TypeDef &tx_dma_, &rx_dma_;
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_I2C_DMA_H_
