#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_QIA128_UART_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_QIA128_UART_H_

#include "../../torque_sensor.h"
#include "../../logger.h"
#include <initializer_list>

static uint8_t crc8(uint8_t *p);

class QIA128_UART : public TorqueSensorBase {
 public:
    QIA128_UART(USART_TypeDef &regs) : regs_(regs) {}
    bool init() {
        for (int i=0; i<1; i++) {
            // takes a while for this sensor to turn on
            ms_delay(200);
            IWDG->KR = 0xAAAA;
        }        
        
        clear_uart_receive_buffer();
        uart_tx({0, 5, 0, 1, 0xE}); // check for loopback on this value
        uart_rx_print(5);
        ms_delay(10);

        // other stuff
        uart_tx({00, 07, 03, 0x11, 00, 00, 0x5B}); 
        uint8_t gain = uart_rx(6)[4];
        logger.log_printf("qia128 gain: %d", gain);
        ms_delay(10);

        // normal initialization
        uart_tx({0, 6, 0, 0x0c, 0, 0x3c}); // set stream state off

        ms_delay(100);
        

        uart_tx({0, 7, 3, 0x19, 0, 0, 0x7b}); // get adc cal 0

        uart_rx_uint32(); // 0,9,3,19
        offset_ = uart_rx_uint32();
        uart_rx_uint8(); // checksum
        
        ms_delay(10);
        
        uint8_t crc[] = {0x7b, 0x81, 0x87, 0x8D, 0x93, 0x99, 0x9F, 0xA5, 0xAB, 0xB1,
                0xB7, 0xBD, 0xC3, 0xC9};
        for (uint8_t i=0; i<13; i++) {
            uart_tx({0, 7, 3, 0x19, 0, i, crc[i]}); // get adc cal 1

            uart_rx_uint32();
            uint32_t tmp = uart_rx_uint32();
            uart_rx_uint8();
            logger.log_printf("qia128 cal%d: %d",i, tmp);
            ms_delay(10);
            IWDG->KR = 0xAAAA;
        }        

        uart_tx({0, 7, 3, 0x19, 0, 5, 0x99}); // get adc cal 5
        uart_rx_uint32();
        full_scale_ = uart_rx_uint32();
        uart_rx_uint8();

        ms_delay(10);
        uart_tx({0, 5, 1, 0, 0xd}); // get device serial number
        uart_rx_uint32();
        uint32_t serial_number = uart_rx_uint32();
        uart_rx_uint8();

        logger.log_printf("qia128 serial number: %d", serial_number);

        ms_delay(10);
        uart_tx({0, 5, 1, 0, 0xd}); // get device serial number
        uart_rx_uint32();
        uint32_t profile_serial_number = uart_rx_uint32();
        uart_rx_uint8();

        logger.log_printf("qia128 profile_serial number: %d", profile_serial_number);

#ifdef IGNORE_QIA128_CALIBRATION
        offset_ = 8388608;
        full_scale_ = offset_+1;
#endif
        logger.log_printf("qia128 offset: %d, full scale: %d", offset_, full_scale_);

        ms_delay(10);
        uart_tx({0, 7, 4, 0x1e, 0, 7, 0xbc});   // set sampling rate 1300 sps
        uart_rx_uint8();
        uart_rx_uint32();

        ms_delay(100);
        uart_tx({0, 6, 0, 0xc, 1, 0x41});       // set stream state on
        uart_rx_uint8();
        uart_rx_uint32();


        regs_.CR3 = 2 << USART_CR3_RXFTCFG_Pos; // 4 bytes fifo threshold



        if ((int32_t) offset_ > 0 && (int32_t) full_scale_ > 0) {
            return true;
        } else {
            return false;
        }
    }

    uint8_t set_gain() {
        logger.log_printf("setting qia128 gain to 7");
        uart_tx({00, 0x08, 04, 0x11, 00, 00, 0x07, 0x91});
        ms_delay(100);
        uart_tx({00, 07, 03, 0x11, 00, 00, 0x5B}); 
        uint8_t gain = uart_rx(6)[4];
        logger.log_printf("qia128 gain: %d", gain);
        ms_delay(10);
        return gain;
    }

    // fifo size 8 bytes
    void uart_tx(std::initializer_list<uint8_t> data) {
        // std::vector<uint8_t> bytes{data};
        // std::string s = bytes_to_hex(bytes);
        // logger.log_printf("qia128 tx");
        // logger.log(bytes_to_hex(bytes));
        for (uint8_t b : data) {
            regs_.TDR = b;
        }
    }

    uint8_t uart_rx_uint8() {
        wait_while_false_with_timeout_us(regs_.ISR & USART_ISR_RXNE, 10000);
        return regs_.RDR;
    }

    std::vector<uint8_t> uart_rx(uint8_t count) {
        std::vector<uint8_t> bytes(count);
        for (int i=0; i<count; i++) {
            bytes[i] = uart_rx_uint8();
        }
        return bytes;
    }

    void uart_rx_print(uint8_t count) {
        std::vector<uint8_t> bytes = uart_rx(count);
        logger.log_printf("qia128 rx %d", count);
        logger.log(bytes_to_hex(bytes));
    }

    uint32_t uart_rx_uint32() {
        uint32_t value = 0;
        for (int i=0; i<4; i++) {
            value |= uart_rx_uint8() << 8*(3-i);
        }
        return value;
    }

    void clear_uart_receive_buffer() {
        while(regs_.ISR & USART_ISR_RXNE) {
            regs_.RDR;
        }
        uart_rx_flush();
    }

    void uart_rx_flush() {
        regs_.RQR = USART_RQR_RXFRQ;
    }

    float read() {
        if (state_ == WAIT) {
            // read character
            if (regs_.ISR & USART_ISR_RXNE) {
                uart_rx_flush();
                wait_count_ = 0;
                wait_error_++;
            } else {
                wait_count_++;
            }
            if (wait_count_ >= 3) {
                wait_count_ = 0;
                state_ = READ;
            }
        } else {
            if (regs_.ISR & USART_ISR_RXFT) {
                read_timeout_count_ = 0;
                read_len_ = 0;
                // 4 bytes received
                for (int i=0; i<3; i++) {
                    if (!(regs_.ISR & USART_ISR_RXNE)) { // move to read uint32 or 24
                        read_error_++;
                    } else {
                        read_len_++;
                    }
                    raw_bytes_[i] = regs_.RDR;
                }
                raw_ = raw_bytes_[0] << 16 | raw_bytes_[1] << 8 | raw_bytes_[2];
                if (regs_.ISR & USART_ISR_RXNE) {
                    read_len_++;
                    crc_read_ = regs_.RDR;
                    full_raw_ = raw_<<8 | crc_read_;
                    if (crc8(raw_bytes_) != crc_read_) {
                        crc_calc_ = crc8(raw_bytes_);
                        crc_error_++;
                    }
                } else {
                    read_error_++;
                }
                while (regs_.ISR & USART_ISR_RXNE) {
                        read_error_++;
                        regs_.RDR;
                        read_len_++;
                }
                regs_.RQR = USART_RQR_RXFRQ;
                regs_.ISR &= ~USART_ISR_RXFT;
                torque_ = (float) ((int32_t) raw_- (int32_t) offset_)/(full_scale_ - offset_)*gain_ + bias_;
                // TODO: remove this in favor of reading if the sensor is connected
                if (!std::isfinite(torque_)) {
                    torque_ = 0;
                }
                state_ = WAIT;
            } else {
                read_timeout_count_++;
                if (read_timeout_count_ > 10) {
                    read_timeout_count_ = 0;
                    timeout_error_++;
                }
            }
        }
        return torque_;
    }
    void clear_faults() {
        timeout_error_ = 0;
        read_error_ = 0;
        crc_error_ = 0;
        wait_error_ = 0;
    }
    float get_value() const { return torque_; }
    float torque_ = 0;
    uint32_t offset_ = 0;
    uint32_t full_scale_ = 0;
    uint32_t raw_ = 0;
    uint8_t raw_bytes_[3] = {};
    uint32_t wait_error_ = 0;
    uint32_t read_error_ = 0;
    uint32_t crc_error_ = 0;
    uint32_t timeout_error_ = 0;
    uint32_t read_len_ = 0;
    uint8_t crc_read_ = 0;
    uint8_t crc_calc_ = 0;
    uint32_t full_raw_ = 0;
    uint32_t read_timeout_count_ = 0;
 private:
    volatile USART_TypeDef& regs_;
    enum State {WAIT, READ} state_ = WAIT;
    uint8_t wait_count_ = 0;
    
};

// From direct communications with Futek
uint8_t crc8(uint8_t *p) {
    return p[0]*1 + p[1]*2 + p[2]*3;
}

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_QIA128_UART_H_
