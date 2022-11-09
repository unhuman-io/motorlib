#pragma once
#include "../../torque_sensor.h"
#include "../../logger.h"

class QIA128_UART : public TorqueSensorBase {
 public:
    QIA128_UART(USART_TypeDef &regs) : regs_(regs) {}
    bool init() {
        for (uint8_t b : {0, 6, 0, 0x0c, 0, 0x3c}) {
            regs_.TDR = b;
        }

        ms_delay(100);
        while(regs_.ISR & USART_ISR_RXNE) {
            regs_.RDR;
        }

        for (uint8_t b : {0, 7, 3, 0x19, 0, 0, 0x7b}) {
            regs_.TDR = b;
        }

        for (int i=0; i<4; i++) {
            while(!(regs_.ISR & USART_ISR_RXNE));
            regs_.RDR;
            
        }
        for (int i=0; i<4; i++) {
            while(!(regs_.ISR & USART_ISR_RXNE));
            offset_ |= regs_.RDR << 8*(3-i);
        }
        while(!(regs_.ISR & USART_ISR_RXNE));
        regs_.RDR;
        
        ms_delay(10);

        for (uint8_t b : {0, 7, 3, 0x19, 0, 5, 0x99}) {
            regs_.TDR = b;
        }

        for (int i=0; i<4; i++) {
            while(!(regs_.ISR & USART_ISR_RXNE));
            regs_.RDR;
            
        }
        for (int i=0; i<4; i++) {
            while(!(regs_.ISR & USART_ISR_RXNE));
            full_scale_ |= regs_.RDR << 8*(3-i);
        }
        while(!(regs_.ISR & USART_ISR_RXNE));
        regs_.RDR;

        logger.log_printf("qia128 offset: %d, full scale: %d", offset_, full_scale_);
        full_scale_ = 16777215;
        offset_ = 8356324;

        ms_delay(10);
        for (uint8_t b : {0, 7, 4, 0x1e, 0, 7, 0xbc}) {
            regs_.TDR = b;
        }
        
        for (int i=0; i<5; i++) {
            while(!(regs_.ISR & USART_ISR_RXNE));
            regs_.RDR;
        }

        ms_delay(100);
        for (uint8_t b : {0, 6, 0, 0xc, 1, 0x41}) {
            regs_.TDR = b;
        }

        for (int i=0; i<5; i++) {
            while(!(regs_.ISR & USART_ISR_RXNE));
            regs_.RDR;
        }

        regs_.CR3 = 2 << USART_CR3_RXFTCFG_Pos; // 4 bytes fifo threshold



        // for (int i; i<4; i++) {
        //     while(!(regs_.ISR & USART_ISR_RXNE));
        //     offset_ |= regs_.RDR << 8*(3-i);
        // }
        return true;
    }

    float read() {
        if (state_ == WAIT) {
            // read character
            if (regs_.ISR & USART_ISR_RXNE) {
                regs_.RQR = USART_RQR_RXFRQ;
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
                
                raw_ = 0;
                read_len_ = 0;
                // 4 bytes received
                for (int i=0; i<3; i++) {
                    if (!(regs_.ISR & USART_ISR_RXNE)) {
                        read_error_++;
                    } else {
                        read_len_++;
                    }
                    raw_ |= regs_.RDR << 8*(2-i);
                }
                while (regs_.ISR & USART_ISR_RXNE) {
                        read_error2_++;
                        regs_.RDR;
                        read_len_++;
                }
                regs_.RQR = USART_RQR_RXFRQ;
                regs_.ISR &= ~USART_ISR_RXFT;
                torque_ = (float) ((int32_t) raw_- (int32_t) offset_)/(full_scale_ - offset_)*25;
                state_ = WAIT;
            }
        }
        return torque_;
    }
    float get_value() const { return torque_; }
    float torque_ = 0;
    uint32_t offset_ = 0;
    uint32_t full_scale_ = 0;
    uint32_t raw_ = 0;
    uint32_t wait_error_ = 0;
    uint32_t read_error_ = 0;
    uint32_t read_error2_ = 0;
    uint32_t read_len_ = 0;
 private:
    volatile USART_TypeDef& regs_;
    enum State {WAIT, READ} state_ = WAIT;
    uint8_t wait_count_ = 0;
    
};