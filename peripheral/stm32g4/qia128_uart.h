#pragma once
#include "../../torque_sensor.h"
#include "../../logger.h"

uint8_t crc8(uint8_t *p, uint8_t len);

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
                read_len_ = 0;
                // 4 bytes received
                for (int i=0; i<3; i++) {
                    if (!(regs_.ISR & USART_ISR_RXNE)) {
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
                    if (crc8(raw_bytes_, 3) != crc_read_) {
                        crc_calc_ = crc8(raw_bytes_, 3);
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
    uint8_t raw_bytes_[3] = {};
    uint32_t wait_error_ = 0;
    uint32_t read_error_ = 0;
    uint32_t crc_error_ = 0;
    uint32_t read_len_ = 0;
    uint8_t crc_read_ = 0;
    uint8_t crc_calc_ = 0;
 private:
    volatile USART_TypeDef& regs_;
    enum State {WAIT, READ} state_ = WAIT;
    uint8_t wait_count_ = 0;
    
};

// from QIA128 spi communication guide
uint8_t const crc_table[256] = {
0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
0xfa, 0xfd, 0xf4, 0xf3
};

// -------------------------------------------------------------------------------------------------
uint8_t crc8(uint8_t *p, uint8_t len){
    uint16_t i;
    uint16_t crc = 0x0;
    while (len--) {
        i = (crc ^ *p++) & 0xFF;
        crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
    }
    return crc & 0xFF;
}