#pragma once

struct USART_SPI {
    USART_TypeDef &regs_;

    void write(uint8_t data) {
        regs_.TDR = data;
    }

    void write(uint16_t data) {
        regs_.TDR = data >> 8;
        regs_.TDR = data & 0xff;
    }

    uint8_t read_u8() {
        return regs_.RDR;
    }

    uint16_t read_u16() {
        return read_u8() << 8 | read_u8();
    }

    bool data_available() {
        return regs_.ISR & USART_ISR_RXFT;
    }
};

