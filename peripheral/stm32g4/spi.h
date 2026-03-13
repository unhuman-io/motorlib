#pragma once

struct SPI {
    SPI_TypeDef &regs_;

    void write(uint32_t data) {
        regs_.DR = data;
    }

    uint32_t read_u8() {
        return regs_.DR;
    }

    uint32_t read_u16() {
        return regs_.DR;
    }

    bool data_available() {
        return regs_.SR & SPI_SR_RXNE;
    }
};

