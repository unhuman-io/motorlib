#pragma once

#include "../max11254.h"

class MAX11254DMA : public MAX11254<> {
 public:
    MAX11254DMA(SPIDMA &spi_dma, DMAMUX_Channel_TypeDef &dmamux_tx_regs, DMAMUX_Channel_TypeDef &dmamux_rx_regs, uint8_t exti_num) : MAX11254(spi_dma, 0, false), tx_dma_(spi_dma.tx_dma_), rx_dma_(spi_dma_.rx_dma_),
        dmamux_tx_regs_(dmamux_tx_regs), dmamux_rx_regs_(dmamux_rx_regs), exti_num_(exti_num) {
            register_address dr = {.rw = 1, .addr = 14, .bits2 = 3};
            dma_buf_out_[0][0] = dr.word;
            dma_buf_out_[1][0] = dr.word;
        }
    bool init() {
        register_address stat_read = {.rw = 1, .addr = 0, .bits2 = 3};
        uint8_t data_out[5] = {stat_read.word};
        uint8_t data_in[5];
        spi_dma_.readwrite(data_out, data_in, 5);
        logger.log_printf("max11254 stat: %02x %02x %02x", data_in[2], data_in[3], data_in[4]);
        bool ret_val = true;
        //ret_val &= write_reg(8, 0x40); // in 1
        // reset sequence
        ret_val &= write_reg(1, 0x30); // reset
        command conv = {.rate=0b1111, .mode=1, .b7=1};
        spi_dma_.readwrite(&conv.word, data_in, 1);
        ms_delay(28);
        spi_dma_.readwrite(data_out, data_in, 5);
        logger.log_printf("max11254 stat: %02x %02x %02x", data_in[2], data_in[3], data_in[4]);

        cr1_register cr1 = {.scycle=0, .format=1};
        ret_val &= write_reg(1, cr1.word);   // continuous conversion
        // pga128
        cr2_register cr2 = {.pga=7, .pgaen=1, .ldoen=1};
        ret_val &= write_reg(2, cr2.word);
        ret_val &= write_reg(9, 0x1); //  GPO0 on

        spi_dma_.readwrite(data_out, data_in, 5);
        logger.log_printf("max11254 stat: %02x %02x %02x", data_in[2], data_in[3], data_in[4]);

        command conv2 = {.rate=0b1100, .mode=3, .b7=1}; // 8khz rate
        spi_dma_.readwrite(&conv2.word, data_in, 1);

        // spi_dma_.readwrite(data_out, data_in, 5);
        // logger.log_printf("max11254 stat: %02x %02x %02x", data_in[2], data_in[3], data_in[4]);
        start_continuous_dma();

        return ret_val;
    }

    void start_continuous_dma() {
        dmamux_tx_regs_.CCR |= exti_num_ << DMAMUX_CxCR_SYNC_ID_Pos | 4 << DMAMUX_CxCR_NBREQ_Pos | 2 << DMAMUX_CxCR_SPOL_Pos | DMAMUX_CxCR_SE;
        dmamux_rx_regs_.CCR |= 4 << DMAMUX_CxCR_NBREQ_Pos | DMAMUX_CxCR_EGE;
        spi_dma_.start_continuous_readwrite(dma_buf_out_[0], dma_buf_in_[0], 10);
    }

    void trigger(){}
    float read() {
        uint8_t *data_in = dma_buf_in_[get_new_buf_ptr()];
        raw_value_ = data_in[2] << 16 | data_in[3] << 8 | data_in[4];
        signed_value_ = raw_value_ - 0x7FFFFF;
        torque_ = signed_value_ * gain_;

        // stale values are currently the only fault mechanism
        if (last_new_signed_value_ != signed_value_) {
            last_new_signed_value_ = signed_value_;
            stale_value_count_ = 0;
        } else {
            stale_value_count_++;
            if (stale_value_count_ > 3) {
                timeout_error_++;
                stale_value_count_ = 0;
            }
        }
        if (raw_value_ == 0 || raw_value_ == 0xFFFFFF) {
            read_error_++;
        }
        return torque_;
    }

    bool is_new_read() {
        return stale_value_count_ == 0;
    }

    uint8_t get_new_buf_ptr() {
        // if CNDTR <= 5, return 1
        //   else return 0
        return rx_dma_.CNDTR <= 5;
    }

    DMA_Channel_TypeDef &tx_dma_, &rx_dma_;
    uint8_t dma_buf_in_[2][5] = {};
    uint8_t dma_buf_out_[2][5] = {};
    int8_t last_buf_ptr_ = -1;
    DMAMUX_Channel_TypeDef &dmamux_tx_regs_, &dmamux_rx_regs_;
    uint8_t exti_num_;

};
