#include "can.h"
#include <cstring>
#include <algorithm>
#include "st_device.h"

CAN::CAN(CAN_INST inst, ArbitrationBaudRate arb, DataBaudRate data) : 
    regs_(*reinterpret_cast<FDCAN_GlobalTypeDef *>(inst*FDCAN_SIZE + FDCAN1_BASE)), 
    ram_(*reinterpret_cast<FDCANMessageRam *>(inst*FDCAN_MSG_RAM_SIZE + SRAMCAN_BASE)) {
    // Only if not reset
    // regs_.CCCR |= FDCAN_CCCR_INIT;
    // while (!(regs_.CCCR & FDCAN_CCCR_INIT));
    regs_.CCCR |= FDCAN_CCCR_CCE;


    // NTSEG1 + NTSEG2 + 3 = 170/(prescaler+1)/baud rate
    // (ntseg1 + 2) / (ntseg1 + ntseg2 + 3) = 0.75
    // ntseg1 max = 255, ntseg2 max = 127, nsjw max = 127
    if (CPU_FREQUENCY_HZ == 170000000) {
        switch (arb) {
            case ArbitrationBaudRate::ARB_1M:
                // samp paint = 0.7471
                regs_.NBTP = 120 << FDCAN_NBTP_NSJW_Pos | 125 << FDCAN_NBTP_NTSEG1_Pos | 42 << FDCAN_NBTP_NTSEG2_Pos;
                break;
            case ArbitrationBaudRate::ARB_2M:
                // samp point = 0.7529
                regs_.NBTP = 60 << FDCAN_NBTP_NSJW_Pos | 62 << FDCAN_NBTP_NTSEG1_Pos | 20 << FDCAN_NBTP_NTSEG2_Pos; 
                break;
            default:
                break;
        }
        // dtseg1 + dtseg2 + 3 = 170/(dbrp+1)*baud rate2
        // dtseg1 max = 31, dtseg2 max = 15, dsjw max = 15, dbrp max = 31 (beware values > 0 may not work with TDC)
        // normal transmitter delay 180 ns, glitch filter 150ns*170MHz = 25 (TDCF)
        // TDCO max is 127, it should be (2+DTSEG1)*(psc+1)
        switch (data) {
            case DataBaudRate::DATA_1M:
                // psc = 4, nq = 34 samp point = 0.7647
                regs_.DBTP = 24 << FDCAN_DBTP_DTSEG1_Pos | 7 << FDCAN_DBTP_DTSEG2_Pos | 15 << FDCAN_DBTP_DSJW_Pos | 4 << FDCAN_DBTP_DBRP_Pos;
                break;
            case DataBaudRate::DATA_2M:
                // psc = 4, nq = 17 samp point = 0.7647
                regs_.DBTP = 11 << FDCAN_DBTP_DTSEG1_Pos | 3 << FDCAN_DBTP_DTSEG2_Pos | 10 << FDCAN_DBTP_DSJW_Pos | 4 << FDCAN_DBTP_DBRP_Pos;
                break;
            case DataBaudRate::DATA_5M:
                // psc = 0, nq = 34  samp point = 0.7647
                regs_.DBTP = 24 << FDCAN_DBTP_DTSEG1_Pos | 7 << FDCAN_DBTP_DTSEG2_Pos | 15 << FDCAN_DBTP_DSJW_Pos | FDCAN_DBTP_TDC | 0 << FDCAN_DBTP_DBRP_Pos;
                regs_.TDCR = 26 << FDCAN_TDCR_TDCO_Pos | 25 << FDCAN_TDCR_TDCF_Pos;
                break;
            case DataBaudRate::DATA_8M:
                // psc = 0, nq = 21  samp point = 0.7619, actual bitrate 8.095
                regs_.DBTP = 14 << FDCAN_DBTP_DTSEG1_Pos | 4 << FDCAN_DBTP_DTSEG2_Pos | 13 << FDCAN_DBTP_DSJW_Pos | FDCAN_DBTP_TDC | 0 << FDCAN_DBTP_DBRP_Pos;
                regs_.TDCR = 16 << FDCAN_TDCR_TDCO_Pos | 25 << FDCAN_TDCR_TDCF_Pos;
                break;
            default:
                break;
        }
    }

    regs_.CCCR |= FDCAN_CCCR_BRSE | FDCAN_CCCR_FDOE; // bit rate switch, fd mode


    regs_.TSCC = 1 << FDCAN_TSCC_TSS_Pos; // start counter
    regs_.RXGFC = 4 << FDCAN_RXGFC_LSS_Pos | FDCAN_RXGFC_ANFS | FDCAN_RXGFC_ANFE | FDCAN_RXGFC_RRFE; // 4 acceptance filters, reject everything else
    regs_.TXBC |= FDCAN_TXBC_TFQM; // transmit fifo request queue mode


    regs_.CCCR &= ~FDCAN_CCCR_INIT;
}

int CAN::read(uint8_t fifo, uint16_t id, uint8_t* data) {
    uint8_t ind = 0;
    RX_FIFO* buffer;
    switch (fifo) {
        case 0 :
            if (!(regs_.RXF0S & FDCAN_RXF0S_F0FL)) {
                return 0;
            }
            ind = (regs_.RXF0S >> FDCAN_RXF0S_F0GI_Pos) & 0x3;
            buffer = reinterpret_cast<RX_FIFO*>(ram_.RX_FIFO0[ind]);
            break;
        case 1:
            if (!(regs_.RXF1S & FDCAN_RXF1S_F1FL)) {
                return 0;
            }
            ind = (regs_.RXF1S >> FDCAN_RXF1S_F1GI_Pos) & 0x3;
            buffer = reinterpret_cast<RX_FIFO*>(ram_.RX_FIFO1[ind]);
            break;
        default:
            return 0;
            break;
    }
    int length = -1;
    if (buffer->id == id) {
        length = dlc_to_length(buffer->dlc);
        std::memcpy(data, buffer->data, length);
        switch (fifo) {
            // acknowledge
            case 0:
                regs_.RXF0A = ind;
                break;
            case 1:
                regs_.RXF1A = ind;
                break;
        }
    }
    return length;
}

void CAN::write(uint16_t id, uint8_t* data, uint8_t length) {
    if (regs_.TXFQS & FDCAN_TXFQS_TFQF) {
        // queue full
        return;
    }
    uint8_t buf_num = (regs_.TXFQS >> FDCAN_TXFQS_TFQPI_Pos) & 3; // get current fifo
    if (buf_num > 2) {
        // shouldn't occur?
        return;
    }
    TX_BUFFER* buffer = reinterpret_cast<TX_BUFFER*>(ram_.TX_BUFFER[buf_num]);
    TX_BUFFER::TXWord1 word1 = {
        .id = id,
    };
    TX_BUFFER::TXWord2 word2 = {
        .dlc = length_to_dlc(length),
        .brs = 1, // bit rate switch
        .fdf = 1, // CAN FD
        .efc = 1, // event
    };
    buffer->word1.word = word1.word;
    buffer->word2.word = word2.word;

    uint8_t len_copy = std::min(std::min(length, (uint8_t) sizeof(buffer->data)), (uint8_t) 64);
    uint8_t len_copy32 = (len_copy+3) / 4;
    //std::memcpy(buffer->data, data, len_copy);
    // can buffer memory requires 32 bit writes
    for (int i = 0; i < len_copy32; i++) {
        buffer->data32[i] = *reinterpret_cast<uint32_t *>(&data[i*4]);
    }
    asm("dmb");

    // send
    regs_.TXBAR = 1 << buf_num;
}

bool CAN::add_acceptance_filter(uint16_t id, uint8_t fifo) {
    if (acceptance_filter_num_ >= 28) {
        return false;
    }
    FLSSA flssa = {
        .sfid2 = 0x7ff, // mask
        .sfid1 = id,
        .sfec = (uint32_t) (fifo + 1),
        .sft = 2, // classic filter
    };
    uint32_t *ptr = reinterpret_cast<uint32_t*>(&flssa);
    ram_.FLSSA[acceptance_filter_num_++] = *ptr;
    return true;
}

uint8_t CAN::dlc_to_length(uint8_t dlc) {
    uint8_t length = 0;
    switch (dlc) {
        case 15:
            length = 64;
            break;
        case 14:
            length = 48;
            break;
        case 13:
            length = 32;
            break;
        case 12:
            length = 24;
            break;
        case 11:
            length = 20;
            break;
        case 10:
            length = 16;
            break;
        case 9:
            length = 12;
            break;
        default:
            length = dlc;
            break;
    }
    return length;
}

uint8_t CAN::length_to_dlc(uint8_t length) {
    uint8_t dlc = length;
    if (length > 48) {
        dlc = 15;
    } else if (length > 32) {
        dlc = 14;
    } else if (length > 24) {
        dlc = 13;
    } else if (length > 20) {
        dlc = 12;
    } else if (length > 16) {
        dlc = 11;
    } else if (length > 12) {
        dlc = 10;
    } else if (length > 8) {
        dlc = 9;
    }
    return dlc;
}

void CAN::interrupt() {
}