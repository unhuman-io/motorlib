#include "can.h"
#include <cstring>

CAN::CAN(CAN_INST inst) : 
    regs_(*reinterpret_cast<FDCAN_GlobalTypeDef *>(inst*FDCAN_SIZE + FDCAN1_BASE)), 
    ram_(*reinterpret_cast<FDCANMessageRam *>(inst*FDCAN_MSG_RAM_SIZE + SRAMCAN_BASE)) {
    // enable queue
}

int CAN::read(uint8_t fifo, uint16_t id, uint8_t* data) {
    uint8_t ind = 0;
    RX_FIFO* buffer;
    switch (fifo) {
        case 0 :
            buffer = reinterpret_cast<RX_FIFO*>(ram_.RX_FIFO0[ind]);
            break;
        case 1:
            buffer = reinterpret_cast<RX_FIFO*>(ram_.RX_FIFO1[ind]);
            break;
        default:
            return 0;
            break;
    }
    uint8_t length = 0;
    if (buffer->id == id) {
        length = dlc_to_length(buffer->dlc);
        std::memcpy(data, buffer->data, length);
    }
    return length;
}

void CAN::write(uint16_t id, uint8_t* data, uint8_t length) {
    uint8_t fifo = 0; // get current fifo
    TX_BUFFER* buffer = reinterpret_cast<TX_BUFFER*>(ram_.TX_BUFFER[fifo]);
    ram_.TX_BUFFER[fifo][0] = 0;
    ram_.TX_BUFFER[fifo][1] = 0;
    buffer->id = id;
    buffer->efc = 0; // no event
    buffer->fdf = 1; // CAN FD
    buffer->brs = 1; // bit rate switch
    buffer->dlc = length_to_dlc(length);

    std::memcpy(buffer->data, data, sizeof(buffer->data));

    // send
}

bool CAN::add_acceptance_filter(uint16_t id, uint8_t fifo) {
    if (acceptance_filter_num_ >= 28) {
        return false;
    }
    FLSSA flssa = {
        .sfid2 = 0x7ff,
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