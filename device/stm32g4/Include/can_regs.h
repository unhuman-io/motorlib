#pragma once
// Device layer for the shared can_init module: FDCAN instance and message RAM
// addresses, and the bit definitions the module checks itself against.
#include "st_device.h"

inline constexpr uint32_t can_instance_base[] = {FDCAN1_BASE, FDCAN2_BASE, FDCAN3_BASE};
inline constexpr uint32_t can_instance_stride = FDCAN2_BASE - FDCAN1_BASE;
inline constexpr uint32_t can_ram_base = SRAMCAN_BASE;
inline constexpr uint32_t can_ram_stride = 0x350;   // one message RAM block per instance

namespace can_bits {
inline constexpr uint32_t CCCR_INIT     = FDCAN_CCCR_INIT;
inline constexpr uint32_t CCCR_CCE      = FDCAN_CCCR_CCE;
inline constexpr uint32_t CCCR_FDOE     = FDCAN_CCCR_FDOE;
inline constexpr uint32_t CCCR_BRSE     = FDCAN_CCCR_BRSE;
inline constexpr uint32_t NBTP_NSJW_Pos = FDCAN_NBTP_NSJW_Pos;
inline constexpr uint32_t DBTP_TDC      = FDCAN_DBTP_TDC;
inline constexpr uint32_t TDCR_TDCO_Pos = FDCAN_TDCR_TDCO_Pos;
inline constexpr uint32_t RXGFC_ANFS    = FDCAN_RXGFC_ANFS;
inline constexpr uint32_t TXFQS_TFQF    = FDCAN_TXFQS_TFQF;
}
