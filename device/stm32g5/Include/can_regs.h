#pragma once
// Device layer for the shared can_init module: FDCAN instance and message RAM
// addresses, and the bit definitions the module checks itself against.
#include "st_device.h"

inline constexpr uint32_t can_instance_base[] = {FDCAN1_BASE, FDCAN2_BASE, FDCAN3_BASE};
inline constexpr uint32_t can_instance_stride = FDCAN2_BASE - FDCAN1_BASE;
inline constexpr uint32_t can_ram_base = 0x4000AC00;   // FDCAN message RAM, not in the SVD header
inline constexpr uint32_t can_ram_stride = 0x350;      // one message RAM block per instance

namespace can_bits {
inline constexpr uint32_t CCCR_INIT     = FDCAN1_FDCAN_CCCR_INIT_Msk;
inline constexpr uint32_t CCCR_CCE      = FDCAN1_FDCAN_CCCR_CCE_Msk;
inline constexpr uint32_t CCCR_FDOE     = FDCAN1_FDCAN_CCCR_FDOE_Msk;
inline constexpr uint32_t CCCR_BRSE     = FDCAN1_FDCAN_CCCR_BRSE_Msk;
inline constexpr uint32_t NBTP_NSJW_Pos = FDCAN1_FDCAN_NBTP_NSJW_Pos;
inline constexpr uint32_t DBTP_TDC      = FDCAN1_FDCAN_DBTP_TDC_Msk;
inline constexpr uint32_t TDCR_TDCO_Pos = FDCAN1_FDCAN_TDCR_TDCO_Pos;
inline constexpr uint32_t RXGFC_ANFS    = FDCAN1_FDCAN_RXGFC_ANFS_Msk;
inline constexpr uint32_t TXFQS_TFQF    = FDCAN1_FDCAN_TXFQS_TFQF_Msk;
}
