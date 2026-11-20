module;
#include <cstdint>
#include <cstring>
#include <type_traits>
#include "can_regs.h"

export module device.can_init;

export {
// Compile-time FDCAN configuration, shared between devices: the register
// block (Bosch M_CAN) and the fixed message RAM layout are the same on all
// ST FDCAN parts; the device's can_regs.h supplies the addresses. Bit timings
// are computed from the kernel clock at compile time.

enum class CANEnable : bool {
    DISABLED,
    ENABLED
};

struct CANInit {
    uint8_t instance = 0;                                   // FDCAN1 = 0
    uint32_t kernel_clock_hz = 0;                           // fdcan_ker_ck, see the RCC clock init
    uint32_t nominal_bitrate_hz = 1'000'000;                // arbitration phase
    uint32_t data_bitrate_hz = 8'000'000;                   // data phase with bit rate switching
    uint8_t nominal_sample_point_percent = 75;
    uint8_t data_sample_point_percent = 75;
    CANEnable fd = CANEnable::ENABLED;                      // CCCR FDOE
    CANEnable bit_rate_switch = CANEnable::ENABLED;         // CCCR BRSE
    CANEnable transmitter_delay_compensation = CANEnable::ENABLED;   // DBTP TDC, needed above ~2 Mbps
    uint16_t transmitter_delay_filter_ns = 150;             // TDCR TDCF
};

// Same register layout on every ST FDCAN; names follow the stm32g4 header
struct CANRegs {
    volatile uint32_t CREL, ENDN, RESERVED1, DBTP, TEST, RWD, CCCR, NBTP, TSCC, TSCV, TOCC, TOCV,
        RESERVED2[4], ECR, PSR, TDCR, RESERVED3, IR, IE, ILS, ILE, RESERVED4[8],
        RXGFC, XIDAM, HPMS, RESERVED5, RXF0S, RXF0A, RXF1S, RXF1A, RESERVED6[8],
        TXBC, TXFQS, TXBRP, TXBAR, TXBCR, TXBTO, TXBCF, TXBTIE, TXBCIE, TXEFS, TXEFA, RESERVED7[5],
        CKDIV;
};
static_assert(offsetof(CANRegs, RXGFC) == 0x80 && offsetof(CANRegs, TXBC) == 0xC0 && offsetof(CANRegs, CKDIV) == 0x100);

struct CANTxElement {
    uint32_t id;      // ESI:31 XTD:30 RTR:29 ID:28..0 (standard id in 28..18)
    uint32_t control; // MM:31..24 EFC:23 FDF:21 BRS:20 DLC:19..16
    union {
        uint8_t data[64];
        uint32_t data32[16];
    };
};

struct CANMessageRam {   // fixed layout: 28 std filters, 8 ext filters, 3+3 rx fifo, 3 tx events, 3 tx buffers
    uint32_t std_filter[28];
    uint32_t ext_filter[8][2];
    uint32_t rx_fifo0[3][18];
    uint32_t rx_fifo1[3][18];
    uint32_t tx_event[3][2];
    CANTxElement tx_buffer[3];
};
static_assert(sizeof(CANMessageRam) == can_ram_stride);

inline CANRegs& can_regs(int instance) {
    return *reinterpret_cast<CANRegs*>(can_instance_base[instance]);
}
inline CANMessageRam& can_ram(int instance) {
    return *reinterpret_cast<CANMessageRam*>(can_ram_base + instance * can_ram_stride);
}

// Bit positions are the M_CAN ones; a few are checked against the device header
namespace can_bit {
inline constexpr uint32_t CCCR_INIT = 1u << 0, CCCR_CCE = 1u << 1, CCCR_FDOE = 1u << 8, CCCR_BRSE = 1u << 9;
inline constexpr uint32_t NBTP_NTSEG2_Pos = 0, NBTP_NTSEG1_Pos = 8, NBTP_NBRP_Pos = 16, NBTP_NSJW_Pos = 25;
inline constexpr uint32_t DBTP_DSJW_Pos = 0, DBTP_DTSEG2_Pos = 4, DBTP_DTSEG1_Pos = 8, DBTP_DBRP_Pos = 16, DBTP_TDC = 1u << 23;
inline constexpr uint32_t TDCR_TDCF_Pos = 0, TDCR_TDCO_Pos = 8;
inline constexpr uint32_t RXGFC_RRFE = 1u << 0, RXGFC_ANFE = 1u << 2, RXGFC_ANFS = 1u << 4;
inline constexpr uint32_t TXFQS_TFQPI_Pos = 16, TXFQS_TFQF = 1u << 21;
inline constexpr uint32_t TX_DLC_Pos = 16, TX_BRS = 1u << 20, TX_FDF = 1u << 21, TX_XTD = 1u << 30, TX_STDID_Pos = 18;
}

struct CANBitTiming {
    uint32_t prescaler, tseg1, tseg2, sjw;   // in time quanta; tseg1 includes the sync segment
};

void can_bit_timing_not_possible();   // not constexpr: calling it fails the constant evaluation

// Closest achievable timing for a bit rate: smallest prescaler whose quanta
// count fits the segment limits, within 1.5 % of the requested rate.
inline consteval CANBitTiming can_bit_timing(uint32_t kernel_hz, uint32_t bitrate_hz, uint8_t sample_point_percent,
                                             uint32_t max_prescaler, uint32_t max_tseg1, uint32_t max_tseg2, uint32_t max_sjw) {
    for (uint32_t prescaler = 1; prescaler <= max_prescaler; prescaler++) {
        const uint32_t tq = (kernel_hz + prescaler * bitrate_hz / 2) / (prescaler * bitrate_hz);
        if (tq < 8 || tq > max_tseg1 + max_tseg2 + 1) continue;
        const uint32_t actual = kernel_hz / (prescaler * tq);
        const uint32_t error_permille = (actual > bitrate_hz ? actual - bitrate_hz : bitrate_hz - actual) * 1000 / bitrate_hz;
        if (error_permille > 15) continue;
        uint32_t tseg2 = (tq * (100 - sample_point_percent) + 50) / 100;
        if (tseg2 < 1) tseg2 = 1;
        if (tseg2 > max_tseg2) tseg2 = max_tseg2;
        const uint32_t tseg1 = tq - 1 - tseg2;
        if (tseg1 < 1 || tseg1 > max_tseg1) continue;
        return {prescaler, tseg1, tseg2, tseg2 < max_sjw ? tseg2 : max_sjw};
    }
    can_bit_timing_not_possible();
    return {};
}

template<const CANInit c>
inline consteval CANBitTiming can_nominal_timing() {
    return can_bit_timing(c.kernel_clock_hz, c.nominal_bitrate_hz, c.nominal_sample_point_percent, 512, 256, 128, 128);
}
template<const CANInit c>
inline consteval CANBitTiming can_data_timing() {
    return can_bit_timing(c.kernel_clock_hz, c.data_bitrate_hz, c.data_sample_point_percent, 32, 32, 16, 16);
}

template<const CANInit c>
inline void init_can() {
    using namespace can_bit;
    static_assert(c.kernel_clock_hz != 0, "set kernel_clock_hz to the fdcan kernel clock selected in the RCC");
    constexpr CANBitTiming n = can_nominal_timing<c>();
    constexpr CANBitTiming d = can_data_timing<c>();
    auto& regs = can_regs(c.instance);

    regs.CCCR |= CCCR_INIT;
    while (!(regs.CCCR & CCCR_INIT));
    regs.CCCR |= CCCR_CCE;

    regs.NBTP = (n.sjw - 1) << NBTP_NSJW_Pos | (n.prescaler - 1) << NBTP_NBRP_Pos |
        (n.tseg1 - 1) << NBTP_NTSEG1_Pos | (n.tseg2 - 1) << NBTP_NTSEG2_Pos;
    constexpr bool tdc = c.transmitter_delay_compensation == CANEnable::ENABLED;
    regs.DBTP = (d.prescaler - 1) << DBTP_DBRP_Pos | (d.tseg1 - 1) << DBTP_DTSEG1_Pos |
        (d.tseg2 - 1) << DBTP_DTSEG2_Pos | (d.sjw - 1) << DBTP_DSJW_Pos | (tdc ? DBTP_TDC : 0);
    if constexpr (tdc) {
        // secondary sample point at the data sample point, measured from the start of the bit
        constexpr uint32_t tdco = d.tseg1 * d.prescaler > 127 ? 127 : d.tseg1 * d.prescaler;
        constexpr uint32_t tdcf = static_cast<uint64_t>(c.kernel_clock_hz) * c.transmitter_delay_filter_ns / 1'000'000'000 > 127 ? 127 :
            static_cast<uint64_t>(c.kernel_clock_hz) * c.transmitter_delay_filter_ns / 1'000'000'000;
        regs.TDCR = tdco << TDCR_TDCO_Pos | tdcf << TDCR_TDCF_Pos;
    }
    if constexpr (c.fd == CANEnable::ENABLED) regs.CCCR |= CCCR_FDOE;
    if constexpr (c.bit_rate_switch == CANEnable::ENABLED) regs.CCCR |= CCCR_BRSE;

    regs.RXGFC = RXGFC_ANFS | RXGFC_ANFE | RXGFC_RRFE;   // no filters: reject everything received
    regs.TXBC = 0;                                        // tx fifo
    regs.CCCR &= ~CCCR_INIT;
}

template<CANInit can_init>
inline void device_can_config() {
    init_can<can_init>();
}

// CAN FD data length coding: lengths above 8 round up to the next FD size
inline constexpr uint8_t can_fd_lengths[] = {12, 16, 20, 24, 32, 48, 64};
inline constexpr uint32_t can_dlc(uint32_t length) {
    if (length <= 8) return length;
    uint32_t dlc = 9;
    while (dlc < 15 && can_fd_lengths[dlc - 9] < length) dlc++;
    return dlc;
}
inline constexpr uint32_t can_dlc_length(uint32_t dlc) { return dlc <= 8 ? dlc : can_fd_lengths[dlc - 9]; }

// Bus-off (no other node acknowledging) needs a software restart; done here so
// queued frames go out as soon as a node appears. Returns the tx fifo put
// index, or -1 if the fifo is full.
template<const CANInit c>
inline int can_tx_slot() {
    using namespace can_bit;
    auto& regs = can_regs(c.instance);
    if (regs.CCCR & CCCR_INIT) regs.CCCR &= ~CCCR_INIT;
    const uint32_t fqs = regs.TXFQS;
    if (fqs & TXFQS_TFQF) return -1;
    return (fqs >> TXFQS_TFQPI_Pos) & 3;
}

// Queue one CAN FD frame with bit rate switching. Standard id unless
// extended; the payload type fixes the length at compile time (<= 64 bytes,
// lengths above 8 are padded to the next FD size). Returns false if the tx
// fifo is full.
template<const CANInit c, bool extended = false, typename T>
inline bool can_write(uint32_t id, const T& payload) {
    using namespace can_bit;
    using Payload = std::remove_cv_t<T>;   // volatile payloads are read once, here
    static_assert(std::is_trivially_copyable_v<Payload> && sizeof(Payload) <= 64, "payload must be a trivially copyable type of up to 64 bytes");
    constexpr uint32_t dlc = can_dlc(sizeof(Payload));
    constexpr uint32_t words = (can_dlc_length(dlc) + 3) / 4;
    const int index = can_tx_slot<c>();
    if (index < 0) return false;
    CANTxElement& e = can_ram(c.instance).tx_buffer[index];
    if constexpr (extended) {
        e.id = TX_XTD | (id & 0x1FFFFFFF);
    } else {
        e.id = (id & 0x7FF) << TX_STDID_Pos;
    }
    e.control = dlc << TX_DLC_Pos | TX_FDF | TX_BRS;
    const Payload copy = payload;
    uint32_t w[words] = {};
    std::memcpy(w, &copy, sizeof(Payload));
    for (uint32_t i = 0; i < words; i++) e.data32[i] = w[i];   // message ram wants word writes
    can_regs(c.instance).TXBAR = 1u << index;
    return true;
}

// Runtime length variant
template<const CANInit c, bool extended = false>
inline bool can_write(uint32_t id, const void* data, uint8_t length) {
    using namespace can_bit;
    const int index = can_tx_slot<c>();
    if (index < 0) return false;
    CANTxElement& e = can_ram(c.instance).tx_buffer[index];
    const uint32_t dlc = can_dlc(length < 64 ? length : 64);
    if constexpr (extended) {
        e.id = TX_XTD | (id & 0x1FFFFFFF);
    } else {
        e.id = (id & 0x7FF) << TX_STDID_Pos;
    }
    e.control = dlc << TX_DLC_Pos | TX_FDF | TX_BRS;
    uint32_t w[16] = {};
    std::memcpy(w, data, length < 64 ? length : 64);
    for (uint32_t i = 0; i < (can_dlc_length(dlc) + 3) / 4; i++) e.data32[i] = w[i];
    can_regs(c.instance).TXBAR = 1u << index;
    return true;
}
}
