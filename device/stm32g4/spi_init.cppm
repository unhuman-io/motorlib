module;
#include <cstdint>
#include "stm32g474xx.h"

export module device.spi_init;

export {
// Compile-time SPI configuration for the stm32g4 (the stm32g5 SPI is a
// different peripheral). Master only, with a gpio as the chip select: the
// transfer functions leave it to the caller. One SPIInit holds all four.

enum class SPIEnable : bool {
    DISABLED,
    ENABLED
};

enum class SPIBaudRate : uint8_t {      // CR1 BR: SCK from the bus clock of the instance
    PCLK_DIV2,
    PCLK_DIV4,
    PCLK_DIV8,
    PCLK_DIV16,
    PCLK_DIV32,
    PCLK_DIV64,
    PCLK_DIV128,
    PCLK_DIV256,
};

enum class SPIClockPolarity : uint8_t { // CR1 CPOL
    IDLE_LOW,
    IDLE_HIGH,
};

enum class SPIClockPhase : uint8_t {    // CR1 CPHA: the clock edge that samples
    FIRST_EDGE,
    SECOND_EDGE,
};

// Defaults are the register reset values, apart from the frame length
struct SPIPortInit {
    SPIEnable enable = SPIEnable::DISABLED;                 // CR1 SPE, as master with software slave management
    SPIBaudRate baud_rate = SPIBaudRate::PCLK_DIV2;
    SPIClockPolarity clock_polarity = SPIClockPolarity::IDLE_LOW;
    SPIClockPhase clock_phase = SPIClockPhase::FIRST_EDGE;
    uint8_t data_bits = 16;                                 // CR2 DS: 9 to 16. Shorter frames need byte accesses to DR, which the transfer functions don't do.
};

struct SPIInit {
    SPIPortInit spi1, spi2, spi3, spi4;
};

// spi_regs<3>() is SPI3
template<int spi>
inline SPI_TypeDef& spi_regs() {
    static_assert(spi >= 1 && spi <= 4, "no such spi on this device");
    if constexpr (spi == 1) return *SPI1;
    else if constexpr (spi == 2) return *SPI2;
    else if constexpr (spi == 3) return *SPI3;
    else return *SPI4;
}

inline consteval uint32_t get_spi_cr1(const SPIPortInit& c) {
    return SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_SPE |
        static_cast<uint32_t>(c.baud_rate) << SPI_CR1_BR_Pos |
        (c.clock_polarity == SPIClockPolarity::IDLE_HIGH ? SPI_CR1_CPOL : 0) |
        (c.clock_phase == SPIClockPhase::SECOND_EDGE ? SPI_CR1_CPHA : 0);
}

// The instance's clock (RCC enable) must be running. The frame length goes in
// before the enable.
template<const SPIPortInit c, int spi>
inline void init_spi_port() {
    if constexpr (c.enable == SPIEnable::ENABLED) {
        static_assert(c.data_bits >= 9 && c.data_bits <= 16, "frames are 9 to 16 bits");
        spi_regs<spi>().CR2 = static_cast<uint32_t>(c.data_bits - 1) << SPI_CR2_DS_Pos;
        spi_regs<spi>().CR1 = get_spi_cr1(c);
    }
}

template<const SPIInit h>
inline void init_spi() {
    init_spi_port<h.spi1, 1>();
    init_spi_port<h.spi2, 2>();
    init_spi_port<h.spi3, 3>();
    init_spi_port<h.spi4, 4>();
}

template<SPIInit spi_init>
inline void device_spi_config() {
    init_spi<spi_init>();
}

// One frame, in three steps so that the wait can be spent elsewhere:
// spi_write<3>(value) starts it on SPI3, spi_ready<3>() tells when the
// received frame is in, spi_read<3>() returns it.
template<int spi>
inline void spi_write(uint16_t value) {
    spi_regs<spi>().DR = value;
}
template<int spi>
inline bool spi_ready() {
    return (spi_regs<spi>().SR & SPI_SR_RXNE) != 0;
}
template<int spi>
inline uint16_t spi_read() {
    return spi_regs<spi>().DR;
}

// The same, waiting for the frame
template<int spi>
inline uint16_t spi_transfer(uint16_t value) {
    spi_write<spi>(value);
    while (!spi_ready<spi>());
    return spi_read<spi>();
}
}
