module;
#include <cstddef>
#include <cstdint>
#include <utility>
#include "irq_regs.h"

export module device.irq_init;

export {
// Compile-time interrupt configuration, shared between devices: one IRQInit
// names the handler and priority of each interrupt line. The vector table and
// the NVIC setup are both made from it, so a line is enabled exactly when it
// has a handler. The device's irq_regs.h supplies the number of lines; they
// are indexed by the IRQn names of the device header (ADC3_IRQn, ...).

using IRQHandler = void (*)();

// Defaults are the reset state
struct IRQLineInit {
    IRQHandler handler = nullptr;   // nullptr: the line is not used and stays disabled
    uint8_t priority = 0;           // NVIC IPR: 0 is the most urgent
};

struct IRQInit {
    IRQLineInit irq[irq_count];
};

// Where the vectors without a handler go, the faults among them
inline void irq_default_handler() {
    while (true) __asm__ volatile("");
}

// Vectors 1..15: reset, the faults and the other system exceptions
inline constexpr int irq_exception_count = 15;

template<int count>
struct VectorTable {
    const void* stack;              // vector 0: the initial stack pointer
    IRQHandler vector[count];       // reset, the system exceptions, then the interrupt lines
};
static_assert(sizeof(VectorTable<irq_exception_count>) == (1 + irq_exception_count) * sizeof(IRQHandler));

// Interrupt lines in the table: it ends after the last one with a handler
template<const IRQInit h>
inline consteval int irq_line_count() {
    int count = 0;
    for (int n = 0; n < irq_count; n++) {
        if (h.irq[n].handler != nullptr) count = n + 1;
    }
    return count;
}

// The table for the .isr_vector section, with reset as the entry point:
//   __attribute__((section(".isr_vector"), used))
//   constinit const auto vector_table = get_vector_table<irq_init>(&_estack, Reset_Handler);
template<const IRQInit h>
inline consteval VectorTable<irq_exception_count + irq_line_count<h>()> get_vector_table(const void* stack, IRQHandler reset) {
    VectorTable<irq_exception_count + irq_line_count<h>()> table{};
    table.stack = stack;
    for (IRQHandler& vector : table.vector) vector = irq_default_handler;
    table.vector[0] = reset;
    for (int n = 0; n < irq_line_count<h>(); n++) {
        if (h.irq[n].handler != nullptr) table.vector[irq_exception_count + n] = h.irq[n].handler;
    }
    return table;
}

// NVIC ISER word: the lines with a handler
template<const IRQInit h>
inline consteval uint32_t get_irq_enable(int word) {
    uint32_t v = 0;
    for (int n = 32 * word; n < 32 * (word + 1) && n < irq_count; n++) {
        if (h.irq[n].handler != nullptr) v |= 1u << (n - 32 * word);
    }
    return v;
}

// Only what differs from reset is written
template<int irq, bool used, uint8_t priority>
inline void init_irq_priority() {
    static_assert(priority < (1 << irq_priority_bits), "interrupt priority out of range");
    static_assert(used || priority == 0, "priority on an interrupt without a handler");
    if constexpr (priority != 0) irq_priority_reg(irq) = static_cast<uint8_t>(priority << (8 - irq_priority_bits));
}

template<int word, uint32_t enable>
inline void init_irq_enable() {
    if constexpr (enable != 0) NVIC->ISER[word] = enable;   // writing 0 bits leaves lines alone
}

template<const IRQInit h>
inline void init_irq() {
    []<size_t... n>(std::index_sequence<n...>) {
        (init_irq_priority<n, h.irq[n].handler != nullptr, h.irq[n].priority>(), ...);
    }(std::make_index_sequence<irq_count>{});
    []<size_t... word>(std::index_sequence<word...>) {
        (init_irq_enable<word, get_irq_enable<h>(word)>(), ...);
    }(std::make_index_sequence<(irq_count + 31) / 32>{});
}

// Makes the table the active one (VTOR), so it need not be at address 0, then
// enables the lines. Call it last: the handlers can run from here on.
template<IRQInit irq_init, int count>
inline void device_irq_config(const VectorTable<count>& table) {
    SCB->VTOR = reinterpret_cast<uint32_t>(&table);
    init_irq<irq_init>();
}
}
