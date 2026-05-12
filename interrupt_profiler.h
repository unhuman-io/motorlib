#pragma once

#include <cstdint>
#include <atomic>
#include <cstring>
#include "util.h" // For get_clock()

struct CycleStats {
    uint32_t max;
    uint32_t min;
    uint32_t accum_internal; // Accumulator for Net time
    uint32_t accum_gross;    // Accumulator for Wall time
    uint32_t count;
};

struct ProcessedStats {
    float avg_internal;
    float avg_total;
    float avg_cpu;
    uint32_t max;
    uint32_t min;
    uint32_t count;
};

struct AllProcessedStats {
    ProcessedStats fastloop;
    ProcessedStats mainloop;
    ProcessedStats systemloop;
    ProcessedStats comint;
};

struct InterruptStats {
    CycleStats fastloop_stats;
    CycleStats mainloop_stats;
    CycleStats systemloop_stats;
    CycleStats comint_stats;
};

// --- Storage ---
inline InterruptStats stats_buffer[2] = {
    { .fastloop_stats = {0, 0xFFFFFFFF, 0, 0, 0}, 
      .mainloop_stats = {0, 0xFFFFFFFF, 0, 0, 0}, 
      .systemloop_stats = {0, 0xFFFFFFFF, 0, 0, 0}, 
      .comint_stats = {0, 0xFFFFFFFF, 0, 0, 0} },
    { .fastloop_stats = {0, 0xFFFFFFFF, 0, 0, 0}, 
      .mainloop_stats = {0, 0xFFFFFFFF, 0, 0, 0}, 
      .systemloop_stats = {0, 0xFFFFFFFF, 0, 0, 0}, 
      .comint_stats = {0, 0xFFFFFFFF, 0, 0, 0} }
};

inline std::atomic<InterruptStats*> active_stats{&stats_buffer[0]};
inline std::atomic<uint32_t> g_stolen_cycles{0};

// --- Helpers ---

static inline uint32_t get_v_tick() {
    uint32_t s1, s2, c;
    do {
        s1 = g_stolen_cycles.load(std::memory_order_relaxed);
        c  = get_clock();
        s2 = g_stolen_cycles.load(std::memory_order_relaxed);
    } while (s1 != s2); 
    return (c - s1);
}

static inline void update_stolen_cycles(uint32_t net_to_add) {
    g_stolen_cycles.fetch_add(net_to_add, std::memory_order_relaxed);
}

static inline void update_stats(CycleStats* s, uint32_t net, uint32_t gross) {
    if (net > s->max) s->max = net;
    if (net < s->min) s->min = net;
    s->accum_internal += net;
    s->accum_gross += gross;
    s->count++;
}

// --- The Profiler Template ---

template <bool IsHighestPriority = false>
struct Profiler {
    uint32_t t_start_v;
    uint32_t t_start_wall;

    inline __attribute__((always_inline)) void start() {
        t_start_wall = get_clock();
        if constexpr (IsHighestPriority) {
            t_start_v = t_start_wall; // No odometer read needed
        } else {
            t_start_v = get_v_tick();
        }
    }

    inline __attribute__((always_inline)) void end(CycleStats& stats, 
                                                   uint32_t& legacy_exec, 
                                                   uint32_t& legacy_period, 
                                                   uint32_t& legacy_count,
                                                   uint32_t& last_start_wall) {
        uint32_t t_end_wall = get_clock();
        uint32_t t_end_v;

        if constexpr (IsHighestPriority) {
            t_end_v = t_end_wall;
        } else {
            t_end_v = get_v_tick();
        }

        uint32_t net = t_end_v - t_start_v;
        uint32_t gross = t_end_wall - t_start_wall;

        // Even high priority must inform the odometer so lower priority 
        // tasks can subtract this time.
        update_stolen_cycles(net);

        // Update legacy globals
        legacy_exec = net;
        legacy_period = t_start_wall - last_start_wall;
        legacy_count += net;
        last_start_wall = t_start_wall;

        // Update double-buffered stats
        InterruptStats* s_ptr = active_stats.load(std::memory_order_relaxed);
        update_stats(&stats, net, gross);
    }
};

inline AllProcessedStats get_exec_stats() {
    static InterruptStats* spare_buffer = &stats_buffer[1];
    static uint32_t last_call_time = 0;
    
    // 1. Calculate how many cycles actually passed since the last report
    uint32_t current_time = get_clock();
    uint32_t elapsed_cycles = current_time - last_call_time;
    last_call_time = current_time;

    // 2. Swap buffers
    InterruptStats* filled_stats = active_stats.exchange(spare_buffer);
    
    // 3. Helper lambda to process and calculate %
    auto process = [&](const CycleStats& cs) {
        ProcessedStats ps = {0};
        ps.count = cs.count;
        if (cs.count > 0) {
            ps.avg_internal = (float)cs.accum_internal / cs.count;
            ps.avg_total    = (float)cs.accum_gross / cs.count;
            ps.max          = cs.max;
            ps.min          = cs.min;
            // Use internal cycles for CPU load to avoid double-counting preemption
            ps.avg_cpu     = ((float)cs.accum_internal / elapsed_cycles) * 100.0f;
        }
        return ps;
    };

    // 4. Populate results
    AllProcessedStats result;
    result.fastloop   = process(filled_stats->fastloop_stats);
    result.mainloop   = process(filled_stats->mainloop_stats);
    result.systemloop = process(filled_stats->systemloop_stats);
    result.comint     = process(filled_stats->comint_stats);

    // 6. Reset and swap
    std::memset(filled_stats, 0, sizeof(InterruptStats));
    filled_stats->fastloop_stats.min = 0xFFFFFFFF;
    filled_stats->mainloop_stats.min = 0xFFFFFFFF;
    filled_stats->systemloop_stats.min = 0xFFFFFFFF;
    filled_stats->comint_stats.min = 0xFFFFFFFF;
    spare_buffer = filled_stats;

    return result;
}