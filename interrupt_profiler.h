#pragma once

#include <cstdint>
#include <atomic>
#include <cstring>
#include <algorithm>
#include "util.h"

struct CycleStats {
    uint32_t max_internal;
    uint32_t min_internal;
    uint32_t max_period;
    uint32_t min_period;
    uint32_t accum_internal;
    uint32_t accum_gross;
    uint32_t accum_period;
    uint32_t count;
};

struct ProcessedStats {
    float avg_internal;
    float avg_total;
    float avg_period;
    float avg_cpu;
    uint32_t jitter_period; // max_period - min_period
    uint32_t max_internal;
    uint32_t min_internal;
    uint32_t count;
};

struct AllProcessedStats {
    ProcessedStats fastloop;
    ProcessedStats mainloop;
    ProcessedStats systemloop;
    ProcessedStats comint;
    float total_cpu_per;
};

struct InterruptStats {
    CycleStats fastloop_stats;
    CycleStats mainloop_stats;
    CycleStats systemloop_stats;
    CycleStats comint_stats;
};

// --- Storage ---
inline InterruptStats stats_buffer[2] = {
    { .fastloop_stats = {0, 0xFFFFFFFF, 0, 0xFFFFFFFF, 0, 0, 0, 0}, 
      .mainloop_stats = {0, 0xFFFFFFFF, 0, 0xFFFFFFFF, 0, 0, 0, 0}, 
      .systemloop_stats = {0, 0xFFFFFFFF, 0, 0xFFFFFFFF, 0, 0, 0, 0}, 
      .comint_stats = {0, 0xFFFFFFFF, 0, 0xFFFFFFFF, 0, 0, 0, 0} },
    { .fastloop_stats = {0, 0xFFFFFFFF, 0, 0xFFFFFFFF, 0, 0, 0, 0}, 
      .mainloop_stats = {0, 0xFFFFFFFF, 0, 0xFFFFFFFF, 0, 0, 0, 0}, 
      .systemloop_stats = {0, 0xFFFFFFFF, 0, 0xFFFFFFFF, 0, 0, 0, 0}, 
      .comint_stats = {0, 0xFFFFFFFF, 0, 0xFFFFFFFF, 0, 0, 0, 0} }
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

// --- The Profiler Template ---

template <bool IsHighestPriority = false>
struct Profiler {
    uint32_t t_start_v;
    uint32_t t_start_wall;
    uint32_t cached_period;

    inline __attribute__((always_inline)) void start(uint32_t& last_start_wall) {
        t_start_wall = get_clock();
        
        if constexpr (IsHighestPriority) {
            t_start_v = t_start_wall;
        } else {
            t_start_v = get_v_tick();
        }

        // Calculate period. If last_start_wall is 0 (first run), we'll ignore it in end()
        cached_period = (last_start_wall == 0) ? 0 : (t_start_wall - last_start_wall);
        last_start_wall = t_start_wall;
    }

    inline __attribute__((always_inline)) void end(CycleStats& stats) {
        uint32_t t_end_wall = get_clock();
        uint32_t t_end_v;

        if constexpr (IsHighestPriority) {
            t_end_v = t_end_wall;
        } else {
            t_end_v = get_v_tick();
        }

        uint32_t net = t_end_v - t_start_v;
        uint32_t gross = t_end_wall - t_start_wall;

        update_stolen_cycles(net);

        // Update Internal Timing Stats
        if (net > stats.max_internal) stats.max_internal = net;
        if (net < stats.min_internal) stats.min_internal = net;
        
        // Update Period Stats (Ignore 0-period from first run)
        if (cached_period > 0) {
            if (cached_period > stats.max_period) stats.max_period = cached_period;
            if (cached_period < stats.min_period) stats.min_period = cached_period;
            stats.accum_period += cached_period;
        }

        stats.accum_internal += net;
        stats.accum_gross += gross;
        stats.count++;
    }
};

inline AllProcessedStats get_exec_stats() {
    static InterruptStats* spare_buffer = &stats_buffer[1];
    static uint32_t last_call_time = 0;
    
    uint32_t current_time = get_clock();
    uint32_t elapsed_cycles = (last_call_time == 0) ? 0 : (current_time - last_call_time);
    last_call_time = current_time;

    InterruptStats* filled_stats = active_stats.exchange(spare_buffer);
    
    auto process = [&](const CycleStats& cs) {
        ProcessedStats ps = {0};
        ps.count = cs.count;
        if (cs.count > 0) {
            ps.avg_internal = (float)cs.accum_internal / cs.count;
            ps.avg_total    = (float)cs.accum_gross / cs.count;
            ps.avg_period   = (float)cs.accum_period / cs.count;
            ps.max_internal = cs.max_internal;
            ps.min_internal = cs.min_internal;
            
            // Calculate jitter: only valid if we had more than 1 sample
            ps.jitter_period = (cs.max_period > cs.min_period) ? (cs.max_period - cs.min_period) : 0;
            
            if (elapsed_cycles > 0) {
                ps.avg_cpu = ((float)cs.accum_internal / elapsed_cycles) * 100.0f;
            }
        }
        return ps;
    };

    AllProcessedStats result;
    result.fastloop   = process(filled_stats->fastloop_stats);
    result.mainloop   = process(filled_stats->mainloop_stats);
    result.systemloop = process(filled_stats->systemloop_stats);
    result.comint     = process(filled_stats->comint_stats);
    
    result.total_cpu_per = result.fastloop.avg_cpu + result.mainloop.avg_cpu + 
                           result.systemloop.avg_cpu + result.comint.avg_cpu;

    // Reset buffer (note: mins are set to 0xFFFFFFFF)
    std::memset(filled_stats, 0, sizeof(InterruptStats));
    filled_stats->fastloop_stats.min_internal = 0xFFFFFFFF;
    filled_stats->fastloop_stats.min_period   = 0xFFFFFFFF;
    filled_stats->mainloop_stats.min_internal   = 0xFFFFFFFF;
    filled_stats->mainloop_stats.min_period     = 0xFFFFFFFF;
    filled_stats->systemloop_stats.min_internal = 0xFFFFFFFF;
    filled_stats->systemloop_stats.min_period   = 0xFFFFFFFF;
    filled_stats->comint_stats.min_internal     = 0xFFFFFFFF;
    filled_stats->comint_stats.min_period       = 0xFFFFFFFF;
    
    spare_buffer = filled_stats;

    return result;
}
