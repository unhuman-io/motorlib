#pragma once

#include <coroutine>
#include <cstdint>
#include <cstddef>
#include "util.h"

// Provided by your hardware/HAL
constexpr uint32_t cpu_frequency_hz = CPU_FREQUENCY_HZ;

class CycleScheduler {
private:
    struct Sleeper {
        std::coroutine_handle<> handle = nullptr;
        uint32_t target_time = 0; // Unified absolute target
    };

    static constexpr size_t MAX_TASKS = 4;
    Sleeper sleepers[MAX_TASKS];

public:
    void poll() {
        uint32_t now = get_clock();
        
        for (auto& s : sleepers) {
            if (s.handle != nullptr) {
                // Safe 32-bit wrap-around comparison for absolute deadlines
                if ((int32_t)(now - s.target_time) >= 0) {
                    auto h = s.handle;
                    s.handle = nullptr; // Clear slot
                    h.resume();         // Jump back to coroutine
                }
            }
        }
    }

    // 1. Absolute delay (perfectly locked frequency, no phase drift)
    auto delay_until(uint32_t target_time) {
        struct Awaiter {
            CycleScheduler& sched;
            uint32_t target_time;
            
            bool await_ready() const {
                return (int32_t)(get_clock() - target_time) >= 0; 
            }
            
            void await_suspend(std::coroutine_handle<> h) {
                for (auto& s : sched.sleepers) {
                    if (s.handle == nullptr) {
                        s.target_time = target_time;
                        s.handle = h;
                        return; // Success
                    }
                }
                while(1); // Trap: Scheduler sleep queue is full!
            }
            void await_resume() {}
        };

        return Awaiter{ *this, target_time };
    }

    // 2. Relative delay (built cleanly on top of delay_until)
    auto async_delay_us(uint32_t us) {
        uint32_t cycles = us * (cpu_frequency_hz / 1'000'000);
        // Just calculate the absolute deadline and reuse the logic!
        return delay_until(get_clock() + cycles);
    }

    // Yield the CPU for exactly one pass of the main loop
    auto yield() {
        struct YieldAwaiter {
            CycleScheduler& sched;
            
            // ALWAYS suspend to force a context switch
            bool await_ready() const { return false; } 
            
            void await_suspend(std::coroutine_handle<> h) {
                for (auto& s : sched.sleepers) {
                    if (s.handle == nullptr) {
                        // Set target time to current time so it wakes up instantly on the next poll
                        s.target_time = get_clock(); 
                        s.handle = h;
                        return;
                    }
                }
                while(1); // Trap: Scheduler queue full!
            }
            void await_resume() {}
        };

        return YieldAwaiter{ *this };
    }
};

// ============================================================================
// Coroutine Return Base (Solves the void duplication problem)
// ============================================================================
template <typename T>
struct PromiseReturn {
    T value_;
    void return_value(T v) { value_ = v; }
};

template <>
struct PromiseReturn<void> {
    void return_void() {}
};

// ============================================================================
// Unified Task Template
// ============================================================================
template <typename T = void>
struct Task {
    // Inherit the return logic (void vs T) based on the template parameter
    struct promise_type : public PromiseReturn<T> {
        
        Task get_return_object() {
            return Task{std::coroutine_handle<promise_type>::from_promise(*this)};
        }
        
        std::suspend_never initial_suspend() { return {}; }
        std::suspend_always final_suspend() noexcept { return {}; }
        
        void unhandled_exception() { while(1); } 
    };

    std::coroutine_handle<promise_type> handle_;

    explicit Task(std::coroutine_handle<promise_type> h) : handle_(h) {}
    
    ~Task() {
        if (handle_) { handle_.destroy(); } 
    }

    Task(const Task&) = delete;
    Task& operator=(const Task&) = delete;
    
    Task(Task&& other) noexcept : handle_(other.handle_) {
        other.handle_ = nullptr;
    }
    
    Task& operator=(Task&& other) noexcept {
        if (this != &other) {
            if (handle_) handle_.destroy();
            handle_ = other.handle_;
            other.handle_ = nullptr;
        }
        return *this;
    }
    
    bool is_done() const {
        return !handle_ || handle_.done();
    }
};
