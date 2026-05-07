// todo make module. gcc 15 has internal compiler error
// module;

#include <coroutine>
#include <cstdint>
#include <cstddef>
#include "util.h"

//export module task;

// Provided by your hardware/HAL

constexpr uint32_t cpu_frequency_hz = CPU_FREQUENCY_HZ;

class CycleScheduler {
private:
    struct Sleeper {
        std::coroutine_handle<> handle = nullptr;
        uint32_t start_time = 0;
        uint32_t delay_cycles = 0;
    };

    static constexpr size_t MAX_TASKS = 4;
    Sleeper sleepers[MAX_TASKS];

public:
    void poll() {
        uint32_t now = get_clock();
        
        for (auto& s : sleepers) {
            if (s.handle != nullptr) {
                if ((now - s.start_time) >= s.delay_cycles) {
                    auto h = s.handle;
                    s.handle = nullptr; // Clear slot
                    h.resume();         // Jump back to coroutine
                }
            }
        }
    }

    // Helper to delay by microseconds
    auto async_delay_us(uint32_t us) {
        struct Awaiter {
            CycleScheduler& sched;
            uint32_t start_time;
            uint32_t cycles_to_wait;
            
            bool await_ready() const { return cycles_to_wait == 0; }
            
            void await_suspend(std::coroutine_handle<> h) {
                for (auto& s : sched.sleepers) {
                    if (s.handle == nullptr) {
                        s.start_time = start_time;
                        s.delay_cycles = cycles_to_wait;
                        s.handle = h;
                        break;
                    }
                }
            }
            void await_resume() {}
        };

        uint32_t cycles = us * (cpu_frequency_hz / 1'000'000);
        
        return Awaiter{ *this, get_clock(), cycles };
    }
};

struct Task {
    struct promise_type {
        Task get_return_object() { 
            return Task{std::coroutine_handle<promise_type>::from_promise(*this)}; 
        }
        std::suspend_never initial_suspend() { return {}; }
        
        // CRITICAL: Suspend always at the end so the handle isn't destroyed 
        // before we can check handle.done() in the main loop.
        std::suspend_always final_suspend() noexcept { return {}; }
        
        void return_void() {}
        void unhandled_exception() {}
    };

    std::coroutine_handle<promise_type> handle;

    // Clean up memory when the Task object goes out of scope
    ~Task() {
        if (handle) handle.destroy();
    }

    // Check if the coroutine has reached final_suspend
    bool is_done() const {
        return !handle || handle.done();
    }
};
