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

    static constexpr size_t MAX_TASKS = 8;
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

using Scheduler = CycleScheduler;

// ============================================================================
// Coroutine Nesting Core
// ============================================================================

// 1. Base promise types to hold values and continuation handles
template <typename T>
struct PromiseReturn {
    T value_;
    std::coroutine_handle<> continuation_ = std::noop_coroutine();
    void return_value(T v) { value_ = v; }
};

template <>
struct PromiseReturn<void> {
    std::coroutine_handle<> continuation_ = std::noop_coroutine();
    void return_void() {}
};

// 2. The Final Awaiter: Transfers control back to parent instantly if one exists
struct FinalAwaiter {
    bool await_ready() const noexcept { return false; }
    
    template <typename PromiseType>
    std::coroutine_handle<> await_suspend(std::coroutine_handle<PromiseType> h) noexcept {
        return h.promise().continuation_; 
    }
    
    void await_resume() noexcept {}
};

// ============================================================================
// Unified Task Template
// ============================================================================
template <typename T = void>
struct [[nodiscard]] Task {
    // Inherit the return logic (void vs T) based on the template parameter
    struct promise_type : public PromiseReturn<T> {
        
        Task get_return_object() {
            return Task{std::coroutine_handle<promise_type>::from_promise(*this)};
        }
        
        std::suspend_never initial_suspend() { return {}; }
        FinalAwaiter final_suspend() noexcept { return {}; } // Hand off to parent
        
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

    T get_result() {
        if constexpr (!std::is_same_v<T, void>) {
            return handle_.promise().value_;
        }
    }

    // --- Enables `co_await child_task()` ---
    auto operator co_await() const noexcept {
        struct TaskAwaiter {
            std::coroutine_handle<promise_type> child_;

            bool await_ready() const noexcept {
                return !child_ || child_.done();
            }

            void await_suspend(std::coroutine_handle<> parent) noexcept {
                // Link the parent so FinalAwaiter knows who to wake up
                child_.promise().continuation_ = parent;
            }

            T await_resume() {
                if constexpr (!std::is_same_v<T, void>) {
                    return child_.promise().value_;
                }
            }
        };

        return TaskAwaiter{handle_};
    }
};

template <typename T = void>
struct [[nodiscard]] GlobalTask : public Task<T> {

    // 1. We must define a promise_type specific to GlobalTask
    struct promise_type : public Task<T>::promise_type {

        static constexpr std::size_t MAX_FRAME_SIZE = 256;
        alignas(std::max_align_t) static inline std::byte global_frame[MAX_FRAME_SIZE];
        static inline bool is_in_use = false;

        // 2. Override get_return_object to return a GlobalTask, not a Task
        GlobalTask get_return_object() {
            return GlobalTask{std::coroutine_handle<promise_type>::from_promise(*this)};
        }

        // 3. Allocator goes HERE, inside the promise_type
        void* operator new(std::size_t size) {
            if (size > MAX_FRAME_SIZE) {
                while(1); // Frame too large
            }
            if (is_in_use) {
                while(1); // Coroutine already active
            }

            is_in_use = true;
            return global_frame;
        }

        void operator delete(void*, std::size_t) {
            is_in_use = false;
        }
    };

    // 4. Constructor safely casts the derived handle to the base handle expected by Task<T>
    explicit GlobalTask(std::coroutine_handle<promise_type> h)
        : Task<T>(std::coroutine_handle<typename Task<T>::promise_type>::from_address(h.address())) {}
};
