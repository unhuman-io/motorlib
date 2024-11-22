#pragma once

#include <cstdint>
#include <functional>

// shared class between SPIx instances. Provides a signal for a SPIx user to pause all SPIx instances
// start and stop callbacks are provided by the user and generally involve mux config etc.
class SPIPause {
 public:
    void pause() {
        lock_++;
        if (lock_ == 1) {
            stop_callback_();
        }
    }
    void unpause() {
        lock_--;
        if (lock_ == 0) {
            start_callback_();
        }
    }
    bool is_paused() {
        return lock_ > 0;
    }
    std::function<void()> stop_callback_ = []{};
    std::function<void()> start_callback_ = []{};
 private:
    volatile unsigned lock_ = 0;
    
};


template <class T>
class SPIDMABase {
 public:
    SPIDMABase(uint32_t baudrate, SPIPause &pause) : pause_(pause) {}

    void reinit() {
        static_cast<T*>(this)->reinit_impl();
    }

    // main() context only
    void readwrite(const uint8_t * const data_out, uint8_t * const data_in, uint16_t length) {
        claim();
        reinit();
        asm("" : : "m" (*(const uint8_t (*)[]) data_out)); // ensure data_out[] is in memory
        static_cast<T*>(this)->start_readwrite_impl(data_out, data_in, length);
        static_cast<T*>(this)->finish_readwrite_impl();
        asm("" : "=m" (*(uint8_t (*)[]) data_in));
        release();
    }

    // Note, use memory barrier before accesing data_in
    // Example: asm("" : "=m" (*(uint8_t (*)[]) data_in));
    void start_continuous_readwrite(const uint8_t * const data_out, uint8_t * const data_in, uint16_t length) {
        asm("" : : "m" (*(const uint8_t (*)[]) data_out)); // ensure data_out[] is in memory
        static_cast<T*>(this)->start_continuous_readwrite_impl(data_out, data_in, length);
    }

    void start_continuous_write(const uint8_t * const data_out, uint16_t length) {
        asm("" : : "m" (*(const uint8_t (*)[]) data_out)); // ensure data_out[] is in memory
        static_cast<T*>(this)->start_continuous_write_impl(data_out, length);
    }

    void stop_continuous_readwrite() {
        static_cast<T*>(this)->stop_continuous_readwrite_impl();
    }

    // ISR use only
    void start_readwrite_isr(const uint8_t * const data_out, uint8_t * const data_in, uint16_t length) {
        if (!pause_.is_paused()) {
            reinit();
            asm("" : : "m" (*(const uint8_t (*)[]) data_out)); // ensure data_out[] is in memory
            static_cast<T*>(this)->start_readwrite_impl(data_out, data_in, length);
            asm("" : "=m" (*(uint8_t (*)[]) data_in));
        }
    }

    // ISR use only
    void start_write_isr(const uint8_t * const data_out, uint16_t length) {
        if (!pause_.is_paused()) {
            reinit();
            asm("" : : "m" (*(const uint8_t (*)[]) data_out)); // ensure data_out[] is in memory
            static_cast<T*>(this)->start_write_impl(data_out, length);
        }
    }

    // ISR use only
    void finish_readwrite_isr() {
        if (!pause_.is_paused()) {
            static_cast<T*>(this)->finish_readwrite_impl();
        }
    }

    void claim() {
        pause_.pause();
    }

    void release() {
        pause_.unpause();
    }

    SPIPause &pause_;
};
