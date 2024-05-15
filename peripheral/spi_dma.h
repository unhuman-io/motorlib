#pragma once

#include <cstdint>
#include <functional>

// shared class between SPIx instances. Provides a signal for a SPIx user to pause all SPIx instances
// start and stop callbacks are provided by the user and generally involve mux config etc.
class SPIPause {
 public:
    void pause() {
        lock_++;
        stop_callback_();
    }
    void unpause() {
        lock_--;
        start_callback_();
    }
    bool is_paused() {
        return lock_ > 0;
    }
    std::function<void()> stop_callback_ = []{};
    std::function<void()> start_callback_ = []{};
 private:
    unsigned lock_ = 0;
    
};


template <class T>
class SPIDMABase {
 public:
    SPIDMABase(uint32_t baudrate, SPIPause &pause) : pause_(pause) {
        reinit();
    }

    void reinit() {
        static_cast<T*>(this)->reinit_impl();
    }

    void readwrite(const uint8_t * const data_out, uint8_t * const data_in, uint16_t length) {
        start_readwrite(data_out, data_in, length);
        finish_readwrite();
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

    void start_readwrite(const uint8_t * const data_out, uint8_t * const data_in, uint16_t length) {
        if (!pause_.is_paused() || claimed_) {
            reinit();
            asm("" : : "m" (*(const uint8_t (*)[]) data_out)); // ensure data_out[] is in memory
            static_cast<T*>(this)->start_readwrite_impl(data_out, data_in, length);
            asm("" : "=m" (*(uint8_t (*)[]) data_in));
        }
    }

    void start_write(const uint8_t * const data_out, uint16_t length) {
        if (!pause_.is_paused() || claimed_) {
            reinit();
            asm("" : : "m" (*(const uint8_t (*)[]) data_out)); // ensure data_out[] is in memory
            static_cast<T*>(this)->start_write_impl(data_out, length);
        }
    }

    void finish_readwrite() {
        if (!pause_.is_paused()) {
            static_cast<T*>(this)->finish_readwrite_impl();
        }
    }

    void claim() {
        pause_.pause();
        claimed_ = true;
    }

    void release() {
        claimed_ = false;
        pause_.unpause();
    }

    SPIPause &pause_;
    bool claimed_ = false;
};
