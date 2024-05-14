#pragma once

#include <cstdint>

class Lock {
    public:
    void wait_lock() {
        unsigned tmp = 0;
        do {
            asm volatile("ldrex     %1, %0\n\t"
                         "add       %1, #1\n\t"
                         "strex     %1, %1, %0" : "+m" (lock_), "=r" (tmp) : "m" (lock_));
        } while (tmp == 0);
    }
    void unlock() {
        unsigned tmp;
        asm volatile("ldrex     %1, %0\n\t"
                     "sub       %1, #1\n\t"
                     "strex     %1, %1, %0" : "+m" (lock_), "=r" (tmp) : "m" (lock_));
    }
    bool try_lock() {
        unsigned tmp;
        asm volatile("ldrex     %1, %0\n\t"
                     "add       %1, #1\n\t"
                     "strex     %1, %1, %0" : "+m" (lock_), "=r" (tmp) : "m" (lock_));
        return tmp == 1;
    }
    bool is_locked() {
        asm volatile("" : "=m" (lock_));
        return lock_ > 0;
    }
    private:
    unsigned lock_ = 0;
};


template <class T>
class SPIDMABase {
 public:
    SPIDMABase(uint32_t baudrate, Lock &lock) : lock_(lock) {
        reinit();
    }

    void reinit() {
        static_cast<T*>(this)->reinit();
    }

    void save_state() {
        static_cast<T*>(this)->save_state();
    }

    void restore_state() {
        static_cast<T*>(this)->restore_state();
    }

    void readwrite(const uint8_t * const data_out, uint8_t * const data_in, uint16_t length) {
        start_readwrite(data_out, data_in, length);
        finish_readwrite();
    }

    // Note, use memory barrier before accesing data_in
    // Example: asm("" : "=m" (*(uint8_t (*)[]) data_in));
    void start_continuous_readwrite(const uint8_t * const data_out, uint8_t * const data_in, uint16_t length) {
        lock_.wait_lock();
        asm("" : : "m" (*(const uint8_t (*)[]) data_out)); // ensure data_out[] is in memory
        static_cast<T*>(this)->start_continuous_readwrite(data_out, data_in, length);
    }

    void start_continuous_write(const uint8_t * const data_out, uint16_t length) {
        lock_.wait_lock();
        asm("" : : "m" (*(const uint8_t (*)[]) data_out)); // ensure data_out[] is in memory
        static_cast<T*>(this)->start_continuous_write(data_out, length);
    }

    void stop_continuous_readwrite() {
        static_cast<T*>(this)->stop_continuous_readwrite();
        lock_.unlock();
    }

    // call only if guaranteed not to be interrupted
    void start_readwrite_isr(const uint8_t * const data_out, uint8_t * const data_in, uint16_t length) {
        if (!lock_.is_locked()) {
            reinit();
            asm("" : : "m" (*(const uint8_t (*)[]) data_out)); // ensure data_out[] is in memory
            static_cast<T*>(this)->start_readwrite(data_out, data_in, length);
            asm("" : "=m" (*(uint8_t (*)[]) data_in));
        }
    }

    void start_readwrite(const uint8_t * const data_out, uint8_t * const data_in, uint16_t length) {
        if (lock_.try_lock()) {
            reinit();
            asm("" : : "m" (*(const uint8_t (*)[]) data_out)); // ensure data_out[] is in memory
            static_cast<T*>(this)->start_readwrite(data_out, data_in, length);
            asm("" : "=m" (*(uint8_t (*)[]) data_in));
        }
    }

    void start_write(const uint8_t * const data_out, uint16_t length) {
        if (lock_.try_lock()) {
            reinit();
            asm("" : : "m" (*(const uint8_t (*)[]) data_out)); // ensure data_out[] is in memory
            static_cast<T*>(this)->start_write(data_out, length);
        }
    }

    void finish_readwrite() {
        if (lock_.is_locked()) {
            static_cast<T*>(this)->finish_readwrite();
            lock_.unlock();
        }
        
    }

    Lock &lock_;

};
