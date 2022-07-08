#ifndef USB_H
#define USB_H

#include <cstdint>

// signal to exit
extern uint32_t go_to_bootloader;
struct usb_control_request;
class USB1 {
 public:
    USB1();
    void connect();
    // limited to 64 bytes
    void send_data(uint8_t endpoint, const uint8_t * const data, uint16_t length, bool wait = true, uint32_t wait_timeout_us = 10000);

    // receive up to length bytes from endpoint, return number of bytes read
    int receive_data(uint8_t endpoint, uint8_t * const data, uint8_t length);

    void send_string(uint8_t endpoint, const char * const str, uint8_t length);

    bool tx_active(uint8_t endpoint);

    void interrupt();

    bool new_rx_data(uint8_t endpoint) const { return new_rx_data_[endpoint]; }

    void cancel_transfer(uint8_t endpoint);

 private:
     // send tx stall
    void send_stall(uint8_t endpoint);

    void handle_setup_packet(usb_control_request *setup_data);

    uint8_t device_address_ = 0;
    uint16_t interface_ = 0;
    volatile bool new_rx_data_[3] = {};
    uint8_t count_rx_[3] = {};
    uint8_t rx_buffer_[3][64] = {};
    uint32_t error_count_ = 0;
    friend class System;
};

#endif
