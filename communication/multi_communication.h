#pragma once

#include "../communication.h"
#include <tuple>
#include "../logger.h"

// iterate the template communication interaces
template<class... Args>
class MultiCommunication : public CommunicationBase {
 public:
    MultiCommunication(Args&... args) : comms_{args...} {}

    int receive_data(ReceiveData* const data) {
        int retval = std::apply([&data](auto&&... comms) {
            int retval;
            ((retval = comms.receive_data(data), retval) || ...);
            return retval;
        }, comms_);
        return retval;
    }

    void send_data(const SendData& data) {
        std::apply([&data](auto&&... comms) {
            (comms.send_data(data), ...);
        }, comms_);
    }

    bool any_new_rx_data() {
        std::apply([](auto&&... comms) {
            return (comms.any_new_rx_data() || ...);
        }, comms_);
        return false;
    }

    int receive_string(char* const string) {
        int i = -1;
        int retval = std::apply([&string, &i](auto&&... comms) {
            int retval;
            ((i++, retval = comms.receive_string(string), retval) || ...);
            return retval;
        }, comms_);
        if (retval) {
            active_str_comms_ = i;
        }
        return retval;
    }

    bool send_string(const char* string, uint16_t length) {
        int i = 0;
        int active_str_comms = active_str_comms_;
        bool retval = std::apply([&string, &length, &i, &active_str_comms](auto&&... comms) {
            bool retval = false;
            ((active_str_comms == i++ ? retval = comms.send_string(string, length) : 0) || ...);
            return retval;
        }, comms_);
        return retval;
    }

    bool send_string_active() const {
        std::apply([](auto&&... comms) {
            return (comms.send_string_active() || ...);
        }, comms_);
        return false;
    }

    void cancel_send_string() {
        std::apply([](auto&&... comms) {
            (comms.cancel_send_string(), ...);
        }, comms_);
    }

    bool tx_data_ack() {
        std::apply([](auto&&... comms) {
            return (comms.tx_data_ack() || ...);
        }, comms_);
        return false;
    }
 private:
    std::tuple<Args&...> comms_;
    int active_str_comms_ = 0;
};
