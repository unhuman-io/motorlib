#ifndef UNHUMAN_MOTORLIB_ROUND_ROBIN_LOGGER_H_
#define UNHUMAN_MOTORLIB_ROUND_ROBIN_LOGGER_H_

#include "messages.h"

#define RR_DATA_LENGTH  ROUND_ROBIN_LENGTH
// an indexed pile of data meant to be read in round robin fashion
class RoundRobinLogger {
 public:
    RoundRobinLogger() {
      for (int i=0; i<RR_DATA_LENGTH; i++) {
         data_[i].index = i;
         data_[i].type = FLOAT;
         data_[i].data = 0;
      }
    }
    void get_next_data(RoundRobinData *data) {
       if (data_[index].type == FLAGS32_T) {
          data_[index].data_u32 = 0;
       }
       if (++index_ == RR_DATA_LENGTH) {
         index_ = 0;
       } 
       *data = data_[index_];
    }
    void log_data(uint8_t index, float data) {
      data_[index].data = data;
    }
    void log_data(uint8_t index, uint32_t data) {
      data_[index].data_u32 = data;
      data_[index].type = UINT32_T;
    }
    void log_data(uint8_t index, int32_t data) {
      data_[index].data_i32 = data;
      data_[index].type = INT32_T;
    }
    void log_flags(uint8_t index, uint32_t data) {
      data_[index].data_u32 |= data;
      data_[index].type = FLAGS32_T;
    }
 private:
    uint8_t index_;
    RoundRobinData data_[RR_DATA_LENGTH];
};

extern RoundRobinLogger round_robin_logger;

#endif  // UNHUMAN_MOTORLIB_ROUND_ROBIN_LOGGER_H_
