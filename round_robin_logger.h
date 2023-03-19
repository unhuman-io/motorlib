#ifndef UNHUMAN_MOTORLIB_ROUND_ROBIN_LOGGER_H_
#define UNHUMAN_MOTORLIB_ROUND_ROBIN_LOGGER_H_

#include "messages.h"

#define RR_DATA_LENGTH  20
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
       if (++index_ == RR_DATA_LENGTH) {
         index_ = 0;
       } 
       *data = data_[index_];
    }
    void log_data(uint8_t index, float data) {
      data_[index].data = data;
    }
 private:
    uint8_t index_;
    RoundRobinData data_[RR_DATA_LENGTH];
};

extern RoundRobinLogger round_robin_logger;

#endif  // UNHUMAN_MOTORLIB_ROUND_ROBIN_LOGGER_H_
