#ifndef UNHUMAN_MOTORLIB_ROUND_ROBIN_LOGGER_H_
#define UNHUMAN_MOTORLIB_ROUND_ROBIN_LOGGER_H_

#include "messages.h"

#define RR_DATA_LENGTH  ROUND_ROBIN_LENGTH

// an indexed pile of data meant to be read in round robin fashion
class RoundRobinLogger {
 public:
    RoundRobinLogger() {};
    void zero_data(uint8_t index, uint8_t length) {
      for (int i=0; i<length; i++) {
         data_index_[index]->data_[i].index = index;
         data_index_[index]->data_[i].subindex = i;
         data_index_[index]->data_[i].type = FLOAT;
         data_index_[index]->data_[i].data = 0;
      }
    }
    void add_index(uint8_t index, uint8_t length) {
      if (index >= rr_max_index_) {
        rr_max_index_ = index;
      }
      data_index_[index]->data_ = new RoundRobinData[length];
      data_index_[index]->length = length;
      zero_data(index, length);
    }
    void get_next_data(RoundRobinData *data) {
      do {
       if (++subindex_ == data_index_[index_]->length) {
          subindex_ = 0;
          if (++index_ == rr_max_index_) {
            index_ = 0;
          }
       }
      } while (data_index_[index_]->length == 0); // skip empty indexes
      *data = data_index_[index_]->data_[subindex_];
    }
    void log_data(uint8_t index, uint8_t subindex, float data) {
      data_index_[index]->data_[subindex].data = data;
    }
    void log_data(uint8_t index, uint8_t subindex, uint32_t data) {
      data_index_[index]->data_[subindex].data_u32 = data;
      data_index_[index]->data_[subindex].type = UINT32_T;
    }
    void log_data(uint8_t index, uint8_t subindex, int32_t data) {
      data_index_[index]->data_[subindex].data_i32 = data;
      data_index_[index]->data_[subindex].type = INT32_T;
    }
 private:
    uint8_t index_;
    uint8_t subindex_;
    uint8_t rr_max_index_ = 0;
    struct RoundRobinIndex {
      RoundRobinData *data_;
      uint8_t length;
    };
    RoundRobinIndex *data_index_[RR_DATA_LENGTH] = {};
};

extern RoundRobinLogger round_robin_logger;

#endif  // UNHUMAN_MOTORLIB_ROUND_ROBIN_LOGGER_H_
