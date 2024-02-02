/**
 * Copyright 2024 Figure AI, Inc
 */

#include "firmware/lib/obot-parser/obot-parser.h"

#include <cstring>
#include <iostream>

#include "firmware/lib/crc/crc.h"

namespace figure {

void ObotParser::process(const uint8_t latest_idx) {
  tail_ = (latest_idx + 1) % buffer_size_;
  if (head_ == tail_) {
    // No new data to process
    return;
  }

  // Calculate number of bytes to process
  int bytes_to_process = 0;
  if (latest_idx >= head_) {
    bytes_to_process = latest_idx - head_ + 1;
  } else {
    // Tail has wrapped around to the beginning of the buffer
    bytes_to_process = buffer_size_ - head_ + latest_idx + 1;
  }
  // Copy the buffer into a local buffer so the packet is contiguous in memory
  // This is necessary for the crc16 function
  if (latest_idx >= head_) {
    std::memcpy(packet_buffer_, buffer_ + head_, bytes_to_process);
  } else {
    // Data wraps around. Copy the data in two parts.
    std::memcpy(packet_buffer_, buffer_ + head_, buffer_size_ - head_);
    std::memcpy(packet_buffer_ + buffer_size_ - head_, buffer_,
                bytes_to_process - buffer_size_ - head_);
  }
  int start_index = 0;
  int end_index = bytes_to_process;  // One past the last element in the local buffer
  std::cout << "in process. head_ = " << head_ << ", tail_ = " << tail_
            << ", start_index = " << start_index << std::endl;

  // Keep searching through local buffer until we've reached the end.
  while (bytes_to_process > 0) {
    // Loop through packet_buffer_ beginning at start_index and find the start of a packet
    for (int i = start_index; i < end_index; i++) {
      if (i == end_index - 1) {
        // Reached the end of the buffer without finding a start byte.
        // Discard any bytes that have been processed by setting the head_ to tail_.
        head_ = tail_;
        return;
      }
      if (packet_buffer_[i] == kStartByte1) {
        std::cout << "Found start byte: i=" << i << std::endl;
        // Check that the buffer contains enough bytes to check up to the payload length
        if (i + 3 > end_index) {
          // Buffer does not contain enough bytes to read the payload length.
          // Update the head_ and return.
          head_ = start_index;
          std::cout << "returning: 1" << std::endl;
          return;
        }

        // Check for second start byte
        if (packet_buffer_[i + 1] == kStartByte2) {
          // Found the start of a packet
          // Parse frame ID
          uint8_t frame_id = packet_buffer_[i + 2];

          // Validate frame ID
          if (frame_id >= kMaxFrameIds) {
            // Invalid frame ID
            // TODO(kyle-figure): Add this
            // start_index = i;
            // head_ = i;
            // bytes_to_process -= 3;
            // std::cout << "break: 1" << std::endl;
            // TODO(kyle-figure): Track this error
            break;
          }

          // Parse payload length
          int payload_length = packet_buffer_[i + 3];

          // Check if the buffer contains the entire packet
          if ((i + payload_length + 6) - start_index > bytes_to_process) {
            // Buffer does not contain the entire packet
            std::cout << "returning: 3" << std::endl;
            return;
          }

          int start_of_payload = i + 4;
          int end_of_payload = i + payload_length + 4;  // One past the last byte of the payload

          // Parse the crc
          uint16_t crc = (buffer_[end_of_payload] << 8) | buffer_[end_of_payload + 1];

          // Check CRC
          if (crc16(&packet_buffer_[i], payload_length + 4) != crc) {
            // CRC is incorrect
            // TODO(kyle-figure): Track this error
            // Update the head_ and start searching for the next packet
            head_ = (latest_idx + i + 1) % buffer_size_;
            start_index = i + payload_length + 6;
            bytes_to_process = end_index - start_index;
            std::cout << "Calculated CRC: " << crc16(&packet_buffer_[i], payload_length + 6)
                      << std::endl;
            std::cout << "Expected CRC: " << crc << std::endl;
            std::cout << "break: 2" << std::endl;
            break;
          }

          // Call the callback for the packet
          if (callbacks_[frame_id] != nullptr) {
            callbacks_[frame_id](&packet_buffer_[start_of_payload], payload_length);
          } else {
            // No callback registered for the packet
            // TODO(kyle-figure): Track this error
          }

          // Update the head_ index
          head_ = (head_ + payload_length + 6) % buffer_size_;
          start_index = i + payload_length + 6;
          bytes_to_process = end_index - start_index;
          break;
        }
      }
    }
  }
}

void ObotParser::registerCallback(uint8_t frame_id, callback_t&& callback) {
  if (frame_id >= kMaxFrameIds) {
    // Invalid frame ID
    // TODO(kyle-figure): Propagate this error
    return;
  }
  callbacks_[frame_id] = callback;
}

}  // namespace figure
