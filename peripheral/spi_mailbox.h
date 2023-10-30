#pragma once

#include "spi_slave.h"
#include <stdint.h>
#include <stddef.h>

class SpiMailbox
{
  public:
    static const size_t kBufferSize = 64U;
    static const size_t kMaxMailboxId = 8U;

    struct Buffer
    {
      uint32_t  sequence;
      uint8_t   mailboxId;
      uint8_t   buffer[kBufferSize];
      uint8_t   length;
      bool      hasData;
      bool      busy;
    };

    struct Pool
    {
      uint8_t mailboxIds[4U];
      Buffer* buffers;
      size_t  buffersCount;
    };

    SpiMailbox(Pool* pools, size_t pool_count);
    ~SpiMailbox(){};

    void write(uint8_t mailbox_id, const uint8_t* buffer, size_t length);
    size_t read(uint8_t mailbox_id, uint8_t* buffer, size_t buffer_size);

  private:
    uint32_t  sequence_;
    Pool*     mailbox_pool_table_[kMaxMailboxId];
};
