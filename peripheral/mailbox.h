#pragma once

#include "macro.h"
#include <stdint.h>
#include <stddef.h>

class Mailbox
{
  public:
    static const size_t kBufferSize = 64U;
    static const size_t kMaxMailboxId = 8U;

    typedef volatile struct
    {
      uint32_t  sequence;
      uint8_t   mailboxId;
      uint8_t   buffer[kBufferSize];
      uint8_t   length;
      bool      hasData;
      bool      busy;
    }Buffer;

    struct Pool
    {
      uint8_t mailboxIds[4U];
      Buffer* buffers;
      size_t  buffersCount;
    };

    Mailbox(Pool* pools, size_t pool_count);
    ~Mailbox(){};

    void write(uint8_t mailbox_id, const uint8_t* buffer, size_t length);
    size_t read(uint8_t mailbox_id, uint8_t* buffer, size_t buffer_size);

  private:
    uint32_t  sequence_;
    Pool*     mailbox_pool_table_[kMaxMailboxId];
};
