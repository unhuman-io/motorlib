#include "mailbox.h"

#include <cstring>

Mailbox::Mailbox(Pool* pools, size_t pool_count) :
      sequence_(0),
      mailbox_pool_table_{NULL}
{
  for(size_t pool_index = 0; pool_index < pool_count; pool_index++)
  {
    auto& pool = pools[pool_index];

    for(size_t mailbox_id_index = 0; mailbox_id_index < FIGURE_COUNTOF(pool.mailboxIds); mailbox_id_index++)
    {
      auto mailbox_id = pool.mailboxIds[mailbox_id_index];

      if(mailbox_id != 0)
      {
        FIGURE_ASSERT(mailbox_id < kMaxMailboxId); // Invalid mailbox ID
        mailbox_pool_table_[mailbox_id] = &pool;
      }
    }
  }
}

void Mailbox::write(uint8_t mailbox_id, const uint8_t* buffer, size_t length)
{
  FIGURE_ASSERT(buffer != NULL); // Invalid arg
  FIGURE_ASSERT(length <= kBufferSize); // Invalid arg

  size_t i;
  size_t oldest_sequence = 0xFFFFFFFFU;
  Buffer* oldest_buffer = NULL;
  Buffer* pool_buffer;

  if(mailbox_id < kMaxMailboxId)
  {
    auto pool = mailbox_pool_table_[mailbox_id];

    // If mailbox is supported
    if(pool != NULL)
    {
      for(i = 0; i < pool->buffersCount; i++)
      {
        pool_buffer = &pool->buffers[i];

        // TODO: Testing/setting of the "busy" flag needs to be inside of a critical section
        if(!pool_buffer->busy)
        {
          // If buffer is free - go ahead and grab it
          if(!pool_buffer->hasData)
          {
            break;
          }
          // otherwise find the oldest buffer with the same mailbox Id
          else if(pool_buffer->mailboxId == mailbox_id)
          {
            if(pool_buffer->sequence < oldest_sequence)
            {
              oldest_sequence = pool_buffer->sequence;
              oldest_buffer = pool_buffer;
            }
          }
        }
      }

      // If a free buffer has been found
      if(i < pool->buffersCount)
      {
        pool_buffer = &pool->buffers[i];
      }
      else if(oldest_buffer != NULL) // If there are no free buffers - overwrite the oldest one
      {
        pool_buffer = oldest_buffer;
      }
      else // If there is no buffer to overwrite - bail out
      {
        pool_buffer = NULL;
      }

      if(pool_buffer != NULL)
      {
        // Lock the buffer
        pool_buffer->busy = true;

        // Copy data
        memcpy((void*)pool_buffer->buffer, buffer, length);
        pool_buffer->length = length;
        pool_buffer->mailboxId = mailbox_id;
        pool_buffer->sequence = sequence_++;
        pool_buffer->hasData = true;

        // Unlock the buffer
        pool_buffer->busy = false;
      }
    }
  }
}

size_t Mailbox::read(uint8_t mailbox_id, uint8_t* buffer, size_t buffer_size)
{
  FIGURE_ASSERT(buffer != NULL); // Invalid arg

  size_t i;
  size_t length = 0;

  size_t oldest_sequence = 0xFFFFFFFFU;
  Buffer* oldest_buffer = NULL;
  Buffer* pool_buffer;

  if(mailbox_id < kMaxMailboxId)
  {
    auto pool = mailbox_pool_table_[mailbox_id];

    // If mailbox is supported
    if(pool != NULL)
    {
      // Find a buffer with a matching Id and the oldest sequence number.
      // TODO: Testing/setting of the "busy" flag needs to be inside of a critical section
      for(i = 0; i < pool->buffersCount; i++)
      {
        pool_buffer = &pool->buffers[i];

        if(  (!pool_buffer->busy)
           &&(pool_buffer->hasData)
           &&(pool_buffer->mailboxId == mailbox_id) )
        {
          if(pool_buffer->sequence < oldest_sequence)
          {
            oldest_sequence = pool_buffer->sequence;
            oldest_buffer = pool_buffer;
          }
        }
      }

      // If a buffer has been found
      if(oldest_buffer != NULL)
      {
        // Lock the buffer
        oldest_buffer->busy = true;

        length = (oldest_buffer->length <= buffer_size) ? oldest_buffer->length : buffer_size;

        // Copy data
        memcpy(buffer, (void*)oldest_buffer->buffer, length);

        oldest_buffer->hasData = false;

        // Unlock the buffer
        oldest_buffer->busy = false;
      }
    }
  }

  return length;
}
