#pragma once

#include <cstdint>
#include <cstddef>

class Comms
{
  public:
    struct BufferDescriptor
    {
      const uint8_t* txBuffer;
      uint8_t* rxBuffer;
      size_t   length;
    };

    typedef void (*transactionCompleteCallback)(void* param);

    virtual void init() = 0;
    virtual void reset() = 0;
    virtual void startTransaction(BufferDescriptor descriptor) = 0;
    virtual void setTransactionCompletedCallback(transactionCompleteCallback callback, void* param) = 0;

    virtual void resetTransactionCounter() = 0;
    virtual uint8_t getTransactionCounter() = 0;
};
