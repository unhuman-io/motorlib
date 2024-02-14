#pragma once

#include <cstdint>
#include <cstddef>

class Comms
{
  public:
    struct BufferDescriptor
    {
      const volatile uint8_t* txBuffer;
      volatile uint8_t*       rxBuffer;
      size_t                  length;
    };

    typedef void (*commsCallback)(void* param);

    virtual void init() = 0;
    virtual void reset() = 0;
    virtual void startTransaction(BufferDescriptor descriptor) = 0;
    virtual void abortTransaction() = 0;
    virtual void setTransactionCompletedCallback(commsCallback callback, void* param) = 0;
    virtual void setErrorCallback(commsCallback callback, void* param) = 0;

    virtual void resetTransactionCounter() = 0;
    virtual uint8_t getTransactionCounter() = 0;
};
