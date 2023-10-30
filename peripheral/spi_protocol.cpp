#include "spi_protocol.h"
#include <cstring>

void SpiProtocol::transactionCompleteCallback(void* param)
{
  FIGURE_ASSERT(param != NULL, "Invalid arg");
  SpiProtocol* spi_protocol = (SpiProtocol*)param;

  spi_protocol->processState();
}

void SpiProtocol::processState()
{
  State previous_state;

  // This state machine is entirely interrupt driven.
  // That means that it absolutely has to start a SPI transaction
  // every time it goes through the states or the entire thing
  // will freeze.
  // The transaction counter helps to keep track of whether a transaction has
  // been started or not.
  spi_slave_.resetTransactionCounter();

  // Keep cranking the state machine until the state stops changing.
  do
  {
    previous_state = state_;

    FIGURE_ASSERT(state_ < kStateCount, "Invalid state");

#if SPI_PROTOCOL_TRACE==1
    if(state_ != state_trace_buffer_[0].state)
    {
      // Shift trace buffer by 1, place current state in the first element
      for(size_t i = 1; i < FIGURE_COUNTOF(state_trace_buffer_); i++)
      {
        state_trace_buffer_[i] = state_trace_buffer_[i - 1];
      }
      state_trace_buffer_[0].state = state_;
      state_trace_buffer_[0].cycles = 1;
    }
    else
    {
      state_trace_buffer_[0].cycles++;
    }
#endif

    // Find a handler for the current state in the state_table and run it
    if(state_table_[state_].stateHandler != NULL)
    {
      state_ = (this->*state_table_[state_].stateHandler)(first_run_);
      first_run_ = (state_ != previous_state);
    }
  }
  while(state_ != previous_state);

  // After completing one iteration of the state machine, exactly 1 SPI transaction
  // should have been started.
  FIGURE_ASSERT(spi_slave_.getTransactionCounter() == 1U, "Logic error");
}

void SpiProtocol::init()
{
  spi_slave_.setTransactionCompletedCallback(transactionCompleteCallback, this);

  // Fire up the state machine
  processState();
}

