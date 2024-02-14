#include "protocol.h"
#include "../util.h"
#include <cstring>

#pragma GCC push_options
#pragma GCC optimize ("O0")
// Global pointer for easy debugging access to the Protocol structure
Protocol* protocol_instance = NULL;
#pragma GCC pop_options

Protocol::Protocol(Comms& comms, Mode mode_, Mailbox::Pool* mailbox_pools, size_t mailbox_pools_count) :
  mailboxes(mailbox_pools, mailbox_pools_count),
  comms_(comms),
  state_(kStateWaitForSync),
  first_run_(true),
  send_nack_(false),
  state_after_ack_(kStateWaitForCommand),
  current_command_(Command::kSync),
  state_table_(NULL),
  tx_protocol_buffer_{0},
  rx_protocol_buffer_{0}
{
  protocol_instance = this;

  switch(mode_)
  {
    case Mode::kSpi:
      send_dummy_byte_ = true;
      state_table_ = state_table_spi_;
      break;

    case Mode::kUart:
      send_dummy_byte_ = false;
      state_table_ = state_table_uart_;
      break;

    default:
      FIGURE_ASSERT(false); // Unsupported mode
  }
};

void Protocol::transactionCompleteCallback(void* param)
{
  FIGURE_ASSERT(param != NULL, "Invalid arg");
  Protocol* protocol = (Protocol*)param;

  protocol->processState();
}

void Protocol::errorCallback(void* param)
{
  FIGURE_ASSERT(param != NULL, "Invalid arg");
  Protocol* protocol = (Protocol*)param;

  // Reset state
  protocol->state_ = kStateWaitForCommand;
  protocol->first_run_ = true;
  protocol->send_nack_ = false;
  protocol->state_after_ack_ = kStateWaitForCommand;
  protocol->current_command_ = Command::kSync;

  protocol->processState();
}

void Protocol::processState()
{
  State previous_state;

  // This state machine is entirely interrupt driven.
  // That means that it absolutely has to start a SPI transaction
  // every time it goes through the states or the entire thing
  // will freeze.
  // The transaction counter helps to keep track of whether a transaction has
  // been started or not.
  comms_.resetTransactionCounter();

  // Keep cranking the state machine until the state stops changing.
  do
  {
    previous_state = state_;

    FIGURE_ASSERT(state_ < kStateCount, "Invalid state");

#if PROTOCOL_TRACE==1
    if(state_ != state_trace_buffer_[0].state)
    {
      // Shift trace buffer by 1, place current state in the first element
      for(size_t i = FIGURE_COUNTOF(state_trace_buffer_) - 1; i > 0; i--)
      {
        state_trace_buffer_[i] = state_trace_buffer_[i - 1];
      }
      state_trace_buffer_[0].state = state_;
      state_trace_buffer_[0].cycles = 1;
      state_trace_buffer_[0].timestamp = get_clock();
    }
    else
    {
      if(state_trace_buffer_[0].cycles < 0xFF)
      {
        state_trace_buffer_[0].cycles++;
      }
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
  FIGURE_ASSERT(comms_.getTransactionCounter() == 1U, "Logic error");
}

void Protocol::init()
{
  DEBUG_PINS_INIT();

  comms_.setTransactionCompletedCallback(transactionCompleteCallback, this);
  comms_.setErrorCallback(errorCallback, this);

  // Fire up the state machine
  processState();
}

