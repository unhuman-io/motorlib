#include <protocol.h>


const Protocol::StateTableEntry Protocol::state_table_uart_[kStateCount] =
{
  [kStateInit           ] = {NULL                                      },
  [kStateWaitForSync    ] = {&Protocol::stateWaitForSyncHandlerUart    },
  [kStateWaitForAck     ] = {&Protocol::stateWaitForAckHandlerUart     },
  [kStateSendAckNack    ] = {&Protocol::stateSendAckNackHandlerUart    },
  [kStateWaitForCommand ] = {&Protocol::stateWaitForCommandHandlerUart },
  [kStateRunCommand     ] = {&Protocol::stateRunCommandHandlerUart     },
};

Protocol::State Protocol::stateWaitForSyncHandlerUart(bool first_run)
{
  State new_state = kStateWaitForSync;

  // Reset communication interface
  comms_.reset();

  if(  (!first_run)
     &&(rx_protocol_buffer_[0] == (uint8_t)Command::kStart) )
  {
    new_state = sendAckNack(false, kStateWaitForCommand);
  }
  else
  {
    // Read 1 byte from Uart looking for Start byte
    comms_.startTransaction({
      .txBuffer = NULL,
      .rxBuffer = rx_protocol_buffer_,
      .length = 1
    });
  }

  return new_state;
}

Protocol::State Protocol::stateSendAckNackHandlerUart(bool first_run)
{
  static uint8_t response_ack[1] = {(uint8_t)Command::kAck};
  static uint8_t response_nack[1] = {(uint8_t)Command::kNack};

  State new_state = kStateSendAckNack;

  if(first_run)
  {
    // Send Ack/Nack
    comms_.startTransaction({
      .txBuffer = (send_nack_) ? response_nack : response_ack,
      .rxBuffer = NULL,
      .length = 1
    });
  }
  else
  {
    new_state = state_after_ack_;
  }

  return new_state;
}

Protocol::State Protocol::stateWaitForAckHandlerUart(bool first_run)
{
  FIGURE_ASSERT(false); // This state is not used in UART mode

  return kStateWaitForAck;
}

Protocol::State Protocol::stateWaitForCommandHandlerUart(bool first_run)
{
  State new_state = kStateWaitForCommand;
  size_t i;

  if(first_run)
  {
    // Clear the buffer
    bytes_read_ = 0;
    rx_protocol_buffer_[0] = 0xAA;
    rx_protocol_buffer_[1] = 0xAA;

    // Receive bytes one by one in case if there is a loss of synchronization
    comms_.startTransaction({
      .txBuffer = NULL,
      .rxBuffer = &rx_protocol_buffer_[1],
      .length = 1
    });
  }
  else
  {
    bool command_found = false;
    bytes_read_++;

    if(bytes_read_ >= 2U) // If there are at least 2 bytes - attempt to parse the command
    {
      if((rx_protocol_buffer_[0] ^ rx_protocol_buffer_[1]) == 0xFF)
      {
        current_command_ = (Command)rx_protocol_buffer_[0];

        // Check if the command is in the command_table
        for(i = 0; i < FIGURE_COUNTOF(command_table_); i++)
        {
          if(command_table_[i].command == current_command_)
          {
            command_found = true;

            // Call the init method of the command
            if(command_table_[i].init != NULL)
            {
              (this->*command_table_[i].init)();
            }

            // Send ACK, transition into kRunCommand state
            new_state = sendAckNack(false, kStateRunCommand);
            break;
          }
        }
//
//        // If command has not been found - send Nack, go back to kWaitForCommand
//        if(i >= FIGURE_COUNTOF(command_table_))
//        {
//          new_state = sendAckNack(true, kStateWaitForCommand);
//        }
      }
//      else
//      {
//        new_state = sendAckNack(true, kStateWaitForCommand);
//      }
    }

    if(!command_found)
    {
      // If there is still no reasonable command after 3 bytes - error out
      if(bytes_read_ >= 3)
      {
        new_state = sendAckNack(true, kStateWaitForCommand);
      }
      else
      {
        // Shift the data inside the buffer
        rx_protocol_buffer_[0] = rx_protocol_buffer_[1];

        // Receive another byte
        comms_.startTransaction({
          .txBuffer = NULL,
          .rxBuffer = &rx_protocol_buffer_[1],
          .length = 1
        });
      }
    }
  }

  return new_state;
}

Protocol::State Protocol::stateRunCommandHandlerUart(bool first_run)
{
  State new_state = kStateRunCommand;
  size_t i;

  // Iterate through the command table, look for a command
  for(i = 0; i < FIGURE_COUNTOF(command_table_); i++)
  {
    if(command_table_[i].command == current_command_)
    {
      if(command_table_[i].stateHandler != NULL)
      {
        // Call the command state handler
        new_state = (this->*command_table_[i].stateHandler)(first_run);
        break;
      }
    }
  }

  // Command should be guaranteed to be in the table at this point
  FIGURE_ASSERT(i < FIGURE_COUNTOF(command_table_), "Logic error");

  return new_state;
}
