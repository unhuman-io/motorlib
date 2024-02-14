#include <protocol.h>


const Protocol::CommandTableEntry Protocol::command_table_[3U] =
{
  {Command::kGet          , &Protocol::getCommandInit          , &Protocol::getCommandStateHandler          },
  {Command::kReadMailbox  , &Protocol::readMailboxCommandInit  , &Protocol::readMailboxCommandStateHandler  },
  {Command::kWriteMailbox , &Protocol::writeMailboxCommandInit , &Protocol::writeMailboxCommandStateHandler }
};


Protocol::State Protocol::sendAckNack(bool send_nack, State state_after_ack)
{
  send_nack_ = send_nack;
  state_after_ack_ = state_after_ack;
  return kStateSendAckNack;
}

void Protocol::getCommandInit()
{
  size_t total_length = 0;

  if(send_dummy_byte_)
  {
    tx_protocol_buffer_[total_length++] = 0x00; // Dummy byte
  }

  tx_protocol_buffer_[total_length++] = 0x00; // Reserved for the length byte
  tx_protocol_buffer_[total_length++] = 0x81; // Protocol version

  // Write all the supported command IDs from the command_table_
  for(size_t i = 0; i < FIGURE_COUNTOF(command_table_); i++)
  {
    tx_protocol_buffer_[total_length++] = (uint8_t)command_table_[i].command;
  }

  // Insert the length byte
  if(send_dummy_byte_)
  {
    tx_protocol_buffer_[1] = total_length - 3; // Number of bytes - 1 excluding dummy and length
  }
  else
  {
    tx_protocol_buffer_[0] = total_length - 2; // Number of bytes - 1 excluding length
  }


  get_command_context_.totalLength = total_length;
}

Protocol::State Protocol::getCommandStateHandler(bool first_run)
{
  State new_state = kStateRunCommand;

  if(first_run)
  {
    comms_.startTransaction({
      .txBuffer = tx_protocol_buffer_,
      .rxBuffer = NULL,
      .length = get_command_context_.totalLength
    });
  }
  else
  {
    new_state = sendAckNack(false, kStateWaitForCommand);
  }

  return new_state;
}

void Protocol::readMailboxCommandInit()
{
  mailbox_command_context_ = {
    .state      = mailbox_command_context_.State::kReadId,
    .mailboxId  = 0,
    .length     = 0,
    .dataSent   = false
  };
}

Protocol::State Protocol::readMailboxCommandStateHandler(bool first_run)
{
  State new_state = kStateRunCommand;
  auto& context = mailbox_command_context_;

  switch(context.state)
  {
    case context.State::kReadIdAndLength:
    case context.State::kReadData:
      FIGURE_ASSERT(false, "Invalid state");
      break;

    case context.State::kReadId:
      if(first_run)
      {
        comms_.startTransaction({
          .txBuffer = NULL,
          .rxBuffer = rx_protocol_buffer_,
          .length = 2 // mbox_id + checksum
          });
      }
      else
      {
        context.mailboxId = rx_protocol_buffer_[0];

        context.length = mailboxes.read(
          context.mailboxId,
          (uint8_t*)tx_buffer_,
          sizeof(tx_buffer_)
        );

        if(context.length > 0)
        {
          context.state = context.State::kWriteLengthAndData;
          new_state = sendAckNack(false, kStateRunCommand);
        }
        else
        {
          new_state = sendAckNack(true, kStateWaitForCommand);
        }
      }
      break;

    case context.State::kWriteLengthAndData:
      if(first_run)
      {
        if(send_dummy_byte_)
        {
          tx_protocol_buffer_[0] = 0x00; // Dummy byte
          tx_protocol_buffer_[1] = context.length - 1; // Length

          comms_.startTransaction({
            .txBuffer = tx_protocol_buffer_,
            .rxBuffer = NULL,
            .length = 2 // dummy byte + length
          });
        }
        else // No dummy byte
        {
          tx_protocol_buffer_[0] = context.length - 1; // Length

          comms_.startTransaction({
            .txBuffer = tx_protocol_buffer_,
            .rxBuffer = NULL,
            .length = 1 // dummy byte + length
          });
        }
      }
      else
      {
        if(!context.dataSent)
        {
          comms_.startTransaction({
            .txBuffer = tx_buffer_,
            .rxBuffer = NULL,
            .length = context.length + 1 // extra byte for the checksum
          });
          context.dataSent = true;
        }
        else
        {
          new_state = sendAckNack(false, kStateWaitForCommand);
        }
      }
      break;
  }

  return new_state;
}

void Protocol::writeMailboxCommandInit()
{
  mailbox_command_context_ = {
    .state      = mailbox_command_context_.State::kReadIdAndLength,
    .mailboxId  = 0,
    .length     = 0,
    .dataSent   = false
  };
}

Protocol::State Protocol::writeMailboxCommandStateHandler(bool first_run)
{
  State new_state = kStateRunCommand;
  auto& context = mailbox_command_context_;

  switch(context.state)
  {
    case context.State::kReadId:
    case context.State::kWriteLengthAndData:
      FIGURE_ASSERT(false, "Invalid state");
      break;

    case context.State::kReadIdAndLength:
      if(first_run)
      {
        comms_.startTransaction({
          .txBuffer = NULL,
          .rxBuffer = rx_protocol_buffer_,
          .length = 3 // mbox_id + length + checksum
          });
      }
      else
      {
        context.mailboxId = rx_protocol_buffer_[0];
        context.length = rx_protocol_buffer_[1] + 1;

        if(context.length <= mailboxes.kBufferSize)
        {
          context.state = context.State::kReadData;
          new_state = sendAckNack(false, kStateRunCommand);
        }
        else
        {
          new_state = sendAckNack(true, kStateWaitForCommand);
        }
      }
      break;

    case context.State::kReadData:
      if(first_run)
      {
        comms_.startTransaction({
          .txBuffer = NULL,
          .rxBuffer = rx_buffer_,
          .length = context.length + 1 // extra byte for the checksum
        });
      }
      else
      {
        mailboxes.write(
          context.mailboxId,
          (const uint8_t*)rx_buffer_,
          context.length
        );

        new_state = sendAckNack(false, kStateWaitForCommand);
      }
    break;
  }

  return new_state;
}
