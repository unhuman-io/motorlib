#include "spi_protocol.h"

const SpiProtocol::CommandTableEntry SpiProtocol::command_table_[] =
{
  {Command::kGet          , &SpiProtocol::getCommandInit          , &SpiProtocol::getCommandStateHandler          },
  {Command::kReadMemory   , &SpiProtocol::readCommandInit         , &SpiProtocol::readCommandStateHandler         },
  {Command::kReadMailbox  , &SpiProtocol::readMailboxCommandInit  , &SpiProtocol::readMailboxCommandStateHandler  },
  {Command::kWriteMailbox , &SpiProtocol::writeMailboxCommandInit , &SpiProtocol::writeMailboxCommandStateHandler }
};

void SpiProtocol::getCommandInit()
{
  size_t total_length = 0;

  tx_protocol_buffer_[total_length++] = 0x00; // Dummy byte
  tx_protocol_buffer_[total_length++] = 0x00; // Reserved for the length byte
  tx_protocol_buffer_[total_length++] = 0x81; // Protocol version

  // Write all the supported command IDs from the command_table_
  for(size_t i = 0; i < FIGURE_COUNTOF(command_table_); i++)
  {
    tx_protocol_buffer_[total_length++] = (uint8_t)command_table_[i].command;
  }

  // Insert the length byte
  tx_protocol_buffer_[1] = total_length - 3; // Number of bytes - 1 excluding dummy and length

  get_command_context_.totalLength = total_length;
}

SpiProtocol::State SpiProtocol::getCommandStateHandler(bool first_run)
{
  State new_state = kStateRunCommand;

  if(first_run)
  {
    spi_slave_.startTransaction({
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

void SpiProtocol::readCommandInit()
{
  read_command_context_ = {
    .state     = read_command_context_.State::kReadAddress,
    .address   = 0,
    .length    = 0,
    .dataSent  = false
  };
}

SpiProtocol::State SpiProtocol::readCommandStateHandler(bool first_run)
{
  State new_state = kStateRunCommand;
  auto& context = read_command_context_;

  switch(context.state)
  {
    case context.State::kReadAddress:
      if(first_run)
      {
        spi_slave_.startTransaction({
          .txBuffer = NULL,
          .rxBuffer = rx_protocol_buffer_,
          .length = 5
        });
      }
      else
      {
        context.address =
          (rx_protocol_buffer_[0] << 24) |
          (rx_protocol_buffer_[1] << 16) |
          (rx_protocol_buffer_[2] << 8) |
          (rx_protocol_buffer_[3]);

        context.state = context.State::kReadLength;
        new_state = sendAckNack(false, kStateRunCommand);
      }
      break;

    case context.State::kReadLength:
      if(first_run)
      {
        spi_slave_.startTransaction({
          .txBuffer = NULL,
          .rxBuffer = rx_protocol_buffer_,
          .length = 2
        });
      }
      else
      {
        context.length = rx_protocol_buffer_[0] + 1;
        context.state = context.State::kSendData;
        new_state = sendAckNack(false, kStateRunCommand);
      }
      break;

    case context.State::kSendData:
      if(first_run)
      {
        // Send a dummy byte
        spi_slave_.startTransaction({
          .txBuffer = NULL,
          .rxBuffer = NULL,
          .length = 1
        });
      }
      else
      {
        if(!context.dataSent)
        {
          // Send the data out
          spi_slave_.startTransaction({
            .txBuffer = tx_buffer_,
            .rxBuffer = NULL,
            .length   = context.length
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

void SpiProtocol::readMailboxCommandInit()
{
  mailbox_command_context_ = {
    .state      = mailbox_command_context_.State::kReadId,
    .mailboxId  = 0,
    .length     = 0,
    .dataSent   = false
  };
}

SpiProtocol::State SpiProtocol::readMailboxCommandStateHandler(bool first_run)
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
        spi_slave_.startTransaction({
          .txBuffer = NULL,
          .rxBuffer = rx_protocol_buffer_,
          .length = 2 // mbox_id + checksum
          });
      }
      else
      {
        context.mailboxId = rx_protocol_buffer_[0];
        context.length = mailboxes.read(context.mailboxId, tx_buffer_, sizeof(tx_buffer_));

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
        tx_protocol_buffer_[0] = 0x00; // Dummy byte
        tx_protocol_buffer_[1] = context.length - 1; // Length

        spi_slave_.startTransaction({
          .txBuffer = tx_protocol_buffer_,
          .rxBuffer = NULL,
          .length = 2 // dummy byte + length
          });
      }
      else
      {
        if(!context.dataSent)
        {
          spi_slave_.startTransaction({
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

void SpiProtocol::writeMailboxCommandInit()
{
  mailbox_command_context_ = {
    .state      = mailbox_command_context_.State::kReadIdAndLength,
    .mailboxId  = 0,
    .length     = 0,
    .dataSent   = false
  };
}

SpiProtocol::State SpiProtocol::writeMailboxCommandStateHandler(bool first_run)
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
        spi_slave_.startTransaction({
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
        spi_slave_.startTransaction({
          .txBuffer = NULL,
          .rxBuffer = rx_buffer_,
          .length = context.length + 1 // extra byte for the checksum
        });
      }
      else
      {
        mailboxes.write(context.mailboxId, rx_buffer_, context.length);
        new_state = sendAckNack(false, kStateWaitForCommand);
      }
    break;
  }

  return new_state;
}
