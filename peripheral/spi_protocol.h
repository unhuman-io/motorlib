#pragma once

#include "spi_slave.h"
#include "spi_mailbox.h"
#include <stdint.h>
#include <stddef.h>

#define SPI_PROTOCOL_TRACE 1

class SpiProtocol
{
  public:
    SpiMailbox mailboxes;

    SpiProtocol(SpiSlave& spi_slave, SpiMailbox::Pool* mailbox_pools, size_t mailbox_pools_count) :
      mailboxes(mailbox_pools, mailbox_pools_count),
      spi_slave_(spi_slave),
      state_(kStateWaitForSync),
      first_run_(true),
      send_nack_(false),
      state_after_ack_(kStateWaitForCommand),
      current_command_(Command::kSync),
      tx_protocol_buffer_{0},
      rx_protocol_buffer_{0}
    {};

    ~SpiProtocol(){};

    void init();

  private:
    enum State
    {
      kStateInit,
      kStateWaitForSync,
      kStateWaitForAck,
      kStateSendAckNack,
      kStateWaitForCommand,
      kStateRunCommand,
      kStateCount
    };

    enum class Command: uint8_t
    {
      kSync = 0x5A,
      kDummy = 0xA5,
      kAck = 0x79,
      kNack = 0x1F,

      kGet = 0x00,
      kReadMemory = 0x11,
      kWriteMemory = 0x31,
      kReadMailbox = 0xF0,
      kWriteMailbox = 0xF1
    };

    struct StateTableEntry
    {
      State (SpiProtocol::*stateHandler)(bool first_run);
    };
    static const StateTableEntry state_table_[kStateCount];

    struct CommandTableEntry
    {
      Command command;
      void  (SpiProtocol::*init)();
      State (SpiProtocol::*stateHandler)(bool first_run);
    };
    static const CommandTableEntry command_table_[4U];

    SpiSlave& spi_slave_;
    State     state_;
    bool      first_run_;
    bool      send_nack_;
    State     state_after_ack_;
    uint8_t   wait_for_ack_cycles_;
    Command   current_command_;

    uint8_t rx_buffer_[256U];
    uint8_t tx_buffer_[256U];

    uint8_t tx_protocol_buffer_[10U];
    uint8_t rx_protocol_buffer_[3U];

#if SPI_PROTOCOL_TRACE==1
    struct
    {
      State    state;
      uint32_t cycles;
    }state_trace_buffer_[10];
#endif

    struct
    {
      size_t totalLength;
    }get_command_context_;

    struct
    {
      enum class State: uint8_t
      {
        kReadAddress,
        kReadLength,
        kSendData
      };

      State    state;
      uint32_t address;
      size_t   length;
      bool     dataSent;
    }read_command_context_;

    struct
    {
      enum class State: uint8_t
      {
        kReadIdAndLength,
        kReadId,
        kWriteLengthAndData,
        kReadData
      };

      State    state;
      uint8_t  mailboxId;
      size_t   length;
      bool     dataSent;
    }mailbox_command_context_;

    volatile bool data_processing_finished_;

    static void transactionCompleteCallback(void* param);
    void processState();
    State sendAckNack(bool send_nack, State state_after_ack);

    State stateWaitForSyncHandler(bool first_run);
    State stateSendAckNackHandler(bool first_run);
    State stateWaitForAckHandler(bool first_run);
    State stateWaitForCommandHandler(bool first_run);
    State stateRunCommandHandler(bool first_run);

    // Command handlers
    void  getCommandInit();
    State getCommandStateHandler(bool first_run);

    void  readCommandInit();
    State readCommandStateHandler(bool first_run);

    void  readMailboxCommandInit();
    State readMailboxCommandStateHandler(bool first_run);

    void  writeMailboxCommandInit();
    State writeMailboxCommandStateHandler(bool first_run);
};