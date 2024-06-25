#pragma once

#include "macro.h"
#include "comms.h"
#include "mailbox.h"
#include <stdint.h>
#include <stddef.h>

#define PROTOCOL_TRACE 1

class Protocol
{
  public:
    enum class Mode
    {
      kSpi,
      kUart
    };

    Mailbox mailboxes;

    Protocol(Comms& comms, Mode mode_, Mailbox::Pool* mailbox_pools, size_t mailbox_pools_count);
    ~Protocol(){};

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
      kStart = 0x7F,
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
      State (Protocol::*stateHandler)(bool first_run);
    };

    static const StateTableEntry state_table_uart_[kStateCount];
    static const StateTableEntry state_table_spi_[kStateCount];

    struct CommandTableEntry
    {
      Command command;
      void  (Protocol::*init)();
      State (Protocol::*stateHandler)(bool first_run);
    };
    static const CommandTableEntry command_table_[3U];

    Comms&    comms_;
    bool      send_dummy_byte_;
    State     state_;
    bool      first_run_;
    bool      send_nack_;
    State     state_after_ack_;
    uint8_t   wait_for_ack_cycles_;
    uint8_t   bytes_read_;
    Command   current_command_;

    const StateTableEntry*  state_table_;

    volatile uint8_t rx_buffer_[256U];
    volatile uint8_t tx_buffer_[256U];

    volatile uint8_t tx_protocol_buffer_[10U];
    volatile uint8_t rx_protocol_buffer_[3U];

#if PROTOCOL_TRACE==1
    struct
    {
      State    state;
      uint8_t  cycles;
      uint32_t timestamp;
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
    static void errorCallback(void* param);
    void processState();

    State sendAckNack(bool send_nack, State state_after_ack);

    State stateWaitForSyncHandlerSpi(bool first_run);
    State stateSendAckNackHandlerSpi(bool first_run);
    State stateWaitForAckHandlerSpi(bool first_run);
    State stateWaitForCommandHandlerSpi(bool first_run);
    State stateRunCommandHandlerSpi(bool first_run);

    State stateWaitForSyncHandlerUart(bool first_run);
    State stateSendAckNackHandlerUart(bool first_run);
    State stateWaitForAckHandlerUart(bool first_run);
    State stateWaitForCommandHandlerUart(bool first_run);
    State stateRunCommandHandlerUart(bool first_run);

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
