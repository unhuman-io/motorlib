// Copyright 2024 Figure AI, Inc

#include "firmware/lib/obot-parser/obot-parser.h"

#include <gtest/gtest.h>

class ObotParserTest : public ::testing::Test {
 public:
  void SetUp() override { packets_processed_ = 0; }

  int packets_processed_{0};
};

TEST_F(ObotParserTest, noNewData) {
  uint8_t buffer[2048] = {};
  figure::ObotParser parser(buffer, sizeof(buffer));
  parser.registerCallback(
      0, [this](const uint8_t* /*packet*/, uint16_t /*len*/) { this->packets_processed_++; });
  parser.process(0);
  EXPECT_EQ(packets_processed_, 0);
  parser.process(0);
  EXPECT_EQ(packets_processed_, 0);
}

TEST_F(ObotParserTest, singlePacket) {
  uint8_t buffer[2048] = {0xCA, 0xFE, 0x00, 0x01, 0x00, 0x68, 0x8C};
  figure::ObotParser parser(buffer, sizeof(buffer));
  parser.registerCallback(0, [this](const uint8_t* packet, uint16_t len) {
    EXPECT_EQ(len, 1);
    EXPECT_EQ(packet[0], 0);
    this->packets_processed_++;
  });
  parser.process(6);
  EXPECT_EQ(packets_processed_, 1);
  parser.process(6);
  EXPECT_EQ(packets_processed_, 1);
}

// TEST_F(ObotParserTest, multipleConsecutivePackets) {
//   uint8_t buffer[2048] = {0xCA, 0xFE, 0x00, 0x01, 0x00, 0x68, 0x8C,
//                           0xCA, 0xFE, 0x00, 0x01, 0x00, 0x68, 0x8C};
//   figure::ObotParser parser(buffer, sizeof(buffer));
//   parser.registerCallback(0, [this](const uint8_t* packet, uint16_t len) {
//     EXPECT_EQ(len, 1);
//     EXPECT_EQ(packet[0], 0);
//     this->packets_processed_++;
//   });
//   parser.process(13);
//   EXPECT_EQ(packets_processed_, 2);
//   parser.process(13);
//   EXPECT_EQ(packets_processed_, 2);
//   parser.process(14);
//   EXPECT_EQ(packets_processed_, 2);
//   // Wraparound
//   parser.process(13);
//   EXPECT_EQ(packets_processed_, 4);
// }

// TEST_F(ObotParserTest, wraparoundPacket1) {
//   uint8_t buffer[2048] = {0xFE, 0x00, 0x01, 0x00, 0x68, 0x8C};
//   buffer[2047] = 0xCA;
//   figure::ObotParser parser(buffer, sizeof(buffer));
//   parser.registerCallback(0, [this](const uint8_t* packet, uint16_t len) {
//     EXPECT_EQ(len, 1);
//     EXPECT_EQ(packet[0], 0);
//     this->packets_processed_++;
//   });
//   parser.process(6);
//   EXPECT_EQ(packets_processed_, 0);
//   // Wraparound
//   parser.process(5);
//   EXPECT_EQ(packets_processed_, 1);
// }

// TEST_F(ObotParserTest, wraparoundPacket2) {
//   uint8_t buffer[2048] = {0x00, 0x01, 0x00, 0x68, 0x8C};
//   buffer[2046] = 0xCA;
//   buffer[2047] = 0xFE;
//   figure::ObotParser parser(buffer, sizeof(buffer));
//   parser.registerCallback(0, [this](const uint8_t* packet, uint16_t len) {
//     EXPECT_EQ(len, 1);
//     EXPECT_EQ(packet[0], 0);
//     this->packets_processed_++;
//   });
//   parser.process(5);
//   EXPECT_EQ(packets_processed_, 0);
//   // Wraparound
//   parser.process(4);
//   EXPECT_EQ(packets_processed_, 1);
// }

// TEST_F(ObotParserTest, wraparoundPacket3) {
//   uint8_t buffer[2048] = {0x01, 0x00, 0x68, 0x8C};
//   buffer[2045] = 0xCA;
//   buffer[2046] = 0xFE;
//   buffer[2047] = 0x00;
//   figure::ObotParser parser(buffer, sizeof(buffer));
//   parser.registerCallback(0, [this](const uint8_t* packet, uint16_t len) {
//     EXPECT_EQ(len, 1);
//     EXPECT_EQ(packet[0], 0);
//     this->packets_processed_++;
//   });
//   parser.process(4);
//   EXPECT_EQ(packets_processed_, 0);
//   // Wraparound
//   parser.process(3);
//   EXPECT_EQ(packets_processed_, 1);
// }

// TEST_F(ObotParserTest, wraparoundPacket4) {
//   uint8_t buffer[2048] = {0x68, 0x8C};
//   buffer[2044] = 0xCA;
//   buffer[2045] = 0xFE;
//   buffer[2046] = 0x00;
//   buffer[2047] = 0x01;
//   figure::ObotParser parser(buffer, sizeof(buffer));
//   parser.registerCallback(0, [this](const uint8_t* packet, uint16_t len) {
//     EXPECT_EQ(len, 1);
//     EXPECT_EQ(packet[0], 0);
//     this->packets_processed_++;
//   });
//   parser.process(3);
//   EXPECT_EQ(packets_processed_, 0);
//   // Wraparound
//   parser.process(2);
//   EXPECT_EQ(packets_processed_, 1);
// }

// TEST_F(ObotParserTest, wraparoundPacket5) {
//   uint8_t buffer[2048] = {0x8C};
//   buffer[2043] = 0xCA;
//   buffer[2044] = 0xFE;
//   buffer[2045] = 0x00;
//   buffer[2046] = 0x01;
//   buffer[2047] = 0x68;
//   figure::ObotParser parser(buffer, sizeof(buffer));
//   parser.registerCallback(0, [this](const uint8_t* packet, uint16_t len) {
//     EXPECT_EQ(len, 1);
//     EXPECT_EQ(packet[0], 0);
//     this->packets_processed_++;
//   });
//   parser.process(2);
//   EXPECT_EQ(packets_processed_, 0);
//   // Wraparound
//   parser.process(1);
//   EXPECT_EQ(packets_processed_, 1);
// }
