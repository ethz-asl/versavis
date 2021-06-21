////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  NmeaParser.h
////////////////////////////////////////////////////////////////////////////////
//
//  Parse NMEA messages received on Serial.
//
// Example:
// 
// #include <NmeaParser.h>
//
// NmeaParser nmea_parser;
//
// void setup() {
//   while(!SerialUSB);
//   Serial.begin(115200);
// }
//
// void loop() {
//   while (Serial.available()) {
//     auto sentence_type = nmea_parser.parseChar(Serial.read());
//     if (sentence_type == NmeaParser::SentenceType::kGpZda) {
//       SerialUSB.println(nmea_parser.getGpZdaMessage().str);
//     }
//   }
//
//   delay(100);
// }
//
////////////////////////////////////////////////////////////////////////////////

#ifndef NmeaParser_h_
#define NmeaParser_h_

#include <cstdint>

#include "nmea_parser/msg/ZdaMessage.h"

class NmeaParser {
public:
  enum class SentenceType { kGpZda, kUnknown };

  NmeaParser();
  // Parse an individual character from serial buffer. If sentence is finished
  // return the sentence type.
  SentenceType parseChar(const char c);

  inline ZdaMessage getGpZdaMessage() { return gp_zda_message_; }

private:
  // NMEA description https://resources.winsystems.com/software/nmea.pdf
  // $->ID->MSG->','->Dn->*->CS->[CR][LF]
  enum class State { kUnknown, kId, kMsg, kDataField, kCheckSum, kSuccess };
  enum class IdType { kGps, kUnknown };
  enum class MsgType { kZda, kUnknown };

  // Sentence storage.
  static const uint8_t kIdSize = 2;
  static const uint8_t kMsgSize = 3;
  static const uint8_t kCsSize = 2;
  // Max size minus minimum info.
  static const uint8_t kDataFieldSize = 79 - kIdSize - kMsgSize - kCsSize - 1;
  // +1 for null termination.
  char id_[kIdSize + 1];
  char msg_[kMsgSize + 1];
  char cs_[kCsSize + 1];
  char data_field_[kDataFieldSize + 1];
  uint8_t cs_calculated_ = 0x00;

  // State and message info.
  State state_ = State::kUnknown;
  IdType id_type_ = IdType::kUnknown;
  MsgType msg_type_ = MsgType::kUnknown;
  SentenceType sentence_type_ = SentenceType::kUnknown;
  uint8_t df_idx_ = 0; // The index of the data field in the current sentence.

  void resetSentence();
  void resetWord();
  void transitionState(const State new_state);
  void addCharacter(const char c, char *field, const uint8_t len);
  void addToCheckSum(const char c);

  bool terminateId();
  bool terminateMsg();
  bool terimateDataFieldAndStartNext();
  bool terminateDataFieldAndStartCs();

  bool processIdType();
  bool processMsgType();
  bool processDataField();
  bool processCheckSum();
  bool processSentenceType();

  bool processZdaMessage();

  // Received messages.
  ZdaMessage gp_zda_message_;
};

#endif
