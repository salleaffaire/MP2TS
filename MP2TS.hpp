/*
 * MP2TS.hpp
 *
 *  Created on: Jul 3, 2015 ... 2018
 *      Author: luc.martel@irdeto.com
 */

#ifndef MP2TS_HPP_
#define MP2TS_HPP_

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <vector>

// Logger
// #define LOGURU_IMPLEMENTATION 1
// #include "loguru.hpp"

#include "binaryfile.hpp"
#include "hexio.hpp"

#define PES_PACKET_MAXSIZE 2048 * 1024

namespace MP2TS {

#include "media-elements.hpp"

#define TS_PACKET_LENGTH 188

static const uint32_t PAT_PID = 0;
static const uint32_t CAT_PID = 1;
static const uint32_t TSDT_PID = 2;
static const uint32_t IPMP_PID = 3;

// Errors
// -----------------------------------------------------------------------------------------
enum class ErrorCode {
  NONE = 0,
  INCOMPLETE_PACKET = 0x100,
  INVALID_START_CODE = 0x101
};

// MPEG-2 TS Multiplexer Main Class
// -----------------------------------------------------------------------------------------
class Mux : public BinaryOutFile {
 public:
  Mux(std::string filename);

  bool Put(PES_Packet *packet);

  bool BuildPMT(PES_Packet *packet);

  bool BuildPAT(PES_Packet *packet);

  bool AddChannel(uint32_t channel_number, uint32_t pmt_pid);

  bool AddStreamToChannel(uint32_t channel_number, uint32_t pid,
                          uint32_t stream_type);

 private:
  //
  std::map<uint32_t, uint32_t> mPMTPIDtoChannelNumber;
  std::map<uint32_t, Channel> mChannelMap;

  bool IsAChannel(uint32_t channel_number);

  // This is where the TS MUX writes
  uint8_t mCurrentTSPacket[TS_PACKET_LENGTH];
};

// MPEG-2 TS Demultiplexer Main Class
// -----------------------------------------------------------------------------------------
class Demux : public BinaryInFile {
 public:
  Demux(std::string filename);

  Demux(std::vector<uint8_t> &data);

  ~Demux();

  bool Get(PES_Packet *&returned_packet);

  bool GetDangling(PES_Packet *&returned_packet);

  bool Parse(PES_Packet *packet, bool outputPESPacketStart = false);

  bool ParsePAT(PES_Packet *packet);

  uint32_t NumberOfTSPackets();

  uint32_t GetPID();

  bool IsEOF();

  ErrorCode Error();

  std::map<uint32_t, Channel> &GetChannelMap();

  void SetVerbosity(bool x);

  void SetOutputPacket(bool x);

  void SetOutputTSPacket(bool x);

 private:
  bool mVerbose;
  bool mOutputPacket;
  bool mOutputTSPacket;

  uint32_t mCurrentPointer;

  ErrorCode mError;
  bool mEOF;
  uint32_t mNumberOfTSPackets;
  uint32_t mNumberOfTSPacketsPerPESPacket;

  bool mIsPATParsed;

  // Transport Stream Header
  uint32_t mPID;
  uint32_t mTEI;
  uint32_t mPayloadUnitStartIndicator;
  uint32_t mTransportPriority;

  uint32_t mScramblingControl;
  uint32_t mAdaptationFieldExist;
  uint32_t mContainsPayload;
  uint32_t mContinuityCounter;

  // Adaptation Field
  bool mAdaptationFieldWasPresent;
  uint32_t mAdaptationFieldLength;
  uint32_t mDiscontinuityIndicator;
  uint32_t mRandomAccessIndicator;
  uint32_t mElementaryStreamPriorityIndicator;
  uint32_t mPCRFlag;
  uint32_t mOPCRFlag;
  uint32_t mSplicingPointFlag;
  uint32_t mTransportPrivateDataFlag;
  uint32_t mAdaptationFieldExtensionFlag;
  uint64_t mPCR;
  uint64_t mOPCR;
  uint64_t mSpliceCountDown;

  uint8_t mAdaptationFieldPrivateData[188];

  std::map<uint32_t, uint32_t> mPMTPIDtoChannelNumber;
  std::map<uint32_t, Channel> mChannelMap;

  bool IsAPMTPID(uint32_t pid);

  // CRC holder
  //
  uint8_t *mCRCPointer;
  uint32_t mCRCLength;

  uint32_t mFillerLength;

  uint32_t mTableId;
  uint32_t mTableIdExtenxion;

  uint32_t mSectionNumber;
  uint32_t mLastSectionNumber;

  // Maps <PID, PES_Packet *>
  std::map<uint32_t, PES_Packet *> mPESPacketsMap;

  bool AsAPIDPacket(uint32_t pid);

  // Returns the number of bytes parsed
  uint32_t ParsePSIHeader(PES_Packet *packet);

  bool ParseAdaptationField(uint8_t *data, uint32_t size);

  bool ParsePMT(PES_Packet *packet);

  bool ParseAssigned(PES_Packet *packet, bool outputPESPacketStart = false);
};

}  // namespace MP2TS

#endif /* MP2TS_HPP_ */
