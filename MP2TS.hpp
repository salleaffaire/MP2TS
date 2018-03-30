/*
 * MP2TS.hpp
 *
 *  Created on: Jul 3, 2015
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

#include "BinaryFile.hpp"
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
  Mux(std::string filename) : BinaryOutFile(filename) {}

  bool Put(PES_Packet *packet) {
    bool rval = true;

    // Build a series of TS packets
    uint32_t bytes_left = packet->mSize;
    uint32_t pid = packet->mPID;
    do {
    } while (bytes_left);

    return rval;
  }

  bool BuildPMT(PES_Packet *packet) {
    bool rval = true;

    return rval;
  }

  bool BuildPAT(PES_Packet *packet) {
    bool rval = true;

    return rval;
  }

  bool AddChannel(uint32_t channel_number, uint32_t pmt_pid) {
    bool rval = true;

    // Add the channel to the data structures
    mPMTPIDtoChannelNumber[pmt_pid] = channel_number;
    mChannelMap[channel_number] = Channel(pmt_pid);

    return rval;
  }

  bool AddStreamToChannel(uint32_t channel_number, uint32_t pid,
                          uint32_t stream_type) {
    bool rval = true;

    if (IsAChannel(channel_number)) {
      mChannelMap[channel_number].mStreams.push_back(Stream(stream_type, pid));
    } else {
      rval = false;
    }

    return rval;
  }

 private:
  //
  std::map<uint32_t, uint32_t> mPMTPIDtoChannelNumber;
  std::map<uint32_t, Channel> mChannelMap;

  bool IsAChannel(uint32_t channel_number) {
    return !(mChannelMap.find(channel_number) == mChannelMap.end());
  }

  // This is where the TS MUX writes
  uint8_t mCurrentTSPacket[TS_PACKET_LENGTH];
};

// MPEG-2 TS Demultiplexer Main Class
// -----------------------------------------------------------------------------------------
class Demux : public BinaryInFile {
 public:
  Demux(std::string filename)
      : BinaryInFile(filename),
        mVerbose(true),
        mOutputPacket(false),
        mOutputTSPacket(false),
        mCurrentPointer(0),
        mError(ErrorCode::NONE),
        mEOF(false),
        mNumberOfTSPackets(0),
        mIsPATParsed(false) {}

  ~Demux() {
    // Cleaning unused or unreturned packets
    for (auto packet : mPESPacketsMap) {
      delete packet.second;
    }
  }

  bool Get(PES_Packet *&returned_packet) {
    bool rval = true;

    // PES Write Pointer
    uint32_t temp;
    returned_packet = (PES_Packet *)0;

    // Find first TS packet with Start Indicator = 1
    while ((mData[mCurrentPointer] == 0x47) &&
           !(mData[mCurrentPointer + 1] & 0x40)) {
      // Skip to next packet
      mCurrentPointer += TS_PACKET_LENGTH;
    }

    // Keep track of the number of TS packets per PES packet
    mNumberOfTSPacketsPerPESPacket = 0;

    bool packet_complete = false;
    mAdaptationFieldWasPresent = false;
    do {
      // Advance to the first sync byte 0x47
      uint32_t bytes_skipped = 0;
      while (((mSize - mCurrentPointer) > 0) &&
             (mData[mCurrentPointer] != 0x47)) {
        mCurrentPointer++;
        bytes_skipped++;
      }
      if (mVerbose) {
        if (bytes_skipped > 0) {
          std::cout << "Skipped " << bytes_skipped << " bytes. " << std::endl;
        }
      }

      uint32_t start_pointer = mCurrentPointer;
      // If there is enough room fo ran extra TS packet
      // PS packet size = 188 bytes
      if ((mSize - mCurrentPointer) >= TS_PACKET_LENGTH) {
        if (mData[mCurrentPointer++] == 0x47) {
          // Output the content of the entire TS packet
          if (mOutputTSPacket) {
            std::cout << "TS packet #" << mNumberOfTSPackets << std::endl;
            outputHex(&(mData[mCurrentPointer - 1]), 188);
          }
          // Parsing The Tansport Stream Header (First 32 bits)
          // -----------------------------------------------------------------
          // Read PID
          temp = ((mData[mCurrentPointer]) << 8) | mData[mCurrentPointer + 1];
          mPID = temp & 0x1fff;
          mTEI = temp & 0x8000;
          mPayloadUnitStartIndicator = temp & 0x4000;
          mTransportPriority = temp & 0x2000;
          mCurrentPointer += 2;

          // Next 8 bits
          temp = mData[mCurrentPointer++];
          mScramblingControl = (temp & 0xC0) >> 6;
          mAdaptationFieldExist =
              (temp & 0x20) >> 5;  // Adaptation field control msb
          mContainsPayload =
              (temp & 0x10) >> 4;  // Adaptation field control lsb
          mContinuityCounter = (temp & 0x0f);

          // For debugging only
          // if ((mNumberOfTSPackets == 113)) {
          //  std::cout << "Found the broken packet" << std::endl;
          // }
          if (mVerbose) {
            std::cout << mPID << " @  TS Packet " << mNumberOfTSPackets
                      << std::endl;
          }
          // If Present - Parse the Adaptation Field
          // ----------------------------------------------------------------
          if (mAdaptationFieldExist) {
            mAdaptationFieldWasPresent = true;

            // In a TS packet the adaptation field may or may not be present.
            // When it is present, the first byte is the length of the
            // adaptation field in bytes following the length. Well, encoders
            // use adaptation field length = 0, to insert 1 single byte of
            // stuffing. It's a thing
            // You have to guard against that
            // So the size will always be checked against within
            // ParseAdaptationField()
            mAdaptationFieldLength = mData[mCurrentPointer++];

            if (mVerbose) {
              std::cout << "Found adaptation field of length "
                        << mAdaptationFieldLength << " bytes" << std::endl;
            }

            // Parse the adaptation field
            ParseAdaptationField(&(mData[mCurrentPointer]),
                                 mAdaptationFieldLength);

            // Skip it
            mCurrentPointer += mAdaptationFieldLength;
          }

          // Find a PES Packet to write the data into
          // --------------------------------------------------------------------
          PES_Packet *current_packet = (PES_Packet *)0;

          // If it is the start of a new payload
          if (mPayloadUnitStartIndicator) {
            // std::cout << "Creating a PES_Packet for PID : " << mPID
            //          << std::endl;
            current_packet = new PES_Packet;
            current_packet->mPID = mPID;
            current_packet->mContinuityCounter = mContinuityCounter;
            // If we already have a PES_Packet with the same PID
            if (AsAPIDPacket(mPID)) {
              packet_complete = true;
              returned_packet = mPESPacketsMap[mPID];
            }
            mPESPacketsMap[mPID] = current_packet;
          } else {
            if (AsAPIDPacket(mPID)) {
              current_packet = mPESPacketsMap[mPID];
            }
          }
          // --------------------------------------------------------------------

          uint32_t bytes_left =
              TS_PACKET_LENGTH - (mCurrentPointer - start_pointer);
          // If we have a current packet
          if ((current_packet) && (mContainsPayload)) {
            // Check continuity counter
            if (mContinuityCounter != current_packet->mContinuityCounter) {
              current_packet->mState |= MP2TS::PESPacketState::CONTINUITY_ERROR;
              if (mVerbose) {
                std::cout << "Error: Invalid continuity counter" << std::endl;
                std::cout << "  Expected : "
                          << (int)current_packet->mContinuityCounter
                          << " but found : " << mContinuityCounter << std::endl;
              }
            }
            // Update the continuity counter to the expected value for the next
            // TS packet
            current_packet->mContinuityCounter =
                (current_packet->mContinuityCounter + 1) & 0xf;

            // Go to the end of the current_packet data
            uint8_t *write_pointer =
                (current_packet->mData) + (current_packet->mSize);
            // Copy Payload into the write buffer
            // std::cout << "Bytes left : " << bytes_left << " written." <<
            // std::endl;  std::cout << "In a packet of size : " <<
            // current_packet->mSize << std::endl;
            std::memcpy(write_pointer, &(mData[mCurrentPointer]), bytes_left);
            current_packet->mSize += bytes_left;

            // Output the content of the TS payload
            if (mOutputTSPacket) {
              std::cout << "TS packet #" << mNumberOfTSPackets << " payload"
                        << std::endl;
              outputHex(&(mData[mCurrentPointer]), bytes_left);
            }
          }
          mCurrentPointer += bytes_left;

          mNumberOfTSPacketsPerPESPacket++;
          mNumberOfTSPackets++;
        } else {
          mError = ErrorCode::INVALID_START_CODE;
          break;
        }
      } else {
        mEOF = true;
      }

      // Loop until either we reach the end of file or if we find a packet
    } while ((!mEOF) && (!packet_complete));

    rval = packet_complete;
    return rval;
  }

  bool GetDangling(PES_Packet *&returned_packet) {
    bool rval = true;
    if (mPESPacketsMap.size()) {
      returned_packet = mPESPacketsMap.begin()->second;
      mPESPacketsMap.erase(mPESPacketsMap.begin());
    } else {
      rval = false;
    }
    return rval;
  }

  bool Parse(PES_Packet *packet, bool outputPESPacketStart = false) {
    bool rval = true;

    // Decode Program Association Table
    if (packet->mPID == PAT_PID) {
      ParsePAT(packet);
    }
    // TODO: Conditional Access Table
    else if (packet->mPID == CAT_PID) {
    }
    // TODO: Transport Stream Description Table
    else if (packet->mPID == TSDT_PID) {
    }
    // TODO: IPMP Control Information Table
    else if (packet->mPID == IPMP_PID) {
    }
    // Reserved - this should not happen
    else if ((packet->mPID >= 0x4) && (mPID <= 0xf)) {
    }
    // DVB Metadata
    else if ((packet->mPID >= 0x10) && (mPID <= 0x1f)) {
    }
    // Assigned by PMT
    else if (((packet->mPID >= 0x20) && (mPID <= 0x1ffa))) {
      if (mVerbose) {
        std::cout
            << "Parsing Assigned PID *********************************** 0x"
            << std::hex << std::setfill('0') << std::setw(6) << mPID << std::dec
            << std::endl;
      }
      ParseAssigned(packet, outputPESPacketStart);
    }
    // TODO: Used by DigiCypher 2/ATSC MGT Metadata
    else if (packet->mPID == 0x1ffb) {
    }
    // Assigned by PMT
    else if (((packet->mPID >= 0x1ffc) && (mPID <= 0x1ffe))) {
      if (mVerbose) {
        std::cout
            << "Parsing Assigned PID *********************************** 0x"
            << std::hex << std::setfill('0') << std::setw(6) << mPID << std::dec
            << std::endl;
      }
      ParseAssigned(packet, outputPESPacketStart);
    }
    // TODO: Null Packet
    else if (packet->mPID == 0x1fff) {
    }
    return rval;
  }

  bool ParsePAT(PES_Packet *packet) {
    bool rval = true;

    if (mVerbose) {
      std::cout << "Parsing PAT" << std::endl;
      outputHex(packet->mData, 184);
    }

    // Parser the PSI Header
    ParsePSIHeader(packet);

    // After the PSI Header the pointer needs to advance by
    // mFillerLength + 8
    uint8_t *data = packet->mData + mFillerLength + 8;

    // Start decoding PAT
    // bytes left excluding the CRC is
    // Section Length - (5) Syntax - (4) CRC
    // Section Length = CRC Length + 1
    // Bytes Left = CRC Length - 8
    uint32_t bytes_left = mCRCLength - 8;

    // Number of PAT Specific Data is bytes_left / 4
    uint32_t PAT_entries = bytes_left >> 2;

    //
    if (mVerbose == true) {
      std::cout << " " << PAT_entries << " PAT entries" << std::endl;
    }
    for (uint32_t i = 0; i < PAT_entries; i++) {
      unsigned int program_number = 0;
      program_number |= *data << 8;
      data++;
      program_number |= *data;
      data++;
      if (mVerbose == true) {
        std::cout << "   Program Number = " << program_number << std::endl;
      }
      unsigned int pmt_pid = 0;
      pmt_pid |= *data << 8;
      data++;
      pmt_pid |= *data;
      data++;
      if (mVerbose == true) {
        std::cout << "   PMT_PID= " << (pmt_pid & 0x1fff) << std::endl;
      }
      mPMTPIDtoChannelNumber[pmt_pid & 0x1fff] = program_number;
      mChannelMap.insert(std::map<uint32_t, Channel>::value_type(
          program_number, Channel(pmt_pid & 0x1fff)));
      // mChannelMap[program_number] = Channel(pmt_pid);
    }
    mIsPATParsed = true;
    return rval;
  }

  uint32_t NumberOfTSPackets() { return mNumberOfTSPacketsPerPESPacket; }

  uint32_t GetPID() { return mPID; }

  bool IsEOF() { return mEOF; }

  ErrorCode Error() { return mError; }

  std::map<uint32_t, Channel> &GetChannelMap() { return mChannelMap; }

  void SetVerbosity(bool x) { mVerbose = x; }

  void SetOutputPacket(bool x) { mOutputPacket = x; }

  void SetOutputTSPacket(bool x) { mOutputTSPacket = x; }

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

  bool IsAPMTPID(uint32_t pid) {
    bool rval = false;
    for (auto channel : mChannelMap) {
      if (channel.second.mPMTPID == pid) {
        rval = true;
        break;
      }
    }
    return rval;
  }

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

  bool AsAPIDPacket(uint32_t pid) {
    return !(mPESPacketsMap.find(pid) == mPESPacketsMap.end());
  }

  // Returns the number of bytes parsed
  uint32_t ParsePSIHeader(PES_Packet *packet) {
    uint8_t *data = packet->mData;

    // Skip filler
    unsigned int pointer_field = *data++;
    data += pointer_field;
    mFillerLength = 1 + pointer_field;

    // Set the start of the payload to include in the CRC calculation
    mCRCPointer = data;

    // Parse Header
    // Table ID
    mTableId = *data++;
    // std::cout << "Table ID = " << table_id << std::endl;

    // Section Length
    unsigned int section_length = 0;
    section_length |= *data << 8;
    data++;
    section_length |= *data;
    data++;
    section_length &= 0x3FF;  // 10-bits
    // std::cout << "Section length = " << section_length << std::endl;

    // Calculate the CRC Length
    // Section Length is the number of syntaxt bytes including the CRC
    // (3) Table Header Length
    // (section_length) = Syntax Length including CRC
    // (4) CRC Length
    // crc_length = 3 + section_length - 4;
    mCRCLength = section_length - 1;

    // Calculate current CRC
    // crc = calculate_crc32(crc_payload, crc_length, 0xffffffff);
    // std::cout << "PAT CRC = " << std::hex << std::setfill('0') <<
    // std::setw(8) << crc << std::endl;  std::cout << std::dec;

    // Table ID Extension
    mTableIdExtenxion = 0;
    mTableIdExtenxion |= *data << 8;
    data++;
    mTableIdExtenxion |= *data;
    data++;
    // std::cout << "Table ID Extension = " << table_id_extension <<
    // std::endl;

    // Skip (Reserved bits (0b11), Version Number, Current/Next indicator)
    data++;

    mSectionNumber = *data++;
    // std::cout << "Section Number = " << section_number << std::endl;

    mLastSectionNumber = *data++;
    // std::cout << "Last Section Number = " << last_section_number <<
    // std::endl;

    uint32_t bytes_read = (data - packet->mData);
    return bytes_read;
  }

  bool ParseAdaptationField(uint8_t *data, uint32_t size) {
    if (size) {
      uint8_t temp = *data++;
      --size;
      mDiscontinuityIndicator = (temp & 0x80) >> 7;
      mRandomAccessIndicator = (temp & 0x40) >> 6;
      mElementaryStreamPriorityIndicator = (temp & 0x20) >> 5;
      mPCRFlag = (temp & 0x10) >> 4;
      mOPCRFlag = (temp & 0x08) >> 3;
      mSplicingPointFlag = (temp & 0x04) >> 2;
      mTransportPrivateDataFlag = (temp & 0x02) >> 1;
      mAdaptationFieldExtensionFlag = (temp & 0x01);
    }

    if (size) {
      if (mPCRFlag) {
        mPCR = *data++;                //
        mPCR = (mPCR << 8) | *data++;  //
        mPCR = (mPCR << 8) | *data++;  //
        mPCR = (mPCR << 8) | *data++;  //
        mPCR = (mPCR << 8) | *data++;  //
        mPCR = (mPCR << 8) | *data++;  //
        size -= 6;
      }
    }

    if (size) {
      if (mOPCRFlag) {
        mOPCR = *data++;                 //
        mOPCR = (mOPCR << 8) | *data++;  //
        mOPCR = (mOPCR << 8) | *data++;  //
        mOPCR = (mOPCR << 8) | *data++;  //
        mOPCR = (mOPCR << 8) | *data++;  //
        mOPCR = (mOPCR << 8) | *data++;  //
        size -= 6;
      }
    }

    if (size) {
      if (mSplicingPointFlag) {
        mSpliceCountDown = *data++;
        --size;
      }
    }

    // If adaptation field private data is present
    if (size) {
      if (mTransportPrivateDataFlag) {
        unsigned int transportPrivateDataLength = *data++;
        --size;
        for (int i = 0; i < transportPrivateDataLength; i++) {
          mAdaptationFieldPrivateData[i] = *data++;
          --size;
        }
        if (mVerbose) {
          std::cout << "Found Adaptation Field Private Data" << std::endl;
          for (int i = 0; i < transportPrivateDataLength; i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << (unsigned int)mAdaptationFieldPrivateData[i] << " ";
            std::cout << std::dec;
          }
          std::cout << std::endl;
        }
      }
    }
    return true;
  }

  bool ParsePMT(PES_Packet *packet) {
    bool rval = 0;

    if (mVerbose) {
      std::cout << "Parsing PMT" << std::endl;
    }

    // Parser the PSI Header
    ParsePSIHeader(packet);

    // After the PSI Header the pointer needs to advance by
    // mFillerLength + 8
    uint8_t *data = packet->mData + mFillerLength + 8;

    // Start decoding PMT

    uint32_t pcr_pid = 0;
    pcr_pid |= *data << 8;
    data++;
    pcr_pid |= *data;
    data++;
    // std::cout << "PRC PID = " << (pcr_pid & 0x1fff) << std::endl;

    uint32_t program_info_length = 0;
    program_info_length |= *data << 8;
    data++;
    program_info_length |= *data;
    data++;
    program_info_length &= 0x03ff;

    // Skip program descriptor bytes
    data += program_info_length;

    // For all stream
    // TODO :: This toooo simple - right now only expecting 1 video and 1 audio
    // ES

    // Bytes left excluding the CRC is
    // Section Length - (5) Syntax PIS Header - (4) CRC - (4) Syntax PMT -
    // program_info_length Section Length = CRC Length + 1 Bytes Left = CRC
    // Length + 1 - 5 - 4 - 4 - program_info_length Bytes Left = CRC Length - 12
    // - program_info_length
    uint32_t bytes_left = mCRCLength - 12 - program_info_length;

    // std::cout << "Bytes left in PMT ------------------------------ " <<
    // bytes_left << std::endl;

    Channel channel;
    channel.mPMTPID = packet->mPID;

    while (bytes_left > 0) {
      unsigned int stream_type = 0;
      stream_type = *data;
      data++;

      if (mVerbose) {
        std::cout << "Stream Type : = " << stream_type << std::endl;
      }

      unsigned int pid = 0;
      pid |= *data << 8;
      data++;
      pid |= *data;
      data++;
      pid &= 0x1fff;

      if (mVerbose) {
        std::cout << "PID = " << pid << std::endl;
      }

      channel.mStreams.push_back(std::move(Stream(stream_type, pid)));

      // Skip to next
      unsigned int elementary_stream_descriptor_length = 0;
      elementary_stream_descriptor_length |= *data << 8;
      data++;
      elementary_stream_descriptor_length |= *data;
      data++;

      elementary_stream_descriptor_length &= 0x3ff;
      data += elementary_stream_descriptor_length;

      bytes_left -= (5 + elementary_stream_descriptor_length);
    }
    mChannelMap[mPMTPIDtoChannelNumber[channel.mPMTPID]] = std::move(channel);

    // Update the CRC
    // CRC will be located at data[5+crc_length]
    // crc = gCRC(crc_payload, crc_length);
    // data[5+crc_length+0] = (crc >> 24) & 0xFF;
    // data[5+crc_length+1] = (crc >> 16) & 0xFF;
    // data[5+crc_length+2] = (crc >> 8)  & 0xFF;
    // data[5+crc_length+3] = (crc)       & 0xFF;

    return rval;
  }

  bool ParseAssigned(PES_Packet *packet, bool outputPESPacketStart = false) {
    bool rval = true;

    // If PAT is not yet parsed, look into the packet map if there is a
    // packet with PID=0
    if (!mIsPATParsed) {
      // Do we have a PAT?
      if (AsAPIDPacket(0)) {
        // Parse PAT
        ParsePAT(mPESPacketsMap[0]);
      }
      // Do we have any PMT? If yes, parse them.
      for (auto channel : mChannelMap) {
        if (AsAPIDPacket(channel.second.mPMTPID)) {
          ParsePMT(mPESPacketsMap[channel.second.mPMTPID]);
        }
      }
    }

    if (IsAPMTPID(packet->mPID)) {
      ParsePMT(packet);
    } else {
      // Is there a PES start code?
      uint8_t *data = packet->mData;

      uint32_t start_code = 0;
      start_code = *data;
      start_code = (start_code << 8) | *(data + 1);
      start_code = (start_code << 8) | *(data + 2);
      data += 3;
      if (mVerbose) {
        std::cout << "  Start Code : 0x" << std::hex << std::setfill('0')
                  << std::setw(6) << start_code << std::dec << std::endl;
      }
      // Found a stream
      if (start_code == 0x000001) {
        packet->mStreamId = *data;
        data++;

        if (mVerbose) {
          std::cout << "  Stream ID : 0x" << std::hex << std::setfill('0')
                    << std::setw(2) << (int)packet->mStreamId << std::dec
                    << std::endl;
        }

        uint16_t pes_packet_length = *data << 8;
        data++;
        pes_packet_length |= *data;
        data++;

        if (mVerbose) {
          std::cout << "  PES Packet length : " << pes_packet_length
                    << std::endl;
        }

        uint8_t temp = *data;
        data++;

        if (mVerbose) {
          std::cout << "  Marker bits (must be 0x2 or 0b11) : "
                    << ((temp & 0xc0) >> 6) << std::endl;
          std::cout << "  Scrmbling control (0 if not scrambled): "
                    << ((temp & 0x30) >> 4) << std::endl;
          std::cout << "  Priority: " << ((temp & 0x08) >> 3) << std::endl;
          std::cout << "  Data alignement indicator : " << ((temp & 0x04) >> 2)
                    << std::endl;
          std::cout << "  Copyright: " << ((temp & 0x02) >> 1) << std::endl;
          std::cout << "  Original or copy (1 implies copy) : " << (temp & 0x01)
                    << std::endl;
        }

        temp = *data;
        data++;

        packet->mPTSDTSIndicator = (temp & 0xc0) >> 6;
        uint8_t escr_flag = (temp & 0x20) >> 5;
        uint8_t es_flag = (temp & 0x10) >> 4;
        uint8_t dms_trick_mode_flag = (temp & 0x08) >> 3;
        uint8_t additional_copy_info_flag = (temp & 0x04) >> 2;
        uint8_t crc_flag = (temp & 0x02) >> 1;
        uint8_t extension_flag = (temp & 0x01);

        if (mVerbose) {
          std::cout << "  PTS present : "
                    << ((packet->mPTSDTSIndicator & 0x2) ? "YES" : "NO")
                    << std::endl;
          std::cout << "  DTS present : "
                    << ((packet->mPTSDTSIndicator & 0x1) ? "YES" : "NO")
                    << std::endl;
        }

        uint8_t pes_header_raimaining_bytes = *data;
        data++;

        // Store the current data pointer
        uint8_t *tempData = data;

        // Skip over the remaining of the PES header, will use tempData to
        // finish parsing
        data += pes_header_raimaining_bytes;

        if (mVerbose) {
          std::cout << "   Remaining bytes = "
                    << (int)pes_header_raimaining_bytes << std::endl;
        }

        // If PTS present
        if (packet->mPTSDTSIndicator & 0x2) {
          uint8_t high = *tempData;
          tempData++;
          uint16_t mid = (*tempData << 8) + (*(tempData + 1));
          tempData += 2;
          uint16_t low = (*tempData << 8) + (*(tempData + 1));
          tempData += 2;

          // std::cout << "    -> HIGH : " << std::hex << (int)high <<
          // std::endl;  std::cout << "    -> MID : " << std::hex << (int)mid <<
          // std::endl;  std::cout << "    -> LOW : " << std::hex << (int)low <<
          // std::endl;  std::cout << std::dec << std::endl;

          packet->mPTS = ((uint64_t(high) & 0x0e) >> 1) << 30;
          packet->mPTS |= ((uint64_t(mid) & 0xfffe) >> 1) << 15;
          packet->mPTS |= ((uint64_t(low) & 0xfffe) >> 1);
        }
        // If DTS present
        if (packet->mPTSDTSIndicator & 0x1) {
          uint8_t high = *tempData;
          tempData++;
          uint16_t mid = (*tempData << 8) + (*(tempData + 1));
          tempData += 2;
          uint16_t low = (*tempData << 8) + (*(tempData + 1));
          tempData += 2;

          packet->mDTS = ((uint64_t(high) & 0x0e) >> 1) << 30;
          packet->mDTS |= ((uint64_t(mid) & 0xfffe) >> 1) << 15;
          packet->mDTS |= ((uint64_t(low) & 0xfffe) >> 1);
        }

        if (mVerbose) {
          if (packet->mPTSDTSIndicator & 0x2) {
            std::cout << "   PTS : " << packet->mPTS << std::endl;
          }
          if (packet->mPTSDTSIndicator & 0x1) {
            std::cout << "   DTS : " << packet->mDTS << std::endl;
          }
        }
        if (outputPESPacketStart) {
          std::cout << "Size of packet payload : "
                    << (unsigned int)packet->mSize - (data - packet->mData)
                    << std::endl;
          if (mVerbose) {
            outputHex(data, (packet->mSize - (data - packet->mData)) > 128
                                ? 128
                                : (packet->mSize - (data - packet->mData)));
          }
        }
      }
    }

    return rval;
  }
};

}  // namespace MP2TS

#endif /* MP2TS_HPP_ */
