/*
 * MP2TS.hpp
 *
 *  Created on: Jul 3, 2015
 *      Author: luc.martel
 */

#ifndef MP2TS_HPP_
#define MP2TS_HPP_

#include <cstdint>
#include <cstring>
#include <list>
#include <map>
#include <algorithm>
#include <memory>

#include "BinaryFile.hpp"

#define PES_PACKET_MAXSIZE 2048 * 1024

namespace MP2TS {

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

enum PESPacketState {
   OK = 0x00,
   CONTINUITY_ERROR = 0x01
};

// Media Classes
// -----------------------------------------------------------------------------------------
class PES_Packet {
public:
   PES_Packet() : mSize(0), mContinuityCounter(0), mState(0), mPID(0) {
      mData = mWritePointer = new uint8_t [PES_PACKET_MAXSIZE];

   }
   ~PES_Packet() {
      if (mData) delete [] mData;
   }
   uint8_t *mData;
   uint8_t *mWritePointer;
   uint32_t mSize;
   uint8_t  mContinuityCounter;
   uint32_t mState;

   // Forward protocol layering
   uint32_t mPID;
};

class Stream {
public:
   Stream(uint32_t type, uint32_t pid) : mType(type),
                                         mPID(pid) {

   }

   uint32_t mType;
   uint32_t mPID;
};

class Channel {
public:
   Channel() : mPMTPID(0) {}
   Channel(uint32_t pmtpid) : mPMTPID(pmtpid) {}

   uint32_t            mPMTPID;
   std::list<Stream>   mStreams;

};

// MPEG-2 TS Multiplexer Main Class
// -----------------------------------------------------------------------------------------
class Mux : public BinaryOutFile {
public:
   Mux(std::string filename) : BinaryOutFile(filename) {

   }

   bool Put(PES_Packet &) {
      bool rval = true;



      return rval;
   }

private:
};

// MPEG-2 TS Demultiplexer Main Class
// -----------------------------------------------------------------------------------------
class Demux : public BinaryInFile {
public:
   Demux(std::string filename) : BinaryInFile(filename),
                                 mVerbose(true), mCurrentPointer(0),
                                 mError(ErrorCode::NONE), mEOF(false), mNumberOfTSPackets(0) {

   }

   ~Demux() {
      // Cleaning unused or unreturned packets
      for (auto packet: mPESPacketsMap) {
         delete packet.second;
      }
   }

   bool Get(PES_Packet *&returned_packet) {
      bool rval = true;

      // PES Write Pointer
      uint32_t temp;
      returned_packet = (PES_Packet *)0;

      // Find first TS packet with Start Indicator = 1
      while ((mData[0] == 0x47) && !(mData[1] & 0x40)) {
         // Skip to next packet
         mData += 188;
      }

      // Keep track of the number of TS packets per PES packet
      mNumberOfTSPacketsPerPESPacket = 0;

      bool packet_complete = false;
      mAdaptationFieldWasPresent = false;
      do {
         // Advance to the first sync byte 0x47
         uint32_t bytes_skipped = 0;
         while (((mSize - mCurrentPointer) > 0) && (mData[mCurrentPointer] != 0x47)) {
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
         if ((mSize - mCurrentPointer) >= 188) {
            if (mData[mCurrentPointer++] == 0x47) {
               // Parsing The Tansport Stream Header (First 32 bits)
               // -----------------------------------------------------------------
               // Read PID
               temp = ((mData[mCurrentPointer]) << 8) | mData[mCurrentPointer+1];
               mPID = temp & 0x1fff;
               mTEI = temp & 0x8000;
               mPayloadUnitStartIndicator = temp & 0x4000;
               mTransportPriority = temp & 0x2000;
               mCurrentPointer += 2;

               // Next 8 bits
               temp = mData[mCurrentPointer++];
               mScramblingControl = (temp & 0xC0) >> 6;
               mAdaptationFieldExist = (temp & 0x20) >> 5;
               mContainsPayload = (temp & 0x10) >> 4;
               mContinuityCounter = (temp & 0x0f);

               //std::cout << mPID << " @  TS Packet " <<  mNumberOfTSPackets << std::endl;

               // If Present - Parse the Adaptation Field
               // ----------------------------------------------------------------
               if (mAdaptationFieldExist) {
                  mAdaptationFieldWasPresent = true;
                  mAdaptationFieldLength = mData[mCurrentPointer++];

                  // Adaptation Field Length is the number of bytes in the AF following the
                  // Adaptation Field Length field
                  uint32_t starting_pointer = mCurrentPointer;

                  temp = mData[mCurrentPointer++];
                  mDiscontinuityIndicator = (temp & 0x80) >> 7;
                  mRandomAccessIndicator = (temp & 0x40) >> 6;
                  mElementaryStreamPriorityIndicator = (temp & 0x20) >> 5;
                  mPCRFlag = (temp & 0x10) >> 4;
                  mOPCRFlag = (temp & 0x08) >> 3;
                  mSplicingPointFlag = (temp & 0x04) >> 2;
                  mTransportPrivateDataFlag = (temp & 0x02) >> 1;
                  mAdaptationFieldExtensionFlag = (temp & 0x01);

                  if (mPCRFlag) {
                     mPCR = mData[mCurrentPointer++];                //
                     mPCR = (mPCR << 8) | mData[mCurrentPointer++];  //
                     mPCR = (mPCR << 8) | mData[mCurrentPointer++];  //
                     mPCR = (mPCR << 8) | mData[mCurrentPointer++];  //
                     mPCR = (mPCR << 8) | mData[mCurrentPointer++];  //
                     mPCR = (mPCR << 8) | mData[mCurrentPointer++];  //
                  }

                  if (mOPCRFlag) {
                     mOPCR = mData[mCurrentPointer++];                //
                     mOPCR = (mOPCR << 8) | mData[mCurrentPointer++];  //
                     mOPCR = (mOPCR << 8) | mData[mCurrentPointer++];  //
                     mOPCR = (mOPCR << 8) | mData[mCurrentPointer++];  //
                     mOPCR = (mOPCR << 8) | mData[mCurrentPointer++];  //
                     mOPCR = (mOPCR << 8) | mData[mCurrentPointer++];  //
                  }

                  if (mSplicingPointFlag) {
                     mSpliceCountDown = mData[mCurrentPointer++];
                  }

                  // Skip the stuffing bytes
                  uint32_t number_of_stuffing_bytes = mAdaptationFieldLength - (mCurrentPointer - starting_pointer);
                  mCurrentPointer += number_of_stuffing_bytes;
               }

               // Find a PES Packet to wriet the data into
               // --------------------------------------------------------------------
               PES_Packet *current_packet = (PES_Packet *)0;

               // If it is the start of a new payload
               if (mPayloadUnitStartIndicator) {
                  current_packet = new PES_Packet;
                  current_packet->mPID = mPID;
                  current_packet->mContinuityCounter = mContinuityCounter;
                  // If we already have a PES_Packet with the same PID
                  if (AsAPIDPacket(mPID)) {
                     packet_complete = true;
                     returned_packet = mPESPacketsMap[mPID];
                  }
                  mPESPacketsMap[mPID] = current_packet;
               }
               else {
                  if (AsAPIDPacket(mPID)) {
                     current_packet = mPESPacketsMap[mPID];
                  }
               }
               // --------------------------------------------------------------------

               uint32_t bytes_left = 188 - (mCurrentPointer - start_pointer);
               // If we have a current packet
               if ((current_packet) && (mContainsPayload)) {
                  // Check continuity counter
                  if (mContinuityCounter != current_packet->mContinuityCounter) {
                     current_packet->mState |= MP2TS::PESPacketState::CONTINUITY_ERROR;
                     if (mVerbose) {
                        std::cout << "Error: Invalid continuity counter" << std::endl;
                        std::cout << "  Expected : " << (int) current_packet->mContinuityCounter << " but found : "
                                  <<  mContinuityCounter << std::endl;
                     }
                  }
                  current_packet->mContinuityCounter = (current_packet->mContinuityCounter + 1) & 0xf;

                  uint8_t *write_pointer = (current_packet->mData)+(current_packet->mSize);
                  // Copy Payload into the write buffer
                  //std::cout << "Bytes left : " << bytes_left << " written." << std::endl;
                  //std::cout << "In a packet of size : " << current_packet->mSize << std::endl;
                  std::memcpy(write_pointer, &(mData[mCurrentPointer]), bytes_left);
                  current_packet->mSize += bytes_left;
               }
               mCurrentPointer += bytes_left;

               mNumberOfTSPacketsPerPESPacket++;
               mNumberOfTSPackets++;
            }
            else {
               mError = ErrorCode::INVALID_START_CODE;
               break;
            }
         }
         else {
            // This may also be an error
            // TODO: Verify that the file is a multiple of 188
            packet_complete = true;

            mEOF = true;
         }

      } while (!packet_complete);

      rval = packet_complete;
      return rval;
   }

   bool Parse(PES_Packet *packet) {
      bool rval = true;

      // Decode Program Association Table
      if (packet->mPID == PAT_PID) {
         if (mVerbose) {
            std::cout << "Parsing PAT" << std::endl;
         }
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
            std::cout << "Parsing Assigned PID *********************************** 0x"
                      << std::hex << std::setfill('0') << std::setw(6) << mPID << std::dec << std::endl;
         }
         ParseAssigned(packet);
      }
      // TODO: Used by DigiCypher 2/ATSC MGT  Metadata
      else if (packet->mPID == 0x1ffb) {

      }
      // Assigned by PMT
      else if (((packet->mPID >= 0x1ffc) && (mPID <= 0x1ffe))) {
         if (mVerbose) {
            std::cout << "Parsing Assigned PID *********************************** 0x"
                      << std::hex << std::setfill('0') << std::setw(6) << mPID << std::dec << std::endl;
         }
         ParseAssigned(packet);
      }
      // TODO: Null Packet
      else if (packet->mPID == 0x1fff) {

      }
      return rval;
   }



   bool ParsePAT(PES_Packet *packet) {
      bool rval = true;

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
      for (uint32_t i=0;i<PAT_entries;i++) {
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
         mChannelMap.insert( std::map< uint32_t, Channel >::value_type ( program_number, Channel(pmt_pid & 0x1fff) ) );
         //mChannelMap[program_number] = Channel(pmt_pid);
      }

      return rval;
   }

   bool ParsePMT(PES_Packet *packet) {
      bool rval = 0;

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
      //std::cout << "PRC PID = " << (pcr_pid & 0x1fff) << std::endl;

      uint32_t program_info_length = 0;
      program_info_length |= *data << 8;
      data++;
      program_info_length |= *data;
      data++;
      program_info_length &= 0x03ff;

      // Skip program descriptor bytes
      data += program_info_length;

      // For all stream
      // TODO :: This toooo simple - right now only expecting 1 video and 1 audio ES

      // Bytes left excluding the CRC is
      // Section Length - (5) Syntax PIS Header - (4) CRC - (4) Syntax PMT - program_info_length
      // Section Length = CRC Length + 1
      // Bytes Left = CRC Length + 1 - 5 - 4 - 4 - program_info_length
      // Bytes Left = CRC Length - 12 - program_info_length
      uint32_t bytes_left = mCRCLength - 12 - program_info_length;

      std::cout << "Bytes left in PMT ------------------------------ " << bytes_left << std::endl;

      Channel channel;
      channel.mPMTPID = packet->mPID;

      while (bytes_left > 0) {

         unsigned int stream_type = 0;
         stream_type = *data;
         data++;

         std::cout << "Stream Type : = "  << stream_type << std::endl;

         unsigned int pid = 0;
         pid |= *data << 8;
         data++;
         pid |= *data;
         data++;
         pid &= 0x1fff;

         std::cout << "PID = "  << pid << std::endl;
         channel.mStreams.push_back(std::move(Stream(stream_type, pid)));

         // Skip to next
         unsigned int elementary_stream_descriptor_length = 0;
         elementary_stream_descriptor_length |= *data << 8;
         data++;
         elementary_stream_descriptor_length |= *data;
         data++;

         elementary_stream_descriptor_length &= 0x3ff;
         data += elementary_stream_descriptor_length;

         bytes_left -= (5+elementary_stream_descriptor_length);
      }
      mChannelMap[mPMTPIDtoChannelNumber[channel.mPMTPID]] = std::move(channel);

      // Update the CRC
      // CRC will be located at data[5+crc_length]
      //crc = gCRC(crc_payload, crc_length);
      //data[5+crc_length+0] = (crc >> 24) & 0xFF;
      //data[5+crc_length+1] = (crc >> 16) & 0xFF;
      //data[5+crc_length+2] = (crc >> 8)  & 0xFF;
      //data[5+crc_length+3] = (crc)       & 0xFF;


      return rval;
   }

   bool ParseAssigned(PES_Packet *packet) {
      bool rval = true;

      if (IsAPMTPID(packet->mPID)) {
         ParsePMT(packet);
      }
      else {
         // Is there a PES start code?
         uint8_t *data = packet->mData;

         uint32_t start_code = 0;
         start_code = *data;
         start_code = (start_code << 8) | *(data+1);
         start_code = (start_code << 8) | *(data+2);

         //std::cout << "Start Code : 0x" << std::hex << std::setfill('0') << std::setw(6)
         //          << start_code << std::dec << std::endl;

         // Found a stream
         if (start_code == 0x000001) {
            // Skip start code
            data += 3;
            //std::cout << "Stream ID : 0x" << std::hex << std::setfill('0') << std::setw(2)
            //          << (int)*data << std::dec << std::endl;
         }
      }
      return rval;
   }

   uint32_t NumberOfTSPackets() {
      return mNumberOfTSPacketsPerPESPacket;
   }

   uint32_t GetPID() {
      return mPID;
   }

   bool IsEOF() {
      return mEOF;
   }

   ErrorCode Error() {
      return mError;
   }

   std::map<uint32_t, Channel> &GetChannelMap() {
      return mChannelMap;
   }

private:
   bool     mVerbose;

   uint32_t mCurrentPointer;

   ErrorCode mError;
   bool      mEOF;
   uint32_t  mNumberOfTSPackets;
   uint32_t  mNumberOfTSPacketsPerPESPacket;


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
   bool     mAdaptationFieldWasPresent;
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

   std::map<uint32_t, uint32_t> mPMTPIDtoChannelNumber;
   std::map<uint32_t, Channel> mChannelMap;

   bool IsAPMTPID(uint32_t pid) {
      bool rval = false;
      for (auto channel: mChannelMap) {
         if (channel.second.mPMTPID == pid) {
            rval = true;
            break;
         }
      }
      return rval;
   }

   // CRC holder
   //
   uint8_t  *mCRCPointer;
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
      mFillerLength = 1+pointer_field;

      // Set the start of the payload to include in the CRC calculation
      mCRCPointer = data;

      // Parse Header
      // Table ID
      mTableId = *data++;
      //std::cout << "Table ID = " << table_id << std::endl;

      // Section Length
      unsigned int section_length = 0;
      section_length |= *data << 8;
      data++;
      section_length |= *data;
      data++;
      section_length &= 0x3FF; // 10-bits
      //std::cout << "Section length = " << section_length << std::endl;

      // Calculate the CRC Length
      // Section Length is the number of syntaxt bytes including the CRC
      // (3) Table Header Length
      // (section_length) = Syntax Length including CRC
      // (4) CRC Length
      // crc_length = 3 + section_length - 4;
      mCRCLength = section_length - 1;

      // Calculate current CRC
      //crc = calculate_crc32(crc_payload, crc_length, 0xffffffff);
      //std::cout << "PAT CRC = " << std::hex << std::setfill('0') << std::setw(8) << crc << std::endl;
      //std::cout << std::dec;

      // Table ID Extension
      mTableIdExtenxion = 0;
      mTableIdExtenxion |= *data << 8;
      data++;
      mTableIdExtenxion |= *data;
      data++;
      //std::cout << "Table ID Extension = " << table_id_extension << std::endl;

      // Skip (Reserved bits (0b11), Version Number, Current/Next indicator)
      data++;

      mSectionNumber = *data++;
      //std::cout << "Section Number = " << section_number << std::endl;

      mLastSectionNumber = *data++;
      //std::cout << "Last Section Number = " << last_section_number << std::endl;

      uint32_t bytes_read = (data-packet->mData);
      return bytes_read;
   }
};


}

#endif /* MP2TS_HPP_ */
