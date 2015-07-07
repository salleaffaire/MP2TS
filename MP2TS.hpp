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

#define PES_PACKET_MAXSIZE 64 * 1024

namespace MP2TS {

const uint32_t PAT_PID = 0;
const uint32_t CAT_PID = 1;
const uint32_t TSDT_PID = 2;
const uint32_t IPMP_PID = 3;


// Errors
enum ErrorCode {
   NONE = 0,
   INCOMPLETE_PACKET = 0x100,
   INVALID_START_CODE = 0x101,
};

class PES_Packet {
public:
   uint8_t  mData[PES_PACKET_MAXSIZE];
   uint32_t mSize;

   // Forward protocol layering
   uint32_t mPID;
};

class Channel {
public:
   Channel() : mPMTPID(0) {}
   Channel(uint32_t pmtpid) : mPMTPID(pmtpid) {}

   uint32_t            mPMTPID;
   std::list<uint32_t> mVideoPID;
   std::list<uint32_t> mAudioPID;
};

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

class Demux : public BinaryInFile {
public:
   Demux(std::string filename) : BinaryInFile(filename),
                                 mVerbose(true), mCurrentPointer(0),
                                 mError(NONE), mEOF(false) {

   }

   bool Get(PES_Packet &packet) {
      bool rval = true;

      // PES Write Pointer
      uint8_t *write_pointer = packet.mData;
      uint32_t temp;

      // Find first TS packet with Start Indicator = 1
      while ((mData[0] == 0x47) && !(mData[1] & 0x40)) {
         // Skip to next packet
         mData += 188;
      }

      // Keep track of the number of TS packets per PES packet
      mNumberOfTSPackets = 0;

      bool packet_complete = false;
      mAdaptationFieldWasPresent = false;
      do {
         uint32_t start_pointer = mCurrentPointer;
         // If there is enough room fo ran extra TS packet
         // PS packet size = 188 bytes
         if ((mSize - mCurrentPointer) >= 188) {
            if (mData[mCurrentPointer++] == 0x47) {
               // Parsing The Tansport Stream Header (First 32 bits)
               // -----------------------------------------------------------------
               // Read PID
               temp = ((mData[mCurrentPointer]) << 8) | mData[mCurrentPointer+1];
               packet.mPID = mPID = temp & 0x1fff;
               mTEI = temp & 0x8000;
               mPayloadUnitStartIndicator = temp & 0x4000;
               mTransportPriority = temp & 0x2000;
               mCurrentPointer += 2;

               // Next 8 bits
               temp = mData[mCurrentPointer++];
               mScramblingControl = (temp & 0xC0) >> 6;
               mAdaptationFieldExist = (temp & 0x2) >> 5;
               mContainsPayload = (temp & 0x10) >> 4;
               mContinuityCounter = (temp & 0x0f);

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

               if (mContainsPayload) {
                  // Copy Payload into the write buffer
                  uint32_t bytes_left = 188 - (mCurrentPointer - start_pointer);
                  std::memcpy(write_pointer, &(mData[mCurrentPointer]), bytes_left);

                  // Advance both the read and write pointers by bytes_left
                  write_pointer += bytes_left;
                  mCurrentPointer += bytes_left;

                  //
               }

               // Test if next packet is the start of a new PES Packet
               if ((mData[mCurrentPointer+0] == 0x47) && (mData[mCurrentPointer+1] & 0x40)) {
                  packet_complete = true;
               }

               mNumberOfTSPackets++;
            }
            else {
               mError = INVALID_START_CODE;
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

   bool Parse(PES_Packet &packet) {
      bool rval = true;

      // Decode Program Association Table
      if (mPID == PAT_PID) {
         if (mVerbose) {
            std::cout << "Parsing PAT" << std::endl;
         }
         ParsePAT(packet);
      }
      // TODO: Conditional Access Table
      else if (mPID == CAT_PID) {

      }
      // TODO: Transport Stream Description Table
      else if (mPID == TSDT_PID) {

      }
      // TODO: IPMP Control Information Table
      else if (mPID == IPMP_PID) {

      }
      // Reserved - this should not happen
      else if ((mPID >= 0x4) && (mPID <= 0xf)) {

      }
      // DVB Metadata
      else if ((mPID >= 0x10) && (mPID <= 0x1f)) {

      }
      // Assigned by PMT
      else if (((mPID >= 0x20) && (mPID <= 0x1ffa))) {
         if (mVerbose) {
            std::cout << "Parsing Assigned PID" << std::endl;
         }
         ParseAssigned(packet);
      }
      // TODO: Used by DigiCypher 2/ATSC MGT  Metadata
      else if (mPID == 0x1ffb) {

      }
      // Assigned by PMT
      else if (((mPID >= 0x1ffc) && (mPID <= 0x1ffe))) {
         if (mVerbose) {
            std::cout << "Parsing Assigned PID" << std::endl;
         }
         ParseAssigned(packet);
      }
      // TODO: Null Packet
      else if (mPID == 0x1fff) {

      }
      return rval;
   }

   bool ParsePAT(PES_Packet &packet) {
      bool rval = true;

      // Parser the PSI Header
      ParsePSIHeader(packet);

      // After the PSI Header the pointer needs to advance by
      // mFillerLength + 8
      uint8_t *data = packet.mData + mFillerLength + 8;

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
         mChannelMap.insert( std::map< uint32_t, Channel >::value_type ( program_number, Channel(pmt_pid & 0x1fff) ) );
         //mChannelMap[program_number] = Channel(pmt_pid);
      }

      return rval;
   }

   bool ParseAssigned(PES_Packet &packet) {
      bool rval = true;

      return rval;
   }

   uint32_t NumberOfTSPackets() {
      return mNumberOfTSPackets;
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

   std::map<uint32_t, Channel> mChannelMap;

   // CRC holder
   //
   uint8_t  *mCRCPointer;
   uint32_t mCRCLength;

   uint32_t mFillerLength;

   uint32_t mTableId;
   uint32_t mTableIdExtenxion;

   uint32_t mSectionNumber;
   uint32_t mLastSectionNumber;

   // Returns the number of bytes parsed
   uint32_t ParsePSIHeader(PES_Packet &packet) {
      uint8_t *data = packet.mData;

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

      uint32_t bytes_read = (data-packet.mData);
      return bytes_read;
   }
};


}

#endif /* MP2TS_HPP_ */
