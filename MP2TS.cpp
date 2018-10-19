/*
 * MP2TS.cpp
 *
 *  Created on: Jul 12, 2015
 *      Author: lmartel
 */

#include <string>
#include <vector>

#include "MP2TS.hpp"

// CRC32 lookup table for polynomial 0x04c11db7
/// ---------------------------------------------------------------------------------------
static uint32_t crc_table[256] = {
    0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9, 0x130476dc, 0x17c56b6b,
    0x1a864db2, 0x1e475005, 0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61,
    0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd, 0x4c11db70, 0x48d0c6c7,
    0x4593e01e, 0x4152fda9, 0x5f15adac, 0x5bd4b01b, 0x569796c2, 0x52568b75,
    0x6a1936c8, 0x6ed82b7f, 0x639b0da6, 0x675a1011, 0x791d4014, 0x7ddc5da3,
    0x709f7b7a, 0x745e66cd, 0x9823b6e0, 0x9ce2ab57, 0x91a18d8e, 0x95609039,
    0x8b27c03c, 0x8fe6dd8b, 0x82a5fb52, 0x8664e6e5, 0xbe2b5b58, 0xbaea46ef,
    0xb7a96036, 0xb3687d81, 0xad2f2d84, 0xa9ee3033, 0xa4ad16ea, 0xa06c0b5d,
    0xd4326d90, 0xd0f37027, 0xddb056fe, 0xd9714b49, 0xc7361b4c, 0xc3f706fb,
    0xceb42022, 0xca753d95, 0xf23a8028, 0xf6fb9d9f, 0xfbb8bb46, 0xff79a6f1,
    0xe13ef6f4, 0xe5ffeb43, 0xe8bccd9a, 0xec7dd02d, 0x34867077, 0x30476dc0,
    0x3d044b19, 0x39c556ae, 0x278206ab, 0x23431b1c, 0x2e003dc5, 0x2ac12072,
    0x128e9dcf, 0x164f8078, 0x1b0ca6a1, 0x1fcdbb16, 0x018aeb13, 0x054bf6a4,
    0x0808d07d, 0x0cc9cdca, 0x7897ab07, 0x7c56b6b0, 0x71159069, 0x75d48dde,
    0x6b93dddb, 0x6f52c06c, 0x6211e6b5, 0x66d0fb02, 0x5e9f46bf, 0x5a5e5b08,
    0x571d7dd1, 0x53dc6066, 0x4d9b3063, 0x495a2dd4, 0x44190b0d, 0x40d816ba,
    0xaca5c697, 0xa864db20, 0xa527fdf9, 0xa1e6e04e, 0xbfa1b04b, 0xbb60adfc,
    0xb6238b25, 0xb2e29692, 0x8aad2b2f, 0x8e6c3698, 0x832f1041, 0x87ee0df6,
    0x99a95df3, 0x9d684044, 0x902b669d, 0x94ea7b2a, 0xe0b41de7, 0xe4750050,
    0xe9362689, 0xedf73b3e, 0xf3b06b3b, 0xf771768c, 0xfa325055, 0xfef34de2,
    0xc6bcf05f, 0xc27dede8, 0xcf3ecb31, 0xcbffd686, 0xd5b88683, 0xd1799b34,
    0xdc3abded, 0xd8fba05a, 0x690ce0ee, 0x6dcdfd59, 0x608edb80, 0x644fc637,
    0x7a089632, 0x7ec98b85, 0x738aad5c, 0x774bb0eb, 0x4f040d56, 0x4bc510e1,
    0x46863638, 0x42472b8f, 0x5c007b8a, 0x58c1663d, 0x558240e4, 0x51435d53,
    0x251d3b9e, 0x21dc2629, 0x2c9f00f0, 0x285e1d47, 0x36194d42, 0x32d850f5,
    0x3f9b762c, 0x3b5a6b9b, 0x0315d626, 0x07d4cb91, 0x0a97ed48, 0x0e56f0ff,
    0x1011a0fa, 0x14d0bd4d, 0x19939b94, 0x1d528623, 0xf12f560e, 0xf5ee4bb9,
    0xf8ad6d60, 0xfc6c70d7, 0xe22b20d2, 0xe6ea3d65, 0xeba91bbc, 0xef68060b,
    0xd727bbb6, 0xd3e6a601, 0xdea580d8, 0xda649d6f, 0xc423cd6a, 0xc0e2d0dd,
    0xcda1f604, 0xc960ebb3, 0xbd3e8d7e, 0xb9ff90c9, 0xb4bcb610, 0xb07daba7,
    0xae3afba2, 0xaafbe615, 0xa7b8c0cc, 0xa379dd7b, 0x9b3660c6, 0x9ff77d71,
    0x92b45ba8, 0x9675461f, 0x8832161a, 0x8cf30bad, 0x81b02d74, 0x857130c3,
    0x5d8a9099, 0x594b8d2e, 0x5408abf7, 0x50c9b640, 0x4e8ee645, 0x4a4ffbf2,
    0x470cdd2b, 0x43cdc09c, 0x7b827d21, 0x7f436096, 0x7200464f, 0x76c15bf8,
    0x68860bfd, 0x6c47164a, 0x61043093, 0x65c52d24, 0x119b4be9, 0x155a565e,
    0x18197087, 0x1cd86d30, 0x029f3d35, 0x065e2082, 0x0b1d065b, 0x0fdc1bec,
    0x3793a651, 0x3352bbe6, 0x3e119d3f, 0x3ad08088, 0x2497d08d, 0x2056cd3a,
    0x2d15ebe3, 0x29d4f654, 0xc5a92679, 0xc1683bce, 0xcc2b1d17, 0xc8ea00a0,
    0xd6ad50a5, 0xd26c4d12, 0xdf2f6bcb, 0xdbee767c, 0xe3a1cbc1, 0xe760d676,
    0xea23f0af, 0xeee2ed18, 0xf0a5bd1d, 0xf464a0aa, 0xf9278673, 0xfde69bc4,
    0x89b8fd09, 0x8d79e0be, 0x803ac667, 0x84fbdbd0, 0x9abc8bd5, 0x9e7d9662,
    0x933eb0bb, 0x97ffad0c, 0xafb010b1, 0xab710d06, 0xa6322bdf, 0xa2f33668,
    0xbcb4666d, 0xb8757bda, 0xb5365d03, 0xb1f740b4};

uint32_t crc32(char *data, int len) {
  int i;
  uint32_t crc = 0xffffffff;

  for (i = 0; i < len; i++)
    crc = (crc << 8) ^ crc_table[((crc >> 24) ^ *data++) & 0xff];

  return crc;
}
/// ---------------------------------------------------------------------------------------

namespace MP2TS {

// MPEG-2 TS Multiplexer
// -----------------------------------------------------------------------------------------
Mux::Mux(std::string filename) : BinaryOutFile(filename) {}

bool Mux::Put(PES_Packet *packet) {
  bool rval = true;

  // Build a series of TS packets
  uint32_t bytes_left = packet->mSize;
  uint32_t pid = packet->mPID;
  do {
  } while (bytes_left);

  return rval;
}

bool Mux::BuildPMT(PES_Packet *packet) {
  bool rval = true;

  return rval;
}

bool Mux::BuildPAT(PES_Packet *packet) {
  bool rval = true;

  return rval;
}

bool Mux::AddChannel(uint32_t channel_number, uint32_t pmt_pid) {
  bool rval = true;

  // Add the channel to the data structures
  mPMTPIDtoChannelNumber[pmt_pid] = channel_number;
  mChannelMap[channel_number] = Channel(pmt_pid);

  return rval;
}

bool Mux::AddStreamToChannel(uint32_t channel_number, uint32_t pid,
                             uint32_t stream_type) {
  bool rval = true;

  if (IsAChannel(channel_number)) {
    mChannelMap[channel_number].mStreams.push_back(Stream(stream_type, pid));
  } else {
    rval = false;
  }

  return rval;
}

bool Mux::IsAChannel(uint32_t channel_number) {
  return !(mChannelMap.find(channel_number) == mChannelMap.end());
}

// MPEG-2 TS Demultiplexer Main Class
// -----------------------------------------------------------------------------------------

Demux::Demux(std::string filename)
    : BinaryInFile(filename),
      mVerbose(true),
      mOutputPacket(false),
      mOutputTSPacket(false),
      mCurrentPointer(0),
      mError(ErrorCode::NONE),
      mEOF(false),
      mNumberOfTSPackets(0),
      mIsPATParsed(false) {}

Demux::Demux(std::vector<uint8_t> &data)
    : BinaryInFile(data),
      mVerbose(true),
      mOutputPacket(false),
      mOutputTSPacket(false),
      mCurrentPointer(0),
      mError(ErrorCode::NONE),
      mEOF(false),
      mNumberOfTSPackets(0),
      mIsPATParsed(false) {}

Demux::~Demux() {
  // Cleaning unused or unreturned packets
  for (auto packet : mPESPacketsMap) {
    delete packet.second;
  }
}

bool Demux::Get(PES_Packet *&returned_packet) {
  bool rval = true;

  // PES Write Pointer
  uint32_t temp;
  returned_packet = (PES_Packet *)0;

  // TODO :: This doesn't work at all. Actually, it's a bug
  //         Need to implement a proper sychronization method
  // -------------------------------------------------------------------------
  // Find first TS packet with Start Indicator = 1
  // while ((mData[0] == 0x47) && !(mData[1] & 0x40)) {
  //   // Skip to next packet
  //   mData += TS_PACKET_LENGTH;
  // }
  // -------------------------------------------------------------------------

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
            (temp & 0x20) >> 5;                 // Adaptation field control msb
        mContainsPayload = (temp & 0x10) >> 4;  // Adaptation field control lsb
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

          if (mAdaptationFieldLength > TS_PACKET_LENGTH - (4 + 1)) {
            if (mVerbose) {
              std::cout << "Found adaptation field of illegal length"
                        << std::endl;
              std::cout << "Reducing the length to 0 to prevent any issues"
                        << std::endl;
            }
            mAdaptationFieldLength = 0;
          }

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

    // Early exit for debugging only
    // --------------------------------------------------------------------------
    // if (mNumberOfTSPackets == 1) {
    //   mEOF = true;
    // }

    // Loop until either we reach the end of file or if we find a packet
  } while ((!mEOF) && (!packet_complete));

  rval = packet_complete;
  return rval;
}

bool Demux::GetDangling(PES_Packet *&returned_packet) {
  bool rval = true;
  if (mPESPacketsMap.size()) {
    returned_packet = mPESPacketsMap.begin()->second;
    mPESPacketsMap.erase(mPESPacketsMap.begin());
  } else {
    rval = false;
  }
  return rval;
}

uint32_t Demux::NumberOfTSPackets() { return mNumberOfTSPacketsPerPESPacket; }

uint32_t Demux::GetPID() { return mPID; }

bool Demux::IsEOF() { return mEOF; }

ErrorCode Demux::Error() { return mError; }

std::map<uint32_t, Channel> &Demux::GetChannelMap() { return mChannelMap; }

void Demux::SetVerbosity(bool x) { mVerbose = x; }

void Demux::SetOutputPacket(bool x) { mOutputPacket = x; }

void Demux::SetOutputTSPacket(bool x) { mOutputTSPacket = x; }

bool Demux::IsAPMTPID(uint32_t pid) {
  bool rval = false;
  for (auto channel : mChannelMap) {
    if (channel.second.mPMTPID == pid) {
      rval = true;
      break;
    }
  }
  return rval;
}

bool Demux::AsAPIDPacket(uint32_t pid) {
  return !(mPESPacketsMap.find(pid) == mPESPacketsMap.end());
}

bool Demux::Parse(PES_Packet *packet, bool outputPESPacketStart) {
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
      std::cout << "Parsing Assigned PID *********************************** 0x"
                << std::hex << std::setfill('0') << std::setw(6) << mPID
                << std::dec << std::endl;
    }
    ParseAssigned(packet, outputPESPacketStart);
  }
  // TODO: Used by DigiCypher 2/ATSC MGT Metadata
  else if (packet->mPID == 0x1ffb) {
  }
  // Assigned by PMT
  else if (((packet->mPID >= 0x1ffc) && (mPID <= 0x1ffe))) {
    if (mVerbose) {
      std::cout << "Parsing Assigned PID *********************************** 0x"
                << std::hex << std::setfill('0') << std::setw(6) << mPID
                << std::dec << std::endl;
    }
    ParseAssigned(packet, outputPESPacketStart);
  }
  // TODO: Null Packet
  else if (packet->mPID == 0x1fff) {
  }
  return rval;
}

bool Demux::ParsePAT(PES_Packet *packet) {
  bool rval = true;

  if (mVerbose) {
    std::cout << "Parsing PAT" << std::endl;
    outputHex(packet->mData, 184);
  }

  // Parser the PSI Header
  uint32_t bytes_read = ParsePSIHeader(packet);

  if (bytes_read) {
    // Check CRC
    uint32_t crc_calc = crc32((char *)mCRCPointer, mCRCLength);
    uint32_t crc_infile = 0;
    crc_infile |= (*(mCRCPointer + mCRCLength + 3) & 0xFF);
    crc_infile |= (*(mCRCPointer + mCRCLength + 2) & 0xFF) << 8;
    crc_infile |= (*(mCRCPointer + mCRCLength + 1) & 0xFF) << 16;
    crc_infile |= (*(mCRCPointer + mCRCLength + 0) & 0xFF) << 24;

    // std::cout << "PAT CRC32 Calculated = " << std::hex << crc_calc <<
    // std::endl; std::cout << "PAT CRC32 In File    = " << std::hex <<
    // crc_infile
    //           << std::endl
    //           << std::dec;

    if (crc_infile == crc_calc) {
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
      rval = true;
    } else {
      rval = false;
    }
  }
  return rval;
}

bool Demux::ParsePMT(PES_Packet *packet) {
  bool rval = false;

  if (mVerbose) {
    std::cout << "Parsing PMT" << std::endl;
  }

  // Parser the PSI Header
  uint32_t bytes_read = ParsePSIHeader(packet);

  if (bytes_read) {
    // Check CRC
    uint32_t crc_calc = crc32((char *)mCRCPointer, mCRCLength);
    uint32_t crc_infile = 0;
    crc_infile |= (*(mCRCPointer + mCRCLength + 3) & 0xFF);
    crc_infile |= (*(mCRCPointer + mCRCLength + 2) & 0xFF) << 8;
    crc_infile |= (*(mCRCPointer + mCRCLength + 1) & 0xFF) << 16;
    crc_infile |= (*(mCRCPointer + mCRCLength + 0) & 0xFF) << 24;

    // std::cout << "PMT CRC32 Calculated = " << std::hex << crc_calc <<
    // std::endl; std::cout << "PMT CRC32 In File    = " << std::hex <<
    // crc_infile
    //           << std::endl
    //           << std::dec;

    if (crc_infile == crc_calc) {
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
      // TODO :: This toooo simple - right now only expecting 1 video and 1
      // audio ES

      // Bytes left excluding the CRC is
      // Section Length - (5) Syntax PIS Header - (4) CRC - (4) Syntax PMT -
      // program_info_length Section Length = CRC Length + 1 Bytes Left = CRC
      // Length + 1 - 5 - 4 - 4 - program_info_length Bytes Left = CRC Length -
      // 12
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

      rval = true;
    } else {
      rval = false;
    }
  }

  return rval;
}

// Returns the number of bytes parsed
uint32_t Demux::ParsePSIHeader(PES_Packet *packet) {
  uint32_t rval = 0;

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

  if ((section_length > 1021) || (section_length < 5)) {
    rval = 0;
  } else {
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
    rval = (data - packet->mData);
  }

  return rval;
}

bool Demux::ParseAdaptationField(uint8_t *data, uint32_t size) {
  if (size >= 1) {
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

  if (size >= 6) {
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

  if (size >= 6) {
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

  if (size >= 1) {
    if (mSplicingPointFlag) {
      mSpliceCountDown = *data++;
      --size;
    }
  }

  // If adaptation field private data is present
  if (size >= 1) {
    if (mTransportPrivateDataFlag) {
      unsigned int transportPrivateDataLength = *data++;
      --size;

      if (transportPrivateDataLength > size) {
        if (mVerbose) {
          std::cout << "Found adaptation field private data of illegal length"
                    << std::endl;
          std::cout << "Reducing the length to 0 to prevent any issues"
                    << std::endl;
        }
        transportPrivateDataLength = 0;
      }

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
}

bool Demux::ParseAssigned(PES_Packet *packet, bool outputPESPacketStart) {
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
        std::cout << "  PES Packet length : " << pes_packet_length << std::endl;
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
      // Set the start of the PES packet payload
      packet->mDataPESPayload = data;

      if (mVerbose) {
        std::cout << "   Remaining bytes = " << (int)pes_header_raimaining_bytes
                  << std::endl;
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

std::vector<std::string> gStreamTypeName{
    /* 0x00 */ "Reserved",
    /* 0x01 */ "ISO/IEC 11172-2 (MPEG-1 Video)",
    /* 0x02 */ "ITU-T Rec. H.262 and ISO/IEC 13818-2",
    /* 0x03 */ "ISO/IEC 11172-3 (MPEG-1 audio)",
    /* 0x04 */ "ISO/IEC 13818-3 (MPEG-2 halved sample rate audio)",
    /* 0x05 */ "ITU-T Rec. H.222 and ISO/IEC 13818-1 (MPEG-2 tabled data)",
    /* 0x06 */ "ITU-T Rec. H.222 and ISO/IEC 13818-1 (MPEG-2 packetized data)",
    /* 0x07 */ "ISO/IEC 13522 (MHEG)",
    /* 0x08 */ "ITU-T Rec. H.222 and ISO/IEC 13818-1 ",
    /* 0x09 */ "ITU-T Rec. H.222 and ISO/IEC 13818-1/11172-1 auxiliary data",
    /* 0x0a */ "ISO/IEC 13818-6 DSM CC multiprotocol encapsulation",
    /* 0x0b */ "ISO/IEC 13818-6 DSM CC U-N messages",
    /* 0x0c */ "ISO/IEC 13818-6 DSM CC stream descriptors",
    /* 0x0d */ "ISO/IEC 13818-6 DSM CC tabled data",
    /* 0x0e */ "ISO/IEC 13818-1 auxiliary data",
    /* 0x0f */ "ISO/IEC 13818-7 ADTS AAC (MPEG-2 lower bit-rate audio)",
    /* 0x10 */ "ISO/IEC 14496-2 (MPEG-4 H.263 based video)",
    /* 0x11 */ "ISO/IEC 14496-3 (MPEG-4 LOAS multi-format framed audio)",
    /* 0x12 */ "ISO/IEC 14496-1 (MPEG-4 FlexMux) in packetized stream",
    /* 0x13 */ "ISO/IEC 14496-1 (MPEG-4 FlexMux) in ISO/IEC 14496 tables",
    /* 0x14 */ "ISO/IEC 13818-6 DSM CC synchronized download protocol",
    /* 0x15 */ "Packetized metadata",
    /* 0x16 */ "Sectioned metadata",
    /* 0x17 */ "ISO/IEC 13818-6 DSM CC Data Carousel metadata",
    /* 0x18 */ "ISO/IEC 13818-6 DSM CC Object Carousel metadata",
    /* 0x19 */ "ISO/IEC 13818-6 Synchronized Download Protocol metadata",
    /* 0x1a */ "ISO/IEC 13818-11 IPMP",
    /* 0x1b */ "ITU-T Rec. H.264 and ISO/IEC 14496-10 (lower bit-rate video)",
    /* 0x1c */ "Warning: Unknown or Reserved Stream Type",
    /* 0x1d */ "Warning: Unknown or Reserved Stream Type",
    /* 0x1e */ "Warning: Unknown or Reserved Stream Type",
    /* 0x1f */ "Warning: Unknown or Reserved Stream Type",
    /* 0x20 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x21 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x22 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x23 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x24 */ "ITU-T Rec. H.265 and ISO/IEC 23008-2 (Ultra HD video)",
    /* 0x25 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x26 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x27 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x28 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x29 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x2a */ "Warning: Unknown or Reserved Stream Type",
    /* 0x2b */ "Warning: Unknown or Reserved Stream Type",
    /* 0x2c */ "Warning: Unknown or Reserved Stream Type",
    /* 0x2d */ "Warning: Unknown or Reserved Stream Type",
    /* 0x2e */ "Warning: Unknown or Reserved Stream Type",
    /* 0x2f */ "Warning: Unknown or Reserved Stream Type",
    /* 0x30 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x31 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x32 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x33 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x34 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x35 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x36 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x37 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x38 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x39 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x3a */ "Warning: Unknown or Reserved Stream Type",
    /* 0x3b */ "Warning: Unknown or Reserved Stream Type",
    /* 0x3c */ "Warning: Unknown or Reserved Stream Type",
    /* 0x3d */ "Warning: Unknown or Reserved Stream Type",
    /* 0x3e */ "Warning: Unknown or Reserved Stream Type",
    /* 0x3f */ "Warning: Unknown or Reserved Stream Type",
    /* 0x40 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x41 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x42 */ "Chinese Video Standard",
    /* 0x43 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x44 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x45 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x46 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x47 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x48 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x49 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x4a */ "Warning: Unknown or Reserved Stream Type",
    /* 0x4b */ "Warning: Unknown or Reserved Stream Type",
    /* 0x4c */ "Warning: Unknown or Reserved Stream Type",
    /* 0x4d */ "Warning: Unknown or Reserved Stream Type",
    /* 0x4e */ "Warning: Unknown or Reserved Stream Type",
    /* 0x4f */ "Warning: Unknown or Reserved Stream Type",
    /* 0x50 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x51 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x52 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x53 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x54 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x55 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x56 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x57 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x58 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x59 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x5a */ "Warning: Unknown or Reserved Stream Type",
    /* 0x5b */ "Warning: Unknown or Reserved Stream Type",
    /* 0x5c */ "Warning: Unknown or Reserved Stream Type",
    /* 0x5d */ "Warning: Unknown or Reserved Stream Type",
    /* 0x5e */ "Warning: Unknown or Reserved Stream Type",
    /* 0x5f */ "Warning: Unknown or Reserved Stream Type",
    /* 0x60 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x61 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x62 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x63 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x64 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x65 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x66 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x67 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x68 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x69 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x6a */ "Warning: Unknown or Reserved Stream Type",
    /* 0x6b */ "Warning: Unknown or Reserved Stream Type",
    /* 0x6c */ "Warning: Unknown or Reserved Stream Type",
    /* 0x6d */ "Warning: Unknown or Reserved Stream Type",
    /* 0x6e */ "Warning: Unknown or Reserved Stream Type",
    /* 0x6f */ "Warning: Unknown or Reserved Stream Type",
    /* 0x70 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x71 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x72 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x73 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x74 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x75 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x76 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x77 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x78 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x79 */ "Warning: Unknown or Reserved Stream Type",
    /* 0x7a */ "Warning: Unknown or Reserved Stream Type",
    /* 0x7b */ "Warning: Unknown or Reserved Stream Type",
    /* 0x7c */ "Warning: Unknown or Reserved Stream Type",
    /* 0x7d */ "Warning: Unknown or Reserved Stream Type",
    /* 0x7e */ "Warning: Unknown or Reserved Stream Type",
    /* 0x7f */ "Warning: Unknown or Reserved Stream Type",
    /* 0x80 */
    "ITU-T Rec. H.262 and ISO/IEC 13818-2 for DigiCipher II or PCM audio for "
    "Blu-ray",
    /* 0x81 */ "Dolby Digital up to six channel audio for ATSC and Blu-ray",
    /* 0x82 */ "SCTE subtitle or DTS 6 channel audio for Blu-ray",
    /* 0x83 */ "Dolby TrueHD lossless audio for Blu-ray",
    /* 0x84 */ "Dolby Digital Plus up to 16 channel audio for Blu-ray",
    /* 0x85 */ "DTS 8 channel audio for Blu-ray",
    /* 0x86 */ "DTS 8 channel lossless audio for Blu-ray",
    /* 0x87 */ "Dolby Digital Plus up to 16 channel audio for ATSC",
    /* 0x88 */ "Warning: Privately defined Stream Id",
    /* 0x89 */ "Warning: Privately defined Stream Id",
    /* 0x8a */ "Warning: Privately defined Stream Id",
    /* 0x8b */ "Warning: Privately defined Stream Id",
    /* 0x8c */ "Warning: Privately defined Stream Id",
    /* 0x8d */ "Warning: Privately defined Stream Id",
    /* 0x8e */ "Warning: Privately defined Stream Id",
    /* 0x8f */ "Warning: Privately defined Stream Id",
    /* 0x90 */ "Blu-ray Presentation Graphic Stream (subtitling)",
    /* 0x91 */ "Warning: Privately defined Stream Id",
    /* 0x92 */ "Warning: Privately defined Stream Id",
    /* 0x93 */ "Warning: Privately defined Stream Id",
    /* 0x94 */ "Warning: Privately defined Stream Id",
    /* 0x95 */ "ATSC DSM CC Network Resources table",
    /* 0x96 */ "Warning: Privately defined Stream Id",
    /* 0x97 */ "Warning: Privately defined Stream Id",
    /* 0x98 */ "Warning: Privately defined Stream Id",
    /* 0x99 */ "Warning: Privately defined Stream Id",
    /* 0x9a */ "Warning: Privately defined Stream Id",
    /* 0x9b */ "Warning: Privately defined Stream Id",
    /* 0x9c */ "Warning: Privately defined Stream Id",
    /* 0x9d */ "Warning: Privately defined Stream Id",
    /* 0x9e */ "Warning: Privately defined Stream Id",
    /* 0x9f */ "Warning: Privately defined Stream Id",
    /* 0xa0 */ "Warning: Privately defined Stream Id",
    /* 0xa1 */ "Warning: Privately defined Stream Id",
    /* 0xa2 */ "Warning: Privately defined Stream Id",
    /* 0xa3 */ "Warning: Privately defined Stream Id",
    /* 0xa4 */ "Warning: Privately defined Stream Id",
    /* 0xa5 */ "Warning: Privately defined Stream Id",
    /* 0xa6 */ "Warning: Privately defined Stream Id",
    /* 0xa7 */ "Warning: Privately defined Stream Id",
    /* 0xa8 */ "Warning: Privately defined Stream Id",
    /* 0xa9 */ "Warning: Privately defined Stream Id",
    /* 0xaa */ "Warning: Privately defined Stream Id",
    /* 0xab */ "Warning: Privately defined Stream Id",
    /* 0xac */ "Warning: Privately defined Stream Id",
    /* 0xad */ "Warning: Privately defined Stream Id",
    /* 0xae */ "Warning: Privately defined Stream Id",
    /* 0xaf */ "Warning: Privately defined Stream Id",
    /* 0xb0 */ "Warning: Privately defined Stream Id",
    /* 0xb1 */ "Warning: Privately defined Stream Id",
    /* 0xb2 */ "Warning: Privately defined Stream Id",
    /* 0xb3 */ "Warning: Privately defined Stream Id",
    /* 0xb4 */ "Warning: Privately defined Stream Id",
    /* 0xb5 */ "Warning: Privately defined Stream Id",
    /* 0xb6 */ "Warning: Privately defined Stream Id",
    /* 0xb7 */ "Warning: Privately defined Stream Id",
    /* 0xb8 */ "Warning: Privately defined Stream Id",
    /* 0xb9 */ "Warning: Privately defined Stream Id",
    /* 0xba */ "Warning: Privately defined Stream Id",
    /* 0xbb */ "Warning: Privately defined Stream Id",
    /* 0xbc */ "Warning: Privately defined Stream Id",
    /* 0xbd */ "Warning: Privately defined Stream Id",
    /* 0xbe */ "Warning: Privately defined Stream Id",
    /* 0xbf */ "Warning: Privately defined Stream Id",
    /* 0xc0 */ "DigiCipher II text",
    /* 0xc1 */ "Warning: Privately defined Stream Id",
    /* 0xc2 */ "ATSC DSM CC synchronous data",
    /* 0xc3 */ "Warning: Privately defined Stream Id",
    /* 0xc4 */ "Warning: Privately defined Stream Id",
    /* 0xc5 */ "Warning: Privately defined Stream Id",
    /* 0xc6 */ "Warning: Privately defined Stream Id",
    /* 0xc7 */ "Warning: Privately defined Stream Id",
    /* 0xc8 */ "Warning: Privately defined Stream Id",
    /* 0xc9 */ "Warning: Privately defined Stream Id",
    /* 0xca */ "Warning: Privately defined Stream Id",
    /* 0xcb */ "Warning: Privately defined Stream Id",
    /* 0xcc */ "Warning: Privately defined Stream Id",
    /* 0xcd */ "Warning: Privately defined Stream Id",
    /* 0xce */ "Warning: Privately defined Stream Id",
    /* 0xcf */ "Warning: Privately defined Stream Id",
    /* 0xd0 */ "Warning: Privately defined Stream Id",
    /* 0xd1 */ "BBC Dirac (Ultra HD video)",
    /* 0xd2 */ "Warning: Privately defined Stream Id",
    /* 0xd3 */ "Warning: Privately defined Stream Id",
    /* 0xd4 */ "Warning: Privately defined Stream Id",
    /* 0xd5 */ "Warning: Privately defined Stream Id",
    /* 0xd6 */ "Warning: Privately defined Stream Id",
    /* 0xd7 */ "Warning: Privately defined Stream Id",
    /* 0xd8 */ "Warning: Privately defined Stream Id",
    /* 0xd9 */ "Warning: Privately defined Stream Id",
    /* 0xda */ "Warning: Privately defined Stream Id",
    /* 0xdb */ "Warning: Privately defined Stream Id",
    /* 0xdc */ "Warning: Privately defined Stream Id",
    /* 0xdd */ "Warning: Privately defined Stream Id",
    /* 0xde */ "Warning: Privately defined Stream Id",
    /* 0xdf */ "Warning: Privately defined Stream Id",
    /* 0xe0 */ "Warning: Privately defined Stream Id",
    /* 0xe1 */ "Warning: Privately defined Stream Id",
    /* 0xe2 */ "Warning: Privately defined Stream Id",
    /* 0xe3 */ "Warning: Privately defined Stream Id",
    /* 0xe4 */ "Warning: Privately defined Stream Id",
    /* 0xe5 */ "Warning: Privately defined Stream Id",
    /* 0xe6 */ "Warning: Privately defined Stream Id",
    /* 0xe7 */ "Warning: Privately defined Stream Id",
    /* 0xe8 */ "Warning: Privately defined Stream Id",
    /* 0xe9 */ "Warning: Privately defined Stream Id",
    /* 0xea */ "Microsoft Windows Media Video 9 (lower bit-rate video)",
    /* 0xeb */ "Warning: Privately defined Stream Id",
    /* 0xec */ "Warning: Privately defined Stream Id",
    /* 0xed */ "Warning: Privately defined Stream Id",
    /* 0xee */ "Warning: Privately defined Stream Id",
    /* 0xef */ "Warning: Privately defined Stream Id",
    /* 0xf0 */ "Warning: Privately defined Stream Id",
    /* 0xf1 */ "Warning: Privately defined Stream Id",
    /* 0xf2 */ "Warning: Privately defined Stream Id",
    /* 0xf3 */ "Warning: Privately defined Stream Id",
    /* 0xf4 */ "Warning: Privately defined Stream Id",
    /* 0xf5 */ "Warning: Privately defined Stream Id",
    /* 0xf6 */ "Warning: Privately defined Stream Id",
    /* 0xf7 */ "Warning: Privately defined Stream Id",
    /* 0xf8 */ "Warning: Privately defined Stream Id",
    /* 0xf9 */ "Warning: Privately defined Stream Id",
    /* 0xfa */ "Warning: Privately defined Stream Id",
    /* 0xfb */ "Warning: Privately defined Stream Id",
    /* 0xfc */ "Warning: Privately defined Stream Id",
    /* 0xfd */ "Warning: Privately defined Stream Id",
    /* 0xfe */ "Warning: Privately defined Stream Id",
    /* 0xff */ "Warning: Privately defined Stream Id"

};
}  // namespace MP2TS
