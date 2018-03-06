/*
 * main.cpp
 *
 *  Created on: Jun 30, 2015
 *      Author: luc.martel
 */

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>

#include "BinaryFile.hpp"
#include "MP2TS.hpp"

// Command Line Argument parser of the poor
// --------------------------------------------------------------------------
char* getCmdOption(char** begin, char** end, const std::string& option) {
  char** itr = std::find(begin, end, option);
  if (itr != end && ++itr != end) {
    return *itr;
  }
  return 0;
}

bool cmdOptionExists(char** begin, char** end, const std::string& option) {
  return std::find(begin, end, option) != end;
}
// --------------------------------------------------------------------------

int main(int argc, char* argv[]) {
  bool verbose = false;
  bool outputChannelMap = false;

  char* filename = getCmdOption(argv, argv + argc, "-f");
  // Parse command line arguments
  if (cmdOptionExists(argv, argv + argc, "-v")) {
    verbose = true;
  }
  if (cmdOptionExists(argv, argv + argc, "-m")) {
    outputChannelMap = true;
  }

  std::list<uint64_t> PTSList;

  // If filename exists
  if (filename) {
    // Create the MPEG-2 TS parser
    MP2TS::Demux tsdemux(filename);

    // Make is quiet
    tsdemux.SetVerbosity(verbose);

    // Processing Loop
    std::cout << "PES Packets Processing Loop" << std::endl;
    std::cout << "-----------------------------------------------------"
              << std::endl;
    unsigned int total_ts_packets = 0;

    // The behavior fo the demux is that each time ::Get(...) is called, a
    // pointer to a PES_PAcket is returned and the responsibility of destroying
    // the returned pointer is the caller's, not the Demux.
    // ---------------
    // It is a bit of an anti-pattern but that is the way it works for now.
    // Also, a PES packet will be returned only when a new TS starting payload
    // is encountered for the same PID. This means that, when the end of file is
    // reached, there are still unprocessed packets in the demuxer. The should
    // be accessed by ::GetDangling(...)

    // Read the file and parse all packets
    // ----------------------------------------------------------------
    MP2TS::PES_Packet* packet;
    while (!tsdemux.IsEOF()) {
      tsdemux.Get(packet);
      if (tsdemux.Error() == MP2TS::ErrorCode::NONE) {
        // std::cout << tsdemux.NumberOfTSPackets() << " TS packets have been
        // read" << std::endl;
        total_ts_packets += tsdemux.NumberOfTSPackets();
        // std::cout << "Total TS Packet Read = " << total_ts_packets <<
        // std::endl;

        if (packet) {
          std::cout << "Received a " << packet->mSize << " bytes packet."
                    << std::endl;
          std::cout << "Packet State : " << packet->mState << std::endl;
          tsdemux.Parse(packet);

          // Output PTS if present in the PES header
          if (packet->AsAPTS()) {
            std::cout << "PTS          : " << packet->mPTS << std::endl;
            PTSList.push_back(packet->mPTS);
          }

          std::cout << "PID          : " << packet->mPID << std::endl;
          std::cout << std::endl;

          delete packet;
        }
      } else {
        std::cout << "Error !!" << std::endl;
        break;
      }
    }

    // At the end, take care of the dangling packets
    // ----------------------------------------------------------------
    // Cleaning up dangling PES packets
    while (tsdemux.GetDangling(packet)) {
      std::cout << " -- Dangling -- Received a " << packet->mSize
                << " bytes packet." << std::endl;
      std::cout << "Packet State : " << packet->mState << std::endl;
      tsdemux.Parse(packet);

      // Output PTS if present in the PES header
      if (packet->AsAPTS()) {
        std::cout << "PTS          : " << packet->mPTS << std::endl;
        PTSList.push_back(packet->mPTS);
      }

      std::cout << "PID          : " << packet->mPID << std::endl;
      std::cout << std::endl;

      delete packet;
    }

    // Output channel map
    // ----------------------------------------------------------------
    if (outputChannelMap) {
      std::cout << "Channel Map" << std::endl;
      std::cout << "-----------------------------------------------------"
                << std::endl;
      for (auto& channel : tsdemux.GetChannelMap()) {
        std::cout << channel.first << " @ PID " << channel.second.mPMTPID
                  << std::endl;
        std::cout << "  Channel PMT PDI = " << std::hex << std::setfill('0')
                  << std::setw(4) << channel.second.mPMTPID << std::dec
                  << std::endl;

        unsigned int stream_num = 0;
        for (auto& stream : channel.second.mStreams) {
          std::cout << "    " << stream_num << ": Stream Type 0x" << std::hex
                    << std::setfill('0') << std::setw(4)
                    << stream.GetStreamTypeName() << std::dec << std::endl;
          std::cout << "    " << stream_num << ": Stream PID  0x" << std::hex
                    << std::setfill('0') << std::setw(4) << stream.mPID
                    << std::dec << std::endl;
          ++stream_num;
        }
      }
    }
  } else {
    std::cout << "Usage : ts-parser -f <filaname> [-v]" << std::endl;
    std::cout << "        -f <name of the .ts file to parse>" << std::endl;
    std::cout << "        -v if present, runs the parser in verbose mode"
              << std::endl;
  }

#if 0
   unsigned char crc_buffer[] = {0x0,0xb0,0x0d,0x00,0x01,0xc1,0x0,0x0,0x0,0x1,0xe0,0x20};
   //unsigned char crc_buffer[] = {0x0,0xb0,0x0d,0x59,0x81,0xeb,0x0,0x0,0x0,0x1,0xe0,0x42};

   // Calculate current CRC
   unsigned long crc = calculate_crc32(crc_buffer, 12);
   std::cout << "PAT CRC = " << std::hex << std::setfill('0') << std::setw(8) << crc << std::endl;
   std::cout << std::dec;
#endif
}
