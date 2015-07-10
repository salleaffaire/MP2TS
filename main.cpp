/*
 * main.cpp
 *
 *  Created on: Jun 30, 2015
 *      Author: luc.martel
 */

#include <fstream>
#include <iostream>
#include <iomanip>

#include "BinaryFile.hpp"
#include "MP2TS.hpp"
#include "repid.hpp"

int
main(int argc, char *argv[])
{

#if 0
   BinaryInFile ts_file("Stream1-2.ts");

   lm_pid_remux(ts_file.GetDataPointer(), ts_file.GetSize());

   ts_file.Save("output.ts");
#endif


#if 1
   MP2TS::Demux tsdemux("Stream1-2.ts");
   //MP2TS::Demux tsdemux("football.ts");
   //MP2TS::Demux tsdemux("decode_test_background_20120726_480p-1M.ts");

   unsigned int total_ts_packets = 0;
   MP2TS::PES_Packet *packet;
   while (!tsdemux.IsEOF()) {
      tsdemux.Get(packet);
      if (tsdemux.Error() == MP2TS::ErrorCode::NONE) {
         //std::cout << tsdemux.NumberOfTSPackets() << " TS packets have been read" << std::endl;
         total_ts_packets += tsdemux.NumberOfTSPackets();
         //std::cout << "Total TS Packet Read = " << total_ts_packets << std::endl;

         if (packet) {
            std::cout << "Received a " << packet->mSize << " bytes packet." << std::endl;
            tsdemux.Parse(packet);
            delete packet;
         }
      }
      else {
         std::cout << "Error !!" << std::endl;
         break;
      }
   }

   // Output channel map
   for (auto &channel: tsdemux.GetChannelMap()) {
      std::cout << channel.first << " @ PID "<< channel.second.mPMTPID << std::endl;
      std::cout << "  Channel PMT PDI = " << std::hex << std::setfill('0') << std::setw(4)
                << channel.second.mPMTPID << std::dec << std::endl;

      for (auto &stream: channel.second.mStreams) {
         std::cout << "    Stream Type 0x" << std::hex << std::setfill('0') << std::setw(4) << stream.mType << std::dec << std::endl;
         std::cout << "    Stream PID  0x" << std::hex << std::setfill('0') << std::setw(4) << stream.mPID << std::dec << std::endl;
      }
   }

#endif

#if 0
   unsigned char crc_buffer[] = {0x0,0xb0,0x0d,0x00,0x01,0xc1,0x0,0x0,0x0,0x1,0xe0,0x20};
   //unsigned char crc_buffer[] = {0x0,0xb0,0x0d,0x59,0x81,0xeb,0x0,0x0,0x0,0x1,0xe0,0x42};

   // Calculate current CRC
   unsigned long crc = calculate_crc32(crc_buffer, 12);
   std::cout << "PAT CRC = " << std::hex << std::setfill('0') << std::setw(8) << crc << std::endl;
   std::cout << std::dec;
#endif
}

