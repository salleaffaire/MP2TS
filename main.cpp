/*
 * main.cpp
 *
 *  Created on: Jun 30, 2015
 *      Author: luc.martel
 */

#include <fstream>
#include <iostream>
#include <iomanip>

#include "crc32.hpp"
#include "BinaryFile.hpp"
#include "MP2TS.hpp"

// --------------                 START                  ---------------
// ----------------------------------------------------------------------
// Set these to the PID values for PMT, PCR, Video and Audio respectively
//

#define LM_PMT_PID 32
#define LM_PCR_PID 33
#define LM_VIDEO_PID 33
#define LM_AUDIO_PID 34


// MPEG-2 CRC32
// ------------------------------------------------------------------------------------------------------
// width=32 poly=0x04c11db7 init=0xffffffff refin=false refout=false xorout=0x00000000 check=0x0376e6e7 name="CRC-32/MPEG-2"
// See: http://reveng.sourceforge.net/crc-catalogue/all.htm

static CRC32<0xffffffff, 0x00000000> gCRC(0x04c11db7);

// TODO: This implementation is extremly simple and assumes that PMT and PATs are small enough to fit in 1
// 188-byte TS packet. Give that the nmber of channels in the stream has to be 1 and that streams only
// have 1 video and 1 audio stream, this assumtion should hold.
//
// Next version will properly demux the TS layer and remultiplex the updates PSI tables.
// -------------------------------------------------------------------------------------------------------

void lm_parse_PSI_header(unsigned char *&payload,
                         unsigned char *&crc_payload,
                         unsigned int &crc,
                         unsigned int &crc_length) {


   // Parse Header
   // Table ID
   unsigned int table_id = *payload++;
   //std::cout << "Table ID = " << table_id << std::endl;

   // Section Length
   unsigned int section_length = 0;
   section_length |= *payload << 8;
   payload++;
   section_length |= *payload;
   payload++;
   section_length &= 0x3FF; // 10-bits
   //std::cout << "Section length = " << section_length << std::endl;

   // Calculate the CRC Length
   // Section Length is the number of syntaxt bytes including the CRC
   // (3) Table Header Length
   // (section_length) = Syntax Length including CRC
   // (4) CRC Length
   // crc_length = 3 + section_length - 4;
   crc_length = section_length - 1;

   // Calculate current CRC
   //crc = calculate_crc32(crc_payload, crc_length, 0xffffffff);
   //std::cout << "PAT CRC = " << std::hex << std::setfill('0') << std::setw(8) << crc << std::endl;
   //std::cout << std::dec;

   // Table ID Extension
   unsigned int table_id_extension = 0;
   table_id_extension |= *payload << 8;
   payload++;
   table_id_extension |= *payload;
   payload++;
   //std::cout << "Table ID Extension = " << table_id_extension << std::endl;

   // Skip (Reserved bits (0b11), Version Number, Current/Next indicator)
   payload++;

   unsigned int section_number = *payload++;
   //std::cout << "Section Number = " << section_number << std::endl;

   unsigned int last_section_number = *payload++;
   //std::cout << "Last Section Number = " << last_section_number << std::endl;
}

/**
 @brief Convert PMT, PCR, Video and Audio PIDs from a 1 channel TS stream.
 @param data pointer to a buffer containing a TS segment
 @param size si of the TS segment in bytes
                      |

 Convert PMT, PCR, Video and Audio PIDs from a 1 channel TS stream.

 PMT PID will be converted to LM_PMT_PID
 PCR PID will be converted to LM_PCR_PID
 Video PID will be converted to LM_VIDEO_PID
 Audio PID will be converted to LM_AUDIO_PID

 See #define above
 Will make it better later.

*/

void lm_pid_remux(unsigned char *data, unsigned int size) {
   const unsigned int ts_packet_length = 188;
   unsigned int error_count = 0;
   unsigned int ts_count = 0;

   while (size >= ts_packet_length) {
      unsigned int pid_in = 0;
      unsigned int pid_out = 0;

      // Test start code
      if (data[0] == 0x47) {
         ++ts_count;

         // Extract PID
         pid_in = pid_out = ((data[1]) << 8) | data[2];
         pid_in &= 0x1fff;
         pid_out &= 0xE000;

         //std::cout << "Found PID = " << pid_in << std::endl;

         unsigned char *crc_payload;
         unsigned int crc = 0;
         unsigned int crc_length = 0;
         unsigned char *payload = data+4;

         switch (pid_in) {
            // Found PAT
            case 0x00:
            {
               //std::cout << "Found PAT" << std::endl;

               // Pointer field
               unsigned int pointer_field = *payload;
               payload++;
               //Skip pointer field bytes
               payload += pointer_field;
               crc_payload = payload;

               lm_parse_PSI_header(payload, crc_payload, crc, crc_length);

               // TABLE DATA (PAT)
               // ------------------------------------------------------------------------

               unsigned int program_number = 0;
               program_number |= *payload << 8;
               payload++;
               program_number |= *payload;
               payload++;
               //std::cout << "Program Number = " << program_number << std::endl;

               unsigned int pmt_pid = 0;
               pmt_pid |= *payload << 8;
               payload++;
               pmt_pid |= *payload;
               payload++;
               //std::cout << "PMT_PID= " << (pmt_pid & 0x1fff) << std::endl;

               // Rewritig the PID
               pmt_pid = 0xE000 | LM_PMT_PID;
               *(payload-1) = pmt_pid & 0xff;
               *(payload-2) = (pmt_pid & 0xff00) >> 8;

               // Update the CRC
               // CRC will be located at data[5+crc_length]
               crc = gCRC(crc_payload, crc_length);
               data[5+crc_length+0] = (crc >> 24) & 0xFF;
               data[5+crc_length+1] = (crc >> 16) & 0xFF;
               data[5+crc_length+2] = (crc >> 8)  & 0xFF;
               data[5+crc_length+3] = (crc)       & 0xFF;
            }
            break;
            // Found PMT (4095)
            case 0xFFF:
            {
               // Create new PID
               pid_out |= (LM_PMT_PID);

               // Write it back
               data[1] = (pid_out & 0xFF00) >> 8;
               data[2] = (pid_out & 0xFF);

               //std::cout << "Found PMT" << std::endl;
               // Pointer field
               unsigned int pointer_field = *payload++;
               // Skip pointer field bytes
               payload += pointer_field;
               crc_payload = payload;

               // Parse PAT Header
               lm_parse_PSI_header(payload, crc_payload, crc, crc_length);

               // PMT TABLE
               // ------------------------------------------------------------------------
               unsigned int pcr_pid = 0;
               pcr_pid |= *payload << 8;
               payload++;
               pcr_pid |= *payload;
               payload++;
               //std::cout << "PRC PID = " << (pcr_pid & 0x1fff) << std::endl;

               // Rewrite it
               // Rewritig the PID
               pcr_pid = 0xE000 | LM_PCR_PID;
               *(payload-1) = pcr_pid & 0xff;
               *(payload-2) = (pcr_pid & 0xff00) >> 8;

               // Skip Reserved and Program Info Length
               payload += 2;

               // For all stream
               // TODO :: This toooo simple - right now only expecting 1 video and 1 audio ES

               for (int i=0;i<2;i++) {
                  unsigned int stream_type = 0;
                  stream_type = *payload;
                  payload++;

                  //std::cout << "Stream Type : = "  << stream_type << std::endl;

                  unsigned int pid = 0;
                  pid |= *payload << 8;
                  payload++;
                  pid |= *payload;
                  payload++;

                  switch (stream_type) {
                  case 0x1B: // Video
                     {
                        //std::cout << "Video PID = " << (pid & 0x1fff) << std::endl;

                        // Rewritig the PID
                        pid = 0xE000 | LM_VIDEO_PID;
                        *(payload-1) = pid & 0xff;
                        *(payload-2) = (pid & 0xff00) >> 8;

                     }
                     break;
                  case 0x03: // Audio
                     {
                        //std::cout << "Audio PID = " << (pid & 0x1fff) << std::endl;

                        // Rewritig the PID
                        pid = 0xE000 | LM_AUDIO_PID;
                        *(payload-1) = pid & 0xff;
                        *(payload-2) = (pid & 0xff00) >> 8;
                     }
                     break;
                  }

                  // Skip to next
                  unsigned int elementary_stream_descriptor_length = 0;
                  elementary_stream_descriptor_length |= *payload << 8;
                  payload++;
                  elementary_stream_descriptor_length |= *payload;
                  payload++;

                  elementary_stream_descriptor_length &= 0x3ff;
                  payload += elementary_stream_descriptor_length;

                  // Update the CRC
                  // CRC will be located at data[5+crc_length]
                  crc = gCRC(crc_payload, crc_length);
                  data[5+crc_length+0] = (crc >> 24) & 0xFF;
                  data[5+crc_length+1] = (crc >> 16) & 0xFF;
                  data[5+crc_length+2] = (crc >> 8)  & 0xFF;
                  data[5+crc_length+3] = (crc)       & 0xFF;
               }
            }
            break;
            // Found Video or PCR (256)
            case 0x100:
            {
               // Create new PID
               pid_out |= (LM_VIDEO_PID);

               // Write it back
               data[1] = (pid_out & 0xFF00) >> 8;
               data[2] = (pid_out & 0xFF);
            }
            break;
            // Found Audio (257)
            case 0x101:
            {
               // Create new PID
               pid_out |= (LM_AUDIO_PID);

               // Write it back
               data[1] = (pid_out & 0xFF00) >> 8;
               data[2] = (pid_out & 0xFF);
            }
            break;
         }
      }
      else {
         // Error
         ++error_count;

      }

      size -= ts_packet_length;
      data += ts_packet_length;
   }

   //std::cout << "Error count = " << error_count << std::endl;
}

// --------------                 END                    ---------------

int
main(int argc, char *argv[])
{

#if 0
   BinaryFile ts_file("Stream1-2.ts");

   lm_pid_remux(ts_file.GetDataPointer(), ts_file.GetSize());

   ts_file.Save("output.ts");
#endif


#if 1
   MP2TS::PES_Packet packet;
   //MP2TS::Demux tsdemux("Stream1-2.ts");
   MP2TS::Demux tsdemux("decode_test_background_20120726_480p-1M.ts");

   unsigned int total_ts_packets = 0;
   while (!tsdemux.IsEOF()) {
      tsdemux.Get(packet);
      if (tsdemux.Error() == MP2TS::NONE) {
         std::cout << "PES Packet of " << tsdemux.NumberOfTSPackets() << " TS packets - PID = "
                   << tsdemux.GetPID() << std::endl;
         total_ts_packets += tsdemux.NumberOfTSPackets();
         std::cout << "Total = " << total_ts_packets << std::endl;

         // Parse PES Packet
         tsdemux.Parse(packet);
      }
      else {
         std::cout << tsdemux.Error() << std::endl;
         break;
      }
   }

   // Output channel map
   for (auto &channel: tsdemux.GetChannelMap()) {
      std::cout << channel.first << " @ PID "<< channel.second.mPMTPID << std::endl;
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

