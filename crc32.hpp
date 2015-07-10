/*
 * crc32.hpp
 *
 *  Created on: Jul 3, 2015
 *      Author: luc.martel
 */

#ifndef CRC32_HPP_
#define CRC32_HPP_

#include <cstdint>

template <int IN, int OUT>
class CRC32 {
public:
   CRC32(uint32_t poly) {
      mTable = new uint32_t [256];
      CreateTable(poly);
   };

   ~CRC32() {
      delete [] mTable;
   }

   uint32_t operator()(unsigned char *buffer, unsigned int length) {
       uint32_t i_crc = IN;
       for(unsigned int i = 0; i < length;i++) {

          i_crc = (i_crc << 8) ^ mTable[((i_crc >> 24) ^ buffer[i]) & 0xff];
       }

       return i_crc ^ OUT;
   }

private:
   uint32_t *mTable;

   void CreateTable(uint32_t poly) {
      uint32_t i, j, k;
      for(i = 0; i < 256; i++) {
         k = 0;
         for(j = (i << 24) | 0x800000; j != 0x80000000; j <<= 1) {
            k = (k << 1) ^ (((k ^ j) & 0x80000000) ? poly : 0);
         }
         mTable[i] = k;
      }
   }

   CRC32() {} // Disallowed
};


#endif /* CRC32_HPP_ */
