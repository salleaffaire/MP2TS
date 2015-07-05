/*
 * BinaryFile.hpp
 *
 *  Created on: Jul 3, 2015
 *      Author: luc.martel
 */

#ifndef BINARYFILE_HPP_
#define BINARYFILE_HPP_

#include <string>

class BinaryFile {
public:
   BinaryFile(std::string filename);

   virtual ~BinaryFile();

   size_t GetSize();

   uint8_t *GetDataPointer();

   uint8_t &operator[](uint32_t index);

   void Save(std::string filename);

protected:

   BinaryFile &self;

   void Open();

   std::string mFileName;
   uint8_t *mData;
   size_t mSize;
};



#endif /* BINARYFILE_HPP_ */
