/*
 * BinaryFile.hpp
 *
 *  Created on: Jul 3, 2015
 *      Author: luc.martel
 */

#ifndef BINARYFILE_HPP_
#define BINARYFILE_HPP_

#include <fstream>
#include <string>

#define OUT_FILE_BUFFER_SIZE (64*1024)

class BinaryInFile {
public:
   BinaryInFile(std::string filename);

   virtual ~BinaryInFile();

   size_t GetSize();

   uint8_t *GetDataPointer();

   uint8_t &operator[](uint32_t index);

   void Save(std::string filename);

protected:

   BinaryInFile &self;

   void Open();

   std::string mFileName;
   uint8_t *mData;
   size_t mSize;
};

class BinaryOutFile {
public:
   BinaryOutFile(std::string filename);

   virtual ~BinaryOutFile();

   bool Write(uint8_t *data, size_t size);
   void Close();

protected:

   std::string mFileName;
   uint8_t *mData;
   size_t   mSize;
   uint32_t mBufferSize;
   uint32_t mPosition;

   std::ofstream mOS;

   void FlushBuffer();
};

#endif /* BINARYFILE_HPP_ */
