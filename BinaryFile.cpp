/*
 * BinaryFile.cpp
 *
 *  Created on: Jul 3, 2015
 *      Author: luc.martel
 */


#include "BinaryFile.hpp"
#include <fstream>

BinaryFile::BinaryFile(std::string filename) : self(*this), mFileName(filename),
                                   mData((uint8_t *)0), mSize(0) {
   Open();
}

BinaryFile::~BinaryFile() {
   if (mData) {
      delete [] mData;
   }
}

size_t BinaryFile::GetSize() {
   return mSize;
}

uint8_t *BinaryFile::GetDataPointer() {
   return mData;
}

uint8_t &BinaryFile::operator[](uint32_t index) {
   return *(mData+index);
}

void BinaryFile::Save(std::string filename) {
   std::ofstream os;
   os.open(filename.c_str(), std::ios::binary);

   os.write((char *)mData, mSize);

   os.close();
}

void BinaryFile::Open() {

   if (mData) {
      delete [] mData;
   }

   std::ifstream is;
   is.open(mFileName.c_str(), std::ios::binary|std::ios::ate);

   mSize = is.tellg();
   is.seekg(0, std::ios::beg);

   mData = new uint8_t [mSize];

   is.read((char *)mData, mSize);

   is.close();
}
