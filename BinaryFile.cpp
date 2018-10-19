/*
 * BinaryFile.cpp
 *
 *  Created on: Jul 3, 2015
 *      Author: luc.martel
 */

#include "binaryfile.hpp"
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>

BinaryInFile::BinaryInFile(std::string filename)
    : self(*this), mFileName(filename), mData((uint8_t *)0), mSize(0) {
  Open();
}

BinaryInFile::BinaryInFile(std::vector<uint8_t> &data)
    : self(*this), mFileName(""), mData((uint8_t *)0), mSize(0) {
  mData = new uint8_t[data.size()];
  mSize = data.size();

  // Copy the data
  // TODO: Optimize later - remove the copy
  std::memcpy(mData, data.data(), mSize);
}

BinaryInFile::~BinaryInFile() {
  if (mData) {
    delete[] mData;
  }
}

size_t BinaryInFile::GetSize() { return mSize; }

uint8_t *BinaryInFile::GetDataPointer() { return mData; }

uint8_t &BinaryInFile::operator[](uint32_t index) { return *(mData + index); }

void BinaryInFile::Save(std::string filename) {
  std::ofstream os;
  os.open(filename.c_str(), std::ios::binary);

  os.write((char *)mData, mSize);

  os.close();
}

void BinaryInFile::Open() {
  if (mData) {
    delete[] mData;
  }

  std::ifstream is;
  is.open(mFileName.c_str(), std::ios::binary | std::ios::ate);

  mSize = is.tellg();
  is.seekg(0, std::ios::beg);

  mData = new uint8_t[mSize];

  is.read((char *)mData, mSize);

  is.close();
}

BinaryOutFile::BinaryOutFile(std::string filename)
    : mFileName(filename),
      mData(0),
      mSize(0),
      mBufferSize(OUT_FILE_BUFFER_SIZE),
      mPosition(0) {
  // Allocate interbal buffer
  mData = new uint8_t[OUT_FILE_BUFFER_SIZE];

  // Open the file
  mOS.open(mFileName.c_str(), std::ios::binary);
}

BinaryOutFile::~BinaryOutFile() {
  if (mData) delete[] mData;
}

bool BinaryOutFile::Write(uint8_t *data, size_t size) {
  bool rval = true;

  uint32_t bytes_left_in_buffer = OUT_FILE_BUFFER_SIZE - mPosition;

  if (size > bytes_left_in_buffer) {
    // First, writes the buffer to file
    FlushBuffer();

    // Write direct to file
    mOS.write((char *)data, size);
  } else {
    std::memcpy(mData + mPosition, data, size);
    mPosition += size;

    if (mPosition == mBufferSize) {
      FlushBuffer();
    }
  }

  return rval;
}

void BinaryOutFile::Close() {
  FlushBuffer();
  mPosition = 0;
  mOS.close();
}

void BinaryOutFile::FlushBuffer() {
  mOS.write((char *)mData, mPosition);
  mPosition = 0;
}
