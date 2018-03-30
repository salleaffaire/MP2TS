#ifndef SAFE_BUFFER_HPP___
#define SAFE_BUFFER_HPP___

#include <algorithm>
#include <cstring>

template <class T>
class SafeBuffer {
 public:
  // Constructors
  // --------------------------------------------------------
  SafeBuffer() : mData() {
    mData = (T *)0;
    mSize = 0;
  }
  SafeBuffer(unsigned int size) : mSize(size) {
    mData = (mSize) ? new T[mSize] : (T *)0;
  }
  // Destructors
  // ---------------------------------------------------------
  virtual ~SafeBuffer() {
    if (mData) {
      delete[] mData;
    }
  }

  // Copy Constructor
  // ---------------------------------------------------------
  SafeBuffer(const SafeBuffer<T> &x) : mSize(x.mSize) {
    if (mSize != 0) {
      mData = new T[mSize];
      std::memcpy(mData, x.mData, mSize);
    } else {
      mData = (T *)0;
    }
  }

  // Move constructor
  // ---------------------------------------------------------
  SafeBuffer(SafeBuffer<T> &&x) {
    mData = x.mData;
    x.mData = (T *)0;
  }

  // Assignment
  // If the assignment is called with a lvalue the copy constructor will be
  // called
  // If the assignment is called with a rvalue the move constructire will be
  // called
  // ---------------------------------------------------------
  SafeBuffer<T> &operator=(const SafeBuffer<T> x) {
    std::swap(mSize, x.mSize);
    std::swap(mData, x.mData);
    return *this;
  }

  const T &operator[](const uint32_t pos) { return mData[pos]; }
  T &&operator[](const uint32_t pos) { return mData[pos]; }

 private:
  T *mData;
  uint32_t mSize;
};

#endif