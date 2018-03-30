#ifndef MEDIA_ELEMENTS_HPP___
#define MEDIA_ELEMENTS_HPP___

#include <list>
#include <string>

// Misc
// -----------------------------------------------------------------------------------------
extern std::vector<std::string> gStreamTypeName;

std::string &GetStreamType(uint8_t st) { return gStreamTypeName[st]; }

enum PESPacketState { OK = 0x00, CONTINUITY_ERROR = 0x01 };

// Media Classes
// -----------------------------------------------------------------------------------------
class PES_Packet {
 public:
  PES_Packet()
      : mSize(0),
        mContinuityCounter(0),
        mState(0),
        mStreamId(0),
        mPTSDTSIndicator(0),
        mPID(0) {
    mData = mWritePointer = new uint8_t[PES_PACKET_MAXSIZE];
  }
  ~PES_Packet() {
    if (mData) delete[] mData;
  }

  bool AsAPTS() { return mPTSDTSIndicator & 0x2; }
  bool AsADTS() { return mPTSDTSIndicator & 0x1; }

  // Points to the beginning
  uint8_t *mData;
  // Points to the end (where to write)
  uint8_t *mWritePointer;

  // Current size in bytes, result of parsing
  uint32_t mSize;
  uint8_t mContinuityCounter;  // used when parsing PES to keep last CC
  uint32_t mState;

  // Stream Identifier
  uint8_t mStreamId;

  // Timestamps
  uint8_t mPTSDTSIndicator;
  uint64_t mPTS;
  uint64_t mDTS;

  // Forward protocol layering
  uint32_t mPID;
};

class Stream {
 public:
  Stream(uint32_t type, uint32_t pid) : mType(type), mPID(pid) {}

  uint32_t mType;
  uint32_t mPID;

  std::string &GetStreamTypeName() { return gStreamTypeName[mType]; }
};

class Channel {
 public:
  Channel() : mPMTPID(0) {}
  Channel(uint32_t pmtpid) : mPMTPID(pmtpid) {}

  uint32_t mPMTPID;
  std::list<Stream> mStreams;
};
#endif
