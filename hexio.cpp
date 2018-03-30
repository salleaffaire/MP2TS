#include "hexio.hpp"
#include <iomanip>
#include <iostream>

void outputHex(unsigned char *p, unsigned int n) {
  unsigned int bytesLeft = n;
  while (bytesLeft) {
    for (unsigned int i = 0; (i < 32) && bytesLeft; ++i, --bytesLeft, ++p) {
      std::cout << std::hex << std::setfill('0') << std::setw(2)
                << (unsigned int)*p << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::dec;
}