#ifndef HUBOCAN_SRC_UTILS_HPP
#define HUBOCAN_SRC_UTILS_HPP

#include <stdint.h>

namespace HuboCan {

inline double doubleFromBytePair(uint8_t data0, uint8_t data1)
{
  unsigned int tmp = 0;
  tmp |= ( ( ((uint16_t)data0) << 8 ) & 0xFFFF );
  tmp |= ( ((uint16_t)data1)  & 0x00FF );

  return (double)( (int16_t)tmp );
}

} // namespace HuboCan

#endif // HUBOCAN_SRC_UTILS_HPP
