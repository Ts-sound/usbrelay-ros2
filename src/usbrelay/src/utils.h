#pragma once 

#include <vector>
#include <string>
#include <sstream>

template <class T>
inline bool GetBit(const T &data, uint8_t pos) {
  return ((data >> pos) & 0x1);
}

template <typename V>
std::string ToString(std::vector<V> &vec)
{
  std::stringstream ss;
  ss << "[ ";
  for (auto &i : vec)
  {
    ss << i << " , ";
  }
  ss << " ]";

  return ss.str();
}
