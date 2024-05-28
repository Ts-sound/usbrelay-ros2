#pragma once

#include <vector>
#include <string>
#include <sstream>

template <class T>
inline bool GetBit(const T &data, uint8_t pos)
{
  return ((data >> pos) & 0x1);
}

std::string ToString(std::vector<uint8_t> &vec)
{
  std::stringstream ss;
  ss << "[ ";
  for (size_t i = 0; i < vec.size() - 1; i++)
  {
    ss << (int)vec[i] << " , ";
  }
  ss << (int)vec.back() << " ]";

  return ss.str();
}

template <typename V>
std::string ToString(std::vector<V> &vec)
{
  std::stringstream ss;
  ss << "[ ";
  for (size_t i = 0; i < vec.size() - 1; i++)
  {
    ss << vec[i] << " , ";
  }
  ss << vec.back() << " ]";

  return ss.str();
}
