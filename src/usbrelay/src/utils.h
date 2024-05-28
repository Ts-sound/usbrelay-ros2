#pragma once 

#include <vector>
#include <string>
#include <sstream>


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
