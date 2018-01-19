#ifndef MP_HASH_FN_HPP_
#define MP_HASH_FN_HPP_

#include "Utils/Definitions.hpp"
#include <cstring>
#include <vector>
#include <utility>
#include <string>
namespace MP
{
  size_t StringHash(const char *s, const int n);

  template <class Key_t> struct HashStruct { };


  template<> struct HashStruct<const std::string>
  {size_t operator()(const std::string &s) const { return StringHash(s.c_str(), s.size());}};

  template<> struct HashStruct<std::string>
  {size_t operator()(std::string &s) const { return StringHash(s.c_str(), s.size());}};

  template<> struct HashStruct<char*>
  {size_t operator()(const char* s) const { return s ? StringHash(s, strlen(s)) : 0;}};

  template<> struct HashStruct<const char*>
  {size_t operator()(const char* s) const { return s ? StringHash(s, strlen(s)) : 0;}};

  template<> struct HashStruct<char>{size_t operator()(char x) const { return StringHash(&x, sizeof(char));}};

  template<> struct HashStruct<unsigned char>
  {size_t operator()(unsigned char x) const { return StringHash((const char *)(&x), sizeof(x));}};

  template<> struct HashStruct<signed char>
  {size_t operator()(unsigned char x) const { return StringHash((const char *)(&x), sizeof(x));}};

  template<> struct HashStruct<short>
  {size_t operator()(short x) const { return StringHash((const char *)(&x), sizeof(x));}};

  template<> struct HashStruct<unsigned short>
  {size_t operator()(unsigned short x) const { return StringHash((const char *)(&x), sizeof(x));}};

  template<> struct HashStruct<int>
  {size_t operator()(int x) const { return StringHash((const char *)(&x), sizeof(x));}};

  template<> struct HashStruct<unsigned int>
  {size_t operator()(unsigned int x) const { return StringHash((const char *)(&x), sizeof(x));}};

  template<> struct HashStruct<long>
  {size_t operator()(long x) const { return StringHash((const char *)(&x), sizeof(x));}};

  template<> struct HashStruct<unsigned long>
  {size_t operator()(unsigned long x) const { return StringHash((const char *)(&x), sizeof(x));}};


  template<> struct HashStruct< std::pair<int, int> >
  {size_t operator()(std::pair<int, int> x) const { return StringHash((const char *)(&x), sizeof(x));}};
    template<> struct HashStruct< std::vector<int> >
    {
      size_t operator()(std::vector<int> x) const
      {
        return StringHash((const char *)(&((x)[0])),x.size());
      }
    };

}

#endif
