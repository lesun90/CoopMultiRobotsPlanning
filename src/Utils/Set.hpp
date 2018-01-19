#ifndef MP_SET_HPP_
#define MP_SET_HPP_

#include "Utils/HashFn.hpp"
#include <unordered_set>

#define IntSet std::unordered_set<int, HashStruct<int> >	
#define UseSet(Key)  std::unordered_set<Key, HashStruct<Key> >

#endif
    
    
    
    







