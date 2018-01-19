/*
  An implementation of APSP with radix heaps
  Author: Stefan Edelkamp, 2014
*/

#ifndef MC_hpp_
#define MC_hpp_

#include <vector>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cfloat>
#include <climits>
#include <limits>
#include <cmath>
#include <cassert>
#include <string>
#include <algorithm>
#include "Utils/BnB.hpp"

namespace MC
{

#define LEVEL 2

class MC {
public:  
  int* visits;     
  int* tour;       
  int tourSize;
  int N;

  long** d;           // distance matrix
  int* moves;                 // static array for successor
  double* value;              // static array for roulette wheel selection
  double** global;          // global tour policy 
  double*** backup; // backup of global tour
  long evaluations;          // counting number of runs/rollouts

    MC(int k, long** goal_dist, long* start_dist);

    ~MC(void);
    
    
    bool check (int i) {
	return !visits[i];
    }
   
    long rollout();

    void adapt(int* sample, int level);

    void print(int* tour);

    Pair* search(int level);
};

 
class Link {
public:
  int succid;
  long weight;
  Link* next;
  Link(int n, long w, Link* p) {
    next = p;
    succid = n;
    weight = w;
  }
};

class Node {
public: 
  enum Label {
    unlabelled, labelled, scanned
  };
  Label state; 
  Link* edges; 
  long element;
  int bucket;
  Node* pred;
  Node* succ;
  long x;
  long y;

  Node(long xcoord, long ycoord) {
    x = xcoord; 
    y = ycoord;
    state = unlabelled;
    pred = succ = 0;
    element = 0;
    edges = 0;
  }
  Node(long v) {
    state = unlabelled;
    pred = succ = 0;
    element = v;
    edges = 0;
  }
};

    
}

#endif
