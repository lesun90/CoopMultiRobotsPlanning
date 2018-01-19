/*
  An implementation of APSP with radix heaps
  Author: Stefan Edelkamp, 2014
*/

#ifndef cBnB_hpp_
#define cBnB_hpp_

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

namespace cBNB
{
#define M 101    
    class State 
    {
    public:
	long h, g;
	int depth, city;
	bool used[M];
	State() 
	{
	    for(int i=0;i<M;i++) 
		used[i] = false;
	    g = h = 0L; depth = city = 0; 
	}

    };
    
    class Memory 
    {
    public:
	int top, max;
	State* old;
	Memory(int n) {
	    max = n*n; top = 0; old = new State[max+1];
	    //    for (int i=0;i<max+1;i++) old[i] = new State();
	}

	~Memory(void)
	{
	    delete[] old;
	}
	
    };
    
    class Pair 
    { 
    public:
	long score; 
	int* tour;
	Pair(int N) {
	    tour = new int[N+1];
	} 
	~Pair() { delete[] tour; } 
    };
    
    class Selective 
    {
    public:  
	int N;
	State* newState;
	Memory* stack;
	int* tour;
	long** dist;
	int** far;
	int* next;
	int start;
	State S;
	bool *used;
	long** cost;
	long** copy;
	long** mask;
	int* rowCover;
	int* colCover;
	int** assignment;
	int* in;
	int* out;
	int* closest;
	
	int* link;
	int** path;
	bool* cycle;
	bool* visited;
	
	int* c;           // color vector
	long** d;           // distance matrix
	int* moves;                 // static array for successor
	int maxColors;          // counting number of runs/rollouts
	long expansions;          // counting number of runs/rollouts

	~Selective();
	Selective(int k, int m, int* goal_color, long** goal_dist, long* start_dist);
	Pair* search(int h, int maxNrExpansions);
    
	int heuristic(int h, int g, int city, int depth) 
	{
	    return 0;
	}
	
	void print(int* tour);
   };

}

#endif
