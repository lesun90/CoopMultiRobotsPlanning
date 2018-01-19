/*
  An implementation of APSP with radix heaps
  Author: Stefan Edelkamp, 2014
*/

#ifndef BnB_hpp_
#define BnB_hpp_

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

namespace BnB
{
    class State 
    {
    public:
	long h, g;
	int depth, city;
	unsigned long used;
	State() { used = 0L; g = h = 0L; depth = city = 0; }
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
    };
    
    
    class Selective 
    {
    public:  
	int N;
	State* newState;
	Memory* stack;
	int* tour;
	long** dist;
	long *cmin;
	
	int** far;
	int* next;
	int start;
	State S;
	unsigned long used;
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
	
	long** d;           // distance matrix
	int* moves;                 // static array for successor
	long expansions;          // counting number of runs/rollouts
	
	Selective(int k, long** goal_dist, long* start_dist);
	~Selective(void);
	
	Pair* search(int h, const int maxNrSteps);
	int heuristic(int h, int g, int city, int depth);
	void print(int* tour);
	long hg (long g, int city, int depth);
	long computeoffset(int depth);

	int column(int g, int city, int depth);
	int incheuristic(int h, int g, int hval, int city, int depth);
	
    };
    
    
    class Link 
    {
    public:
	int succid;
	long weight;
	Link* next;
	Link(int n, long w, Link* p) 
	{
	    next = p;
	    succid = n;
	    weight = w;
	}
    };
    
    class Node 
    {
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
	
	Node(long xcoord, long ycoord) 
	{
	    x = xcoord; 
	    y = ycoord;
	    state = unlabelled;
	    pred = succ = 0;
	    element = 0;
	    edges = 0;
	}
	Node(long v) 
	{
	    state = unlabelled;
	    pred = succ = 0;
	    element = v;
	    edges = 0;
	}
    };
    
    class Radix {
    public:
	long S;
	int B;
	long n; // number of nodes    
	Node** buckets;   
	long* u;   
	long* b;   
	
	Radix() ;
	
	long size() 
	{
	    return n;
	}
	bool empty() 
	{
	    return (n == 0);
	}
	Node* next(Node* p);
	
	void insert_node(Node* p, int i);
	
	void extract_node(Node* p);
	
	void adjust(long m, int t);
	
	int find(Node* p, int i);
	
	Node* top() 
	{
	    return buckets[0];
	}
  
	Node* insert(Node* p);
	
	
	void decrease(Node* x, long k);
	
	Node* extract() ;
	
	Node* extract(Node* x);
	
    };
    
    class Graph 
    {
    public:
	std::vector<Node*> nodes;  
	std::vector<int> goals;  
	int start;
	Graph(int n, int k);
    };

    void dijkstra(Node* s, std::vector<Node*> N);
    
    
}

#endif
