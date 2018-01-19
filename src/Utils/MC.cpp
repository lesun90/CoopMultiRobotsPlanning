/*
  An implementation of APSP with radix heaps
  Author: Stefan Edelkamp, 2014
*/

#include "MC.hpp"

#include <vector>

using namespace std;

namespace MC
{
#define EXP(y) exp(y)
 
#define ITERATIONS 40
#define SHOW (LEVEL-1)
#define ALPHA 1


    MC::~MC(void)
    {
	for(int i = 0; i < N; ++i)
	{
	    delete[] d[i];
	    delete[] global[i];	 
	}

	for(int z = 0; z < LEVEL + 1; ++z)
	{
	    for(int i = 0; i < N; ++i)
		delete[] backup[z][i];
	    delete[] backup[z];
	}
	
	
	delete[] d;
	delete[] moves;
	delete[] value;
	delete[] global;
	delete[] backup;
    }

    
    MC::MC(int k, long** goal_dist, long* start_dist) {
	N = k+1;
	tour = new int[N+1];
	visits = new int[N+1];
	d = new long*[N];           // distance matrix
	moves = new int[N];           // static array for successor
	value = new double[N];          // static array for roulette wheel selection
	global = new double*[N];          // global tour policy 
	backup = new double**[LEVEL+1];      // global tour policy 
	for(int z = 0; z < LEVEL + 1; ++z)
	    backup[z] = new double*[N];
  	for(int i = 0; i < N; i++) { 
	    d[i] = new long[N];
	    global[i] = new double[N];
	    for(int z = 0; z < LEVEL + 1; ++z)
		backup[z][i] = new double[N];

    }
    evaluations = 0;          // counting number of runs/rollouts
    for(int i = 1; i < N; i++) 
      for(int j = 1; j < N; j++) 
      d[i][j] = goal_dist[i-1][j-1];
    for (int i=0;i<N;i++) 
      d[i][i] = 0;
    
    for (int i=1;i<N;i++) {
      d[0][i] = start_dist[i-1];
      d[i][0] = start_dist[i-1];
    }
    for(int i = 0; i < N; i++) 
      for(int j = 0; j < N; j++) 
	global[i][j] = 0.0;
  }

    
    long MC::rollout() {
	for (int j=1;j<N;j++) 
	    visits[j] = 0;
	visits[0] = 1;
	tour[0] = 0;
	tourSize = 1; // start node already visits
	int node = 0;
	int prev = 0;
	long makespan = 0;
	long capacity = 0;
	long violations = 0;
	long cost = 0;
	while(tourSize < N) {
	    double sum = 0.0;
	    int successors = 0;
	    for(int i = 0; i < N; i++) 
		if (check(i))   	
		    moves[successors++] = i;
	    assert(successors != 0);
	    for(int i=0; i<successors; i++) {
		value[i] = EXP(global[node][moves[i]]);
		sum += value[i];
	    }
	    double mrand=(rand()/(double)RAND_MAX)*(sum);
	    int i=0;
	    sum = value[0];
	    while(sum<mrand) 
		sum += value[++i];
	    prev = node;
	    node = moves[i];
	    tour[tourSize++] = node;
	    visits[node] = 1;
	    cost += d[prev][node];
	}
	tour[tourSize++] = 0; // Finish at the depot;
	//    cost += d[node][0];  
	return cost;
    }  

    
    void MC::adapt(int* sample, int level) {   
    for (int j=0;j<N;j++) 
      visits[j] = 0;
    int successors;
    int node = 0;
    for(int j=1; j<N; j++)  {
      successors = 0;    
      for(int i = 1; i < N; i++) 
	if (check(i)) 
	  moves[successors++] = i;
      double factor = ALPHA;
      backup[level][node][sample[j]] += factor;    
      double z = 0.0;
      for(int i=0; i<successors; i++) 
	  z += EXP(global[node][moves[i]]);
	for (int i=0; i<successors; i++) 
	  backup[level][node][moves[i]] -= factor *
	    EXP(global[node][moves[i]])/z;
      node = sample[j];
      visits[node] = 1;
    }
  }

    
    void MC::print(int* tour) {
    long sum = 0;
    long dist = 0;
    for (int l=1;l<N;l++) {
      cout << "[" << tour[l-1] << "," << tour[l] << "],";
      dist += d[tour[l-1]][tour[l]];
    }
    // dist += d[tour[N-1]][0];
    cout << " --- sum dist " << ((double)dist) << endl;
  }

    Pair* MC::search(int level) {
    Pair* best = new Pair(N);
    best->score =  std::numeric_limits<long>::max();
    if (level == 0) {
      long eval = rollout();  // sets tour and visits
      evaluations++;
      best->score = eval;
      for (int j = 0; j < N+1; j++) 
	best->tour[j] = tour[j];	
    }
    else {
      for(int i = 0; i < N; i++)  
        for(int j = 0; j < N; j++)  
	  backup[level][i][j] = global[i][j];
      for(int i=0; i<ITERATIONS; i++) {  
	Pair* r = search(level - 1);
	long score = r->score;
	if (score < best->score) {
	  best->score = score;	  
	  for (int j = 0; j < N+1; j++) 
	    best->tour[j] = r->tour[j];
	  /*if(level > SHOW) {
	    for(int t = 1; t < level - (SHOW); t++)
		cout << "\t";
	      cout << " Level: " << level << "," << i 
		   << ", score: " << score << ", runs: " 
		   << evaluations << endl;
	      print(best->tour);
	      }*/
	  adapt(best->tour,level);
	}
	delete r;
      }
      for(int i = 0; i < N; i++) 
        for(int j = 0; j < N; j++)  
	  global[i][j] = backup[level][i][j];
    }
    return best;
  }  
}
