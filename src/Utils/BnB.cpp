/*
  An implementation of APSP with radix heaps
  Author: Stefan Edelkamp, 2014
*/

#include "BnB.hpp"

#include <vector>

using namespace std;

namespace BnB
{
    Selective::~Selective(void)
    {
	for(int i = 0; i < N; ++i)
	{
	    delete[] d[i];
	    delete[] cost[i];
	    delete[] copy[i];
	    delete[] mask[i];
	    delete[] far[i];
	    delete[] assignment[i];	 
	}
	
	for(int i = 0; i < N*N; ++i)
	    delete[] path[i];
	
	
	delete[] tour;
	delete[] d;
	delete[] cmin;	
	delete[] newState;
	delete stack;
	delete[] far;
	delete[] next;
	delete[] cost;
	delete[] copy;
	delete[] mask;
	delete[] cycle;
	delete[] visited;
	delete[] assignment;
	delete[] in;
	delete[] path;
	delete[] link;
	delete[] out;
	delete[] closest;
	delete[] rowCover;
	delete[] colCover;

    }
    
    Selective::Selective(int k, long** goal_dist, long* start_dist) 
    {
	start = 0;
	N = k+1;
	tour = new int[N+1];
	d = new long*[N];           // distance matrix
	
	cmin = new long[N]; 

	newState = new State[N];
	stack = new Memory(N);
	
	far = new int*[N];
	next = new int[N];
	cost = new long*[N];
	copy = new long*[N];
	mask = new long*[N];
	cycle = new bool[N];
	visited = new bool[N];
	rowCover = new int[N];                  ;
	colCover = new int[N];
	assignment = new int*[N];     
	in = new int[N];
	path = new int*[N*N];
	link = new int[N];
	out = new int[N];
	closest = new int[N];
	
	for(int i = 0; i < N; i++) { 
	    d[i] = new long[N];
	    cost[i] = new long[N];
	    copy[i] = new long[N];
	    mask[i] = new long[N];      
	    far[i] = new int[N];
	    assignment[i] = new int[2];
	}
	for(int i = 0; i < N*N; i++) { 
	    path[i] = new int[2];
	}
	
	expansions = 0;         
	for(int i = 1; i < N; i++) 
	    for(int j = 1; j < N; j++) 
		d[i][j] = goal_dist[i-1][j-1];
	for (int i=0;i<N;i++) 
	    d[i][i] = 0;
	
	for (int i=1;i<N;i++) {
	    d[0][i] = start_dist[i-1];
	    d[i][0] = start_dist[i-1];
	}
	
	used = 0L;
	for (int i = 0; i < N; i++) {
	    newState[i].used = 0L;
	    newState[i].g = newState[i].h = 0;
	    newState[i].depth = newState[i].city = 0;
	    for (int j = 0; j < N; j++)
		far[i][j] = j;
	    for (int j = 0; j < N-1; j++) 
		for (int k = j+1; k < N; k++) 
		    if (d[i][far[i][j]] < d[i][far[i][k]]) 
			std::swap(far[i][j],far[i][k]);
	}
	
	
    }
    
    Pair* Selective::search(int h, const int maxNrSteps) 
    {   
	//printf("maxNrSteps = %d\n", maxNrSteps);
	
	expansions = 0;
	Pair* best = new Pair(N);
	long alpha = std::numeric_limits<long>::max();
     int top = stack->top++;
     long csum = 0;
     stack->old[top].g = stack->old[top].depth = 0;
     stack->old[top].city = start;
     used = 0L;
     stack->old[top].h = heuristic(h,0,0,0);    
     //cout << "Heuristic value at root = " << stack->old[top].h << endl;
     stack->old[top].used = used = (1L << start);
     while (stack->top != 0) {
       if (expansions > maxNrSteps) {
	   //cout << "End of TSP tour optimization " << alpha << endl;
	 return best;
       }       
       top = --stack->top;
       int depth = stack->old[top].depth;
       int city = stack->old[top].city;
       int hval = stack->old[top].h;
       int oldcity = depth > 0 ? tour[depth-1] : -1;
       tour[depth] = city;
       if (depth == N - 1) { 
	 if (stack->old[top].g /* + d[city][start] */  < alpha) { 
	   alpha = stack->old[top].g /* + d[city][start]*/ ; 
	   //cout << " cost: " << alpha << " (" << expansions << ")" << endl;
	   int check = 0;
	   best->score = alpha;
	   for (int j = 0; j < N; j++)  
	     best->tour[j] = tour[j];
	   tour[N] = 0;
	   //print(best->tour);
	   continue;
	 } 	 
       }
       used = stack->old[top].used;
       long cost = stack->old[top].g;
       int opindex = 0;           
       expansions++;
       for(int i=0; i < N; i++) {
	 if (((used >> far[city][i]) & 1L) > 0L) continue;
	 int newcity = far[city][i];
	 newState[opindex].depth = depth+1;
	 next[opindex] = opindex + 1;
	 int g = cost + d[city][newcity];
	 newState[opindex].g = g;
	 newState[opindex].city = newcity;
	 used &= ~(1L << start);
	 newState[opindex].h = 
	   incheuristic(h,g,hval,newcity,depth); 
	 used |= (1L << start);
	 used |= (1L << newcity);
	 newState[opindex].used = used;
	 used &= ~(1L << newcity);
	 opindex++;
       }
       next[opindex-1] = N; 
       for(int i=0; i != N; i = next[i]) {
	 if (newState[i].g + newState[i].h >= alpha) continue;
	 int newtop = stack->top++;
	 stack->old[newtop].city = newState[i].city;
	 stack->old[newtop].used = newState[i].used;
	 stack->old[newtop].g = newState[i].g;
	 stack->old[newtop].h = newState[i].h;
	 stack->old[newtop].depth = newState[i].depth;
       }
     }
     return best;

    }
    
    int Selective::column(int g, int city, int depth) {
	long csum = 0;
	for (int j=1;j<N;j++) {
	    if (((used >> j) & 1UL) > 0L) continue;
	    csum += cmin[j];
	}
	return csum;
    }

      
    int Selective::heuristic(int h, int g, int city, int depth) {
	if (h == 1) return hg(g,city,depth);
	if (h == 2) return column(g,city,depth);
	return 0;
    } 
    
    int Selective::incheuristic(int h, int g, int hval, int city, int depth) {
	if (h == 1) return hg(g,city,depth);
	if (h == 2) return hval - cmin[city]; 
	return 0;
    }

    void Selective::print(int* tour) 
    {
	long sum = 0;
	long dist = 0;
	for (int l=1;l<N;l++) {
	    cout << "[" << tour[l-1] << "--" 
		 << d[tour[l-1]][tour[l]] << "-->" << tour[l] << "]";
	    dist += d[tour[l-1]][tour[l]];
	}
	cout << " --- sum dist " << dist << endl;
    }
    
    long Selective::hg (long g, int city, int depth) 
    {
	int c = N - depth;
	int ci = 0;
	int count = 0;
	long minval = 0;
	int rowcol0;
	int rowcol1;    
	for (int i=0; i<N; i++) {
	    if (((used >> i) & 1UL) > 0) continue;
	    rowCover[ci] = colCover[ci] = 0;
	    int cj = 0;
	    for (int j=0; j<N; j++) {         
		if (((used >> j) & 1UL) > 0L) continue;
		copy[ci][cj] = cost[ci][cj] = d[i][j];
		mask[ci][cj] = 0;
		//   if (porder[j][i]) 
		//     copy[ci][cj] = cost[ci][cj] = std::numeric_limits<long>::max();
		if (j == start) 
		    copy[ci][cj] = cost[ci][cj] = 0;
		if (i == j)
		    copy[ci][cj] = cost[ci][cj] = std::numeric_limits<long>::max();
		if (start != city && i == start) { 
		    if (j == city) 
			copy[ci][cj] = cost[ci][cj] = g;
		    else 
			copy[ci][cj] = cost[ci][cj] = std::numeric_limits<long>::max();
		}
		cj++;
	    }
	    ci++;
	}
	int step = 1;       
	bool finished = false;
	while (!finished)  { 
	    switch (step) {
	    case 1: {
		for (int i=0; i<c; i++) {
		    minval = cost[i][0];
		    for (int j=0; j<c; j++) {
			if (minval > cost[i][j]) 
			    minval = cost[i][j];
		    }
		    for (int j=0; j<c; j++) 
			cost[i][j] -= minval;  // - minval
		}                                   
		for (int i=0; i<c; i++) {
		    for (int j=0; j<c; j++) {
			if ((cost[i][j]==0) && (colCover[j]==0) && 
			    (rowCover[i]==0)) {
			    mask[i][j] = colCover[j] = rowCover[i] = 1;
			}
		    }
		}
		step = 3;
		break;
	    }
	    case 3: {
		for (int i=0; i<c; i++) 
		    rowCover[i] = colCover[i] = 0;
		for (int i=0; i<c; i++) {
		    for (int j=0; j<c; j++) {
			if (mask[i][j] == 1)
			    colCover[j]=1;
		    }
		}           
		count=0;                                                
		for (int j=0; j<c; j++) {  
		    count += colCover[j];
		}
		if (count >= c)
		    finished = true;
		else 
		    step = 4;     
		break;
	    }
	    case 4: {
		while (true) {
		    rowcol0 = -1; 
		    rowcol1 = 0;
		    int i = 0; 
		    bool done = false;
		    while (!done) {
			int j = 0;
			while (j < c) {
			    if (cost[i][j]==0 && 
				rowCover[i]==0 && colCover[j]==0) {
				rowcol0 = i;
				rowcol1 = j;
				done = true;
			    }
			    j++;
			}
			i++;
			if (i >= c) {
			    done = true;
			}
		    }                 
		    if (rowcol0 == -1) {
			step = 6;
			break;
		    }
		    else {
			mask[rowcol0][rowcol1] = 2; 
			bool starInRow = false;
			for (int j=0; j<c; j++) {
			    if (mask[rowcol0][j]==1) {
				starInRow = true;
				rowcol1 = j;            
			    }
			}
			if (starInRow) {
			    rowCover[rowcol0] = 1;
			    colCover[rowcol1] = 0;
			}
			else {
			    step = 5;
			    break;
			}
		    }
		}
		break;
	    }
	    case 5: {
		count = 0;     
		path[count][0] = rowcol0; 
		path[count][1] = rowcol1; 
		bool done = false;
		while (!done) { 
		    int r=-1;     
		    for (int i=0; i<c; i++) {
			if (mask[i][path[count][1]]==1) 
			    r = i;
		    }                             
		    if (r>=0) {
			count++;
			path[count][0] = r;                         
			path[count][1] = path[count-1][1];     
		    }
		    else {
			done = true;
		    }
		    if (!done) {
			int t = -1;
			for (int j=0; j<c; j++) {
			    if (mask[path[count][0]][j]==2) 
				t = j;
			}           
			count++;
			path[count][0] = path [count-1][0]; 
			path[count][1] = t;                 
		    }
		}
		for (int i=0; i<=count; i++) {
		    if (mask[(path[i][0])][(path[i][1])]==1)
			mask[(path[i][0])][(path[i][1])] = 0;
		    else
			mask[(path[i][0])][(path[i][1])] = 1;
		}
		for (int i=0; i<c; i++) {
		    for (int j=0; j<c; j++) {
			if (mask[i][j]==2)
			    mask[i][j] = 0;
		    }
		}
		step = 3;               
		break;
	    }
	    case 6: {
		minval = std::numeric_limits<long>::max();      
		for (int i=0; i<c; i++)  {
		    for (int j=0; j<c; j++) {
			if (rowCover[i]==0 && colCover[j]==0 && 
			    (minval > cost[i][j]))
			    minval = cost[i][j];
		    }
		}
		for (int i=0; i<c; i++) {
		    for (int j=0; j<c; j++) {
			if (rowCover[i]==1) {
			    cost[i][j] += minval;
			}
			if (colCover[j]==0) {
			    cost[i][j] -= minval;
			}
		    }
		}
		step = 4;
		break;
	    }
	    }
	}
	for (int i=0; i<c; i++) {
	    for (int j=0; j<c; j++) {
		if (mask[i][j] == 1) 
		    link[i]=j;
	    }
	}
	long sum = 0; 
	for (int i=0; i<c; i++) 
	    sum += copy[i][link[i]];
	int offset = computeoffset(depth);
	return sum - g + offset;
	
    }
    
    
    long Selective::computeoffset(int depth) 
    {
	int c = N - depth;
	for (int i = 0; i < c; i++) 
	    cycle[i] = visited[i] = false;
	long offset = 0;
	for (int i = 0; i < c; i++) {
	    if (!visited[i]) {
		int len = 0;
		bool first = true;
		for (int j = i; (j != i) || first; j = link[j]) {
		    len++;
		    cycle[j] = visited[j] = true;
		    first = false;
		}
		long minc = std::numeric_limits<long>::max();    
		for (int j = 0; j < c; j++) 
		    if (cycle[j])         
			for (int k = 0; k < c; k++) 
			    if (!cycle[k]) 
				if (minc > cost[j][k])
				    minc = cost[j][k];
		
		for (int k = 0; k < c; k++)
		    cycle[k] = false;
		if (minc != std::numeric_limits<long>::max())
          offset += minc;
      }
    }
    return offset;
 
    }
    
    Radix::Radix() 
    {
	S = std::numeric_limits<long>::max();
	B = (int) (ceil(log(S) / log(2))) + 2;
	buckets = new Node* [B];
	b = new long [B];
	u = new long [B];
	for (int i = 0; i < B; i++) {
	    buckets[i] = 0;
	    b[i] = u[i] = 0;
	}
	b[0] = 1; 
	b[B-1] = std::numeric_limits<long>::max();
	for (int i = 1; i < B-1; i++)
	    b[i] = 1L << (i-1);
	u[B-1] = std::numeric_limits<long>::max();
	n = 0;
    }
    
    Node* Radix::next(Node* p) 
    {
	if (p->succ != 0)
	    return p->succ;
	else {
	    int next = p->bucket + 1;
	    while ((next<B) && (buckets[next] == 0))
		next++;
	    if (next == B)
		return 0;
	    else
		return buckets[next];
	}
    }
    
    void Radix::insert_node(Node* p, int i) {
	p->succ = buckets[i];
	if (buckets[i] != 0)
	    buckets[i]->pred = p;
	p->pred = 0;
	p->bucket = i;
	buckets[i] = p;
    }
    
    void Radix::extract_node(Node* p) {
	if (p->pred != 0) {
	    Node* q = p->pred;
	    q->succ = p->succ;
	}
	else {
	    buckets[p->bucket] = p->succ;
	}
	if (p->succ != 0) {
	    Node* q = p->succ;
	    q->pred = p->pred;
	}
    }
    
    void Radix::adjust(long m, int t) {
	int i;
	u[0] = m;
	for (i = 1; i < t; i++) {
	    u[i] = u[i-1] + b[i];
	    if (u[i] > u[t])
		break;
	}
	for (; i < t; i++)
	    u[i] = u[t];
    }
    
    int Radix::find(Node* p, int i) {
	if (p->element == u[0]) 
	    return 0;
	while (p->element <= u[--i]) ;
	return i+1;
    }

    Node* Radix::insert(Node* p) {
	long k = p->element;
	if (n > 0) {
	    //  if (k < u[0]) cout  << "Error insert" << k << " < " << u[0] << endl;
	    int i = find(p,B-1);
	    insert_node(p,i);	    
	}
	else {
	    adjust(k,B-1);
	    buckets[0] = p;
	    p->bucket = 0;
	}
	n++;
	return p;
    }
    
    void Radix::decrease(Node* x, long k) {
	//	if (k >= x->element) cout  << "Error decrease" << k << " >= " << x->element << endl;
	//	if (k < u[0]) cout  << "Error decrease" << k << " < " << u[0] << endl;
	x->element = k;
	if (k <= u[x->bucket-1]) {
	    extract_node(x);
	    int i = find(x,x->bucket);
	    insert_node(x,i);
	}
    }
    
    Node* Radix::extract() {
	for (int i = 0; i < B; i++) {
	    Node* p = buckets[i];
	    if (p != 0) {
		extract_node(p);
		n--;
		return p;
	    }
	}
	return 0;
    }
    
    Node* Radix::extract(Node* x) {
	int i = x->bucket;
	extract_node(x);
	if ((n > 1) && (i == 0) && (buckets[0] == 0)) {
	    int j = 1;
	    while (buckets[j] == 0)
		j++;	
	    Node* p = buckets[j];
	    Node* d = p->succ;
	    while (d != 0) {
		if (d->element < p->element)
		    p = d;
		d = d->succ;
	    }
	    adjust(p->element,j);
	    extract_node(p);
	    insert_node(p,0);
	    p = buckets[j];
	    while (p != 0) {
		Node* q = p->succ;
		extract_node(p);
		int l = find(p,j);
		insert_node(p,l);
		p = q;
	    }
	}
	n--;
	return x;
    }

    Graph::Graph(int n, int k) {
/*
	for (int i = 0; i < n; i++)
	    nodes.push_back(new Node(rand()%10000,rand()%10000));
	for (int i = 0; i < 30; i++) {
	    for (int j = 0; j < n; j++) {
		double w = 
		    sqrt( (( nodes[i]->x - nodes[j]->x ) * 
			   ( nodes[i]->x - nodes[j]->x )) + 
			  (( nodes[i]->y - nodes[j]->y ) * 
			   ( nodes[i]->y - nodes[j]->y )) );
		nodes[j]->edges = new Link(rand()%n,(long) w,nodes[j]->edges);  
	    }
	}
	start = rand() % n;
	for (int i = 0; i < k; i++) 
	    goals.push_back(rand() % n);

	cout << "......................start = " << start << " n = " << n << " rand = " << rand() << endl;
*/	
    }
    
    void dijkstra(Node* s, vector<Node*> N) {
	int n = N.size();
	int v = 0;
	for (int i=0;i<n;i++) {
	    N[i]->state = Node::unlabelled;
	    N[i]->pred = N[i]->succ = 0;
	}
	//    cout  << "Nodes initialized" << endl;
	s->element = 0;
	s->state = Node::labelled;
	Radix* heap = new Radix();
	heap->insert(s);
	cout  << "Heap initialized" << endl;
	while (heap->size() != 0) {
	    v++;
	    Node* t = heap->top();
	    t->state = Node::scanned;
	    long d = t->element;
	    //	    cout  << "Deleting minimum " << d << endl;
	    for (Link* l = t->edges; l != 0; l = l->next) {
		Node* u = N[l->succid];
		//		cout  << "Next: " << u << endl;
		long c = d + l->weight;
		if (u->state == Node::scanned) 
		    ;
		else if (u->state == Node::unlabelled) {
		    u->element = c;
		    //	        cout  << "Inserting with cost " << c << endl;
		    heap->insert(u);
		    u->state  = Node::labelled;
		}
		else if (u->element > c) {
		    //		  cout  << "Decreasing to cost " << c << endl;
		    heap->decrease(u, c);
		}
	    }
	    heap->extract(t);
	}
	cout  << "Expanded nodes " << v << endl;
    }
}

/*
using namespace BnB;

int main(int argc, char** args) {
    int n = 1000, k = 30; 
    if (argc == 3) {
	n = atoi(args[1]);
	k = atoi(args[2]); 
    }
    cout  << "Generate graph with " << n << " nodes " << k << " goals and " 
	  << n*(n+1)/2 << " edges" << endl; 
    Graph* G = new Graph(n,k);  
    
    cout  << "Done. Now allocating space " << endl; 
    long** distance = new long*[k];
    long** goal_dist = new long*[k];
    long* start_dist = new long[k];
    for (int j = 0; j<k; j++) { 
	distance[j] = new long[n];
	goal_dist[j] = new long[k];
    }
    cout  << "Done. Now searching with Dijkstra's algorithm " << endl; 
    for (int j = 0; j<k; j++) { // assumes
	dijkstra(G->nodes[G->goals[j]],G->nodes);
	cout  << "Saving graph node distances" << endl; 
	for (int i=0;i<n;i++) {
	    distance[j][i] = G->nodes[i]->element;
	}
	cout  << "Saving goal node distances" << endl; 
	for (int i=0;i<k;i++) {
	    goal_dist[j][i] = distance[j][G->goals[i]];
	}
    }
    cout  << "Copying starting state " << endl; 
    for (int i=0;i<k;i++) 
	start_dist[i] = distance[i][G->start];
    cout  << "Done. Now searching TSP search blind heuristic" << endl; 
    Selective S(k,goal_dist,start_dist);
    Pair* l = S.search(0);
    cout << endl << "Best " <<  ((double)l->score) << endl; 
    S.print(l->tour);
    cout  << "Done. Now searching TSP search AP heuristic" << endl; 
    Selective T(k,goal_dist,start_dist);
    Pair* m = T.search(1);
    cout << endl << "Best " <<  ((double)m->score) << endl; 
    T.print(m->tour);
    
    cout << "goals:" << endl;
    
    for(auto it = G->goals.begin(); it != G->goals.end(); ++it)
	cout << (*it) << " ";
    cout << endl;
    cout << "start:" << G->start << endl;
    
    
    return 0;
}

*/
