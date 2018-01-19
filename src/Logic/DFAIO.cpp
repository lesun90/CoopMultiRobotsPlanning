#include "Logic/DFAIO.hpp"
#include "Logic/FormulaIO.hpp"
#include "Utils/PrintMsg.hpp"

namespace MP
{
    DFA* DFAIO::ReadLBT(FILE * const in)
    {
//http://www.tcs.hut.fi/Software/maria/tools/lbt/
	int  nrStates = 0;
	int  nrAccept = 0;
        int  isInit   = 0;
	int  isAccept = 0;
	int  zid      = 0;
	int  zto      = 0;
	
	
	if(fscanf(in, "%d %d", &nrStates, &nrAccept) != 2)
	    OnInputError(printf("expecting nrStates nrAccept\n"));
	
	DFA        *dfa = new DFA();
	DFA::State *s;
	
	dfa->m_states.resize(nrStates);
	
	
	for(int i = 0; i < nrStates; ++i)
	{
	    s = new DFA::State();
//state id
	    if(fscanf(in, "%d", &zid) != 1)
		OnInputError(printf("expecting state id\n"));
	    dfa->m_states[zid] = s;
//is initial
	    if(fscanf(in, "%d", &isInit) != 1)
		OnInputError(printf("expecting isInit\n"));
	    if(isInit == 1 && dfa->m_init == Constants::ID_UNDEFINED)
		dfa->m_init = zid;
	    else if(isInit == 0 && dfa->m_init != Constants::ID_UNDEFINED)
		OnInputError(printf("another init state %d\n", zid));
//is accept
	    if(fscanf(in, "%d", &isAccept) != 1)
		OnInputError(printf("expecting acceptance or -1\n"));
	    if(isAccept != -1)
	    {
		s->m_isAccept = true;
		if(fscanf(in, "%d", &isAccept) != 1 || isAccept != -1)
		    OnInputError(printf("expecting -1 after acceptance\n"));
	    }
	    
//transitions
	    while(fscanf(in, "%d", &zto) == 1 && zto != -1)
	    {
		DFA::Edge *e = new DFA::Edge();
		e->m_zto     = zto;
		e->m_formula = FormulaIO::ReadPrefix(in);
		if(e->m_formula == NULL)
		    OnInputError("expecting formula\n");
		s->m_edges.push_back(e);
	    }
	}
	    
	return dfa;
    }

    void DFAIO::PrintDotty(const DFA * const dfa, FILE * const out)
    {
	fprintf(out, "digraph g {\n");
	
	const int n = dfa->m_states.size();
	for(int i = 0; i < n; ++i)
	{
	    if(i == dfa->m_init)
		fprintf(out, "%d[style=filled,label=\"%d\"];\n", i, i);
	    else if(dfa->m_states[i]->m_isAccept)
		fprintf(out, "%d[label=\"%d\\n0\"];\n", i, i);
	    else
		fprintf(out, "%d[label=\"%d\"]\n", i, i);
	    
	    const int ne = dfa->m_states[i]->m_edges.size();
	    for(int j = 0; j < ne; ++j)
	    {
		fprintf(out, "%d->%d[label=\"", i, dfa->m_states[i]->m_edges[j]->m_zto);
		FormulaIO::PrintPrefix(dfa->m_states[i]->m_edges[j]->m_formula, out);
		fprintf(out, "\"];\n");
	    }
	}
	fprintf(out, "}\n");
	
    }
}
    
    
    

