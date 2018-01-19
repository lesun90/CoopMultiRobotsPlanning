#ifndef MP_LOGIC_FORMULA_AND_HPP_
#define MP_LOGIC_FORMULA_AND_HPP_

#include "Logic/FormulaComposed.hpp"

namespace MP
{
    class FormulaAnd : public FormulaComposed
    {
    public:
	FormulaAnd(void) : FormulaComposed()
	{
	}
	
	FormulaAnd(Formula * const f1, Formula * const f2) : FormulaComposed()
	{
	    m_subformulas.push_back(f1);
	    m_subformulas.push_back(f2);
	}
	
	virtual ~FormulaAnd(void)
	{
	}
	
	virtual bool IsSatisfied(const int n, const int props[]) const
	{
	    const int nf = m_subformulas.size();
	    for(int i = 0; i < nf; ++i)
		if(m_subformulas[i]->IsSatisfied(n, props) == false)
		    return false;
	    return true;
	}	    
    };
}
    
#endif



