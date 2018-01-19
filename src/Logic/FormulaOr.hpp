#ifndef MP_LOGIC_FORMULA_OR_HPP_
#define MP_LOGIC_FORMULA_OR_HPP_

#include "Logic/FormulaComposed.hpp"

namespace MP
{
    class FormulaOr : public FormulaComposed
    {
    public:
	FormulaOr(void) : FormulaComposed()
	{
	}
	
	FormulaOr(Formula * const f1, Formula * const f2) : FormulaComposed()
	{
	    m_subformulas.push_back(f1);
	    m_subformulas.push_back(f2);
	}

	virtual ~FormulaOr(void)
	{
	}
	
	virtual bool IsSatisfied(const int n, const int props[]) const
	{
	    const int nf = m_subformulas.size();
	    for(int i = 0; i < nf; ++i)
		if(m_subformulas[i]->IsSatisfied(n, props))
		    return true;
	    return false;
	}	    
    };
}
    
#endif



