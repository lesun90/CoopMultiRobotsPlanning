#ifndef MP_LOGIC_FORMULA_XOR_HPP_
#define MP_LOGIC_FORMULA_XOR_HPP_

#include "Logic/FormulaComposed.hpp"

namespace MP
{
    class FormulaXOr : public FormulaComposed
    {
    public:
	FormulaXOr(void) : FormulaComposed()
	{
	}
	
	FormulaXOr(Formula * const f1, Formula * const f2) : FormulaComposed()
	{
	    m_subformulas.push_back(f1);
	    m_subformulas.push_back(f2);
	}

	virtual ~FormulaXOr(void)
	{
	}
	
	virtual bool IsSatisfied(const int n, const int props[]) const
	{
	    return 
		m_subformulas[0]->IsSatisfied(n, props) != 
		m_subformulas[1]->IsSatisfied(n, props);
	}	    
    };
}
    
#endif



