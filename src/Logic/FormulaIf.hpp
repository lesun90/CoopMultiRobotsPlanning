#ifndef MP_LOGIC_FORMULA_IF_HPP_
#define MP_LOGIC_FORMULA_IF_HPP_

#include "Logic/FormulaComposed.hpp"

namespace MP
{
    class FormulaIf : public FormulaComposed
    {
    public:
	FormulaIf(void) : FormulaComposed()
	{
	}
	
	FormulaIf(Formula * const f1, Formula * const f2) : FormulaComposed()
	{
	    m_subformulas.push_back(f1);
	    m_subformulas.push_back(f2);
	}

	virtual ~FormulaIf(void)
	{
	}
	
	virtual bool IsSatisfied(const int n, const int props[]) const
	{
	    return 
		m_subformulas[0]->IsSatisfied(n, props) == false ||
		m_subformulas[1]->IsSatisfied(n, props) == true;
	}	    
    };
}
    
#endif



