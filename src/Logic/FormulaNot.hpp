#ifndef MP_LOGIC_FORMULA_NOT_HPP_
#define MP_LOGIC_FORMULA_NOT_HPP_

#include "Logic/FormulaComposed.hpp"

namespace MP
{
    class FormulaNot : public FormulaComposed
    {
    public:
	FormulaNot(void) : FormulaComposed()
	{
	}
	
	FormulaNot(Formula * const f) : FormulaComposed()
	{
	    m_subformulas.push_back(f);
	}

	virtual ~FormulaNot(void)
	{
	}
	
	virtual bool IsSatisfied(const int n, const int props[]) const
	{
	    return m_subformulas[0]->IsSatisfied(n, props) == false; 
	}	    
	
    };
}
    
#endif



