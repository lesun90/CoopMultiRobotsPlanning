#ifndef MP_LOGIC_FORMULA_ATOMIC_HPP_
#define MP_LOGIC_FORMULA_ATOMIC_HPP_

#include "Logic/Formula.hpp"
#include "Utils/Misc.hpp"

namespace MP
{
    class FormulaAtomic : public Formula
    {
    public:
	FormulaAtomic(void) : Formula()
	{
	}

	FormulaAtomic(const int prop) : Formula()
	{
	    m_prop = prop;
	}
	
	
	virtual ~FormulaAtomic(void)
	{
	}
	
	virtual bool IsSatisfied(const int n, const int props[]) const
	{
	    const int pos = FindItem<int>(n, props, m_prop);
	    if(m_prop >= 0)
		return pos >= 0;
	    else
		return pos < 0;
	}
	
	int m_prop;
    };
}
    
#endif



