#ifndef MP_LOGIC_FORMULA_HPP_
#define MP_LOGIC_FORMULA_HPP_

namespace MP
{
    class Formula 
    {
    public:
	Formula(void) 
	{
	}
	
	virtual ~Formula(void)
	{
	}
	
	virtual bool IsSatisfied(const int n, const int props[]) const = 0;	    
    };
    
}
    
#endif



