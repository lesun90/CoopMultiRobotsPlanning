#ifndef MP_LOGIC_FORMULA_IO_HPP_
#define MP_LOGIC_FORMULA_IO_HPP_

#include "Logic/Formula.hpp"
#include <cstdio>

namespace MP
{
    class FormulaIO
    {
    public:
	static Formula* ReadPrefix(FILE * const in);
	static void PrintPrefix(const Formula * const f, FILE * const out);
	static void PrintInfix(const Formula * const f, FILE * const out);
    };
    
}
    
#endif



