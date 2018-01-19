#include "Logic/FormulaIO.hpp"
#include "Logic/FormulaTrue.hpp"
#include "Logic/FormulaFalse.hpp"
#include "Logic/FormulaAtomic.hpp"
#include "Logic/FormulaNot.hpp"
#include "Logic/FormulaOr.hpp"
#include "Logic/FormulaAnd.hpp"
#include "Logic/FormulaIf.hpp"
#include "Logic/FormulaIff.hpp"
#include "Logic/FormulaXOr.hpp"

namespace MP
{
 
    Formula* FormulaIO::ReadPrefix(FILE * const in)
    {
	char     c;
	int      d;
	Formula *f1 = NULL;
	Formula *f2 = NULL;
	Formula *f  = NULL;
	
	while(fscanf(in, "%c", &c) != EOF)
	{
	    switch(c)
	    {
	    case 't': return new FormulaTrue();
	    case 'f': return new FormulaFalse();
		
	    case '!':
		if((f1 = ReadPrefix(in)))
		    return new FormulaNot(f1);
		else
		    return NULL;
		
	    case '|':
	    case '&':
	    case 'i':
	    case 'e':
	    case '^':
	    
		f1 = ReadPrefix(in);
		f2 = f1 ? ReadPrefix(in) : NULL;
		
		if(f1 && f2)
		{
		    if(c == '|')
			return new FormulaOr(f1, f2);
		    else if(c == '&')
			return new FormulaAnd(f1, f2);
		    else if(c == 'i')
			return new FormulaIf(f1, f2);
		    else if(c == 'e')
			return new FormulaIff(f1, f2);
		    else 
			return new FormulaXOr(f1, f2);
		}
		else
		{
		    if(f1)
			delete f1;
		    if(f2)
			delete f2;
		    return NULL;
		}
		
	    case 'p':
		if(fscanf(in, "%d", &d) == 1)
		    return new FormulaAtomic(d);
		else
		    return NULL;	    
		
	    case ' ':
	    case '\t':
	    case '\n':
	    case '\v':
	    case '\r':
	    case '\f':
		break;
		
	    default:
		printf("error: unrecognized character <%c>\n", c);
		return NULL;
	    }
	}
	
	return NULL;
    }   

    
    void FormulaIO::PrintPrefix(const Formula * const f, FILE * const out)
    {
	if(dynamic_cast<const FormulaOr * const>(f) ||
	   dynamic_cast<const FormulaAnd * const>(f) ||
	   dynamic_cast<const FormulaIf * const>(f) ||
	   dynamic_cast<const FormulaIff * const>(f) ||
	   dynamic_cast<const FormulaXOr * const>(f)) 
	{
	    if(dynamic_cast<const FormulaOr * const>(f))
	       fprintf(out, "|");
	    else if(dynamic_cast<const FormulaAnd * const>(f))
	       fprintf(out, "&");
	    else if(dynamic_cast<const FormulaIf * const>(f))
	       fprintf(out, "i");
	    else if(dynamic_cast<const FormulaIff * const>(f))
	       fprintf(out, "e");
	    else if(dynamic_cast<const FormulaXOr * const>(f))
	       fprintf(out, "^");

	    PrintPrefix(dynamic_cast<const FormulaComposed * const>(f)->m_subformulas[0], out);
	    PrintPrefix(dynamic_cast<const FormulaComposed * const>(f)->m_subformulas[1], out);
	}
	else if(dynamic_cast<const FormulaNot * const>(f))
	{
	    fprintf(out, "!");
	    PrintPrefix(dynamic_cast<const FormulaComposed * const>(f)->m_subformulas[0], out);
	}
	else if(dynamic_cast<const FormulaAtomic * const>(f))
	    fprintf(out, "p%d", dynamic_cast<const FormulaAtomic * const>(f)->m_prop);
	else if(dynamic_cast<const FormulaTrue * const>(f))
	    fprintf(out, "t");
	else if(dynamic_cast<const FormulaFalse * const>(f))
	    fprintf(out, "f");

    }

    void FormulaIO::PrintInfix(const Formula * const f, FILE * const out)
    {
	if(dynamic_cast<const FormulaOr * const>(f) ||
	   dynamic_cast<const FormulaAnd * const>(f) ||
	   dynamic_cast<const FormulaIf * const>(f) ||
	   dynamic_cast<const FormulaIff * const>(f) ||
	   dynamic_cast<const FormulaXOr * const>(f)) 
	{
	    fprintf(out, "(");
	    
	    PrintInfix(dynamic_cast<const FormulaComposed * const>(f)->m_subformulas[0], out);

	    if(dynamic_cast<const FormulaOr * const>(f))
	       fprintf(out, " | ");
	    else if(dynamic_cast<const FormulaAnd * const>(f))
	       fprintf(out, " & ");
	    else if(dynamic_cast<const FormulaIf * const>(f))
	       fprintf(out, " i ");
	    else if(dynamic_cast<const FormulaIff * const>(f))
	       fprintf(out, " e ");
	    else if(dynamic_cast<const FormulaXOr * const>(f))
	       fprintf(out, " ^ ");

	    PrintInfix(dynamic_cast<const FormulaComposed * const>(f)->m_subformulas[1], out);

	    fprintf(out, ")");
	}
	else if(dynamic_cast<const FormulaNot * const>(f))
	{
	    fprintf(out, "(! ");
	    PrintInfix(dynamic_cast<const FormulaComposed * const>(f)->m_subformulas[0], out);
	    fprintf(out, ")");
	}
	else if(dynamic_cast<const FormulaAtomic * const>(f))
	    fprintf(out, "(p%d) ", dynamic_cast<const FormulaAtomic * const>(f)->m_prop);
	else if(dynamic_cast<const FormulaTrue * const>(f))
	    fprintf(out, "t");
	else if(dynamic_cast<const FormulaFalse * const>(f))
	    fprintf(out, "f");

    }
    
}



