/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             boOX 1-224_leibniz                             */
/*                                                                            */
/*                  (C) Copyright 2018 - 2020 Pavel Surynek                   */
/*                                                                            */
/*                http://www.surynek.com | <pavel@surynek.com>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* basic_test.cpp / 1-224_leibniz                                             */
/*----------------------------------------------------------------------------*/
//
// Basic initial test.
//
/*----------------------------------------------------------------------------*/


#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <signal.h>
#include <sys/time.h>

#include <z3++.h>

#include "defs.h"
#include "compile.h"
#include "result.h"
#include "version.h"

#include "common/types.h"

#include "test/basic_test.h"


using namespace std;
using namespace boOX;
using namespace z3;


/*----------------------------------------------------------------------------*/

namespace boOX
{




/*----------------------------------------------------------------------------*/

void print_Introduction(void)
{
    printf("----------------------------------------------------------------\n");    
    printf("%s : Basic Test Program\n", sPRODUCT);
    printf("%s\n", sCOPYRIGHT);
    printf("================================================================\n");
}


int test_basic_1(void)
{
    printf("Testing basic 1 ...\n");

    z3::context z_context;
    z3::expr x(z_context.bool_const("x"));
    z3::expr y(z_context.bool_const("y"));
    z3::expr z(z_context.bool_const("z"));    

//    z3::expr lhs(!x || y);
    z3::expr lhs(x || y);    
    z3::expr rhs(implies(x, y));
    z3::expr final(lhs == rhs);
    
    z3::solver z_solver(z_context);
    z_solver.add(!final);

    printf("Printing solver status:\n");
    cout << z_solver << "\n";
    printf("Printing smt status:\n");
    cout << z_solver.to_smt2() << "\n";

    switch (z_solver.check())
    {
    case z3::sat:
    {
	printf("  SATISFIABLE\n");
	break;
    }
    case z3::unsat:	
    {
	printf("  UNSATISFIABLE\n");
	break;
    }
    case z3::unknown:
    {
	printf("  UNKNOWN\n");
	break;
    }
    default:
    {
	break;
    }
    }

    z3::model z_model(z_solver.get_model());
    printf("Printing model:\n");
    cout << z_model << "\n";
    for (sUInt_32 i = 0; i < z_model.size(); ++i)
    {
	printf("Variable:%s\n", z_model[i].name().str().c_str());
	printf("Printing interpretation:\n");
	cout << z_model.get_const_interp(z_model[i]) << "\n";
	
	switch (z_model.get_const_interp(z_model[i]).bool_value())
	{
	case Z3_L_FALSE:
	{
	    printf("   value: FALSE\n");
	    break;
	}
	case Z3_L_TRUE:
	{
	    printf("   value: TRUE\n");
	    break;
	}
	case Z3_L_UNDEF:
	{
	    printf("   value: UNDEF\n");
		break;
	}	    
	default:
	{
		break;
	}
	}
    }
    
    printf("Testing basic 1 ... finished\n");

    return sRESULT_SUCCESS;
}


}  // namespace boOX


/*----------------------------------------------------------------------------*/

int main(int sUNUSED(argc), const char **sUNUSED(argv))
{
    sResult result;

    print_Introduction();

    if (sFAILED(result = test_basic_1()))
    {
	printf("Test basic 1 failed (error:%d).\n", result);
	return result;
    }
    return 0;
}
