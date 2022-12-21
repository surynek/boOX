/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             boOX 2-189_planck                              */
/*                                                                            */
/*                  (C) Copyright 2018 - 2022 Pavel Surynek                   */
/*                                                                            */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* hamilton_solver_main.h / 2-189_planck                                      */
/*----------------------------------------------------------------------------*/
//
// Multi-Agent Hamiltonian Path Finding Solver - main program.
//
// CBS-based and SMT-based solvers for multi-agent hamiltonian
// path finding problem.
//
/*----------------------------------------------------------------------------*/


#ifndef __HAMILTON_SOLVER_MAIN_H__
#define __HAMILTON_SOLVER_MAIN_H__

#include "config.h"
#include "compile.h"
#include "defs.h"
#include "version.h"

using namespace std;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/

    struct sCommandParameters
    {
	enum Algorithm
	{
	    ALGORITHM_CBS_PLUS_PLUS_PLUS,
	    ALGORITHM_CBS_PLUS_PLUS_PLUS_PLUS,
	    ALGORITHM_CBS_ULTRA,
	    ALGORITHM_CBS_ULTRA_PLUS,	    	    
	    ALGORITHM_SMTCBS_PLUS_PLUS,
	    ALGORITHM_SMTCBS_PLUS_PLUS_PLUS,
	    ALGORITHM_SMTCBS_ULTRA,
	    ALGORITHM_SMTCBS_ULTRA_PLUS
	};
	
	sCommandParameters();
        /*--------------------------------*/

	sInt_32 m_cost_limit;
	Algorithm m_algorithm;

	sString m_input_filename;		
	sString m_output_filename;

	sDouble m_timeout;
    };


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void);
    void print_ConcludingMessage(void);
    void print_Help(void);
    
    sResult parse_CommandLineParameter(const sString &parameter, sCommandParameters &parameters);
    sResult solve_MultiAgentHamiltonianPathFindingMission(const sCommandParameters &parameters);


/*----------------------------------------------------------------------------*/

} // namespace boOX


#endif /* __HAMILTON_SOLVER_MAIN_H__ */
