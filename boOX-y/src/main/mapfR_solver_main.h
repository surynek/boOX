/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             boOX 2-213_planck                             */
/*                                                                            */
/*                  (C) Copyright 2018 - 2022 Pavel Surynek                  */
/*                                                                            */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* mapfR_solver_main.h / 2-213_planck                                         */
/*----------------------------------------------------------------------------*/
//
// Continuous Multi-Agent Path Finding Solver (MAPF-R) - main program.
//
// CBS-based and SMT-based solver for Continuous Multi-Agent Path Finding.
//
/*----------------------------------------------------------------------------*/


#ifndef __MAPF_R_SOLVER_MAIN_H__
#define __MAPF_R_SOLVER_MAIN_H__

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
	    ALGORITHM_CBS_R,
	    ALGORITHM_CBS_R_PLUS,
	    ALGORITHM_CBS_R_PLUS_PLUS,	    
	    ALGORITHM_SMTCBS_R,
	    ALGORITHM_SMTCBS_R_PLUS,
	    ALGORITHM_SMTCBS_R_PLUS_PLUS,
	    ALGORITHM_SMTCBS_R_PLUS_PLUS_PLUS,
	    ALGORITHM_SMTCBS_R_4_PLUS,
	    ALGORITHM_SMTCBS_R_STAR,
	    ALGORITHM_SMTCBS_R_STAR_SOC,
	    ALGORITHM_SMTCBS_R_STAR_SOC_PLUS	    
	};
	
	sCommandParameters();
        /*--------------------------------*/

	sDouble m_makespan_limit;
	sDouble m_cost_limit;
	
	Algorithm m_algorithm;

	sString m_mapR_input_filename;
	sString m_kruhoR_input_filename;	
	sString m_output_filename;

	sDouble m_timeout;
    };


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void);
    void print_ConcludingMessage(void);
    void print_Help(void);
    
    sResult parse_CommandLineParameter(const sString &parameter, sCommandParameters &parameters);
    sResult solve_RealMultiAgentPathFindingInstance(const sCommandParameters &parameters);


/*----------------------------------------------------------------------------*/

} // namespace boOX


#endif /* __MAPF_R_SOLVER_MAIN_H__ */
