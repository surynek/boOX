/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                              boOX 0_iskra-140                              */
/*                                                                            */
/*                      (C) Copyright 2018 Pavel Surynek                      */
/*                http://www.surynek.com | <pavel@surynek.com>                */
/*                                                                            */
/*                                                                            */
/*============================================================================*/
/* rota_solver_main.h / 0_iskra-140                                           */
/*----------------------------------------------------------------------------*/
//
// Token Rotation Problem Solver - main program.
//
// A CBS-based solver for token rotation problem (swaps excluding).
//
/*----------------------------------------------------------------------------*/


#ifndef __ROTA_SOLVER_MAIN_H__
#define __ROTA_SOLVER_MAIN_H__

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
	    ALGORITHM_CBS,
	    ALGORITHM_CBS_PLUS,
	    ALGORITHM_CBS_PLUS_PLUS,	    
	    ALGORITHM_SMTCBS
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
    sResult solve_TokenRotationInstance(const sCommandParameters &parameters);


/*----------------------------------------------------------------------------*/

} // namespace boOX


#endif /* __ROTA_SOLVER_MAIN_H__ */
