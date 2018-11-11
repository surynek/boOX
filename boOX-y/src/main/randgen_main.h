/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                              boOX 0_iskra-151                              */
/*                                                                            */
/*                      (C) Copyright 2018 Pavel Surynek                      */
/*                http://www.surynek.com | <pavel@surynek.com>                */
/*                                                                            */
/*                                                                            */
/*============================================================================*/
/* randgen_main.h / 0_iskra-151                                               */
/*----------------------------------------------------------------------------*/
//
// Random Graph Instance Generator - main program.
//
// Generates a Multi Robot instance on a random graph.
//
/*----------------------------------------------------------------------------*/


#ifndef __RANDGEN_MAIN_H__
#define __RANDGEN_MAIN_H__

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
	sCommandParameters();
        /*--------------------------------*/

	bool m_walk;
	int m_N_vertices;	
	int m_N_agents;
	int m_seed;
	double m_edge_probability;	

	sString m_map_filename;
	sString m_cpf_filename;
	sString m_mpf_filename;		
	sString m_pddl_domain_filename;
	sString m_pddl_problem_filename;	
	sString m_bgu_filename;

	sString m_usc_map_filename;
	sString m_usc_agents_filename;	
    };


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void);
    void print_ConcludingMessage(void);
    void print_Help(void);
    
    sResult parse_CommandLineParameter(const sString &parameter, sCommandParameters &parameters);
    sResult generate_RandomInstance(const sCommandParameters &parameters);


/*----------------------------------------------------------------------------*/

} // namespace boOX


#endif /* __RANDGEN_MAIN_H__ */
