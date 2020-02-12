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
/* stargen_main.h / 1-224_leibniz                                             */
/*----------------------------------------------------------------------------*/
//
// Star Graph Instance Generator - main program.
//
// Generates a Multi Robot instance on a star graph.
//
/*----------------------------------------------------------------------------*/


#ifndef __STARGEN_MAIN_H__
#define __STARGEN_MAIN_H__

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
    sResult generate_StarInstance(const sCommandParameters &parameters);


/*----------------------------------------------------------------------------*/

} // namespace boOX


#endif /* __STARGEN_MAIN_H__ */
