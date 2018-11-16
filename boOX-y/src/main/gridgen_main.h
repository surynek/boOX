/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                              boOX 0_iskra-158                              */
/*                                                                            */
/*                      (C) Copyright 2018 Pavel Surynek                      */
/*                http://www.surynek.com | <pavel@surynek.com>                */
/*                                                                            */
/*                                                                            */
/*============================================================================*/
/* gridgen_main.h / 0_iskra-158                                               */
/*----------------------------------------------------------------------------*/
//
// Grid Instance Generator - main program.
//
// Generates a Multi Robot instance on a 4-connected grid with randomly
// placed obstacles.
//
/*----------------------------------------------------------------------------*/


#ifndef __GRIDGEN_MAIN_H__
#define __GRIDGEN_MAIN_H__

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
	int m_x_size;
	int m_y_size;
	int m_N_agents;
	int m_seed;
	double m_obstacle_probability;	
	int m_N_obstacles;

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
    sResult generate_GridInstance(const sCommandParameters &parameters);


/*----------------------------------------------------------------------------*/

} // namespace boOX


#endif /* __GRIDGEN_MAIN_H__ */
