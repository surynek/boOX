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
/* gridgen_main.h / 1-224_leibniz                                             */
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
	sInt_32 m_x_size;
	sInt_32 m_y_size;
	sInt_32 m_N_agents;
	sInt_32 m_seed;
	sDouble m_obstacle_probability;	
	sInt_32 m_N_obstacles;
	sInt_32 m_capacity;

	sString m_map_filename;
	sString m_cpf_filename;
	sString m_mpf_filename;		
	sString m_pddl_domain_filename;
	sString m_pddl_problem_filename;	
	sString m_bgu_filename;

	sString m_usc_map_filename;
	sString m_usc_agents_filename;

	bool is_Input_usc(void) const
	{
	    return ((!m_cpf_filename.empty() || !m_mpf_filename.empty() || !m_bgu_filename.empty()) && !m_usc_map_filename.empty());
	}

	bool is_Output_usc(void) const
	{
	    return (m_cpf_filename.empty() && m_mpf_filename.empty() && m_bgu_filename.empty() && !m_usc_map_filename.empty());
	}
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
