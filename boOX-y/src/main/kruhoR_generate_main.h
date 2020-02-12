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
/* kruhoR_generate_main.h / 1-224_leibniz                                     */
/*----------------------------------------------------------------------------*/
//
// Continuous Multi-Agent Path Finding (MAPF-R) instance (real kruhobot
// configuration, conjunction) generator - main program.
//
// This program takes a continuous map and generated a MAPF-R instance
// on top of that.
//
/*----------------------------------------------------------------------------*/


#ifndef __KRUHO_R_GENERATE_MAIN_H__
#define __KRUHO_R_GENERATE_MAIN_H__

#include "config.h"
#include "compile.h"
#include "defs.h"
#include "version.h"

#include "core/mapR.h"
#include "core/kruhoR.h"

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
	sInt_32 m_N_kruhobots;
	sInt_32 m_seed;

	sDouble m_kruhobot_radius;
	
	sDouble m_kruhobot_linear_velocity;
	sDouble m_kruhobot_linear_acceleration;

	sDouble m_kruhobot_angular_velocity;
	sDouble m_kruhobot_angular_acceleration;
	sDouble m_kruhobot_wait_factor;
	
	sString m_input_mapR_filename;
	sString m_input_xml_map_filename;	
	sString m_input_xml_agent_filename;
	sString m_output_kruhoR_filename;
    };


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void);
    void print_ConcludingMessage(void);
    void print_Help(void);
    
    sResult parse_CommandLineParameter(const sString &parameter, sCommandParameters &parameters);

    
/*----------------------------------------------------------------------------*/
    
    sResult generate_RealKruhobotInstance(const sCommandParameters &parameters);
    
    void generate_RandomKruhobotConjunction(const sCommandParameters &parameters, sInt_32 N_locations, sRealConjunction &conjunction);
    void generate_WalkKruhobotConjunction(const sCommandParameters &parameters, sRealConjunction &initial_conjunction, sRealConjunction &conjunction);    


/*----------------------------------------------------------------------------*/

} // namespace boOX


#endif /* __KRUHO_R_GENERATE_MAIN_H__ */
