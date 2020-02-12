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
/* moviscen_convert_main.h / 1-224_leibniz                                    */
/*----------------------------------------------------------------------------*/
//
// movingai.com scenario convertor - main program.
//
// This program takes a movingai.com scenario and converts it to xml format that
// can be further processed by other programs.
//
/*----------------------------------------------------------------------------*/


#ifndef __MOVISCEN_CONVERT_MAIN_H__
#define __MOVISCEN_CONVERT_MAIN_H__

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

	sInt_32 m_N_kruhobots;
	sInt_32 m_N_agents;	
	
	sString m_input_movi_map_filename;
	sString m_input_movi_scen_filename;	
	sString m_output_xml_scen_filename;
	
	sString m_output_mpf_filename;
	sString m_output_cpf_filename;
	sString m_output_bgu_filename;
    };


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void);
    void print_ConcludingMessage(void);
    void print_Help(void);
    
    sResult parse_CommandLineParameter(const sString &parameter, sCommandParameters &parameters);
    
    sResult convert_MoviScen2XmlTask(const sCommandParameters &parameters);
    sResult convert_MoviScen2MultirobotTask(const sCommandParameters &parameters);    


/*----------------------------------------------------------------------------*/

} // namespace boOX


#endif /* __MOVISCEN_CONVERT_MAIN_H__ */
