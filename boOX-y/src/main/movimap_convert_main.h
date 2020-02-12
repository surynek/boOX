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
/* movimap_convert_main.h / 1-224_leibniz                                     */
/*----------------------------------------------------------------------------*/
//
// movingai.com map convertor - main program.
//
// This program takes a movingai.com map and converts it to xml format that
// can be further processed by other programs.
//
/*----------------------------------------------------------------------------*/


#ifndef __MOVIMAP_CONVERT_MAIN_H__
#define __MOVIMAP_CONVERT_MAIN_H__

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

	sString m_input_movi_map_filename;
	sString m_output_xml_map_filename;
    };


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void);
    void print_ConcludingMessage(void);
    void print_Help(void);
    
    sResult parse_CommandLineParameter(const sString &parameter, sCommandParameters &parameters);
    
    sResult convert_MoviMap2XmlMap(const sCommandParameters &parameters);


/*----------------------------------------------------------------------------*/

} // namespace boOX


#endif /* __MOVIMAP_CONVERT_MAIN_H__ */
