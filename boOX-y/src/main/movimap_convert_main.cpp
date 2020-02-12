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
/* movimap_convert_main.cpp / 1-224_leibniz                                   */
/*----------------------------------------------------------------------------*/
//
// movingai.com map convertor - main program.
//
// This program takes a movingai.com map and converts it to xml format that
// can be further processed by other programs.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <sys/times.h>
#include <unistd.h>

#include "config.h"
#include "compile.h"
#include "defs.h"
#include "version.h"

#include "core/graph.h"
#include "core/mapR.h"

#include "util/statistics.h"

#include "main/movimap_convert_main.h"


using namespace std;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/

  sCommandParameters::sCommandParameters()
  {
     // nothing
  }


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void)
    {
	printf("------------------------------------------------------------\n");	
	printf("%s : movingai.com Map Convertor\n", sPRODUCT); 
	printf("%s\n", sCOPYRIGHT);
	printf("============================================================\n");	
    }


    void print_ConcludingMessage(void)
    {
	printf("------------------------------------------------------------\n");
    }


    void print_Help(void)
    {
	printf("Usage:\n");
	printf("movimap_convert_boOX --input-movi-map-file=<string>\n");
	printf("                     --output-xml-map-file=<string>\n");
	printf("\n");
	printf("Examples:\n");
	printf("movimap_convert_boOX --input-movi-map-file=empty-16-16.map\n");
	printf("                     --output-xml-map-file=empty-16-16-random-1.xml\n");
	printf("\n");
	printf("Defaults: none\n");	
	printf("\n");
    }


    sResult convert_MoviMap2XmlMap(const sCommandParameters &parameters)
    {
	sResult result;
	s2DMap real_Map;

        #ifdef sSTATISTICS
	{
	    s_GlobalStatistics.enter_Phase("CONVERSION");
	}
  	#endif
	
	if (!parameters.m_input_movi_map_filename.empty())
	{
	    result = real_Map.from_File_movi(parameters.m_input_movi_map_filename);

	    if (sFAILED(result))
	    {
		printf("Error: Failed to open movingai.com map file %s (code = %d).\n", parameters.m_input_movi_map_filename.c_str(), result);
		return result;
	    }
	}
	if (!parameters.m_output_xml_map_filename.empty())
	{
	    result = real_Map.to_File_xml(parameters.m_output_xml_map_filename);

	    if (sFAILED(result))
	    {
		printf("Error: Failed to write xml map file %s (code = %d).\n", parameters.m_output_xml_map_filename.c_str(), result);
		return result;
	    }
	}	

        #ifdef sSTATISTICS
	{
	    s_GlobalStatistics.leave_Phase();
	}
	#endif
	
	#ifdef sSTATISTICS
	{
	    s_GlobalStatistics.to_Screen();
	}
	#endif

	return sRESULT_SUCCESS;
    }


    sResult parse_CommandLineParameter(const sString &parameter, sCommandParameters &command_parameters)
    {
	if (parameter.find("--input-movi-map-file=") == 0)
	{
	    command_parameters.m_input_movi_map_filename = parameter.substr(22, parameter.size());
	}
	else if (parameter.find("--output-xml-map-file=") == 0)
	{
	    command_parameters.m_output_xml_map_filename = parameter.substr(22, parameter.size());
	}	
	else
	{
	    return sMOVIMAP_CONVERT_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
	}
	return sRESULT_SUCCESS;
    }


/*----------------------------------------------------------------------------*/

} // namespace boOX


/*----------------------------------------------------------------------------*/
// main program

int main(int argc, char **argv)
{
    sResult result;
    sCommandParameters command_parameters;

    print_IntroductoryMessage();

    if (argc >= 2 && argc <= 11)
    {
	for (int i = 1; i < argc; ++i)
	{
	    result = parse_CommandLineParameter(argv[i], command_parameters);
	    if (sFAILED(result))
	    {
		printf("Error: Cannot parse command line parameters (code = %d).\n", result);
		print_Help();

		return result;
	    }
	}
	if (!command_parameters.m_input_movi_map_filename.empty())
	{
	    if (!command_parameters.m_output_xml_map_filename.empty())
	    {
		result = convert_MoviMap2XmlMap(command_parameters);
	    }
	    else
	    {
		printf("Error: No output xml file specified (code = %d).\n", sMOVIMAP_CONVERT_PROGRAM_NO_XML_MAP_FILE_SPECIFIED_ERROR);
		print_Help();
		
		return sMOVIMAP_CONVERT_PROGRAM_NO_XML_MAP_FILE_SPECIFIED_ERROR;		
	    }
	}
	else
	{
	    printf("Error: No input movimap.com file specified (code = %d).\n", sMOVIMAP_CONVERT_PROGRAM_NO_MOVI_MAP_FILE_SPECIFIED_ERROR);
	    print_Help();

	    return sMOVIMAP_CONVERT_PROGRAM_NO_MOVI_MAP_FILE_SPECIFIED_ERROR;
	}
	if (sFAILED(result))
	{
	    return result;
	}
    }
    else
    {
	print_Help();
    }
    return sRESULT_SUCCESS;
}

