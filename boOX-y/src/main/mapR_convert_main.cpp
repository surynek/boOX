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
/* mapR_convert_main.cpp / 1-224_leibniz                                      */
/*----------------------------------------------------------------------------*/
//
// Continuous Multi-Agent Path Finding (MAPF-R) map convertor - main program.
//
// This program takes a grid map and converts it into a continuous map (mapR)
// with locations having real valued coordinates.
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

#include "main/mapR_convert_main.h"


using namespace std;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/

  sCommandParameters::sCommandParameters()
      : m_neighbor_type(NEIGHBORHOOD_CIRCULAR)
      , m_neighbor_radius(1.0)
      , m_neighbor_k(-1)
  {
     // nothing
  }


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void)
    {
	printf("--------------------------------------------------------------------------------\n");
	printf("%s : Continuous Multi-Agent Path Finding Map (mapR) Convertor\n", sPRODUCT); 
	printf("%s\n", sCOPYRIGHT);
	printf("================================================================================\n");	
    }


    void print_ConcludingMessage(void)
    {
	printf("----------------------------------------------------------------\n");
    }


    void print_Help(void)
    {
	printf("Usage:\n");
	printf("mapR_convert_boOX  --input-map-file=<string>\n");
	printf("                   --input-xml-file=<string>\n");
	printf("                   --output-mapR-file=<string>\n");
	printf("                  [--neighbor-type={circular|radiant}]\n");		
	printf("                  [--neighbor-radius=<double>]\n");
	printf("                  [--neighbor-k=<int>]\n");	
	printf("\n");
	printf("Examples:\n");
	printf("mapR_convert_boOX --input-map-file=ost003d.map\n");
	printf("                  --output-file=ost003d.mapR\n");
	printf("                  --neighbor-type=circular\n");		
	printf("                  --neighbor-radius=2.0\n");	
	printf("\n");
	printf("Defaults: --neighbor-type=circular\n");	
	printf("          --neighbor-radius=1.0\n");	
	printf("\n");
    }


    sResult convert_GridMap2RealMap(const sCommandParameters &parameters)
    {
	sResult result;
	s2DMap real_Map;

        #ifdef sSTATISTICS
	{
	    s_GlobalStatistics.enter_Phase("CONVERSION");
	}
  	#endif
	
	if (!parameters.m_input_map_filename.empty())
	{
	    result = real_Map.from_File_map(parameters.m_input_map_filename);

	    if (sFAILED(result))
	    {
		printf("Error: Failed to open grid map file %s (code = %d).\n", parameters.m_input_map_filename.c_str(), result);
		return result;
	    }
	}
	switch (parameters.m_neighbor_type)
	{
	case sCommandParameters::NEIGHBORHOOD_CIRCULAR:
	{
	    real_Map.populate_NetworkCircular(parameters.m_neighbor_radius);
	    break;
	}
	case sCommandParameters::NEIGHBORHOOD_RADIANT:
	{
	    if (parameters.m_neighbor_k >= 0)
	    {		
		if (parameters.m_neighbor_k >= 2 && parameters.m_neighbor_k < sizeof(s_RADIANT_RADIUS_2K_NEIHBORHOOD) / sizeof(sDouble))
		{
		    sDouble corresponding_radius = s_RADIANT_RADIUS_2K_NEIHBORHOOD[parameters.m_neighbor_k];
		    real_Map.populate_NetworkRadiant(corresponding_radius);		    
		}
		else
		{
		    printf("Error: Specified k in 2^k neighborhood is out of range <2,6> (code = %d).\n", sMAP_R_CONVERT_PROGRAM_K_NEIGHBOR_OUT_OF_RANGE_ERROR);
		    return sMAP_R_CONVERT_PROGRAM_K_NEIGHBOR_OUT_OF_RANGE_ERROR;
		}
	    }
	    else
	    {
		real_Map.populate_NetworkRadiant(parameters.m_neighbor_radius);
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}

	if (!parameters.m_output_mapR_filename.empty())
	{
	    result = real_Map.to_File_mapR(parameters.m_output_mapR_filename);

	    if (sFAILED(result))
	    {
		printf("Error: Failed to write continuous map (mapR) file %s (code = %d).\n", parameters.m_output_mapR_filename.c_str(), result);
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


    sResult convert_XmlMap2RealMap(const sCommandParameters &parameters)
    {
	sResult result;
	s2DMap real_Map;

        #ifdef sSTATISTICS
	{
	    s_GlobalStatistics.enter_Phase("CONVERSION");
	}
  	#endif
	
	if (!parameters.m_input_xml_filename.empty())
	{
	    result = real_Map.from_File_xml(parameters.m_input_xml_filename);

	    if (sFAILED(result))
	    {
		printf("Error: Failed to open xml map file %s (code = %d).\n", parameters.m_input_xml_filename.c_str(), result);
		return result;
	    }
	}
	
	if (real_Map.m_Network.m_Edges.empty())
	{
	    switch (parameters.m_neighbor_type)
	    {
	    case sCommandParameters::NEIGHBORHOOD_CIRCULAR:
	    {
		real_Map.populate_NetworkCircular(parameters.m_neighbor_radius);
		break;
	    }
	    case sCommandParameters::NEIGHBORHOOD_RADIANT:
	    {
		if (parameters.m_neighbor_k >= 0)
		{		
		    if (parameters.m_neighbor_k >= 2 && parameters.m_neighbor_k < sizeof(s_RADIANT_RADIUS_2K_NEIHBORHOOD) / sizeof(sDouble))
		    {
			sDouble corresponding_radius = s_RADIANT_RADIUS_2K_NEIHBORHOOD[parameters.m_neighbor_k];
			real_Map.populate_NetworkRadiant(corresponding_radius);		    
		    }
		    else
		    {
			printf("Error: Specified k in 2^k neighborhood is out of range <2,6> (code = %d).\n", sMAP_R_CONVERT_PROGRAM_K_NEIGHBOR_OUT_OF_RANGE_ERROR);
			return sMAP_R_CONVERT_PROGRAM_K_NEIGHBOR_OUT_OF_RANGE_ERROR;
		    }
		}
		else
		{
		    real_Map.populate_NetworkRadiant(parameters.m_neighbor_radius);
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }
	}

	if (!parameters.m_output_mapR_filename.empty())
	{
	    result = real_Map.to_File_mapR(parameters.m_output_mapR_filename);

	    if (sFAILED(result))
	    {
		printf("Error: Failed to write continuous map (mapR) file %s (code = %d).\n", parameters.m_output_mapR_filename.c_str(), result);
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
	if (parameter.find("--input-map-file=") == 0)
	{
	    command_parameters.m_input_map_filename = parameter.substr(17, parameter.size());
	}
	else if (parameter.find("--input-xml-file=") == 0)
	{
	    command_parameters.m_input_xml_filename = parameter.substr(17, parameter.size());
	}	
	else if (parameter.find("--output-mapR-file=") == 0)
	{
	    command_parameters.m_output_mapR_filename = parameter.substr(19, parameter.size());
	}
	else if (parameter.find("--neighbor-radius=") == 0)
	{
	    command_parameters.m_neighbor_radius = sDouble_from_String(parameter.substr(18, parameter.size()));
	}
	else if (parameter.find("--neighbor-k=") == 0)
	{
	    command_parameters.m_neighbor_k = sInt_32_from_String(parameter.substr(13, parameter.size()));	    
	}	
	else if (parameter.find("--neighbor-type=") == 0)
	{
	    sString neighbor_type_str = parameter.substr(16, parameter.size());

	    if (neighbor_type_str == "circular")
	    {
		command_parameters.m_neighbor_type = sCommandParameters::NEIGHBORHOOD_CIRCULAR;
	    }
	    else if (neighbor_type_str == "radiant")
	    {
		command_parameters.m_neighbor_type = sCommandParameters::NEIGHBORHOOD_RADIANT;
	    }
	    else
	    {
		return sMAPF_R_SOLVER_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
	    }
	}	
	else
	{
	    return sMAP_R_CONVERT_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
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
	if (!command_parameters.m_input_map_filename.empty())
	{
	    result = convert_GridMap2RealMap(command_parameters);
	}
	else if (!command_parameters.m_input_xml_filename.empty())
	{
	    result = convert_XmlMap2RealMap(command_parameters);
	}
	else
	{
	    printf("Error: No input map file specified (code = %d).\n", sMAP_R_CONVERT_PROGRAM_NO_MAP_FILE_SPECIFIED_ERROR);
	    print_Help();

	    return sMAP_R_CONVERT_PROGRAM_NO_MAP_FILE_SPECIFIED_ERROR;
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

