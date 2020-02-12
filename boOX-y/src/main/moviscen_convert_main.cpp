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
/* moviscen_convert_main.cpp / 1-224_leibniz                                  */
/*----------------------------------------------------------------------------*/
//
// movingai.com scenario convertor - main program.
//
// This program takes a movingai.com scenario and converts it to xml format that
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
#include "core/agent.h"
#include "core/mapR.h"
#include "core/kruhoR.h"

#include "util/statistics.h"

#include "main/moviscen_convert_main.h"


using namespace std;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/

  sCommandParameters::sCommandParameters()
      : m_N_kruhobots(-1)
      , m_N_agents(-1)
  {
     // nothing
  }


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void)
    {
	printf("------------------------------------------------------------\n");
	printf("%s : movingai.com Scenario Convertor\n", sPRODUCT); 
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
	printf("moviscen_convert_boOX  --input-movi-map-file=<string>\n");
	printf("                       --input-movi-scen-file=<string>\n");	
	printf("                       --output-xml-scen-file=<string>\n");
	printf("                       --output-mpf-file=<string>\n");
	printf("                       --output-cpf-file=<string>\n");
	printf("                       --output-bgu-file=<string>\n");			
	printf("                      [--N-kruhobots=<int>]\n");
	printf("                      [--N-agents=<int>]\n");		
	printf("\n");
	printf("Examples:\n");
	printf("moviscen_convert_boOX --input-movi-map-file=empty-16-16.map\n");
	printf("                      --input-movi-scen-file=empty-16-16-random-1.scen\n");
	printf("                      --output-xml-file=empty-16-16-random-1.xml\n");
	printf("                      --N-kruhobots=10\n");	
	printf("\n");	
	printf("moviscen_convert_boOX --input-movi-map-file=empty-16-16.map\n");
	printf("                      --input-movi-scen-file=empty-16-16-random-1.scen\n");
	printf("                      --output-mpf-file=empty-16-16-random-1.mpf\n");
	printf("                      --N-agents=10\n");			
	printf("\n");
	printf("Defaults: --N-kruhobots=-1 (unspecified)\n");	
	printf("\n");
    }


    sResult convert_MoviScen2XmlTask(const sCommandParameters &parameters)
    {
	sResult result;
	s2DMap real_Map;

        #ifdef sSTATISTICS
	{
	    s_GlobalStatistics.enter_Phase("CONVERSION");
	}
  	#endif

	if (!parameters.m_output_xml_scen_filename.empty())
	{
	    if (!parameters.m_input_movi_map_filename.empty())
	    {
		result = real_Map.from_File_movi(parameters.m_input_movi_map_filename);
		
		if (sFAILED(result))
		{
		    printf("Error: Failed to open movingai.com map file %s (code = %d).\n", parameters.m_input_movi_map_filename.c_str(), result);
		    return result;
		}
	    }
	    sRealInstance real_Instance(&real_Map);
	    
	    if (!parameters.m_input_movi_scen_filename.empty())
	    {
		result = real_Instance.from_File_movi(parameters.m_input_movi_scen_filename);
		
		if (sFAILED(result))
		{
		    printf("Error: Failed to open movingai.com scenario file %s (code = %d).\n", parameters.m_input_movi_scen_filename.c_str(), result);
		    return result;
		}
	    }
	    if (!parameters.m_output_xml_scen_filename.empty())
	    {
		if (parameters.m_N_kruhobots >= 0)
		{
		    result = real_Instance.to_File_xml(parameters.m_output_xml_scen_filename, parameters.m_N_kruhobots);
		}
		else
		{
		    result = real_Instance.to_File_xml(parameters.m_output_xml_scen_filename);
		}
		if (sFAILED(result))
		{
		    printf("Error: Failed to write xml task file %s (code = %d).\n", parameters.m_output_xml_scen_filename.c_str(), result);
		    return result;
		}
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


    sResult convert_MoviScen2MultirobotTask(const sCommandParameters &parameters)
    {
	sResult result;
	s2DMap real_Map;

        #ifdef sSTATISTICS
	{
	    s_GlobalStatistics.enter_Phase("CONVERSION");
	}
  	#endif

	if (!parameters.m_output_mpf_filename.empty() || !parameters.m_output_cpf_filename.empty() || !parameters.m_output_bgu_filename.empty())
	{
	    sInstance mapf_Instance;

	    if (!parameters.m_input_movi_map_filename.empty())
	    {
		result = mapf_Instance.m_environment.from_File_movi(parameters.m_input_movi_map_filename);
		
		if (sFAILED(result))
		{
		    printf("Error: Failed to open movingai.com map file %s (code = %d).\n", parameters.m_input_movi_map_filename.c_str(), result);
		    return result;
		}
	    }
	    
	    if (!parameters.m_input_movi_scen_filename.empty())
	    {
		result = mapf_Instance.from_File_movi(parameters.m_input_movi_scen_filename, mapf_Instance.m_environment, parameters.m_N_agents);
		
		if (sFAILED(result))
		{
		    printf("Error: Failed to open movingai.com scenario file %s (code = %d).\n", parameters.m_input_movi_scen_filename.c_str(), result);
		    return result;
		}
	    }	    
	    
	    if (!parameters.m_output_mpf_filename.empty())
	    {
		if (parameters.m_N_agents >= 0)
		{
		    result = mapf_Instance.to_File_mpf(parameters.m_output_mpf_filename, parameters.m_N_agents);
		}
		else
		{
		    result = mapf_Instance.to_File_mpf(parameters.m_output_mpf_filename);
		}
		if (sFAILED(result))
		{
		    printf("Error: Failed to write mpf file %s (code = %d).\n", parameters.m_output_mpf_filename.c_str(), result);
		    return result;
		}
	    }

	    if (!parameters.m_output_cpf_filename.empty())
	    {
		if (parameters.m_N_agents >= 0)
		{
		    result = mapf_Instance.to_File_cpf(parameters.m_output_cpf_filename, parameters.m_N_agents);
		}
		else
		{
		    result = mapf_Instance.to_File_cpf(parameters.m_output_cpf_filename);
		}
		if (sFAILED(result))
		{
		    printf("Error: Failed to write cpf file %s (code = %d).\n", parameters.m_output_cpf_filename.c_str(), result);
		    return result;
		}
	    }

	    if (!parameters.m_output_bgu_filename.empty())
	    {
		if (parameters.m_N_agents >= 0)
		{
		    result = mapf_Instance.to_File_bgu(parameters.m_output_bgu_filename, parameters.m_N_agents);
		}
		else
		{
		    result = mapf_Instance.to_File_bgu(parameters.m_output_bgu_filename);
		}
		if (sFAILED(result))
		{
		    printf("Error: Failed to write bgu file %s (code = %d).\n", parameters.m_output_bgu_filename.c_str(), result);
		    return result;
		}
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
	else if (parameter.find("--input-movi-scen-file=") == 0)
	{
	    command_parameters.m_input_movi_scen_filename = parameter.substr(23, parameter.size());
	}	    
	else if (parameter.find("--output-xml-scen-file=") == 0)
	{
	    command_parameters.m_output_xml_scen_filename = parameter.substr(23, parameter.size());
	}
	else if (parameter.find("--output-mpf-file=") == 0)
	{
	    command_parameters.m_output_mpf_filename = parameter.substr(18, parameter.size());
	}
	else if (parameter.find("--output-cpf-file=") == 0)
	{
	    command_parameters.m_output_cpf_filename = parameter.substr(18, parameter.size());
	}
	else if (parameter.find("--output-bgu-file=") == 0)
	{
	    command_parameters.m_output_bgu_filename = parameter.substr(18, parameter.size());
	}			
	else if (parameter.find("--N-kruhobots=") == 0)
	{
	    command_parameters.m_N_kruhobots = sInt_32_from_String(parameter.substr(14, parameter.size()));
	}
	else if (parameter.find("--N-agents=") == 0)
	{
	    command_parameters.m_N_agents = sInt_32_from_String(parameter.substr(11, parameter.size()));
	}		
	else
	{
	    return sMOVISCEN_CONVERT_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
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
	if (command_parameters.m_input_movi_map_filename.empty())
	{
	    printf("Error: No input movingai.com map file specified (code = %d).\n", sMOVISCEN_CONVERT_PROGRAM_NO_MOVI_MAP_FILE_SPECIFIED_ERROR);
	    print_Help();
		
	    return sMOVISCEN_CONVERT_PROGRAM_NO_MOVI_MAP_FILE_SPECIFIED_ERROR;	    
	}

	if (command_parameters.m_input_movi_scen_filename.empty())
	{
	    printf("Error: No input movingai.com scenario file specified (code = %d).\n", sMOVISCEN_CONVERT_PROGRAM_NO_MOVI_SCEN_FILE_SPECIFIED_ERROR);
	    print_Help();
		
	    return sMOVISCEN_CONVERT_PROGRAM_NO_MOVI_SCEN_FILE_SPECIFIED_ERROR;	    
	}	

	    
	if (!command_parameters.m_output_xml_scen_filename.empty())
	{
	    result = convert_MoviScen2XmlTask(command_parameters);
	    
	    if (sFAILED(result))
	    {
		printf("Error: Cannot produce output xml file (code = %d).\n", result);
		return result;
	    }
	}
	
	if (!command_parameters.m_output_mpf_filename.empty() || !command_parameters.m_output_cpf_filename.empty() || !command_parameters.m_output_bgu_filename.empty())
	{
	    result = convert_MoviScen2MultirobotTask(command_parameters);

	    if (sFAILED(result))
	    {
		printf("Error: Cannot produce output mpf file (code = %d).\n", result);		
		return result;
	    }
	}

	if (   command_parameters.m_output_xml_scen_filename.empty()
	    && command_parameters.m_output_mpf_filename.empty()
	    && command_parameters.m_output_cpf_filename.empty()
	    && command_parameters.m_output_bgu_filename.empty())
	{    
	    printf("Error: Neither .xml nor .mpf nor .cpf not .bgu output file specified (code = %d).\n", sMOVISCEN_CONVERT_PROGRAM_NO_OUTPUT_FILE_SPECIFIED_ERROR);
	    print_Help();
	    
	    return sMOVISCEN_CONVERT_PROGRAM_NO_OUTPUT_FILE_SPECIFIED_ERROR;		
	}
    }
    else
    {
	print_Help();
    }
    return sRESULT_SUCCESS;
}

