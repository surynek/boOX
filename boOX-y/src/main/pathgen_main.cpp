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
/* pathgen_main.cpp / 1-224_leibniz                                           */
/*----------------------------------------------------------------------------*/
//
//
// Path Graph Instance Generator - main program.
//
// Generates a Multi Robot instance on a path graph.
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

#include "core/agent.h"
#include "core/graph.h"
#include "util/statistics.h"

#include "main/pathgen_main.h"


using namespace std;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/

  sCommandParameters::sCommandParameters()
      : m_walk(false)
      , m_N_vertices(10)
      , m_N_agents(5)
      , m_seed(0)
  {
      // nothing
  }


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void)
    {
	printf("----------------------------------------------------------------\n");
	printf("%s : Path Instance Generator\n", sPRODUCT);
	printf("%s\n", sCOPYRIGHT);
	printf("================================================================\n");	
    }


    void print_ConcludingMessage(void)
    {
	printf("----------------------------------------------------------------\n");
    }


    void print_Help(void)
    {
	printf("Usage:\n");
	printf("pathgen_boOX  --N-vertices=<int>\n");
	printf("              --N-agents=<int>\n");
	printf("             [--walk]\n");		
	printf("             [--seed=<int>\n");
	printf("             [--cpf-file=<string>]\n");
	printf("             [--mpf-file=<string>]\n");		
	printf("             [--pddl-domain-file=<string>]\n");
	printf("             [--pddl-problem-file=<string>]\n");
	printf("             [--bgu-file=<string>]\n");
	printf("             [--map-file=<string>]\n");
	printf("             [--usc-map-file=<string>]\n");
	printf("             [--usc-agents-file=<string>]\n");
	printf("\n");
	printf("Examples:\n");
	printf("pathgen_boOX --N-vertices=10\n");
	printf("             --N-agents=5\n");
	printf("             --seed=12345\n");
	printf("             --mpf-file=grid_04x04_a05.mpf\n");
	printf("\n");
	printf("Defaults: --N-vertices=10\n");
	printf("          --N-agents=5\n");
	printf("          --seed=0\n");
	printf("\n");
    }


    sResult generate_PathInstance(const sCommandParameters &parameters)
    {
	sResult result;

	srand(parameters.m_seed);

	sInstance instance;
	sUndirectedGraph environment;

	if (!parameters.m_usc_map_filename.empty())
	{
	    if (!parameters.m_usc_agents_filename.empty())
	    {
		result = instance.from_File_usc(parameters.m_usc_map_filename, parameters.m_usc_agents_filename);

		if (sFAILED(result))
		{
		    printf("Error: Failed to read usc map and agents files %s,%s (code = %d).\n", parameters.m_usc_map_filename.c_str(), parameters.m_usc_agents_filename.c_str(), result);
		    return result;
		}
		instance.to_Screen();
	    }
	    else		
	    {
		result = environment.from_File_usc(parameters.m_usc_map_filename);
				
		sConfiguration initial_configuration;		
		if (sFAILED(result = initial_configuration.generate_Nonconflicting(environment.get_VertexCount(), sMIN(parameters.m_N_agents, environment.get_VertexCount()), environment)))
		{
		    printf("Error: Failed to generate initial configuration because of too many conflicts.\n");
		    return result;
		}
		sConfiguration goal_configuration(environment.get_VertexCount(), parameters.m_N_agents);		
		goal_configuration.generate_NovelNonconflictingWalk(initial_configuration, environment);
		instance = sInstance(environment, initial_configuration, goal_configuration);
	    }
	}
	else
	{       
	    if (!parameters.m_map_filename.empty())
	    {	    
		result = environment.from_File_map(parameters.m_map_filename);
		
		if (sFAILED(result))
		{
		    printf("Error: Failed to read map file %s (code = %d).\n", parameters.m_map_filename.c_str(), result);
		    return result;
		}
	    }
	    else
	    {
		environment.generate_Path(parameters.m_N_vertices);
	    }
	    sConfiguration initial_configuration(environment.get_VertexCount(), sMIN(parameters.m_N_agents, environment.get_VertexCount()), true);
	    
	    if (parameters.m_walk)
	    {
		sConfiguration goal_configuration(environment.get_VertexCount(), parameters.m_N_agents);
		goal_configuration.generate_NovelWalk(initial_configuration, environment);
		instance = sInstance(environment, initial_configuration, goal_configuration);
	    }
	    else
	    {
		sConfiguration goal_configuration(environment.get_VertexCount(), sMIN(parameters.m_N_agents, environment.get_VertexCount()), true);
		instance = sInstance(environment, initial_configuration, goal_configuration);
	    }
	}
	
	if (!parameters.m_pddl_problem_filename.empty())
	{
	    result = instance.to_File_problemPDDL(parameters.m_pddl_problem_filename);

	    if (sFAILED(result))
	    {
		printf("Error: Failed to write PDDL problem file %s (code = %d).\n", parameters.m_pddl_problem_filename.c_str(), result);
		return result;
	    }
	}
	if (!parameters.m_pddl_domain_filename.empty())
	{
	    result = instance.to_File_domainPDDL(parameters.m_pddl_domain_filename);

	    if (sFAILED(result))
	    {
		printf("Error: Failed to write PDDL domain file %s (code = %d).\n", parameters.m_pddl_domain_filename.c_str(), result);
		return result;
	    }
	}
	if (!parameters.m_cpf_filename.empty())
	{
	    result = instance.to_File_cpf(parameters.m_cpf_filename);

	    if (sFAILED(result))
	    {
		printf("Error: Failed to write multirobot file %s (code = %d).\n", parameters.m_cpf_filename.c_str(), result);
		return result;
	    }
	}
	if (!parameters.m_mpf_filename.empty())
	{
	    result = instance.to_File_mpf(parameters.m_mpf_filename);

	    if (sFAILED(result))
	    {
		printf("Error: Failed to write multirobot file %s (code = %d).\n", parameters.m_mpf_filename.c_str(), result);
		return result;
	    }
	}	
	if (!parameters.m_bgu_filename.empty())
	{
	    result = instance.to_File_bgu(parameters.m_bgu_filename, "", 1);

	    if (sFAILED(result))
	    {
		printf("Error: Failed to write multirobot file %s (code = %d).\n", parameters.m_bgu_filename.c_str(), result);
		return result;
	    }
	}
	#ifdef sSTATISTICS
	{
	  s_GlobalStatistics.to_Screen();
	}
	#endif

	return sRESULT_SUCCESS;
    }


    sResult parse_CommandLineParameter(const sString &parameter, sCommandParameters &command_parameters)
    {
	if (parameter.find("--N-vertices=") == 0)
	{
	    command_parameters.m_N_vertices = sInt_32_from_String(parameter.substr(13, parameter.size()));
	}
	else if (parameter.find("--N-agents=") == 0)
	{
	    command_parameters.m_N_agents = sInt_32_from_String(parameter.substr(11, parameter.size()));
	}
	else if (parameter.find("--seed=") == 0)
	{
	    command_parameters.m_seed = sInt_32_from_String(parameter.substr(7, parameter.size()));
	}
	else if (parameter.find("--walk") == 0)
	{
	    command_parameters.m_walk = true;
	}	
	else if (parameter.find("--pddl-domain-file=") == 0)
	{
	    command_parameters.m_pddl_domain_filename = parameter.substr(19, parameter.size());
	}
	else if (parameter.find("--pddl-problem-file=") == 0)
	{
	    command_parameters.m_pddl_problem_filename = parameter.substr(20, parameter.size());
	}
	else if (parameter.find("--cpf-file=") == 0)
	{
	    command_parameters.m_cpf_filename = parameter.substr(11, parameter.size());
	}
	else if (parameter.find("--mpf-file=") == 0)
	{
	    command_parameters.m_mpf_filename = parameter.substr(11, parameter.size());
	}	
	else if (parameter.find("--map-file=") == 0)
	{
	    command_parameters.m_map_filename = parameter.substr(11, parameter.size());
	}
	else if (parameter.find("--usc-map-file=") == 0)
	{
	    command_parameters.m_usc_map_filename = parameter.substr(15, parameter.size());
	}
	else if (parameter.find("--usc-agents-file=") == 0)
	{
	    command_parameters.m_usc_agents_filename = parameter.substr(18, parameter.size());
	}		
	else if (parameter.find("--bgu-file=") == 0)
	{
	    command_parameters.m_bgu_filename = parameter.substr(11, parameter.size());
	}
	else
	{
	    return sPATHGEN_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
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
	result = generate_PathInstance(command_parameters);
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
