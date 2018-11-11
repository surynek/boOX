/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                              boOX 0_iskra-149                              */
/*                                                                            */
/*                      (C) Copyright 2018 Pavel Surynek                      */
/*                http://www.surynek.com | <pavel@surynek.com>                */
/*                                                                            */
/*                                                                            */
/*============================================================================*/
/* tosolver_main.cpp / 0_iskra-149                                            */
/*----------------------------------------------------------------------------*/
//
// Token Swapping Problem Solver - main program.
//
// A CBS-based solver for token swapping problem.
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
#include "core/cbs.h"
#include "util/statistics.h"

#include "main/tosolver_main.h"


using namespace std;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/

  sCommandParameters::sCommandParameters()
      : m_cost_limit(128)
  {
      // nothing
  }


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void)
    {
	printf("----------------------------------------------------------------\n");
	printf("%s : Token Swapping Problem Solver\n", sPRODUCT);
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
	printf("tknsolver_boOX --input-file=<string>\n");
	printf("               --output-file=<sting>\n");
	printf("               --cost-limit=<int>\n");
	printf("\n");
	printf("Examples:\n");
	printf("tknsolver_boOX --input-file=grid_02x02_t04.tkn\n");
	printf("               --output-file=output.txt\n");
	printf("\n");
	printf("Defaults: --cost-limit=128\n");
	printf("\n");
    }


    sResult solve_TokenSwappingInstance(const sCommandParameters &parameters)
    {
	sResult result;
	sInstance instance;

        #ifdef sSTATISTICS
	{
	    s_GlobalStatistics.enter_Phase("CBS");
	}
	#endif
	
	if (!parameters.m_input_filename.empty())
	{
	    result = instance.from_File_mpf(parameters.m_input_filename);

	    if (sFAILED(result))
	    {
		printf("Error: Failed to open token swapping %s (code = %d).\n", parameters.m_input_filename.c_str(), result);
		return result;
	    }
	}

	sSolution solution;
	sCBS cbs_Solver(&instance);

	sInt_32 cost = cbs_Solver.find_ShortestNonconflictingSwapping(solution, parameters.m_cost_limit);
	if (cost < 0)
	{
	    printf("The input instance is UNSOLVABLE within a cost smaller than %d.\n", parameters.m_cost_limit);
	}
	else
	{
	    printf("Solution of cost %d FOUND !\n", cost);
	    solution.to_Screen();
	    
	    if (!parameters.m_output_filename.empty())
	    {
		result = solution.to_File_mpf(parameters.m_output_filename);
		
		if (sFAILED(result))
		{
		    printf("Error: Failed to write problem solution to %s (code = %d).\n", parameters.m_output_filename.c_str(), result);
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
	if (parameter.find("--input-file=") == 0)
	{
	    command_parameters.m_input_filename = parameter.substr(13, parameter.size());
	}
	else if (parameter.find("--output-file=") == 0)
	{
	    command_parameters.m_output_filename = parameter.substr(14, parameter.size());
	}
	else if (parameter.find("--cost-limit=") == 0)
	{
	    command_parameters.m_cost_limit = sInt_32_from_String(parameter.substr(13, parameter.size()));
	}	
	else
	{
	    return sTOSOLVER_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
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
	result = solve_TokenSwappingInstance(command_parameters);
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

