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
/* rota_solver_main.cpp / 1-224_leibniz                                       */
/*----------------------------------------------------------------------------*/
//
// Token Rotation Problem Solver - main program.
//
// CBS-based and SMT-based solvers for token rotation problem (swaps excluding).
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
#include "core/smtcbs.h"
#include "core/cnf.h"

#include "util/statistics.h"

#include "main/rota_solver_main.h"


using namespace std;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/

  sCommandParameters::sCommandParameters()
      : m_cost_limit(65536)
      , m_capacitated(false)
      , m_subopt_ratio(-1.0)	
      , m_timeout(-1.0)	
  {
      // nothing
  }


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void)
    {
	printf("----------------------------------------------------------------\n");
	printf("%s : Token Rotation Problem Solver\n", sPRODUCT);
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
	printf("swap_solver_boOX  --input-file=<string>\n");
	printf("                  --output-file=<sting>\n");
	printf("                 [--cost-limit=<int>]\n");
	printf("                 [--algorithm={cbs|cbs+|cbs++|cbs+++|smtcbs|smtcbs+|smtcbs++}]\n");
        printf("		 [--subopt-ratio=<double>]\n");		
        printf("		 [--timeout=<double>]\n");
	printf("		 [--capacitated]\n");
	printf("\n");
	printf("Examples:\n");
	printf("tswap_solver_boOX --input-file=grid_02x02_t04.mpf\n");
	printf("                  --output-file=output.txt\n");
	printf("\n");
	printf("Defaults: --cost-limit=65536\n");
	printf("          --algorithm=cbs\n");	
	printf("          --timeout=-1.0 (unlimited)\n");
	printf("          --subopt-ratio=-1.0 (unused = optimal)\n");				
	printf("\n");
    }


    sResult solve_TokenRotationInstance(const sCommandParameters &parameters)
    {
	sResult result;
	sInstance instance;

	if (!parameters.m_input_filename.empty())
	{
	    if (parameters.m_capacitated)
	    {
		result = instance.from_File_cmpf(parameters.m_input_filename);
	    }
	    else
	    {
		result = instance.from_File_mpf(parameters.m_input_filename);
	    }
	    if (sFAILED(result))
	    {
		printf("Error: Failed to open token rotation instance %s (code = %d).\n", parameters.m_input_filename.c_str(), result);
		return result;
	    }
	}

	sSolution solution;
	sInt_32 cost;

	switch (parameters.m_algorithm)
	{
	case sCommandParameters::ALGORITHM_CBS:
	{
            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.enter_Phase("CBS");
	    }
  	    #endif
	    
	    sCBS cbs_Solver(&instance, parameters.m_timeout);
	    
	    if (parameters.m_capacitated)
	    {
		printf("Wrong combination of command line parameters.\n");
		return sROTA_SOLVER_PROGRAM_WRONG_PARAMETERS_ERROR;
	    }
	    else
	    {
		cost = cbs_Solver.find_ShortestNonconflictingRotation(solution, parameters.m_cost_limit);
	    }
	    break;
	}
	case sCommandParameters::ALGORITHM_CBS_PLUS:
	{
            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.enter_Phase("CBS-PLUS");
	    }
  	    #endif
	    
	    sCBS cbs_Solver(&instance, parameters.m_timeout);
	    
	    if (parameters.m_capacitated)
	    {
		printf("Wrong combination of command line parameters.\n");
		return sROTA_SOLVER_PROGRAM_WRONG_PARAMETERS_ERROR;
	    }
	    else
	    {	    
		cost = cbs_Solver.find_ShortestNonconflictingRotation_Delta(solution, parameters.m_cost_limit);
	    }
	    break;
	}
	case sCommandParameters::ALGORITHM_CBS_PLUS_PLUS:
	{
            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.enter_Phase("CBS-PLUS-PLUS");
	    }
  	    #endif
	    
	    sCBS cbs_Solver(&instance, parameters.m_timeout);

	    if (parameters.m_capacitated)
	    {
		printf("Wrong combination of command line parameters.\n");
		return sROTA_SOLVER_PROGRAM_WRONG_PARAMETERS_ERROR;
	    }
	    else
	    {	    
		cost = cbs_Solver.find_ShortestNonconflictingRotation_DeltaStar(solution, parameters.m_cost_limit);
	    }
	    break;
	}
	case sCommandParameters::ALGORITHM_CBS_PLUS_PLUS_PLUS:
	{
            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.enter_Phase("CBS-PLUS-PLUS-PLUS");
	    }
  	    #endif
	    
	    sCBS cbs_Solver(&instance, parameters.m_timeout);

	    if (parameters.m_capacitated)
	    {
		printf("Wrong combination of command line parameters.\n");
		return sROTA_SOLVER_PROGRAM_WRONG_PARAMETERS_ERROR;
	    }
	    else
	    {	    
		cost = cbs_Solver.find_ShortestNonconflictingRotation_DeltaSuperStar(solution, parameters.m_cost_limit);
	    }
	    break;
	}				
	case sCommandParameters::ALGORITHM_SMTCBS:
	{
            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.enter_Phase("SMTCBS");
	    }
	    #endif
	    
	    sBoolEncoder encoder;
	    sSMTCBS smtcbs_Solver(&encoder, parameters.m_subopt_ratio, &instance, parameters.m_timeout);	    

	    if (parameters.m_capacitated)
	    {
		printf("Wrong combination of command line parameters.\n");
		return sROTA_SOLVER_PROGRAM_WRONG_PARAMETERS_ERROR;
	    }
	    else
	    {	    	    
		cost = smtcbs_Solver.find_ShortestNonconflictingRotation(solution, parameters.m_cost_limit);
	    }
	    break;
	}
	case sCommandParameters::ALGORITHM_SMTCBS_PLUS:
	{
            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.enter_Phase("SMTCBS-PLUS");
	    }
	    #endif
	    
	    sBoolEncoder encoder;
	    sSMTCBS smtcbs_Solver(&encoder, parameters.m_subopt_ratio, &instance, parameters.m_timeout);	    	    

	    if (parameters.m_capacitated)
	    {
		printf("Wrong combination of command line parameters.\n");
		return sROTA_SOLVER_PROGRAM_WRONG_PARAMETERS_ERROR;
	    }
	    else
	    {
		cost = smtcbs_Solver.find_ShortestNonconflictingRotationInverse(solution, parameters.m_cost_limit);
	    }
	    break;
	}
	case sCommandParameters::ALGORITHM_SMTCBS_PLUS_PLUS:
	{
            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.enter_Phase("SMTCBS-PLUS-PLUS");
	    }
	    #endif
	    
	    sBoolEncoder encoder;
	    sSMTCBS smtcbs_Solver(&encoder, parameters.m_subopt_ratio, &instance, parameters.m_timeout);	    	    	    

	    if (parameters.m_capacitated)
	    {
		cost = smtcbs_Solver.find_ShortestNonconflictingCapacitatedRotationInverseDepleted(solution, parameters.m_cost_limit);
	    }
	    else
	    {
		cost = smtcbs_Solver.find_ShortestNonconflictingRotationInverseDepleted(solution, parameters.m_cost_limit);
	    }
	    break;
	}				
	default:
	{
	    sASSERT(false);
	    break;
	}
	}       
	if (cost < 0)
	{
	    if (cost == -1)
	    {
		printf("The input instance is UNSOLVABLE within a cost smaller than %d.\n", parameters.m_cost_limit);
	    }
	    else if (cost == -2)
	    {
		printf("The answer is INDETERMINATE for the input instance under given timeout of %.3f seconds.\n", parameters.m_timeout);
	    }
	    else
	    {
		printf("The instance cannot be solver from UNKNOWN reason.\n");
	    }	    
	}
	else
	{
	    printf("The input instance is SOLVABLE with a solution of cost %d !\n", cost);	    
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
	else if (parameter.find("--capacitated") == 0)
	{
	    command_parameters.m_capacitated = true;
	}	
	else if (parameter.find("--algorithm=") == 0)
	{
	    sString algorithm_str = parameter.substr(12, parameter.size());

	    if (algorithm_str == "cbs")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_CBS;
	    }
	    else if (algorithm_str == "cbs+")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_CBS_PLUS;
	    }
	    else if (algorithm_str == "cbs++")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_CBS_PLUS_PLUS;
	    }
	    else if (algorithm_str == "cbs+++")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_CBS_PLUS_PLUS_PLUS;
	    }	    
	    else if (algorithm_str == "smtcbs")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_SMTCBS;
	    }
	    else if (algorithm_str == "smtcbs+")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_SMTCBS_PLUS;
	    }
	    else if (algorithm_str == "smtcbs++")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_SMTCBS_PLUS_PLUS;
	    }	    	    
	    else
	    {
		return sROTA_SOLVER_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
	    }
	}
	else if (parameter.find("--subopt-ratio=") == 0)
	{
	    command_parameters.m_subopt_ratio = sDouble_from_String(parameter.substr(15, parameter.size()));
	}			
	else if (parameter.find("--timeout=") == 0)
	{
	    command_parameters.m_timeout = sDouble_from_String(parameter.substr(10, parameter.size()));
	}		
	else
	{
	    return sROTA_SOLVER_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
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
	if (command_parameters.m_input_filename.empty())
	{
	    printf("Error: Input file name missing (code = %d).\n", sROTA_SOLVER_PROGRAM_MISSING_INPUT_FILE_ERROR);
	    return sROTA_SOLVER_PROGRAM_MISSING_INPUT_FILE_ERROR;
	}	

	result = solve_TokenRotationInstance(command_parameters);
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
