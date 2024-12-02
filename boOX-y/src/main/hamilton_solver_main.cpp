/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             boOX 2-213_planck                             */
/*                                                                            */
/*                  (C) Copyright 2018 - 2022 Pavel Surynek                  */
/*                                                                            */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* hamilton_solver_main.cpp / 2-213_planck                                    */
/*----------------------------------------------------------------------------*/
//
// Multi-Agent Hamiltonian Path Finding Solver - main program.
//
// CBS-based and SMT-based solvers for multi-agent hamiltonian
// path finding problem.
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
#include "core/cnf.h"
#include "core/smtcbs.h"
#include "util/statistics.h"

#include "main/hamilton_solver_main.h"


using namespace std;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/

  sCommandParameters::sCommandParameters()
      : m_cost_limit(65536)
      , m_algorithm(ALGORITHM_SMTCBS_PLUS_PLUS)
      , m_timeout(-1.0)
  {
      // nothing
  }


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void)
    {
	printf("------------------------------------------------------------------------\n");
	printf("%s : Multi-Agent Hamiltonian Path Finding (MAHPF) Solver\n", sPRODUCT);
	printf("%s\n", sCOPYRIGHT);
	printf("========================================================================\n");	
    }


    void print_ConcludingMessage(void)
    {
	printf("------------------------------------------------------------------------\n");
    }


    void print_Help(void)
    {
	printf("Usage:\n");
	printf("hamilton_solver_boOX  --input-file=<string>\n");
	printf("                      --output-file=<sting>\n");
	printf("                     [--cost-limit=<int>]\n");
	printf("                     [--algorithm={cbs+++|cbs++++|cbs#|cbs#+|smtcbs++|smtcbs+++|smtcbs#|smtcbs#+}]\n");
        printf("		     [--timeout=<double>]\n");
	printf("\n");
	printf("Examples:\n");
	printf("hamilton_solver_boOX --input-file=grid_02x02_a04.mHpf\n");
	printf("                     --output-file=output.txt\n");
	printf("\n");
	printf("Defaults: --cost-limit=65536\n");
	printf("          --algorithm=smtcbs++\n");
	printf("          --timeout=-1.0 (unlimited)\n");
	printf("\n");
    }


    sResult solve_MultiAgentHamiltonianPathFindingMission(const sCommandParameters &parameters)
    {
	sResult result;
	sMission mission;
	
	if (!parameters.m_input_filename.empty())
	{
	    result = mission.from_File_mHpf(parameters.m_input_filename);

	    if (sFAILED(result))
	    {
		printf("Error: Failed to open multi-agent hamiltonian path finding mission %s (code = %d).\n", parameters.m_input_filename.c_str(), result);
		return result;
	    }
	}

	sSolution solution;
	sInt_32 cost;
	
	switch (parameters.m_algorithm)
	{
	case sCommandParameters::ALGORITHM_CBS_PLUS_PLUS_PLUS:
	{
            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.enter_Phase("CBS-PLUS-PLUS-PLUS");
	    }
  	    #endif
	    sCBS cbs_Solver(&mission, parameters.m_timeout);
	    cost = cbs_Solver.find_ShortestNonconflictingHamiltonian_DeltaSuperStar(solution, parameters.m_cost_limit);
	    break;
	}
	case sCommandParameters::ALGORITHM_CBS_PLUS_PLUS_PLUS_PLUS:
	{
            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.enter_Phase("CBS-PLUS-PLUS-PLUS-PLUS");
	    }
  	    #endif
	    sCBS cbs_Solver(&mission, parameters.m_timeout);
	    cost = cbs_Solver.find_ShortestNonconflictingHamiltonian_DeltaHyperStar(solution, parameters.m_cost_limit);
	    break;
	}
	case sCommandParameters::ALGORITHM_CBS_ULTRA:
	{
            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.enter_Phase("CBS-ULTRA");
	    }
  	    #endif
	    sCBS cbs_Solver(&mission, parameters.m_timeout);
	    cost = cbs_Solver.find_ShortestNonconflictingHamiltonian_DeltaUltraStar(solution, parameters.m_cost_limit);
	    break;
	}	
	case sCommandParameters::ALGORITHM_CBS_ULTRA_PLUS:
	{
            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.enter_Phase("CBS-ULTRA-PLUS");
	    }
  	    #endif
	    sCBS cbs_Solver(&mission, parameters.m_timeout);
	    cost = cbs_Solver.find_ShortestNonconflictingHamiltonian_DeltaUltraStarPlus(solution, parameters.m_cost_limit);
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
	    sSMTCBS smtcbs_Solver(&encoder, &mission, parameters.m_timeout);
	    cost = smtcbs_Solver.find_ShortestNonconflictingHamiltonianInverseDepleted(solution, parameters.m_cost_limit);
	    break;
	}
	case sCommandParameters::ALGORITHM_SMTCBS_PLUS_PLUS_PLUS:
	{
            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.enter_Phase("SMTCBS-PLUS-PLUS-PLUS");
	    }
	    #endif
	    
	    sBoolEncoder encoder;
	    sSMTCBS smtcbs_Solver(&encoder, &mission, parameters.m_timeout);
	    cost = smtcbs_Solver.find_ShortestNonconflictingHamiltonianInverseDepletedSpanning(solution, parameters.m_cost_limit);
	    break;
	}	
	case sCommandParameters::ALGORITHM_SMTCBS_ULTRA:
	{
            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.enter_Phase("SMTCBS-ULTRA");
	    }
	    #endif
	    
	    sBoolEncoder encoder;
	    sSMTCBS smtcbs_Solver(&encoder, &mission, parameters.m_timeout);
	    cost = smtcbs_Solver.find_ShortestNonconflictingHamiltonianInverseDepletedHamilton(solution, parameters.m_cost_limit);
	    break;
	}
	case sCommandParameters::ALGORITHM_SMTCBS_ULTRA_PLUS:
	{
            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.enter_Phase("SMTCBS-ULTRA-PLUS");
	    }
	    #endif
	    
	    sBoolEncoder encoder;
	    sSMTCBS smtcbs_Solver(&encoder, &mission, parameters.m_timeout);
	    cost = smtcbs_Solver.find_ShortestNonconflictingHamiltonianInverseDepletedHamiltonPlus(solution, parameters.m_cost_limit);
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
		printf("The input mission is UNSOLVABLE within a cost smaller than %d.\n", parameters.m_cost_limit);
	    }
	    else if (cost == -2)
	    {
		printf("The answer is INDETERMINATE for the input mission under given timeout of %.3f seconds.\n", parameters.m_timeout);
	    }
	    else
	    {
		printf("The mission cannot be solver from UNKNOWN reason.\n");
	    }	    
	}
	else
	{
	    printf("The input mission is SOLVABLE with a solution of cost %d !\n", cost);
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
	else if (parameter.find("--algorithm=") == 0)
	{
	    sString algorithm_str = parameter.substr(12, parameter.size());

	    if (algorithm_str == "cbs+++")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_CBS_PLUS_PLUS_PLUS;
	    }
	    else if (algorithm_str == "cbs++++")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_CBS_PLUS_PLUS_PLUS_PLUS;
	    }
	    else if (algorithm_str == "cbs#")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_CBS_ULTRA;
	    }
	    else if (algorithm_str == "cbs#+")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_CBS_ULTRA_PLUS;
	    }	    	    	    	    
	    else if (algorithm_str == "smtcbs++")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_SMTCBS_PLUS_PLUS;
	    }
	    else if (algorithm_str == "smtcbs+++")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_SMTCBS_PLUS_PLUS_PLUS;
	    }
	    else if (algorithm_str == "smtcbs#")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_SMTCBS_ULTRA;
	    }
	    else if (algorithm_str == "smtcbs#+")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_SMTCBS_ULTRA_PLUS;
	    }	    	    	    
	    else
	    {
		return sHAMILTON_SOLVER_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
	    }
	}
	else if (parameter.find("--timeout=") == 0)
	{
	    command_parameters.m_timeout = sDouble_from_String(parameter.substr(10, parameter.size()));
	}	
	else
	{
	    return sHAMILTON_SOLVER_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
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
	    printf("Error: Input file name missing (code = %d).\n", sHAMILTON_SOLVER_PROGRAM_MISSING_INPUT_FILE_ERROR);
	    return sHAMILTON_SOLVER_PROGRAM_MISSING_INPUT_FILE_ERROR;
	}	
	result = solve_MultiAgentHamiltonianPathFindingMission(command_parameters);
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
