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
/* mapfR_solver_main.cpp / 1-224_leibniz                                      */
/*----------------------------------------------------------------------------*/
//
// Continuous Multi-Agent Path Finding Solver (MAPF-R) - main program.
//
// CBS-based and SMT-based solver for Continuous Multi-Agent Path Finding.
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

#include "core/kruhoR.h"
#include "core/mapR.h"
#include "core/cbsR.h"
#include "core/smtcbsR.h"
#include "util/statistics.h"

#include "main/mapfR_solver_main.h"


using namespace std;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/

  sCommandParameters::sCommandParameters()
      : m_makespan_limit(65536.0)
      , m_algorithm(ALGORITHM_CBS_R)	
      , m_timeout(-1.0)
  {
      // nothing
  }


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void)
    {
	printf("------------------------------------------------------------------------\n");
	printf("%s : Continuous Multi-Agent Path Finding (MAPF-R) Solver\n", sPRODUCT);
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
	printf("mapfR_solver_boOX  --input-mapR-file=<string>\n");
	printf("                   --input-kruhoR-file=<sting>\n");
	printf("                   --output-file=<sting>\n");
	printf("                  [--makespan-limit=<double>]\n");
	printf("                  [--algorithm={cbsR|cbsR+|cbsR++|smtcbsR|smtcbsR+|smtcbsR++|smtcbsR+++|smtcbsR4+|smtcbsR*}]\n");
        printf("	 	  [--timeout=<double>]\n");
	printf("\n");
	printf("Examples:\n");
	printf("mapfR_solver_boOX --input-mapR-file=grid_02x02.mapR\n");
	printf("                  --input-kruhoR-file=grid_02x02.kruR\n"); 
	printf("                  --output-file=output.txt\n");
	printf("\n");
	printf("Defaults: --makespan-limit=65536.0\n");
	printf("          --algorithm=cbsR\n");
	printf("          --timeout=-1.0 (unlimited)\n");		
	printf("\n");
    }


    sResult solve_RealMultiAgentPathFindingInstance(const sCommandParameters &command_parameters)
    {
	sResult result;
	sInstance instance;

	s2DMap real_Map;
	
	if (!command_parameters.m_mapR_input_filename.empty())
	{
	    result = real_Map.from_File_mapR(command_parameters.m_mapR_input_filename);

	    if (sFAILED(result))
	    {
		printf("Error: Failed to open continuous multi-agent path finding (MAPF-R) map (mapR) file %s (code = %d).\n", command_parameters.m_mapR_input_filename.c_str(), result);
		return result;
	    }	    
	}

	sRealInstance real_Instance(&real_Map);

	if (!command_parameters.m_kruhoR_input_filename.empty())
	{
	    result = real_Instance.from_File_mpfR(command_parameters.m_kruhoR_input_filename);

	    if (sFAILED(result))
	    {
		printf("Error: Failed to open continuous multi-agent path finding (MAPF-R) kruhobot (kruR) file %s (code = %d).\n", command_parameters.m_kruhoR_input_filename.c_str(), result);
		return result;
	    }

	    if (!command_parameters.m_mapR_input_filename.empty())
	    {
		s2DMap::LocationIDs_vector selected_location_IDs;

		real_Instance.collect_StartingGoalLocations(selected_location_IDs);

//		real_Map.calc_AllPairsStraightDistances();		
		real_Map.calc_NetworkPairsStraightDistances();

		real_Map.calc_SelectedPairsStraightDistances(selected_location_IDs);		
		real_Map.calc_SelectedPairsShortestDistances(selected_location_IDs);
	    }
	}
       
	sDouble makespan;	
	sRealSMTCBS::KruhobotSchedules_vector kruhobot_Schedules;
	
	switch (command_parameters.m_algorithm)
	{
	case sCommandParameters::ALGORITHM_CBS_R:
	{
            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.enter_Phase("CBS-R");
	    }
  	    #endif

	    sRealCBS real_CBS_Solver(&real_Instance, command_parameters.m_timeout);
	    
	    makespan = real_CBS_Solver.find_ShortestNonconflictingSchedules(kruhobot_Schedules, command_parameters.m_makespan_limit);
	    break;
	}
	case sCommandParameters::ALGORITHM_CBS_R_PLUS:
	{
            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.enter_Phase("CBS-R+");
	    }
  	    #endif

	    sRealCBS real_CBS_Solver(&real_Instance, command_parameters.m_timeout);
	    
	    makespan = real_CBS_Solver.find_ShortestNonconflictingSchedules_smart(kruhobot_Schedules, command_parameters.m_makespan_limit);
	    break;
	}
	case sCommandParameters::ALGORITHM_CBS_R_PLUS_PLUS:
	{
            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.enter_Phase("CBS-R++");
	    }
  	    #endif

	    sRealCBS real_CBS_Solver(&real_Instance, command_parameters.m_timeout);
	    
	    makespan = real_CBS_Solver.find_ShortestNonconflictingSchedules_strong(kruhobot_Schedules, command_parameters.m_makespan_limit);
	    break;
	}		
	case sCommandParameters::ALGORITHM_SMTCBS_R:
	{
            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.enter_Phase("SMT-CBS-R");
	    }
  	    #endif

	    sBoolEncoder boolean_Encoder;
	    sRealSMTCBS real_SMTCBS_Solver(&boolean_Encoder, &real_Instance, command_parameters.m_timeout);
	    
	    makespan = real_SMTCBS_Solver.find_ShortestNonconflictingSchedules(kruhobot_Schedules, command_parameters.m_makespan_limit);
	    break;
	}
	case sCommandParameters::ALGORITHM_SMTCBS_R_PLUS:
	{
            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.enter_Phase("SMT-CBS-R+");
	    }
  	    #endif

	    sBoolEncoder boolean_Encoder;
	    sRealSMTCBS real_SMTCBS_Solver(&boolean_Encoder, &real_Instance, command_parameters.m_timeout);
	    
	    makespan = real_SMTCBS_Solver.find_ShortestNonconflictingSchedules_pruningSmart(kruhobot_Schedules, command_parameters.m_makespan_limit);
	    break;
	}
	case sCommandParameters::ALGORITHM_SMTCBS_R_PLUS_PLUS:
	{
            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.enter_Phase("SMT-CBS-R++");
	    }
  	    #endif

	    sBoolEncoder boolean_Encoder;
	    sRealSMTCBS real_SMTCBS_Solver(&boolean_Encoder, &real_Instance, command_parameters.m_timeout);
	    
	    makespan = real_SMTCBS_Solver.find_ShortestNonconflictingSchedules_pruningStrong(kruhobot_Schedules, command_parameters.m_makespan_limit);
	    break;
	}
	case sCommandParameters::ALGORITHM_SMTCBS_R_PLUS_PLUS_PLUS:
	{
            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.enter_Phase("SMT-CBS-R+++");
	    }
  	    #endif

	    sBoolEncoder boolean_Encoder;
	    sRealSMTCBS real_SMTCBS_Solver(&boolean_Encoder, &real_Instance, command_parameters.m_timeout);
	    
	    makespan = real_SMTCBS_Solver.find_ShortestNonconflictingSchedules_conflictRespectful(kruhobot_Schedules, command_parameters.m_makespan_limit);
	    break;
	}
	case sCommandParameters::ALGORITHM_SMTCBS_R_4_PLUS:
	{
            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.enter_Phase("SMT-CBS-R-4+");
	    }
  	    #endif

	    sBoolEncoder boolean_Encoder;
	    sRealSMTCBS real_SMTCBS_Solver(&boolean_Encoder, &real_Instance, command_parameters.m_timeout);
	    
	    makespan = real_SMTCBS_Solver.find_ShortestNonconflictingSchedules_individualizedConflictRespectful(kruhobot_Schedules, command_parameters.m_makespan_limit);
	    break;
	}
	case sCommandParameters::ALGORITHM_SMTCBS_R_STAR:
	{
            #ifdef sSTATISTICS
	    {
		s_GlobalStatistics.enter_Phase("SMT-CBS-R*");
	    }
  	    #endif

	    sBoolEncoder boolean_Encoder;
	    sRealSMTCBS real_SMTCBS_Solver(&boolean_Encoder, &real_Instance, command_parameters.m_timeout);
	    
	    makespan = real_SMTCBS_Solver.find_ExactShortestNonconflictingSchedules_individualizedConflictRespectful(kruhobot_Schedules, command_parameters.m_makespan_limit);
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}

	if (makespan < 0)
	{
	    if (makespan == -1.0)
	    {
		printf("The input instance is UNSOLVABLE within a makespan smaller than %.3f\n", command_parameters.m_makespan_limit);
	    }
	    else if (makespan == -2.0)
	    {
		printf("The answer is INDETERMINATE for the input instance under given timeout of %.3f seconds.\n", command_parameters.m_timeout);
	    }
	    else if (makespan == -3.0)
	    {
		printf("The answer is INDETERMINATE for the input instance under given makespan limit of %.3f\n", command_parameters.m_makespan_limit);		
	    }
	    else
	    {
		printf("The instance cannot be solver from UNKNOWN reason.\n");
	    }	    
	}
	else
	{
	    printf("The input instance is SOLVABLE with a solution of makespan %.3f !\n", makespan);

	    sRealCBSBase::to_Screen(kruhobot_Schedules);	    
	    
	    if (!command_parameters.m_output_filename.empty())
	    {
		result = sRealCBSBase::to_File(command_parameters.m_output_filename, kruhobot_Schedules);	    
		
		if (sFAILED(result))
		{
		    printf("Error: Failed to write problem solution to %s (code = %d).\n", command_parameters.m_output_filename.c_str(), result);
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
	if (parameter.find("--input-mapR-file=") == 0)
	{
	    command_parameters.m_mapR_input_filename = parameter.substr(18, parameter.size());
	}
	else if (parameter.find("--input-kruhoR-file=") == 0)
	{
	    command_parameters.m_kruhoR_input_filename = parameter.substr(20, parameter.size());
	}	
	else if (parameter.find("--output-file=") == 0)
	{
	    command_parameters.m_output_filename = parameter.substr(14, parameter.size());
	}	
	else if (parameter.find("--makespan-limit=") == 0)
	{
	    command_parameters.m_makespan_limit = sDouble_from_String(parameter.substr(17, parameter.size()));
	}	
	else if (parameter.find("--algorithm=") == 0)
	{
	    sString algorithm_str = parameter.substr(12, parameter.size());

	    if (algorithm_str == "cbsR")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_CBS_R;
	    }
	    else if (algorithm_str == "cbsR+")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_CBS_R_PLUS;
	    }
	    else if (algorithm_str == "cbsR++")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_CBS_R_PLUS_PLUS;
	    }	    	    
	    else if (algorithm_str == "smtcbsR")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_SMTCBS_R;
	    }
	    else if (algorithm_str == "smtcbsR+")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_SMTCBS_R_PLUS;
	    }
	    else if (algorithm_str == "smtcbsR++" || algorithm_str == "smtcbsR2+")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_SMTCBS_R_PLUS_PLUS;
	    }
	    else if (algorithm_str == "smtcbsR+++" || algorithm_str == "smtcbsR3+")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_SMTCBS_R_PLUS_PLUS_PLUS;
	    }
	    else if (algorithm_str == "smtcbsR++++" || algorithm_str == "smtcbsR4+")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_SMTCBS_R_4_PLUS;
	    }
	    else if (algorithm_str == "smtcbsR*" || algorithm_str == "smtcbsR*")
	    {
		command_parameters.m_algorithm = sCommandParameters::ALGORITHM_SMTCBS_R_STAR;
	    }	    	    	    	    	    
	    else
	    {
		return sMAPF_R_SOLVER_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
	    }
	}
	else if (parameter.find("--timeout=") == 0)
	{
	    command_parameters.m_timeout = sDouble_from_String(parameter.substr(10, parameter.size()));
	}	
	else
	{
	    return sMAPF_R_SOLVER_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
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
	if (command_parameters.m_mapR_input_filename.empty())
	{
	    printf("Error: Map file name missing (code = %d).\n", sMAPF_R_SOLVER_PROGRAM_MISSING_MAP_FILE_ERROR);
	    return sMAPF_R_SOLVER_PROGRAM_MISSING_MAP_FILE_ERROR;
	}
	if (command_parameters.m_kruhoR_input_filename.empty())
	{
	    printf("Error: Kruhobot file name missing (code = %d).\n", sMAPF_R_SOLVER_PROGRAM_MISSING_KRUHOBOT_FILE_ERROR);
	    return sMAPF_R_SOLVER_PROGRAM_MISSING_KRUHOBOT_FILE_ERROR;
	}		
	result = solve_RealMultiAgentPathFindingInstance(command_parameters);
	
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

