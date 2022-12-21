/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             boOX 2-189_planck                              */
/*                                                                            */
/*                  (C) Copyright 2018 - 2022 Pavel Surynek                   */
/*                                                                            */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* smtcbsR.cpp / 2-189_planck                                                 */
/*----------------------------------------------------------------------------*/
//
// Conflict based search for a semi-continuous version of MAPF implemented
// on top of SAT-modulo theories.
//
/*----------------------------------------------------------------------------*/


#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#include <map>

#include "config.h"
#include "compile.h"
#include "version.h"
#include "defs.h"
#include "result.h"

#include "common/types.h"
#include "core/smtcbsR.h"
#include "util/statistics.h"


using namespace std;
using namespace boOX;


/*----------------------------------------------------------------------------*/




namespace boOX
{

    
/*----------------------------------------------------------------------------*/
// sRealSMTCBS

    sRealSMTCBS::sRealSMTCBS(sBoolEncoder *solver_Encoder, sRealInstance *real_Instance)
	: sRealCBSBase(real_Instance)
	, sSMTCBSBase(solver_Encoder)
    {
	// nothing
    }


    sRealSMTCBS::sRealSMTCBS(sBoolEncoder *solver_Encoder, sRealInstance *real_Instance, sDouble timeout)
	: sRealCBSBase(real_Instance, timeout)
	, sSMTCBSBase(solver_Encoder)
    {
	// nothing
    }    

    
/*----------------------------------------------------------------------------*/

    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules(sRealSolution &real_Solution, sDouble cost_limit)
    {
	return find_ShortestNonconflictingSchedules(*m_real_Instance, real_Solution, cost_limit);	
    }

  
    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules(const sRealInstance &real_Instance,
							      sRealSolution       &sUNUSED(real_Solution),
							      sDouble              makespan_limit)
    {
	sInt_32 cost;
	KruhobotSchedules_vector kruhobot_Schedules;

	if ((cost = find_ShortestNonconflictingSchedules(real_Instance, kruhobot_Schedules, makespan_limit)) < 0)
	{
	    return cost;
	}
/*
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    #ifdef sDEBUG
	    {
		printf("Agent %d: ", agent_id);
	    }
	    #endif
	    for (sInt_32 i = 1; i < agent_Paths[agent_id].size(); ++i)
	    {
                #ifdef sDEBUG
		{
		    printf("%d ", agent_Paths[agent_id][i - 1]);
		}
                #endif
		if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i])
		{
		    solution.add_Move(i - 1, sSolution::Move(agent_id, agent_Paths[agent_id][i - 1], agent_Paths[agent_id][i]));
		}
	    }
            #ifdef sDEBUG
	    {
		printf("%d\n", *agent_Paths[agent_id].rbegin());
	    }
            #endif	    
	}	
*/
	return cost;	
    }


    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit)
    {
	return find_ShortestNonconflictingSchedules(*m_real_Instance, kruhobot_Schedules, makespan_limit);	
    }


    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules(const sRealInstance      &real_Instance,
							      KruhobotSchedules_vector &kruhobot_Schedules,
							      sDouble                   makespan_limit)
    {
	return find_ShortestNonconflictingSchedules(real_Instance, kruhobot_Schedules, makespan_limit, 0.0);
    }    

    
    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit, sDouble extra_makespan)
    {
	return find_ShortestNonconflictingSchedules(*m_real_Instance, kruhobot_Schedules, makespan_limit, extra_makespan);	
    }

    
    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules(const sRealInstance      &real_Instance,
							      KruhobotSchedules_vector &kruhobot_Schedules,
							      sDouble                   makespan_limit,
							      sDouble                   extra_makespan)
    {
	sDouble solution_makespan;
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();	
	
	sDouble start_time = sStatistics::get_CPU_Seconds();	

	{    
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving MAPF-R (elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
	    }
	    #endif

//	    KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
//	    KruhobotUlinearConflicts_vector kruhobot_linear_Conflicts;

	    KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	    KruhobotUlinearConflicts_upper_vector kruhobot_linear_Conflicts;	    

	    kruhobot_location_Conflicts.resize(N_kruhobots + 1);
	    kruhobot_linear_Conflicts.resize(N_kruhobots + 1);
	    kruhobot_Schedules.resize(N_kruhobots + 1);

	    if ((solution_makespan = find_NonconflictingSchedules(real_Instance,
								  kruhobot_location_Conflicts,
								  kruhobot_linear_Conflicts,
								  kruhobot_Schedules,
								  makespan_limit,
								  extra_makespan)) >= 0.0)
	    {
		return solution_makespan;
	    }
	    
	    if (m_timeout >= 0.0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2.0;
		}
	    }
	}

	return -1.0;	
    }


/*----------------------------------------------------------------------------*/
    
    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_pruningSmart(sRealSolution &real_Solution, sDouble cost_limit)
    {
	return find_ShortestNonconflictingSchedules_pruningSmart(*m_real_Instance, real_Solution, cost_limit);	
    }

  
    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_pruningSmart(const sRealInstance &real_Instance,
									   sRealSolution       &sUNUSED(real_Solution),
									   sDouble              makespan_limit)
    {
	sInt_32 cost;
	KruhobotSchedules_vector kruhobot_Schedules;

	if ((cost = find_ShortestNonconflictingSchedules_pruningSmart(real_Instance, kruhobot_Schedules, makespan_limit)) < 0)
	{
	    return cost;
	}
/*
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    #ifdef sDEBUG
	    {
		printf("Agent %d: ", agent_id);
	    }
	    #endif
	    for (sInt_32 i = 1; i < agent_Paths[agent_id].size(); ++i)
	    {
                #ifdef sDEBUG
		{
		    printf("%d ", agent_Paths[agent_id][i - 1]);
		}
                #endif
		if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i])
		{
		    solution.add_Move(i - 1, sSolution::Move(agent_id, agent_Paths[agent_id][i - 1], agent_Paths[agent_id][i]));
		}
	    }
            #ifdef sDEBUG
	    {
		printf("%d\n", *agent_Paths[agent_id].rbegin());
	    }
            #endif	    
	}	
*/
	return cost;	
    }


    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_pruningSmart(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit)
    {
	return find_ShortestNonconflictingSchedules_pruningSmart(*m_real_Instance, kruhobot_Schedules, makespan_limit);	
    }


    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_pruningSmart(const sRealInstance      &real_Instance,
									   KruhobotSchedules_vector &kruhobot_Schedules,
									   sDouble                   makespan_limit)
    {
	return find_ShortestNonconflictingSchedules_pruningSmart(real_Instance, kruhobot_Schedules, makespan_limit, 0.0);
    }    

    
    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_pruningSmart(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit, sDouble extra_makespan)
    {
	return find_ShortestNonconflictingSchedules_pruningSmart(*m_real_Instance, kruhobot_Schedules, makespan_limit, extra_makespan);	
    }

    
    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_pruningSmart(const sRealInstance      &real_Instance,
									   KruhobotSchedules_vector &kruhobot_Schedules,
									   sDouble                   makespan_limit,
									   sDouble                   extra_makespan)
    {
	sDouble solution_makespan;
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();	
	
	sDouble start_time = sStatistics::get_CPU_Seconds();	

	{    
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving MAPF-R (elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
	    }
	    #endif

//	    KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
//	    KruhobotUlinearConflicts_vector kruhobot_linear_Conflicts;

	    KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	    KruhobotUlinearConflicts_upper_vector kruhobot_linear_Conflicts;	    

	    kruhobot_location_Conflicts.resize(N_kruhobots + 1);
	    kruhobot_linear_Conflicts.resize(N_kruhobots + 1);
	    kruhobot_Schedules.resize(N_kruhobots + 1);

	    if ((solution_makespan = find_NonconflictingSchedules_pruningSmart(real_Instance,
									       kruhobot_location_Conflicts,
									       kruhobot_linear_Conflicts,
									       kruhobot_Schedules,
									       makespan_limit,
									       extra_makespan)) >= 0.0)
	    {
		return solution_makespan;
	    }
	    
	    if (m_timeout >= 0.0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2.0;
		}
	    }
	}

	return -1.0;	
    }


/*----------------------------------------------------------------------------*/
    
    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_pruningStrong(sRealSolution &real_Solution, sDouble cost_limit)
    {
	return find_ShortestNonconflictingSchedules_pruningStrong(*m_real_Instance, real_Solution, cost_limit);	
    }

  
    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_pruningStrong(const sRealInstance &real_Instance,
									   sRealSolution       &sUNUSED(real_Solution),
									   sDouble              makespan_limit)
    {
	sInt_32 cost;
	KruhobotSchedules_vector kruhobot_Schedules;

	if ((cost = find_ShortestNonconflictingSchedules_pruningStrong(real_Instance, kruhobot_Schedules, makespan_limit)) < 0)
	{
	    return cost;
	}
/*
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    #ifdef sDEBUG
	    {
		printf("Agent %d: ", agent_id);
	    }
	    #endif
	    for (sInt_32 i = 1; i < agent_Paths[agent_id].size(); ++i)
	    {
                #ifdef sDEBUG
		{
		    printf("%d ", agent_Paths[agent_id][i - 1]);
		}
                #endif
		if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i])
		{
		    solution.add_Move(i - 1, sSolution::Move(agent_id, agent_Paths[agent_id][i - 1], agent_Paths[agent_id][i]));
		}
	    }
            #ifdef sDEBUG
	    {
		printf("%d\n", *agent_Paths[agent_id].rbegin());
	    }
            #endif	    
	}	
*/
	return cost;	
    }


    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_pruningStrong(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit)
    {
	return find_ShortestNonconflictingSchedules_pruningStrong(*m_real_Instance, kruhobot_Schedules, makespan_limit);	
    }


    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_pruningStrong(const sRealInstance      &real_Instance,
									   KruhobotSchedules_vector &kruhobot_Schedules,
									   sDouble                   makespan_limit)
    {
	return find_ShortestNonconflictingSchedules_pruningStrong(real_Instance, kruhobot_Schedules, makespan_limit, 0.0);
    }    

    
    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_pruningStrong(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit, sDouble extra_makespan)
    {
	return find_ShortestNonconflictingSchedules_pruningStrong(*m_real_Instance, kruhobot_Schedules, makespan_limit, extra_makespan);	
    }

    
    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_pruningStrong(const sRealInstance      &real_Instance,
									   KruhobotSchedules_vector &kruhobot_Schedules,
									   sDouble                   makespan_limit,
									   sDouble                   extra_makespan)
    {
	sDouble solution_makespan;
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();	
	
	sDouble start_time = sStatistics::get_CPU_Seconds();	

	{    
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving MAPF-R (elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
	    }
	    #endif

//	    KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
//	    KruhobotUlinearConflicts_vector kruhobot_linear_Conflicts;

	    KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	    KruhobotUlinearConflicts_upper_vector kruhobot_linear_Conflicts;	    

	    kruhobot_location_Conflicts.resize(N_kruhobots + 1);
	    kruhobot_linear_Conflicts.resize(N_kruhobots + 1);
	    kruhobot_Schedules.resize(N_kruhobots + 1);

	    if ((solution_makespan = find_NonconflictingSchedules_pruningStrong(real_Instance,
									       kruhobot_location_Conflicts,
									       kruhobot_linear_Conflicts,
									       kruhobot_Schedules,
									       makespan_limit,
									       extra_makespan)) >= 0.0)
	    {
		return solution_makespan;
	    }
	    
	    if (m_timeout >= 0.0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2.0;
		}
	    }
	}

	return -1.0;	
    }


/*----------------------------------------------------------------------------*/
    
    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_conflictRespectful(sRealSolution &real_Solution, sDouble cost_limit)
    {
	return find_ShortestNonconflictingSchedules_conflictRespectful(*m_real_Instance, real_Solution, cost_limit);	
    }

  
    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_conflictRespectful(const sRealInstance &real_Instance,
										 sRealSolution       &sUNUSED(real_Solution),
										 sDouble              makespan_limit)
    {
	sInt_32 cost;
	KruhobotSchedules_vector kruhobot_Schedules;

	if ((cost = find_ShortestNonconflictingSchedules_conflictRespectful(real_Instance, kruhobot_Schedules, makespan_limit)) < 0)
	{
	    return cost;
	}
/*
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    #ifdef sDEBUG
	    {
		printf("Agent %d: ", agent_id);
	    }
	    #endif
	    for (sInt_32 i = 1; i < agent_Paths[agent_id].size(); ++i)
	    {
                #ifdef sDEBUG
		{
		    printf("%d ", agent_Paths[agent_id][i - 1]);
		}
                #endif
		if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i])
		{
		    solution.add_Move(i - 1, sSolution::Move(agent_id, agent_Paths[agent_id][i - 1], agent_Paths[agent_id][i]));
		}
	    }
            #ifdef sDEBUG
	    {
		printf("%d\n", *agent_Paths[agent_id].rbegin());
	    }
            #endif	    
	}	
*/
	return cost;	
    }


    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_conflictRespectful(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit)
    {
	return find_ShortestNonconflictingSchedules_conflictRespectful(*m_real_Instance, kruhobot_Schedules, makespan_limit);	
    }


    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_conflictRespectful(const sRealInstance      &real_Instance,
										 KruhobotSchedules_vector &kruhobot_Schedules,
										 sDouble                   makespan_limit)
    {
	return find_ShortestNonconflictingSchedules_conflictRespectful(real_Instance, kruhobot_Schedules, makespan_limit, 0.0);
    }    

    
    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_conflictRespectful(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit, sDouble extra_makespan)
    {
	return find_ShortestNonconflictingSchedules_conflictRespectful(*m_real_Instance, kruhobot_Schedules, makespan_limit, extra_makespan);	
    }

    
    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_conflictRespectful(const sRealInstance      &real_Instance,
										 KruhobotSchedules_vector &kruhobot_Schedules,
										 sDouble                   makespan_limit,
										 sDouble                   extra_makespan)
    {
	sDouble solution_makespan;
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();	
	
	sDouble start_time = sStatistics::get_CPU_Seconds();	

	{    
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving MAPF-R (elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
	    }
	    #endif

//	    KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
//	    KruhobotUlinearConflicts_vector kruhobot_linear_Conflicts;

	    KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	    KruhobotUlinearConflicts_upper_vector kruhobot_linear_Conflicts;	    

	    kruhobot_location_Conflicts.resize(N_kruhobots + 1);
	    kruhobot_linear_Conflicts.resize(N_kruhobots + 1);
	    kruhobot_Schedules.resize(N_kruhobots + 1);

	    if ((solution_makespan = find_NonconflictingSchedules_conflictRespectful(real_Instance,
										     kruhobot_location_Conflicts,
										     kruhobot_linear_Conflicts,
										     kruhobot_Schedules,
										     makespan_limit,
										     extra_makespan)) >= 0.0)
	    {
		return solution_makespan;
	    }
	    
	    if (m_timeout >= 0.0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2.0;
		}
	    }
	}

	return -1.0;	
    }
    
    
/*----------------------------------------------------------------------------*/

    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_individualizedConflictRespectful(sRealSolution &real_Solution, sDouble cost_limit)
    {
	return find_ShortestNonconflictingSchedules_individualizedConflictRespectful(*m_real_Instance, real_Solution, cost_limit);	
    }

  
    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_individualizedConflictRespectful(const sRealInstance &real_Instance,
											       sRealSolution       &sUNUSED(real_Solution),
											       sDouble              makespan_limit)
    {
	sInt_32 cost;
	KruhobotSchedules_vector kruhobot_Schedules;

	if ((cost = find_ShortestNonconflictingSchedules_individualizedConflictRespectful(real_Instance, kruhobot_Schedules, makespan_limit)) < 0)
	{
	    return cost;
	}
/*
	sInt_32 N_agents = instance.m_start_configuration.get_AgentCount();

	for (sInt_32 agent_id = 1; agent_id <= N_agents; ++agent_id)
	{
	    #ifdef sDEBUG
	    { 	
		printf("Agent %d: ", agent_id);
	    }
	    #endif
	    for (sInt_32 i = 1; i < agent_Paths[agent_id].size(); ++i)
	    {
                #ifdef sDEBUG
		{
		    printf("%d ", agent_Paths[agent_id][i - 1]);
		}
                #endif
		if (agent_Paths[agent_id][i - 1] != agent_Paths[agent_id][i])
		{
		    solution.add_Move(i - 1, sSolution::Move(agent_id, agent_Paths[agent_id][i - 1], agent_Paths[agent_id][i]));
		}
	    }
            #ifdef sDEBUG
	    {
		printf("%d\n", *agent_Paths[agent_id].rbegin());
	    }
            #endif	    
	}	
*/
	return cost;	
    }


    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_individualizedConflictRespectful(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit)
    {
	return find_ShortestNonconflictingSchedules_individualizedConflictRespectful(*m_real_Instance, kruhobot_Schedules, makespan_limit);	
    }


    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_individualizedConflictRespectful(const sRealInstance      &real_Instance,
											       KruhobotSchedules_vector &kruhobot_Schedules,
											       sDouble                   makespan_limit)
    {
	return find_ShortestNonconflictingSchedules_individualizedConflictRespectful(real_Instance, kruhobot_Schedules, makespan_limit, 0.0);
    }    

    
    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_individualizedConflictRespectful(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit, sDouble extra_makespan)
    {
	return find_ShortestNonconflictingSchedules_individualizedConflictRespectful(*m_real_Instance, kruhobot_Schedules, makespan_limit, extra_makespan);	
    }

    
    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules_individualizedConflictRespectful(const sRealInstance      &real_Instance,
											       KruhobotSchedules_vector &kruhobot_Schedules,
											       sDouble                   makespan_limit,
											       sDouble                   extra_makespan)
    {
	sDouble solution_makespan;
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();	
	
	sDouble start_time = sStatistics::get_CPU_Seconds();	

	{    
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving MAPF-R (elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
	    }
	    #endif

//	    KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
//	    KruhobotUlinearConflicts_vector kruhobot_linear_Conflicts;

	    KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	    KruhobotUlinearConflicts_upper_vector kruhobot_linear_Conflicts;	    

	    kruhobot_location_Conflicts.resize(N_kruhobots + 1);
	    kruhobot_linear_Conflicts.resize(N_kruhobots + 1);
	    kruhobot_Schedules.resize(N_kruhobots + 1);

	    if ((solution_makespan = find_NonconflictingSchedules_individualizedConflictRespectful(real_Instance,
												   kruhobot_location_Conflicts,
												   kruhobot_linear_Conflicts,
												   kruhobot_Schedules,
												   makespan_limit,
												   extra_makespan)) >= 0.0)
	    {
		return solution_makespan;
	    }
	    
	    if (m_timeout >= 0.0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2.0;
		}
	    }
	}

	return -1.0;	
    }
    
    
/*----------------------------------------------------------------------------*/
    
    
    sDouble sRealSMTCBS::find_NonconflictingSchedules(const sRealInstance              &real_Instance,
						      KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
						      KruhobotUlinearConflicts_vector   &kruhobot_linear_Conflicts,
						      KruhobotSchedules_vector         &kruhobot_Schedules,
						      sDouble                           sUNUSED(makespan_limit),
						      sDouble                           sUNUSED(extra_makespan))
    {
	sInt_32 last_conflict_id = 0;
	
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();
	sDouble start_time = sStatistics::get_CPU_Seconds();

	RealContext context(0.0);
	{
	    RealModel real_sat_Model;	
	    
	    Glucose::Solver *solver;
	    solver = new Glucose::Solver;
	    solver->s_Glucose_timeout = m_timeout;

	    KruhobotDecisionDiagrams_vector kruhobot_RDDs;
	    kruhobot_RDDs.resize(N_kruhobots + 1);

	    KruhobotDecisionMappings_vector kruhobot_RDD_Mappings;
	    kruhobot_RDD_Mappings.resize(N_kruhobots + 1);	    
	
	    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	    {
		build_KruhobotRealDecisionDiagram(real_Instance.m_Kruhobots[kruhobot_id],
						  *real_Instance.m_start_conjunction.m_Map,
						  real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
						  real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],
						  kruhobot_location_Conflicts[kruhobot_id],
						  kruhobot_linear_Conflicts[kruhobot_id],
						  -1.0,
						  kruhobot_RDDs[kruhobot_id],
						  kruhobot_RDD_Mappings[kruhobot_id]);
	    }

	    kruhobot_Schedules.clear();
	    find_InitialNonconflictingSchedules(solver,
						context,					 
						real_Instance,
						kruhobot_RDDs,
						kruhobot_RDD_Mappings,
						real_sat_Model,
						kruhobot_Schedules);
	    augment_Schedules(real_Instance, kruhobot_Schedules);
	    
	    delete solver;

	    if (m_timeout >= 0.0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2.0;
		}
	    }	    
	}
	sDouble cumulative_makespan;
	KruhobotCollisions_mset kruhobot_Collisions;
	KruhobotCollisions_mset next_kruhobot_Collisions;	

	cumulative_makespan = analyze_NonconflictingSchedules_nonprioritized(real_Instance,
									     kruhobot_Schedules,
									     next_kruhobot_Collisions);
	if (next_kruhobot_Collisions.empty())
	{
	    printf("COLLISION-FREE solution found !\n");
	    
            #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
	    }
            #endif
	    
	    return cumulative_makespan;
	}

	sInt_32 effective_conflicts = 0;	

	for (KruhobotCollisions_mset::const_iterator next_collision = next_kruhobot_Collisions.begin(); next_collision != next_kruhobot_Collisions.end(); ++next_collision)
	{		    
	    if (verify_KruhobotCollisionDuplicities(*next_collision, kruhobot_Collisions))
	    {
		KruhobotAffection_pair kruhobot_affection = resolve_KruhobotCollision(real_Instance,
										      next_collision->m_traversal_A,
										      next_collision->m_traversal_B,
										      kruhobot_location_Conflicts,
										      kruhobot_linear_Conflicts,
										      last_conflict_id);

		sInt_32 total_affection = kruhobot_affection.first + kruhobot_affection.second;
		effective_conflicts += total_affection;

		if (total_affection > 0)
		{
		    kruhobot_Collisions.insert(*next_collision);
		}
	    }
	}

	if (effective_conflicts <= 0)
	{
	    printf("COLLISION-FREE solution found !\n");
	    
            #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
	    }
            #endif
	    
	    return cumulative_makespan;
	}			

  	#ifdef sDEBUG
	{
	    /*
	    printf("All collisions:\n");
	    for (KruhobotCollisions_mset::const_iterator collision = kruhobot_Collisions.begin(); collision != kruhobot_Collisions.end(); ++collision)
	    {
		collision->to_Screen();
	    }
	    */
	}
	#endif		

	sDouble makespan_bound = -1.0;

	while (true)
	{
	    if (m_timeout >= 0.0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2.0;
		}
	    }
	    
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving MAPF-R (elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
	    }
	    #endif

            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_macro_search_Steps;
	    }
            #endif
	    
	    {
		KruhobotDecisionDiagrams_vector kruhobot_RDDs;
		kruhobot_RDDs.resize(N_kruhobots + 1);

		KruhobotDecisionMappings_vector kruhobot_RDD_Mappings;
		kruhobot_RDD_Mappings.resize(N_kruhobots + 1);	    	    
    
		for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
		{
		    sDouble kruhobot_makespan_bound = calc_KruhobotMakespanBound(real_Instance.m_Kruhobots[kruhobot_id],
										 kruhobot_location_Conflicts[kruhobot_id],
										 kruhobot_linear_Conflicts[kruhobot_id]);
		    makespan_bound = sMAX(makespan_bound, kruhobot_makespan_bound);
		}
		
//	    makespan_bound = sMIN(makespan_bound, 5.650);	    
//	    makespan_bound = 5.7;
//	    makespan_bound = 5.5;
//	    makespan_bound = 7.0;
//    	    makespan_bound = 4.65;	    
//	    makespan_bound = 5.7;
//	    makespan_bound = 7.0;
	    
		{
		    RealModel real_sat_Model;	
		    
		    Glucose::Solver *solver;
		    solver = new Glucose::Solver;
		    solver->s_Glucose_timeout = m_timeout;		
		    
		    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
		    {
			sDouble nx_makespan_bound;
			nx_makespan_bound = build_KruhobotRealDecisionDiagram(real_Instance.m_Kruhobots[kruhobot_id],
									      *real_Instance.m_start_conjunction.m_Map,
									      real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
									      real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],					    
									      kruhobot_location_Conflicts[kruhobot_id],
									      kruhobot_linear_Conflicts[kruhobot_id],
									      makespan_bound,
									      kruhobot_RDDs[kruhobot_id],
									      kruhobot_RDD_Mappings[kruhobot_id]);
			sASSERT(nx_makespan_bound > 0.0);			
			
//		    to_Screen(kruhobot_RDDs[kruhobot_id]);
/*
		    printf("Kruhobot location conflicts: %d\n", kruhobot_id);
		    for (LocationConflicts__umap::const_iterator location_Conflict = kruhobot_location_Conflicts[kruhobot_id].begin(); location_Conflict != kruhobot_location_Conflicts[kruhobot_id].end(); ++location_Conflict)
		    {
			printf("  %d, %ld\n", location_Conflict->first, location_Conflict->second.size());
			for (LocationConflicts_map::const_iterator location_conflict = location_Conflict->second.begin(); location_conflict != location_Conflict->second.end(); ++location_conflict)
			{
			    location_conflict->second.to_Screen();
			}
		    }
		    printf("Kruhobot linear conflicts: %d\n", kruhobot_id);
		    for (UlinearConflicts__map::const_iterator linear_Conflict = kruhobot_linear_Conflicts[kruhobot_id].begin(); linear_Conflict != kruhobot_linear_Conflicts[kruhobot_id].end(); ++linear_Conflict)
		    {
			printf("  %ld\n", linear_Conflict->second.size());
			for (UlinearConflicts_map::const_iterator linear_conflict = linear_Conflict->second.begin(); linear_conflict != linear_Conflict->second.end(); ++linear_conflict)
			{
			    linear_conflict->second.to_Screen();
			}
		    }	
*/	    
		    }
		    
		    kruhobot_Schedules.clear();
		    if (!find_NextNonconflictingSchedules(solver,
							  context,					 
							  real_Instance,
							  kruhobot_RDDs,
							  kruhobot_RDD_Mappings,
							  kruhobot_Collisions,
							  real_sat_Model,
							  kruhobot_Schedules))
		    {
			printf("No solution !\n");
			return -1.0;
		    }
		    augment_Schedules(real_Instance, kruhobot_Schedules);		
		    
		    delete solver;		

		    sDouble cumulative_makespan;
		    KruhobotCollisions_mset next_kruhobot_Collisions;		

		    cumulative_makespan = analyze_NonconflictingSchedules_nonprioritized(real_Instance,
											 kruhobot_Schedules,
											 next_kruhobot_Collisions);
		    if (next_kruhobot_Collisions.empty())
		    {
			printf("COLLISION-FREE solution found !\n");
			
                        #ifdef sVERBOSE
			{
			    sDouble end_time = sStatistics::get_CPU_Seconds();
			    printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
			}
 	                #endif
		    
			return cumulative_makespan;
		    }

		/*
  	        #ifdef sDEBUG
		{
		    printf("Next collisions:\n");
		    for (KruhobotCollisions_mset::const_iterator collision = next_kruhobot_Collisions.begin(); collision != next_kruhobot_Collisions.end(); ++collision)
		    {
			collision->to_Screen();
		    }
		}
	        #endif
		*/

		    sInt_32 effective_conflicts = 0;

		    for (KruhobotCollisions_mset::const_iterator next_collision = next_kruhobot_Collisions.begin(); next_collision != next_kruhobot_Collisions.end(); ++next_collision)
		    {		    
			if (verify_KruhobotCollisionDuplicities(*next_collision, kruhobot_Collisions))			
			{
			    KruhobotAffection_pair kruhobot_affection = resolve_KruhobotCollision(real_Instance,
												  next_collision->m_traversal_A,
												  next_collision->m_traversal_B,
												  kruhobot_location_Conflicts,
												  kruhobot_linear_Conflicts,
												  last_conflict_id);
			    
			    sInt_32 total_affection = kruhobot_affection.first + kruhobot_affection.second;
			    effective_conflicts += total_affection;
			    
			    if (total_affection > 0)
			    {
				kruhobot_Collisions.insert(*next_collision);
			    }
			}
		    }
		    
		    if (effective_conflicts <= 0)
		    {
			printf("COLLISION-FREE solution found !\n");
		    
  	                #ifdef sVERBOSE
			{
			    sDouble end_time = sStatistics::get_CPU_Seconds();
			    printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
			}
 	                #endif
		    
			return cumulative_makespan;
		    }
		}
	    }	    
	}	
	
	return -1.0;
    }


    sDouble sRealSMTCBS::find_NonconflictingSchedules(const sRealInstance                    &real_Instance,
						      KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
						      KruhobotUlinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
						      KruhobotSchedules_vector               &kruhobot_Schedules,
						      sDouble                                 makespan_limit,
						      sDouble                                 extra_makespan)
    {
	sInt_32 last_conflict_id = 0;
	
	sDouble makespan_bound = 0.0;	    
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();	

	#ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif	

	RealContext real_context(0.0);
	{
	    RealModel real_sat_Model;
	    
	    Glucose::Solver *solver;
	    solver = new Glucose::Solver;
	    solver->s_Glucose_timeout = m_timeout;

	    KruhobotDecisionDiagrams_vector kruhobot_RDDs;
	    kruhobot_RDDs.resize(N_kruhobots + 1);

	    KruhobotDecisionMappings_vector kruhobot_RDD_Mappings;
	    kruhobot_RDD_Mappings.resize(N_kruhobots + 1);

	    sDouble next_makespan_bound = -1.0;
	    sDouble makespan_lower_bound = 0.0;

	    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	    {
		sDouble kruhobot_makespan_lower_bound;
		
		if ((kruhobot_makespan_lower_bound = find_KruhobotIgnoringSchedule(real_Instance.m_Kruhobots[kruhobot_id],
										   *real_Instance.m_start_conjunction.m_Map,
										   real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
										   real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],
										   makespan_limit,
										   extra_makespan,
										   kruhobot_Schedules[kruhobot_id])) < 0.0)
		{
		    return -1.0;
		}
		makespan_lower_bound = (makespan_lower_bound < kruhobot_makespan_lower_bound) ? kruhobot_makespan_lower_bound : makespan_lower_bound;
		printf("Kruhobot lower bound makespan: %.3f\n", kruhobot_makespan_lower_bound);
	    }
	    real_context.m_makespan_bound = makespan_bound = makespan_lower_bound;
	    
	    #ifdef sDEBUG
	    {
		printf("Lower bound makespan: %.3f\n", makespan_lower_bound);
	    }
	    #endif
	    
            #ifdef sVERBOSE	    
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		
		printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current makespan: %.3f)...\n",
		       (end_time - start_time),
		       makespan_bound);
	    }
	    #endif

	    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	    {
		sDouble nx_makespan_bound;
		nx_makespan_bound = build_KruhobotRealDecisionDiagram_pruning(real_Instance.m_Kruhobots[kruhobot_id],
									      *real_Instance.m_start_conjunction.m_Map,
									      real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
									      real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],					    
									      kruhobot_location_Conflicts[kruhobot_id],
									      kruhobot_linear_Conflicts[kruhobot_id],
									      makespan_lower_bound,
									      kruhobot_RDDs[kruhobot_id],
									      kruhobot_RDD_Mappings[kruhobot_id]);
		//sASSERT(nx_makespan_bound > 0.0);
		    
		next_makespan_bound = (next_makespan_bound < 0.0) ? nx_makespan_bound : sMIN(next_makespan_bound, nx_makespan_bound);
	    }
            #ifdef sVERBOSE	    
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		
		printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current makespan: %.3f)...\n",
		       (end_time - start_time),
		       makespan_bound);
	    }
	    #endif

	    kruhobot_Schedules.clear();
	    find_InitialNonconflictingSchedules(solver,
						real_context,					 
						real_Instance,
						kruhobot_RDDs,
						kruhobot_RDD_Mappings,
						real_sat_Model,
						kruhobot_Schedules);
	    augment_Schedules(real_Instance, kruhobot_Schedules);    
	    delete solver;
    
	    if (m_timeout >= 0.0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2.0;
		}
	    }
	    if (makespan_bound > makespan_limit + extra_makespan)
	    {
		return -3.0;
	    }
	}
	
	sDouble cumulative_makespan;
	KruhobotCollisions_mset kruhobot_Collisions;
	KruhobotCollisions_mset next_kruhobot_Collisions;	

	cumulative_makespan = analyze_NonconflictingSchedules_nonprioritized(real_Instance,
									     kruhobot_Schedules,
									     next_kruhobot_Collisions);

	if (next_kruhobot_Collisions.empty())
	{
	    printf("COLLISION-FREE solution found !\n");
	    
            #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
	    }
            #endif
	    
	    return cumulative_makespan;
	}

	sInt_32 effective_conflicts = 0;	

	for (KruhobotCollisions_mset::const_iterator next_collision = next_kruhobot_Collisions.begin(); next_collision != next_kruhobot_Collisions.end(); ++next_collision)
	{		    
	    if (verify_KruhobotCollisionDuplicities(*next_collision, kruhobot_Collisions))
	    {
		/*
		reflect_KruhobotCollision(*next_collision,
					  kruhobot_location_Conflicts,
					  kruhobot_linear_Conflicts,
					  affected_Kruhobots,
					  last_conflict_id);			
		*/
		KruhobotAffection_pair kruhobot_affection = resolve_KruhobotCollision(real_Instance,
										      next_collision->m_traversal_A,
										      next_collision->m_traversal_B,
										      kruhobot_location_Conflicts,
										      kruhobot_linear_Conflicts,
										      last_conflict_id);

		sInt_32 total_affection = kruhobot_affection.first + kruhobot_affection.second;
		effective_conflicts += total_affection;

		if (total_affection > 0)
		{
		    kruhobot_Collisions.insert(*next_collision);
		}
	    }
	}

	if (effective_conflicts <= 0)
	{
	    printf("COLLISION-FREE solution found !\n");
	    
            #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
	    }
            #endif
	    
	    return cumulative_makespan;
	}			

  	#ifdef sDEBUG
	{
	    /*
	    printf("All collisions:\n");
	    for (KruhobotCollisions_mset::const_iterator collision = kruhobot_Collisions.begin(); collision != kruhobot_Collisions.end(); ++collision)
	    {
		collision->to_Screen();
	    }
	    */
	}
	#endif		

	Glucose::Solver *solver = NULL;

	KruhobotDecisionDiagrams_vector kruhobot_RDDs;
	kruhobot_RDDs.resize(N_kruhobots + 1);
	
	KruhobotDecisionMappings_vector kruhobot_RDD_Mappings;
	kruhobot_RDD_Mappings.resize(N_kruhobots + 1);

	RealModel real_sat_Model;
	next_kruhobot_Collisions.clear();

        #ifdef sVERBOSE	    
	{
	    sDouble end_time = sStatistics::get_CPU_Seconds();
			
	    printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current makespan: %.3f)...\n",
		   (end_time - start_time),
		   makespan_bound);
	}
	#endif
		
	while (true)
	{
	    if (m_timeout >= 0.0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2.0;
		}
	    }
	    if (makespan_bound > makespan_limit + extra_makespan)
	    {
		return -3.0;
	    }		
	    
	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current makespan: %.3f)...\n",
			   (end_time - start_time),
			   makespan_bound);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_macro_search_Steps;
	    }
            #endif
	    {
	    KruhobotDecisionDiagrams_vector next_kruhobot_RDDs;
	    next_kruhobot_RDDs.resize(N_kruhobots + 1);

	    KruhobotDecisionMappings_vector next_kruhobot_RDD_Mappings;
	    next_kruhobot_RDD_Mappings.resize(N_kruhobots + 1);	    

	    {
		bool next_iteration = false;
		sDouble next_makespan_bound = -1.0;		

		for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
		{
		    sDouble nx_makespan_bound;
		    nx_makespan_bound = build_KruhobotRealDecisionDiagram_pruning(real_Instance.m_Kruhobots[kruhobot_id],
										  *real_Instance.m_start_conjunction.m_Map,
										  real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
										  real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],	    
										  kruhobot_location_Conflicts[kruhobot_id],
										  kruhobot_linear_Conflicts[kruhobot_id],
										  makespan_bound,
										  next_kruhobot_RDDs[kruhobot_id],
										  next_kruhobot_RDD_Mappings[kruhobot_id]);
		    sASSERT(nx_makespan_bound > 0.0);
		    
		    next_makespan_bound = (next_makespan_bound < 0.0) ? nx_makespan_bound : sMIN(next_makespan_bound, nx_makespan_bound);
		    
		    if (kruhobot_RDDs[kruhobot_id].size() != next_kruhobot_RDDs[kruhobot_id].size())
		    {
			next_iteration = true;
		    }
		    else
		    {
			if (!compare_KruhobotRealDecisionDiagrams(*real_Instance.m_start_conjunction.m_Map, kruhobot_RDDs[kruhobot_id], next_kruhobot_RDDs[kruhobot_id]))
			{
			    next_iteration = true;
			}
		    }
		    #ifdef sDEBUG
		    {
			/*
			to_Screen(kruhobot_RDDs[kruhobot_id]);

			printf("Kruhobot location conflicts: %d\n", kruhobot_id);
			for (LocationConflicts_upper__umap::const_iterator location_Conflict = kruhobot_location_Conflicts[kruhobot_id].begin(); location_Conflict != kruhobot_location_Conflicts[kruhobot_id].end(); ++location_Conflict)
			{
			    printf("  %d, %ld\n", location_Conflict->first, location_Conflict->second.size());
			    for (LocationConflicts_map::const_iterator location_conflict = location_Conflict->second.begin(); location_conflict != location_Conflict->second.end(); ++location_conflict)
			    {
				location_conflict->second.to_Screen();
			    }
			}
			printf("Kruhobot linear conflicts: %d\n", kruhobot_id);
			for (UlinearConflicts_upper__map::const_iterator linear_Conflict = kruhobot_linear_Conflicts[kruhobot_id].begin(); linear_Conflict != kruhobot_linear_Conflicts[kruhobot_id].end(); ++linear_Conflict)
			{
			    printf("  %ld\n", linear_Conflict->second.size());
			    for (UlinearConflicts_map::const_iterator linear_conflict = linear_Conflict->second.begin(); linear_conflict != linear_Conflict->second.end(); ++linear_conflict)
			    {
				linear_conflict->second.to_Screen();
			    }
			}

 		        printf("Next makespan bound: %.3f\n", next_makespan_bound);		
			*/
		    }
		    #endif
		}
	
		if (next_iteration)
		{    
		    RealModel next_real_sat_Model;

		    if (solver != NULL)
		    {
			delete solver;
		    }		
		    solver = new Glucose::Solver;
		    solver->setIncrementalMode();
		    solver->s_Glucose_timeout = m_timeout;		    
		    
		    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
		    {
			kruhobot_RDDs[kruhobot_id] = next_kruhobot_RDDs[kruhobot_id];
			kruhobot_RDD_Mappings[kruhobot_id] = next_kruhobot_RDD_Mappings[kruhobot_id];			
		    }
		    kruhobot_Schedules.clear();

		    if (!find_NextNonconflictingSchedules(solver,
							  real_context,					 
							  real_Instance,
							  kruhobot_RDDs,
							  kruhobot_RDD_Mappings,
							  kruhobot_Collisions,
							  next_real_sat_Model,
							  kruhobot_Schedules))
		    {
			sASSERT(next_makespan_bound > makespan_bound + s_EPSILON);
			
                        #ifdef sVERBOSE	    
			{
			    sDouble end_time = sStatistics::get_CPU_Seconds();
			    
			    printf("%sUnable to solve newly built instance (elapsed time [seconds]: %.3f, current makespan: %.3f, next makespan: %.3f)...\n",
				   s_INDENT.c_str(),
				   (end_time - start_time),
				   makespan_bound,
				   next_makespan_bound);
			}
	                #endif
			real_context.m_makespan_bound = makespan_bound = (next_makespan_bound < 0.0) ? makespan_bound : next_makespan_bound;
			real_sat_Model = next_real_sat_Model;			
			continue;
		    }
		    else
		    {
			real_sat_Model = next_real_sat_Model;
		    }
		}
		else
		{
		    kruhobot_Schedules.clear();
		    if (!find_NearNonconflictingSchedules(solver,
							  real_context,
							  real_Instance,
							  kruhobot_RDDs,
							  kruhobot_RDD_Mappings,
							  kruhobot_Collisions,
							  next_kruhobot_Collisions,
							  real_sat_Model,
							  kruhobot_Schedules))
		    {			
			sASSERT(next_makespan_bound > makespan_bound + s_EPSILON);
			
                        #ifdef sVERBOSE	    
			{
			    sDouble end_time = sStatistics::get_CPU_Seconds();

			    printf("%sUnable to solve augmented instance (elapsed time [seconds]: %.3f, current makespan: %.3f, next makespan: %.3f)...\n",
				   s_INDENT.c_str(),
				   (end_time - start_time),
				   makespan_bound,
				   next_makespan_bound);
			}
	                #endif
			real_context.m_makespan_bound = makespan_bound = (next_makespan_bound < 0.0) ? makespan_bound : next_makespan_bound;
			continue;
		    }
		}
		augment_Schedules(real_Instance, kruhobot_Schedules);

		sDouble cumulative_makespan;
		next_kruhobot_Collisions.clear();
		cumulative_makespan = analyze_NonconflictingSchedules_nonprioritized(real_Instance,
										     kruhobot_Schedules,
										     next_kruhobot_Collisions);
		if (next_kruhobot_Collisions.empty())
		{
		    printf("COLLISION-FREE solution found !\n");

  	            #ifdef sVERBOSE
		    {
			sDouble end_time = sStatistics::get_CPU_Seconds();
			printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
		    }
 	            #endif
		    
		    return cumulative_makespan;
		}

  	        #ifdef sDEBUG
		{
		    /*
		    printf("Collisions:\n");
		    for (KruhobotCollisions_mset::const_iterator collision = kruhobot_Collisions.begin(); collision != kruhobot_Collisions.end(); ++collision)
		    {
			collision->to_Screen(s_INDENT);
		    }
		    
		    printf("Next collisions:\n");
		    for (KruhobotCollisions_mset::const_iterator collision = next_kruhobot_Collisions.begin(); collision != next_kruhobot_Collisions.end(); ++collision)
		    {
			collision->to_Screen(s_INDENT);
		    }
		    */
		}
	        #endif

		sInt_32 effective_conflicts = 0;

		for (KruhobotCollisions_mset::const_iterator next_collision = next_kruhobot_Collisions.begin(); next_collision != next_kruhobot_Collisions.end(); ++next_collision)
		{		    
		    if (verify_KruhobotCollisionDuplicities(*next_collision, kruhobot_Collisions))			
		    {
			KruhobotAffection_pair kruhobot_affection = resolve_KruhobotCollision(real_Instance,
											      next_collision->m_traversal_A,
											      next_collision->m_traversal_B,
											      kruhobot_location_Conflicts,
											      kruhobot_linear_Conflicts,
											      last_conflict_id);
			
			/*
			reflect_KruhobotCollision(*next_collision,
						  kruhobot_location_Conflicts,
						  kruhobot_linear_Conflicts,
						  affected_Kruhobots,
						  last_conflict_id);
			*/
			sInt_32 total_affection = kruhobot_affection.first + kruhobot_affection.second;
			effective_conflicts += total_affection;
			
			if (total_affection > 0)
			{
			    kruhobot_Collisions.insert(*next_collision);
			}
		    }
		}

		if (effective_conflicts <= 0)
		{
		    printf("COLLISION-FREE solution found !\n");

  	            #ifdef sVERBOSE
		    {
			sDouble end_time = sStatistics::get_CPU_Seconds();
			printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
		    }
 	            #endif
		    
		    return cumulative_makespan;
		}
	    }
	    }	    
	}	
	
	return -1.0;
    }


    sDouble sRealSMTCBS::find_NonconflictingSchedules_pruningSmart(const sRealInstance                    &real_Instance,
								   KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
								   KruhobotUlinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
								   KruhobotSchedules_vector               &kruhobot_Schedules,
								   sDouble                                 makespan_limit,
								   sDouble                                 extra_makespan)
    {
	sInt_32 last_conflict_id = 0;
	
	sDouble makespan_bound = 0.0;	    
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();	

	#ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif	

	RealContext real_context(0.0);
	{
	    RealModel real_sat_Model;
	    
	    Glucose::Solver *solver;
	    solver = new Glucose::Solver;
	    solver->s_Glucose_timeout = m_timeout;

	    KruhobotDecisionDiagrams_vector kruhobot_RDDs;
	    kruhobot_RDDs.resize(N_kruhobots + 1);

	    KruhobotDecisionMappings_vector kruhobot_RDD_Mappings;
	    kruhobot_RDD_Mappings.resize(N_kruhobots + 1);

	    sDouble next_makespan_bound = -1.0;
	    sDouble makespan_lower_bound = 0.0;

	    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	    {
		sDouble kruhobot_makespan_lower_bound;
		
		if ((kruhobot_makespan_lower_bound = find_KruhobotIgnoringSchedule(real_Instance.m_Kruhobots[kruhobot_id],
										   *real_Instance.m_start_conjunction.m_Map,
										   real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
										   real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],
										   makespan_limit,
										   extra_makespan,
										   kruhobot_Schedules[kruhobot_id])) < 0.0)
		{
		    return -1.0;
		}
		makespan_lower_bound = (makespan_lower_bound < kruhobot_makespan_lower_bound) ? kruhobot_makespan_lower_bound : makespan_lower_bound;
		printf("Kruhobot lower bound makespan: %.3f\n", kruhobot_makespan_lower_bound);
	    }
	    
	    real_context.m_makespan_bound = makespan_bound = makespan_lower_bound;
	    #ifdef sDEBUG
	    {
		printf("Lower bound makespan: %.3f\n", makespan_lower_bound);
	    }
	    #endif
	    
            #ifdef sVERBOSE	    
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		
		printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current makespan: %.3f)...\n",
		       (end_time - start_time),
		       makespan_bound);
	    }
	    #endif

	    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	    {
		sDouble nx_makespan_bound;
		nx_makespan_bound = build_KruhobotRealDecisionDiagram_pruningSmart(real_Instance.m_Kruhobots[kruhobot_id],
										   *real_Instance.m_start_conjunction.m_Map,
										   real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
										   real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],					    
										   kruhobot_location_Conflicts[kruhobot_id],
										   kruhobot_linear_Conflicts[kruhobot_id],
										   makespan_lower_bound,
										   kruhobot_RDDs[kruhobot_id],
										   kruhobot_RDD_Mappings[kruhobot_id]);
//		sASSERT(nx_makespan_bound > 0.0);
		    
		next_makespan_bound = (next_makespan_bound < 0.0) ? nx_makespan_bound : sMIN(next_makespan_bound, nx_makespan_bound);
	    }
            #ifdef sVERBOSE	    
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		
		printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current makespan: %.3f)...\n",
		       (end_time - start_time),
		       makespan_bound);
	    }
	    #endif

	    kruhobot_Schedules.clear();
	    find_InitialNonconflictingSchedules(solver,
						real_context,					 
						real_Instance,
						kruhobot_RDDs,
						kruhobot_RDD_Mappings,
						real_sat_Model,
						kruhobot_Schedules);
	    augment_Schedules(real_Instance, kruhobot_Schedules);    
	    delete solver;
    
	    if (m_timeout >= 0.0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2.0;
		}
	    }
	    if (makespan_bound > makespan_limit + extra_makespan)
	    {
		return -3.0;
	    }
	}
	
	sDouble cumulative_makespan;
	KruhobotCollisions_mset kruhobot_Collisions;
	KruhobotCollisions_mset next_kruhobot_Collisions;	

	cumulative_makespan = analyze_NonconflictingSchedules_nonprioritized(real_Instance,
									     kruhobot_Schedules,
									     next_kruhobot_Collisions);
	if (next_kruhobot_Collisions.empty())
	{
	    printf("COLLISION-FREE solution found !\n");
	    
            #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
	    }
            #endif
	    
	    return cumulative_makespan;
	}

	sInt_32 effective_conflicts = 0;	

	for (KruhobotCollisions_mset::const_iterator next_collision = next_kruhobot_Collisions.begin(); next_collision != next_kruhobot_Collisions.end(); ++next_collision)
	{		    
	    if (verify_KruhobotCollisionDuplicities(*next_collision, kruhobot_Collisions))
	    {
		/*
		reflect_KruhobotCollision(*next_collision,
					  kruhobot_location_Conflicts,
					  kruhobot_linear_Conflicts,
					  affected_Kruhobots,
					  last_conflict_id);			
		*/
		KruhobotAffection_pair kruhobot_affection = resolve_KruhobotCollision(real_Instance,
										      next_collision->m_traversal_A,
										      next_collision->m_traversal_B,
										      kruhobot_location_Conflicts,
										      kruhobot_linear_Conflicts,
										      last_conflict_id);

		sInt_32 total_affection = kruhobot_affection.first + kruhobot_affection.second;
		effective_conflicts += total_affection;

		if (total_affection > 0)
		{
		    kruhobot_Collisions.insert(*next_collision);
		}
	    }
	}

	if (effective_conflicts <= 0)
	{
	    printf("COLLISION-FREE solution found !\n");
	    
            #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
	    }
            #endif
	    
	    return cumulative_makespan;
	}			

  	#ifdef sDEBUG
	{
	    /*
	    printf("All collisions:\n");
	    for (KruhobotCollisions_mset::const_iterator collision = kruhobot_Collisions.begin(); collision != kruhobot_Collisions.end(); ++collision)
	    {
		collision->to_Screen();
	    }
	    */
	}
	#endif		

	Glucose::Solver *solver = NULL;

	KruhobotDecisionDiagrams_vector kruhobot_RDDs;
	kruhobot_RDDs.resize(N_kruhobots + 1);
	
	KruhobotDecisionMappings_vector kruhobot_RDD_Mappings;
	kruhobot_RDD_Mappings.resize(N_kruhobots + 1);

	RealModel real_sat_Model;
	next_kruhobot_Collisions.clear();

        #ifdef sVERBOSE	    
	{
	    sDouble end_time = sStatistics::get_CPU_Seconds();
			
	    printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current makespan: %.3f)...\n",
		   (end_time - start_time),
		   makespan_bound);
	}
	#endif
		
	while (true)
	{
	    if (m_timeout >= 0.0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2.0;
		}
	    }
	    if (makespan_bound > makespan_limit + extra_makespan)
	    {
		return -3.0;
	    }		
	    
	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current makespan: %.3f)...\n",
			   (end_time - start_time),
			   makespan_bound);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_macro_search_Steps;
	    }
            #endif
	    {
	    KruhobotDecisionDiagrams_vector next_kruhobot_RDDs;
	    next_kruhobot_RDDs.resize(N_kruhobots + 1);

	    KruhobotDecisionMappings_vector next_kruhobot_RDD_Mappings;
	    next_kruhobot_RDD_Mappings.resize(N_kruhobots + 1);	    

	    {
		bool next_iteration = false;
		sDouble next_makespan_bound = -1.0;		

		for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
		{
		    sDouble nx_makespan_bound;
		    nx_makespan_bound = build_KruhobotRealDecisionDiagram_pruningSmart(real_Instance.m_Kruhobots[kruhobot_id],
										       *real_Instance.m_start_conjunction.m_Map,
										       real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
										       real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],	    
										       kruhobot_location_Conflicts[kruhobot_id],
										       kruhobot_linear_Conflicts[kruhobot_id],
										       makespan_bound,
										       next_kruhobot_RDDs[kruhobot_id],
										       next_kruhobot_RDD_Mappings[kruhobot_id]);
		    sASSERT(nx_makespan_bound > 0.0);
		    
		    next_makespan_bound = (next_makespan_bound < 0.0) ? nx_makespan_bound : sMIN(next_makespan_bound, nx_makespan_bound);
		    
		    if (kruhobot_RDDs[kruhobot_id].size() != next_kruhobot_RDDs[kruhobot_id].size())
		    {
			next_iteration = true;
		    }
		    else
		    {
			if (!compare_KruhobotRealDecisionDiagrams_smart(*real_Instance.m_start_conjunction.m_Map, kruhobot_RDDs[kruhobot_id], next_kruhobot_RDDs[kruhobot_id]))
			{
			    next_iteration = true;
			}
		    }
		    #ifdef sDEBUG
		    {
			/*
			to_Screen(kruhobot_RDDs[kruhobot_id]);

			printf("Kruhobot location conflicts: %d\n", kruhobot_id);
			for (LocationConflicts_upper__umap::const_iterator location_Conflict = kruhobot_location_Conflicts[kruhobot_id].begin(); location_Conflict != kruhobot_location_Conflicts[kruhobot_id].end(); ++location_Conflict)
			{
			    printf("  %d, %ld\n", location_Conflict->first, location_Conflict->second.size());
			    for (LocationConflicts_map::const_iterator location_conflict = location_Conflict->second.begin(); location_conflict != location_Conflict->second.end(); ++location_conflict)
			    {
				location_conflict->second.to_Screen();
			    }
			}
			printf("Kruhobot linear conflicts: %d\n", kruhobot_id);
			for (UlinearConflicts_upper__map::const_iterator linear_Conflict = kruhobot_linear_Conflicts[kruhobot_id].begin(); linear_Conflict != kruhobot_linear_Conflicts[kruhobot_id].end(); ++linear_Conflict)
			{
			    printf("  %ld\n", linear_Conflict->second.size());
			    for (UlinearConflicts_map::const_iterator linear_conflict = linear_Conflict->second.begin(); linear_conflict != linear_Conflict->second.end(); ++linear_conflict)
			    {
				linear_conflict->second.to_Screen();
			    }
			}

 		        printf("Next makespan bound: %.3f\n", next_makespan_bound);		
			*/
		    }
		    #endif
		}
	
		if (next_iteration)
		{    
		    RealModel next_real_sat_Model;

		    if (solver != NULL)
		    {
			delete solver;
		    }		
		    solver = new Glucose::Solver;
		    solver->setIncrementalMode();
		    solver->s_Glucose_timeout = m_timeout;		    
		    
		    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
		    {
			kruhobot_RDDs[kruhobot_id] = next_kruhobot_RDDs[kruhobot_id];
			kruhobot_RDD_Mappings[kruhobot_id] = next_kruhobot_RDD_Mappings[kruhobot_id];			
		    }
		    kruhobot_Schedules.clear();

		    if (!find_NextNonconflictingSchedules(solver,
							  real_context,					 
							  real_Instance,
							  kruhobot_RDDs,
							  kruhobot_RDD_Mappings,
							  kruhobot_Collisions,
							  next_real_sat_Model,
							  kruhobot_Schedules))
		    {
			sASSERT(next_makespan_bound > makespan_bound + s_EPSILON);
			
                        #ifdef sVERBOSE	    
			{
			    sDouble end_time = sStatistics::get_CPU_Seconds();
			    
			    printf("%sUnable to solve newly built instance (elapsed time [seconds]: %.3f, current makespan: %.3f, next makespan: %.3f)...\n",
				   s_INDENT.c_str(),
				   (end_time - start_time),
				   makespan_bound,
				   next_makespan_bound);
			}
	                #endif
			real_context.m_makespan_bound = makespan_bound = (next_makespan_bound < 0.0) ? makespan_bound : next_makespan_bound;			
			real_sat_Model = next_real_sat_Model;			
			continue;
		    }
		    else
		    {
			real_sat_Model = next_real_sat_Model;
		    }
		}
		else
		{
		    kruhobot_Schedules.clear();
		    
		    if (!find_NearNonconflictingSchedules(solver,
							  real_context,
							  real_Instance,
							  kruhobot_RDDs,
							  kruhobot_RDD_Mappings,
							  kruhobot_Collisions,
							  next_kruhobot_Collisions,
							  real_sat_Model,
							  kruhobot_Schedules))
		    {			
			sASSERT(next_makespan_bound > makespan_bound + s_EPSILON);
			
                        #ifdef sVERBOSE	    
			{
			    sDouble end_time = sStatistics::get_CPU_Seconds();

			    printf("%sUnable to solve augmented instance (elapsed time [seconds]: %.3f, current makespan: %.3f, next makespan: %.3f)...\n",
				   s_INDENT.c_str(),
				   (end_time - start_time),
				   makespan_bound,
				   next_makespan_bound);
			}
	                #endif
			real_context.m_makespan_bound = makespan_bound = (next_makespan_bound < 0.0) ? makespan_bound : next_makespan_bound;
			continue;
		    }
		}	       
		augment_Schedules(real_Instance, kruhobot_Schedules);

		sDouble cumulative_makespan;
		next_kruhobot_Collisions.clear();

		cumulative_makespan = analyze_NonconflictingSchedules_nonprioritized(real_Instance,
										     kruhobot_Schedules,
										     next_kruhobot_Collisions);
		if (next_kruhobot_Collisions.empty())
		{
		    printf("COLLISION-FREE solution found !\n");

  	            #ifdef sVERBOSE
		    {
			sDouble end_time = sStatistics::get_CPU_Seconds();
			printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
		    }
 	            #endif
		    
		    return cumulative_makespan;
		}

  	        #ifdef sDEBUG
		{
		    /*
		    printf("Collisions:\n");
		    for (KruhobotCollisions_mset::const_iterator collision = kruhobot_Collisions.begin(); collision != kruhobot_Collisions.end(); ++collision)
		    {
			collision->to_Screen(s_INDENT);
		    }
		    
		    printf("Next collisions:\n");
		    for (KruhobotCollisions_mset::const_iterator collision = next_kruhobot_Collisions.begin(); collision != next_kruhobot_Collisions.end(); ++collision)
		    {
			collision->to_Screen(s_INDENT);
		    }
		    */
		}
	        #endif
		
		sInt_32 effective_conflicts = 0;

		for (KruhobotCollisions_mset::const_iterator next_collision = next_kruhobot_Collisions.begin(); next_collision != next_kruhobot_Collisions.end(); ++next_collision)
		{		    
		    if (verify_KruhobotCollisionDuplicities(*next_collision, kruhobot_Collisions))			
		    {
			KruhobotAffection_pair kruhobot_affection = resolve_KruhobotCollision(real_Instance,
											      next_collision->m_traversal_A,
											      next_collision->m_traversal_B,
											      kruhobot_location_Conflicts,
											      kruhobot_linear_Conflicts,
											      last_conflict_id);
			
			/*
			reflect_KruhobotCollision(*next_collision,
						  kruhobot_location_Conflicts,
						  kruhobot_linear_Conflicts,
						  affected_Kruhobots,
						  last_conflict_id);
			*/
			sInt_32 total_affection = kruhobot_affection.first + kruhobot_affection.second;
			effective_conflicts += total_affection;
			
			if (total_affection > 0)
			{
			    kruhobot_Collisions.insert(*next_collision);
			}
		    }
		}

		if (effective_conflicts <= 0)
		{
		    printf("COLLISION-FREE solution found !\n");

  	            #ifdef sVERBOSE
		    {
			sDouble end_time = sStatistics::get_CPU_Seconds();
			printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
		    }
 	            #endif
		    
		    return cumulative_makespan;
		}		
	    }
	    }	    
	}	
	
	return -1.0;
    }


    sDouble sRealSMTCBS::find_NonconflictingSchedules_pruningStrong(const sRealInstance                    &real_Instance,
								    KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
								    KruhobotUlinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
								    KruhobotSchedules_vector               &kruhobot_Schedules,
								    sDouble                                 makespan_limit,
								    sDouble                                 extra_makespan)
    {
	sInt_32 last_conflict_id = 0;
	
	sDouble makespan_bound = 0.0;	    
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();	

	#ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif	

	RealContext real_context(0.0);
	{
	    RealModel real_sat_Model;
	    
	    Glucose::Solver *solver;
	    solver = new Glucose::Solver;
	    solver->s_Glucose_timeout = m_timeout;

	    KruhobotDecisionDiagrams_vector kruhobot_RDDs;
	    kruhobot_RDDs.resize(N_kruhobots + 1);

	    KruhobotDecisionMappings_vector kruhobot_RDD_Mappings;
	    kruhobot_RDD_Mappings.resize(N_kruhobots + 1);

	    sDouble next_makespan_bound = -1.0;
	    sDouble makespan_lower_bound = 0.0;

	    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	    {
		sDouble kruhobot_makespan_lower_bound;
		
		if ((kruhobot_makespan_lower_bound = find_KruhobotIgnoringSchedule_strong(real_Instance.m_Kruhobots[kruhobot_id],
											  *real_Instance.m_start_conjunction.m_Map,
											  real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
											  real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],
											  makespan_limit,
											  extra_makespan,
											  kruhobot_Schedules[kruhobot_id])) < 0.0)
		{
		    return -1.0;
		}
		makespan_lower_bound = (makespan_lower_bound < kruhobot_makespan_lower_bound) ? kruhobot_makespan_lower_bound : makespan_lower_bound;
		printf("Kruhobot lower bound makespan: %.3f\n", kruhobot_makespan_lower_bound);
	    }
	    real_context.m_makespan_bound = makespan_bound = makespan_lower_bound;
	    #ifdef sDEBUG
	    {
		printf("Lower bound makespan: %.3f\n", makespan_lower_bound);
	    }
	    #endif
	    
            #ifdef sVERBOSE	    
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		
		printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current makespan: %.3f)...\n",
		       (end_time - start_time),
		       makespan_bound);
	    }
	    #endif
	    
	    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	    {
		sDouble nx_makespan_bound;
		nx_makespan_bound = build_KruhobotRealDecisionDiagram_pruningStrong(real_Instance.m_Kruhobots[kruhobot_id],
										    *real_Instance.m_start_conjunction.m_Map,
										    real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
										    real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],					    
										    kruhobot_location_Conflicts[kruhobot_id],
										    kruhobot_linear_Conflicts[kruhobot_id],
										    makespan_lower_bound,
										    kruhobot_RDDs[kruhobot_id],
										    kruhobot_RDD_Mappings[kruhobot_id]);
		//sASSERT(nx_makespan_bound > 0.0);
		    
		next_makespan_bound = (next_makespan_bound < 0.0) ? nx_makespan_bound : sMIN(next_makespan_bound, nx_makespan_bound);
	    }
            #ifdef sVERBOSE	    
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		
		printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current makespan: %.3f)...\n",
		       (end_time - start_time),
		       makespan_bound);
	    }
	    #endif

	    kruhobot_Schedules.clear();
	    find_InitialNonconflictingSchedules(solver,
						real_context,					 
						real_Instance,
						kruhobot_RDDs,
						kruhobot_RDD_Mappings,
						real_sat_Model,
						kruhobot_Schedules);
	    augment_Schedules(real_Instance, kruhobot_Schedules);    
	    delete solver;
    
	    if (m_timeout >= 0.0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2.0;
		}
	    }
	    if (makespan_bound > makespan_limit + extra_makespan)
	    {
		return -3.0;
	    }
	}
	
	sDouble cumulative_makespan;
	KruhobotCollisions_mset kruhobot_Collisions;
	KruhobotCollisions_mset next_kruhobot_Collisions;	

	cumulative_makespan = analyze_NonconflictingSchedules_nonprioritized(real_Instance,
									     kruhobot_Schedules,
									     next_kruhobot_Collisions);
	if (next_kruhobot_Collisions.empty())
	{
	    printf("COLLISION-FREE solution found !\n");
	    
            #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
	    }
            #endif
	    
	    return cumulative_makespan;
	}
	
	sInt_32 effective_conflicts = 0;	

	for (KruhobotCollisions_mset::const_iterator next_collision = next_kruhobot_Collisions.begin(); next_collision != next_kruhobot_Collisions.end(); ++next_collision)
	{		    
	    if (verify_KruhobotCollisionDuplicities(*next_collision, kruhobot_Collisions))
	    {
		/*
		reflect_KruhobotCollision(*next_collision,
					  kruhobot_location_Conflicts,
					  kruhobot_linear_Conflicts,
					  affected_Kruhobots,
					  last_conflict_id);			
		*/
		KruhobotAffection_pair kruhobot_affection = resolve_KruhobotCollision(real_Instance,
										      next_collision->m_traversal_A,
										      next_collision->m_traversal_B,
										      kruhobot_location_Conflicts,
										      kruhobot_linear_Conflicts,
										      last_conflict_id);

		sInt_32 total_affection = kruhobot_affection.first + kruhobot_affection.second;
		effective_conflicts += total_affection;

		if (total_affection > 0)
		{
		    kruhobot_Collisions.insert(*next_collision);
		}
	    }
	}

	if (effective_conflicts <= 0)
	{
	    printf("COLLISION-FREE solution found !\n");
	    
            #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
	    }
            #endif
		    
	    return cumulative_makespan;
	}			

  	#ifdef sDEBUG
	{
	    /*
	    printf("All collisions:\n");
	    for (KruhobotCollisions_mset::const_iterator collision = kruhobot_Collisions.begin(); collision != kruhobot_Collisions.end(); ++collision)
	    {
		collision->to_Screen();
	    }
	    */
	}
	#endif		

	Glucose::Solver *solver = NULL;

	KruhobotDecisionDiagrams_vector kruhobot_RDDs;
	kruhobot_RDDs.resize(N_kruhobots + 1);
	
	KruhobotDecisionMappings_vector kruhobot_RDD_Mappings;
	kruhobot_RDD_Mappings.resize(N_kruhobots + 1);

	RealModel real_sat_Model;
	next_kruhobot_Collisions.clear();

        #ifdef sVERBOSE	    
	{
	    sDouble end_time = sStatistics::get_CPU_Seconds();
			
	    printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current makespan: %.3f)...\n",
		   (end_time - start_time),
		   makespan_bound);
	}
	#endif
		
	while (true)
	{
	    if (m_timeout >= 0.0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2.0;
		}
	    }
	    if (makespan_bound > makespan_limit + extra_makespan)
	    {
		return -3.0;
	    }		
	    
	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current makespan: %.3f)...\n",
			   (end_time - start_time),
			   makespan_bound);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_macro_search_Steps;
	    }
            #endif
	    {
	    KruhobotDecisionDiagrams_vector next_kruhobot_RDDs;
	    next_kruhobot_RDDs.resize(N_kruhobots + 1);

	    KruhobotDecisionMappings_vector next_kruhobot_RDD_Mappings;
	    next_kruhobot_RDD_Mappings.resize(N_kruhobots + 1);	    

	    {
		bool next_iteration = false;
		sDouble next_makespan_bound = -1.0;		

		for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
		{
		    sDouble nx_makespan_bound;
		    nx_makespan_bound = build_KruhobotRealDecisionDiagram_pruningStrong(real_Instance.m_Kruhobots[kruhobot_id],
											*real_Instance.m_start_conjunction.m_Map,
											real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
											real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],	    
											kruhobot_location_Conflicts[kruhobot_id],
											kruhobot_linear_Conflicts[kruhobot_id],
											makespan_bound,
											next_kruhobot_RDDs[kruhobot_id],
											next_kruhobot_RDD_Mappings[kruhobot_id]);
		    sASSERT(nx_makespan_bound > 0.0);
		    
		    next_makespan_bound = (next_makespan_bound < 0.0) ? nx_makespan_bound : sMIN(next_makespan_bound, nx_makespan_bound);
		    
		    if (kruhobot_RDDs[kruhobot_id].size() != next_kruhobot_RDDs[kruhobot_id].size())
		    {
			next_iteration = true;
		    }
		    else
		    {
			if (!compare_KruhobotRealDecisionDiagrams_smart(*real_Instance.m_start_conjunction.m_Map, kruhobot_RDDs[kruhobot_id], next_kruhobot_RDDs[kruhobot_id]))
			{
			    next_iteration = true;
			}
		    }
		    #ifdef sDEBUG
		    {
			/*
			to_Screen(kruhobot_RDDs[kruhobot_id]);

			printf("Kruhobot location conflicts: %d\n", kruhobot_id);
			for (LocationConflicts_upper__umap::const_iterator location_Conflict = kruhobot_location_Conflicts[kruhobot_id].begin(); location_Conflict != kruhobot_location_Conflicts[kruhobot_id].end(); ++location_Conflict)
			{
			    printf("  %d, %ld\n", location_Conflict->first, location_Conflict->second.size());
			    for (LocationConflicts_map::const_iterator location_conflict = location_Conflict->second.begin(); location_conflict != location_Conflict->second.end(); ++location_conflict)
			    {
				location_conflict->second.to_Screen();
			    }
			}
			printf("Kruhobot linear conflicts: %d\n", kruhobot_id);
			for (UlinearConflicts_upper__map::const_iterator linear_Conflict = kruhobot_linear_Conflicts[kruhobot_id].begin(); linear_Conflict != kruhobot_linear_Conflicts[kruhobot_id].end(); ++linear_Conflict)
			{
			    printf("  %ld\n", linear_Conflict->second.size());
			    for (UlinearConflicts_map::const_iterator linear_conflict = linear_Conflict->second.begin(); linear_conflict != linear_Conflict->second.end(); ++linear_conflict)
			    {
				linear_conflict->second.to_Screen();
			    }
			}

 		        printf("Next makespan bound: %.3f\n", next_makespan_bound);		
			*/
		    }
		    #endif
		}
	
		if (next_iteration)
		{    
		    RealModel next_real_sat_Model;

		    if (solver != NULL)
		    {
			delete solver;
		    }		
		    solver = new Glucose::Solver;
		    solver->setIncrementalMode();
		    solver->s_Glucose_timeout = m_timeout;		    
		    
		    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
		    {
			kruhobot_RDDs[kruhobot_id] = next_kruhobot_RDDs[kruhobot_id];
			kruhobot_RDD_Mappings[kruhobot_id] = next_kruhobot_RDD_Mappings[kruhobot_id];			
		    }
		    kruhobot_Schedules.clear();

		    if (!find_NextNonconflictingSchedules(solver,
							  real_context,					 
							  real_Instance,
							  kruhobot_RDDs,
							  kruhobot_RDD_Mappings,
							  kruhobot_Collisions,
							  next_real_sat_Model,
							  kruhobot_Schedules))
		    {
			sASSERT(next_makespan_bound > makespan_bound + s_EPSILON);
			
                        #ifdef sVERBOSE	    
			{
			    sDouble end_time = sStatistics::get_CPU_Seconds();
			    
			    printf("%sUnable to solve newly built instance (elapsed time [seconds]: %.3f, current makespan: %.3f, next makespan: %.3f)...\n",
				   s_INDENT.c_str(),
				   (end_time - start_time),
				   makespan_bound,
				   next_makespan_bound);
			}
	                #endif
			//real_context.m_makespan_bound = makespan_bound = (next_makespan_bound < 0.0) ? makespan_bound : next_makespan_bound;
			real_sat_Model = next_real_sat_Model;			
			continue;
		    }
		    else
		    {	       	
			real_sat_Model = next_real_sat_Model;
		    }
		}
		else
		{
		    kruhobot_Schedules.clear();

		    if (!find_NearNonconflictingSchedules(solver,
							  real_context,
							  real_Instance,
							  kruhobot_RDDs,
							  kruhobot_RDD_Mappings,
							  kruhobot_Collisions,
							  next_kruhobot_Collisions,
							  real_sat_Model,
							  kruhobot_Schedules))
		    {			
			sASSERT(next_makespan_bound > makespan_bound + s_EPSILON);
			
                        #ifdef sVERBOSE	    
			{
			    sDouble end_time = sStatistics::get_CPU_Seconds();

			    printf("%sUnable to solve augmented instance (elapsed time [seconds]: %.3f, current makespan: %.3f, next makespan: %.3f)...\n",
				   s_INDENT.c_str(),
				   (end_time - start_time),
				   makespan_bound,
				   next_makespan_bound);
			}
	                #endif
			real_context.m_makespan_bound = makespan_bound = (next_makespan_bound < 0.0) ? makespan_bound : next_makespan_bound;
			continue;
		    }
		}	       
		augment_Schedules(real_Instance, kruhobot_Schedules);

		sDouble cumulative_makespan;
		next_kruhobot_Collisions.clear();

		cumulative_makespan = analyze_NonconflictingSchedules_nonprioritized(real_Instance,
										     kruhobot_Schedules,
										     next_kruhobot_Collisions);
		if (next_kruhobot_Collisions.empty())
		{
		    printf("COLLISION-FREE solution found !\n");

  	            #ifdef sVERBOSE
		    {
			sDouble end_time = sStatistics::get_CPU_Seconds();
			printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
		    }
 	            #endif
		    
		    return cumulative_makespan;
		}

  	        #ifdef sDEBUG
		{
		    /*
		    printf("Collisions:\n");
		    for (KruhobotCollisions_mset::const_iterator collision = kruhobot_Collisions.begin(); collision != kruhobot_Collisions.end(); ++collision)
		    {
			collision->to_Screen(s_INDENT);
		    }
		    
		    printf("Next collisions:\n");
		    for (KruhobotCollisions_mset::const_iterator collision = next_kruhobot_Collisions.begin(); collision != next_kruhobot_Collisions.end(); ++collision)
		    {
			collision->to_Screen(s_INDENT);
		    }
		    */
		}
	        #endif

		effective_conflicts = 0;

		for (KruhobotCollisions_mset::const_iterator next_collision = next_kruhobot_Collisions.begin(); next_collision != next_kruhobot_Collisions.end(); ++next_collision)
		{		    
		    if (verify_KruhobotCollisionDuplicities(*next_collision, kruhobot_Collisions))			
		    {
			KruhobotAffection_pair kruhobot_affection = resolve_KruhobotCollision(real_Instance,
											      next_collision->m_traversal_A,
											      next_collision->m_traversal_B,
											      kruhobot_location_Conflicts,
											      kruhobot_linear_Conflicts,
											      last_conflict_id);
			
			/*
			reflect_KruhobotCollision(*next_collision,
						  kruhobot_location_Conflicts,
						  kruhobot_linear_Conflicts,
						  affected_Kruhobots,
						  last_conflict_id);
			*/
			sInt_32 total_affection = kruhobot_affection.first + kruhobot_affection.second;
			effective_conflicts += total_affection;
			
			if (total_affection > 0)
			{
			    kruhobot_Collisions.insert(*next_collision);
			}
		    }
		}

		if (effective_conflicts <= 0)
		{
		    printf("COLLISION-FREE solution found !\n");

  	            #ifdef sVERBOSE
		    {
			sDouble end_time = sStatistics::get_CPU_Seconds();
			printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
		    }
 	            #endif
		    
		    return cumulative_makespan;
		}
	    }
	    }	    
	}	
	
	return -1.0;
    }

    
    sDouble sRealSMTCBS::find_NonconflictingSchedules_conflictRespectful(const sRealInstance                    &real_Instance,
									 KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
									 KruhobotUlinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
									 KruhobotSchedules_vector               &kruhobot_Schedules,
									 sDouble                                 makespan_limit,
									 sDouble                                 extra_makespan)
    {
	sInt_32 last_conflict_id = 0;
	
	sDouble makespan_bound = 0.0;	    
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();	

	#ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif	

	RealContext real_context(0.0);
	{
	    RealModel real_sat_Model;
	    
	    Glucose::Solver *solver;
	    solver = new Glucose::Solver;
	    solver->s_Glucose_timeout = m_timeout;

	    KruhobotDecisionDiagrams_vector kruhobot_RDDs;
	    kruhobot_RDDs.resize(N_kruhobots + 1);

	    KruhobotDecisionMappings_vector kruhobot_RDD_Mappings;
	    kruhobot_RDD_Mappings.resize(N_kruhobots + 1);

	    sDouble next_makespan_bound = -1.0;
	    sDouble makespan_lower_bound = 0.0;

	    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	    {
		sDouble kruhobot_makespan_lower_bound;
		
		if ((kruhobot_makespan_lower_bound = find_KruhobotIgnoringSchedule_strong(real_Instance.m_Kruhobots[kruhobot_id],
											  *real_Instance.m_start_conjunction.m_Map,
											  real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
											  real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],
											  makespan_limit,
											  extra_makespan,
											  kruhobot_Schedules[kruhobot_id])) < 0.0)
		{
		    return -1.0;
		}
		makespan_lower_bound = (makespan_lower_bound < kruhobot_makespan_lower_bound) ? kruhobot_makespan_lower_bound : makespan_lower_bound;
		printf("Kruhobot lower bound makespan: %.3f\n", kruhobot_makespan_lower_bound);
	    }

	    real_context.m_makespan_bound = makespan_bound = makespan_lower_bound;
	    #ifdef sDEBUG
	    {
		printf("Lower bound makespan: %.3f\n", makespan_lower_bound);
	    }
	    #endif
	    
            #ifdef sVERBOSE	    
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		
		printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current makespan: %.3f)...\n",
		       (end_time - start_time),
		       makespan_bound);
	    }
	    #endif
	    
	    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	    {
		sDouble nx_makespan_bound;
		nx_makespan_bound = build_KruhobotRealDecisionDiagram_conflictRespectfulBucketing(real_Instance.m_Kruhobots[kruhobot_id],
												  *real_Instance.m_start_conjunction.m_Map,
												  real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
												  real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],					    
												  kruhobot_location_Conflicts[kruhobot_id],
												  kruhobot_linear_Conflicts[kruhobot_id],
												  makespan_lower_bound,
												  kruhobot_RDDs[kruhobot_id],
												  kruhobot_RDD_Mappings[kruhobot_id]);
		sASSERT(nx_makespan_bound > 0.0);	
		next_makespan_bound = (next_makespan_bound < 0.0) ? nx_makespan_bound : sMIN(next_makespan_bound, nx_makespan_bound);
	    }
            #ifdef sVERBOSE	    
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		
		printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current makespan: %.3f)...\n",
		       (end_time - start_time),
		       makespan_bound);
	    }
	    #endif

	    kruhobot_Schedules.clear();
	    find_InitialNonconflictingSchedules(solver,
						real_context,					 
						real_Instance,
						kruhobot_RDDs,
						kruhobot_RDD_Mappings,
						real_sat_Model,
						kruhobot_Schedules);
	    augment_Schedules(real_Instance, kruhobot_Schedules);    
	    delete solver;
    
	    if (m_timeout >= 0.0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2.0;
		}
	    }
	    if (makespan_bound > makespan_limit + extra_makespan)
	    {
		return -3.0;
	    }
	}
	
	sDouble cumulative_makespan;
	KruhobotCollisions_mset kruhobot_Collisions;
	KruhobotCollisions_mset next_kruhobot_Collisions;	

	cumulative_makespan = analyze_NonconflictingSchedules_nonprioritized(real_Instance,
									     kruhobot_Schedules,
									     next_kruhobot_Collisions);
	if (next_kruhobot_Collisions.empty())
	{
	    printf("COLLISION-FREE solution found !\n");
	    
            #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
	    }
            #endif
	    
	    return cumulative_makespan;
	}
	/*
	reflect_KruhobotCollisions(kruhobot_Collisions,
				   kruhobot_location_Conflicts,
				   kruhobot_linear_Conflicts,
				   last_conflict_id);
	*/

	sInt_32 effective_conflicts = 0;	

	for (KruhobotCollisions_mset::const_iterator next_collision = next_kruhobot_Collisions.begin(); next_collision != next_kruhobot_Collisions.end(); ++next_collision)
	{		    
	    if (verify_KruhobotCollisionDuplicities(*next_collision, kruhobot_Collisions))
	    {
		/*
		reflect_KruhobotCollision(*next_collision,
					  kruhobot_location_Conflicts,
					  kruhobot_linear_Conflicts,
					  affected_Kruhobots,
					  last_conflict_id);			
		*/
		KruhobotAffection_pair kruhobot_affection = resolve_KruhobotCollision(real_Instance,
										      next_collision->m_traversal_A,
										      next_collision->m_traversal_B,
										      kruhobot_location_Conflicts,
										      kruhobot_linear_Conflicts,
										      last_conflict_id);

		sInt_32 total_affection = kruhobot_affection.first + kruhobot_affection.second;
		effective_conflicts += total_affection;

		if (total_affection > 0)
		{
		    kruhobot_Collisions.insert(*next_collision);
		}
	    }
	}

	if (effective_conflicts <= 0)
	{
	    printf("COLLISION-FREE solution found !\n");
	    
            #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
	    }
            #endif
	    
	    return cumulative_makespan;
	}			

	Glucose::Solver *solver = NULL;

	KruhobotDecisionDiagrams_vector kruhobot_RDDs;
	kruhobot_RDDs.resize(N_kruhobots + 1);
	
	KruhobotDecisionMappings_vector kruhobot_RDD_Mappings;
	kruhobot_RDD_Mappings.resize(N_kruhobots + 1);

	RealModel real_sat_Model;
	next_kruhobot_Collisions.clear();

        #ifdef sVERBOSE	    
	{
	    sDouble end_time = sStatistics::get_CPU_Seconds();
			
	    printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current makespan: %.3f)...\n",
		   (end_time - start_time),
		   makespan_bound);
	}
	#endif	
		
	while (true)
	{
	    if (m_timeout >= 0.0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2.0;
		}
	    }
	    if (makespan_bound > makespan_limit + extra_makespan)
	    {
		return -3.0;
	    }		
	    
	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current makespan: %.3f)...\n",
			   (end_time - start_time),
			   makespan_bound);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_macro_search_Steps;
	    }
            #endif
	    {
	    KruhobotDecisionDiagrams_vector next_kruhobot_RDDs;
	    next_kruhobot_RDDs.resize(N_kruhobots + 1);

	    KruhobotDecisionMappings_vector next_kruhobot_RDD_Mappings;
	    next_kruhobot_RDD_Mappings.resize(N_kruhobots + 1);	    

	    {
		bool next_iteration = false;
		sDouble next_makespan_bound = -1.0;		

		for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
		{
		    sDouble nx_makespan_bound;
		    nx_makespan_bound = build_KruhobotRealDecisionDiagram_conflictRespectfulBucketing(real_Instance.m_Kruhobots[kruhobot_id],
												      *real_Instance.m_start_conjunction.m_Map,
												      real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
												      real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],	    
												      kruhobot_location_Conflicts[kruhobot_id],
												      kruhobot_linear_Conflicts[kruhobot_id],
												      makespan_bound,
												      next_kruhobot_RDDs[kruhobot_id],
												      next_kruhobot_RDD_Mappings[kruhobot_id]);
		    sASSERT(nx_makespan_bound > 0.0);
		    next_makespan_bound = (next_makespan_bound < 0.0) ? nx_makespan_bound : sMIN(next_makespan_bound, nx_makespan_bound);

		    if (kruhobot_RDDs[kruhobot_id].size() != next_kruhobot_RDDs[kruhobot_id].size())
		    {
			next_iteration = true;
		    }
		    else
		    {
			if (!compare_KruhobotRealDecisionDiagrams_smart(*real_Instance.m_start_conjunction.m_Map, kruhobot_RDDs[kruhobot_id], next_kruhobot_RDDs[kruhobot_id]))
			{
			    next_iteration = true;
			}
		    }
		    
		    #ifdef sDEBUG
		    {
			/*
			to_Screen(kruhobot_RDDs[kruhobot_id]);

			printf("Kruhobot location conflicts: %d\n", kruhobot_id);
			for (LocationConflicts_upper__umap::const_iterator location_Conflict = kruhobot_location_Conflicts[kruhobot_id].begin(); location_Conflict != kruhobot_location_Conflicts[kruhobot_id].end(); ++location_Conflict)
			{
			    printf("  %d, %ld\n", location_Conflict->first, location_Conflict->second.size());
			    for (LocationConflicts_map::const_iterator location_conflict = location_Conflict->second.begin(); location_conflict != location_Conflict->second.end(); ++location_conflict)
			    {
				location_conflict->second.to_Screen();
			    }
			}
			printf("Kruhobot linear conflicts: %d\n", kruhobot_id);
			for (UlinearConflicts_upper__map::const_iterator linear_Conflict = kruhobot_linear_Conflicts[kruhobot_id].begin(); linear_Conflict != kruhobot_linear_Conflicts[kruhobot_id].end(); ++linear_Conflict)
			{
			    printf("  %ld\n", linear_Conflict->second.size());
			    for (UlinearConflicts_map::const_iterator linear_conflict = linear_Conflict->second.begin(); linear_conflict != linear_Conflict->second.end(); ++linear_conflict)
			    {
				linear_conflict->second.to_Screen();
			    }
			}

 		        printf("Next makespan bound: %.3f\n", next_makespan_bound);		
			*/
		    }
		    #endif
		}

		if (next_iteration)
		{
		    RealModel next_real_sat_Model;

		    if (solver != NULL)
		    {
			delete solver;
		    }		
		    solver = new Glucose::Solver;
		    solver->setIncrementalMode();
		    solver->s_Glucose_timeout = m_timeout;		    
		    
		    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
		    {
			kruhobot_RDDs[kruhobot_id] = next_kruhobot_RDDs[kruhobot_id];
			kruhobot_RDD_Mappings[kruhobot_id] = next_kruhobot_RDD_Mappings[kruhobot_id];
		    }
		    kruhobot_Schedules.clear();

		    if (!find_NextNonconflictingSchedules(solver,
							  real_context,
							  real_Instance,
							  kruhobot_RDDs,
							  kruhobot_RDD_Mappings,
							  kruhobot_Collisions,
							  next_real_sat_Model,
							  kruhobot_Schedules))
		    {
			sASSERT(next_makespan_bound > makespan_bound + s_EPSILON);			
						
                        #ifdef sVERBOSE	    
			{
			    sDouble end_time = sStatistics::get_CPU_Seconds();

			    printf("%sUnable to solve newly built instance (elapsed time [seconds]: %.3f, current makespan: %.3f, next makespan: %.3f)...\n",
				   s_INDENT.c_str(),
				   (end_time - start_time),
				   makespan_bound,
				   next_makespan_bound);
			}
	                #endif
//			real_context.m_makespan_bound = makespan_bound = (next_makespan_bound < 0.0) ? makespan_bound : next_makespan_bound;

			real_context.m_makespan_bound = makespan_bound = (next_makespan_bound < 0.0) ? makespan_bound : next_makespan_bound;
			real_sat_Model = next_real_sat_Model;			
			continue;
		    }
		    else
		    {
			real_sat_Model = next_real_sat_Model;
		    }
		}
		else
		{
		    kruhobot_Schedules.clear();

		    if (!find_NearNonconflictingSchedules(solver,
							  real_context,
							  real_Instance,
							  kruhobot_RDDs,
							  kruhobot_RDD_Mappings,
							  kruhobot_Collisions,
							  next_kruhobot_Collisions,
							  real_sat_Model,
							  kruhobot_Schedules))
		    {
			sASSERT(next_makespan_bound > makespan_bound + s_EPSILON);
			
                        #ifdef sVERBOSE	    
			{
			    sDouble end_time = sStatistics::get_CPU_Seconds();

			    printf("%sUnable to solve augmented instance (elapsed time [seconds]: %.3f, current makespan: %.3f, next makespan: %.3f)...\n",
				   s_INDENT.c_str(),
				   (end_time - start_time),
				   makespan_bound,
				   next_makespan_bound);
			}
	                #endif
			real_context.m_makespan_bound = makespan_bound = (next_makespan_bound < 0.0) ? makespan_bound : next_makespan_bound;
			continue;
		    }
		}
		augment_Schedules(real_Instance, kruhobot_Schedules);

		sDouble cumulative_makespan;
		next_kruhobot_Collisions.clear();

		cumulative_makespan = analyze_NonconflictingSchedules_nonprioritized(real_Instance,
										     kruhobot_Schedules,
										     next_kruhobot_Collisions);
		if (next_kruhobot_Collisions.empty())
		{
		    printf("COLLISION-FREE solution found !\n");

  	            #ifdef sVERBOSE
		    {
			sDouble end_time = sStatistics::get_CPU_Seconds();
			printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
		    }
 	            #endif
		    
		    return cumulative_makespan;
		}

  	        #ifdef sDEBUG
		{
		    /*
		    printf("Collisions:\n");
		    for (KruhobotCollisions_mset::const_iterator collision = kruhobot_Collisions.begin(); collision != kruhobot_Collisions.end(); ++collision)
		    {
			collision->to_Screen(s_INDENT);
		    }
		    
		    printf("Next collisions:\n");
		    for (KruhobotCollisions_mset::const_iterator collision = next_kruhobot_Collisions.begin(); collision != next_kruhobot_Collisions.end(); ++collision)
		    {
			collision->to_Screen(s_INDENT);
		    }
		    */
		}
	        #endif

		effective_conflicts = 0;
		
		for (KruhobotCollisions_mset::const_iterator next_collision = next_kruhobot_Collisions.begin(); next_collision != next_kruhobot_Collisions.end(); ++next_collision)
		{		    
		    if (verify_KruhobotCollisionDuplicities(*next_collision, kruhobot_Collisions))
		    {
			/*
			kruhobot_Collisions.insert(*next_collision);

			reflect_KruhobotCollision(*next_collision,
						  kruhobot_location_Conflicts,
						  kruhobot_linear_Conflicts,
						  last_conflict_id);			
			*/
			KruhobotAffection_pair kruhobot_affection = resolve_KruhobotCollision(real_Instance,
											      next_collision->m_traversal_A,
											      next_collision->m_traversal_B,
											      kruhobot_location_Conflicts,
											      kruhobot_linear_Conflicts,
											      last_conflict_id);
			
			sInt_32 total_affection = kruhobot_affection.first + kruhobot_affection.second;
			effective_conflicts += total_affection;
			
			if (total_affection > 0)
			{
			    kruhobot_Collisions.insert(*next_collision);
			}			
		    }
		}
		if (effective_conflicts <= 0)
		{
		    printf("COLLISION-FREE solution found !\n");

  	            #ifdef sVERBOSE
		    {
			sDouble end_time = sStatistics::get_CPU_Seconds();
			printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
		    }
 	            #endif
		    
		    return cumulative_makespan;
		}
	    }	    
	    }	    
	}	
	
	return -1.0;
    }


    sDouble sRealSMTCBS::find_NonconflictingSchedules_individualizedConflictRespectful(const sRealInstance                    &real_Instance,
										       KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
										       KruhobotUlinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
										       KruhobotSchedules_vector               &kruhobot_Schedules,
										       sDouble                                 makespan_limit,
										       sDouble                                 extra_makespan)
    {	    
	sInt_32 last_conflict_id = 0;
	
	sDouble makespan_bound = 0.0;	    
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();	

	#ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif
	
	std::vector<sDouble> kruhobot_makespan_lower_Bounds;
	kruhobot_makespan_lower_Bounds.resize(N_kruhobots + 1);

	std::vector<sDouble> kruhobot_next_makespan_Bounds;	
	kruhobot_next_makespan_Bounds.resize(N_kruhobots + 1);	

	KruhobotAffections_vector affected_Kruhobots;
	affected_Kruhobots.resize(N_kruhobots + 1, 1);

	KruhobotDecisionDiagrams_vector kruhobot_RDDs;
	kruhobot_RDDs.resize(N_kruhobots + 1);
	
	KruhobotDecisionMappings_vector kruhobot_RDD_Mappings;
	kruhobot_RDD_Mappings.resize(N_kruhobots + 1);
	
	RealContext real_context(0.0);
	RealModel real_sat_Model;

	Glucose::Solver *solver;
	solver = new Glucose::Solver;
	solver->s_Glucose_timeout = m_timeout;	
	
	{    	    
	    sDouble next_makespan_bound = -1.0;
	    sDouble makespan_lower_bound = 0.0;

	    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	    {
		sDouble kruhobot_makespan_lower_bound;
		
		if ((kruhobot_makespan_lower_bound = find_KruhobotIgnoringSchedule_strong(real_Instance.m_Kruhobots[kruhobot_id],
											  *real_Instance.m_start_conjunction.m_Map,
											  real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
											  real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],
											  makespan_limit,
											  extra_makespan,
											  kruhobot_Schedules[kruhobot_id])) < 0.0)
		{
		    return -1.0;
		}
		makespan_lower_bound = (makespan_lower_bound < kruhobot_makespan_lower_bound) ? kruhobot_makespan_lower_bound : makespan_lower_bound;
		kruhobot_makespan_lower_Bounds[kruhobot_id] = kruhobot_next_makespan_Bounds[kruhobot_id] = kruhobot_makespan_lower_bound;
	    }
	    real_context.m_makespan_bound = makespan_bound = makespan_lower_bound;
	    
            #ifdef sVERBOSE	    
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		
		printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current makespan: %.3f)...\n",
		       (end_time - start_time),
		       makespan_bound);
	    }
	    #endif

	    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	    {
		sDouble nx_makespan_bound;
		nx_makespan_bound = build_KruhobotRealDecisionDiagram_individualizedConflictRespectfulBucketing(real_Instance.m_Kruhobots[kruhobot_id],
														*real_Instance.m_start_conjunction.m_Map,
														real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
														real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],
														kruhobot_location_Conflicts[kruhobot_id],
														kruhobot_linear_Conflicts[kruhobot_id],
														makespan_lower_bound,
														kruhobot_next_makespan_Bounds[kruhobot_id],
														-1,
														kruhobot_RDDs[kruhobot_id],
														kruhobot_RDD_Mappings[kruhobot_id]);
		if (nx_makespan_bound > 0.0)
		{
		    next_makespan_bound = (next_makespan_bound < 0.0) ? nx_makespan_bound : sMIN(next_makespan_bound, nx_makespan_bound);
		}
	    }
//	    to_Screen(kruhobot_RDDs);
		    
	    sASSERT(next_makespan_bound > 0.0);
	    
            #ifdef sVERBOSE	    
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		
		printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current makespan: %.3f)...\n",
		       (end_time - start_time),
		       makespan_bound);
	    }
	    #endif

	    kruhobot_Schedules.clear();
	    find_InitialNonconflictingSchedules(solver,
						real_context,					 
						real_Instance,
						kruhobot_RDDs,
						kruhobot_RDD_Mappings,
						real_sat_Model,
						kruhobot_Schedules);
	    augment_Schedules(real_Instance, kruhobot_Schedules);

//	    delete solver;
    
	    if (m_timeout >= 0.0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2.0;
		}
	    }
	    if (makespan_bound > makespan_limit + extra_makespan)
	    {
		return -3.0;
	    }
	}
	
	sDouble cumulative_makespan;
	KruhobotCollisions_mset kruhobot_Collisions;
	KruhobotCollisions_mset next_kruhobot_Collisions;	

	cumulative_makespan = analyze_NonconflictingSchedules_nonprioritized(real_Instance,
									     kruhobot_Schedules,
									     next_kruhobot_Collisions);
	if (next_kruhobot_Collisions.empty())
	{
	    printf("COLLISION-FREE solution found !\n");
	    
            #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
	    }
            #endif
	    
	    return cumulative_makespan;
	}

	affected_Kruhobots.clear();
	affected_Kruhobots.resize(N_kruhobots + 1, 0);

	sInt_32 effective_conflicts = 0;

	for (KruhobotCollisions_mset::const_iterator next_collision = next_kruhobot_Collisions.begin(); next_collision != next_kruhobot_Collisions.end(); ++next_collision)
	{		    
	    if (verify_KruhobotCollisionDuplicities(*next_collision, kruhobot_Collisions))
	    {
		/*
		kruhobot_Collisions.insert(*next_collision);
		
		reflect_KruhobotCollision(*next_collision,
					  kruhobot_location_Conflicts,
					  kruhobot_linear_Conflicts,
					  affected_Kruhobots,
					  last_conflict_id);
		*/
		KruhobotAffection_pair kruhobot_affection = resolve_KruhobotCollision(real_Instance,
										      next_collision->m_traversal_A,
										      next_collision->m_traversal_B,
										      kruhobot_location_Conflicts,
										      kruhobot_linear_Conflicts,
										      last_conflict_id);

		affected_Kruhobots[sABS(next_collision->m_traversal_A.m_kruhobot_id)] += kruhobot_affection.first;
		affected_Kruhobots[sABS(next_collision->m_traversal_B.m_kruhobot_id)] += kruhobot_affection.second;

		sInt_32 total_affection = kruhobot_affection.first + kruhobot_affection.second;
		effective_conflicts += total_affection;

		if (total_affection > 0)
		{
		    kruhobot_Collisions.insert(*next_collision);
		}
		
	    }
	}

	if (effective_conflicts <= 0)
	{
	    printf("COLLISION-FREE solution found !\n");

  	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
	    }
            #endif
		    
	    return cumulative_makespan;
	}			

	/*
	reflect_KruhobotCollisions(kruhobot_Collisions,
				   kruhobot_location_Conflicts,
				   kruhobot_linear_Conflicts,
				   affected_Kruhobots,
				   last_conflict_id);
	*/
	/*
        #ifdef sDEBUG		
	{
	    printf("Affection [%ld]: ", next_kruhobot_Collisions.size());
	    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	    {
		printf("%d ", affected_Kruhobots[kruhobot_id]);			
	    }
	    printf("\n");
	}
	#endif	
	*/

  	#ifdef sDEBUG
	{
	    /*
	    printf("All collisions:\n");
	    for (KruhobotCollisions_mset::const_iterator collision = kruhobot_Collisions.begin(); collision != kruhobot_Collisions.end(); ++collision)
	    {
		collision->to_Screen();
	    }
	    */
	}
	#endif		

	//Glucose::Solver *solver = NULL;

	/*
	KruhobotDecisionDiagrams_vector kruhobot_RDDs;
	kruhobot_RDDs.resize(N_kruhobots + 1);
	
	KruhobotDecisionMappings_vector kruhobot_RDD_Mappings;
	kruhobot_RDD_Mappings.resize(N_kruhobots + 1);
	*/

        #ifdef sVERBOSE	    
	{
	    sDouble end_time = sStatistics::get_CPU_Seconds();
			
	    printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current makespan: %.3f)...\n",
		   (end_time - start_time),
		   makespan_bound);
	}
	#endif

	//KruhobotAffections_vector affected_Kruhobots;
	//affected_Kruhobots.resize(N_kruhobots + 1, 1);

	sInt_32 fingerprint_limit = 1;
	
	while (true)
	{
	    if (m_timeout >= 0.0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2.0;
		}
	    }
	    if (makespan_bound > makespan_limit + extra_makespan)
	    {
		return -3.0;
	    }
	    
	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current makespan: %.3f)...\n",
			   (end_time - start_time),
			   makespan_bound);
		    verbose_period *= 1.5;
		}
	    }
	    #endif	    

            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_macro_search_Steps;
	    }
            #endif

	    /*
	    #ifdef sDEBUG
	    {
		for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
		{
		    printf("%.3f ", kruhobot_makespan_lower_Bounds[kruhobot_id]);
		}
		printf("\n");

	    }
	    #endif
	    */
	    
	    {
	    KruhobotDecisionDiagrams_vector next_kruhobot_RDDs;
	    next_kruhobot_RDDs.resize(N_kruhobots + 1);

	    KruhobotDecisionMappings_vector next_kruhobot_RDD_Mappings;
	    next_kruhobot_RDD_Mappings.resize(N_kruhobots + 1);	    

	    {
		bool next_iteration = false;
		sDouble next_makespan_bound = -1.0;

		kruhobot_next_makespan_Bounds = kruhobot_makespan_lower_Bounds;
		//sDouble build_begin_time = sStatistics::get_CPU_Seconds();		

		for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
		{
		    sDouble nx_makespan_bound;

		    nx_makespan_bound = build_KruhobotRealDecisionDiagram_individualizedConflictRespectfulBucketing(real_Instance.m_Kruhobots[kruhobot_id],
														    *real_Instance.m_start_conjunction.m_Map,
														    real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
														    real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],	    
														    kruhobot_location_Conflicts[kruhobot_id],
														    kruhobot_linear_Conflicts[kruhobot_id],
														    makespan_bound,
														    kruhobot_next_makespan_Bounds[kruhobot_id],
														    fingerprint_limit,
														    next_kruhobot_RDDs[kruhobot_id],
														    next_kruhobot_RDD_Mappings[kruhobot_id]);

		    /*
		    nx_makespan_bound = makespan_bound;
		    next_kruhobot_RDDs[kruhobot_id] = kruhobot_RDDs[kruhobot_id];
		    next_kruhobot_RDD_Mappings[kruhobot_id] = kruhobot_RDD_Mappings[kruhobot_id];
		    */
		    if (nx_makespan_bound > 0.0)
		    {
			next_makespan_bound = (next_makespan_bound < 0.0) ? nx_makespan_bound : sMIN(next_makespan_bound, nx_makespan_bound);
		    }
		    if (kruhobot_RDDs[kruhobot_id].size() != next_kruhobot_RDDs[kruhobot_id].size())
		    {
			next_iteration = true;
		    }
		    else
		    {
			if (!compare_KruhobotRealDecisionDiagrams_smart(*real_Instance.m_start_conjunction.m_Map, kruhobot_RDDs[kruhobot_id], next_kruhobot_RDDs[kruhobot_id]))
			{
			    next_iteration = true;
			}
		    }
		
		    #ifdef sDEBUG
		    {
			/*
			to_Screen(kruhobot_RDDs[kruhobot_id]);

			printf("Kruhobot location conflicts: %d\n", kruhobot_id);
			for (LocationConflicts_upper__umap::const_iterator location_Conflict = kruhobot_location_Conflicts[kruhobot_id].begin(); location_Conflict != kruhobot_location_Conflicts[kruhobot_id].end(); ++location_Conflict)
			{
			    printf("  %d, %ld\n", location_Conflict->first, location_Conflict->second.size());
			    for (LocationConflicts_map::const_iterator location_conflict = location_Conflict->second.begin(); location_conflict != location_Conflict->second.end(); ++location_conflict)
			    {
				location_conflict->second.to_Screen();
			    }
			}
			printf("Kruhobot linear conflicts: %d\n", kruhobot_id);
			for (UlinearConflicts_upper__map::const_iterator linear_Conflict = kruhobot_linear_Conflicts[kruhobot_id].begin(); linear_Conflict != kruhobot_linear_Conflicts[kruhobot_id].end(); ++linear_Conflict)
			{
			    printf("  %ld\n", linear_Conflict->second.size());
			    for (UlinearConflicts_map::const_iterator linear_conflict = linear_Conflict->second.begin(); linear_conflict != linear_Conflict->second.end(); ++linear_conflict)
			    {
				linear_conflict->second.to_Screen();
			    }
			}

 		        printf("Next makespan bound: %.3f\n", next_makespan_bound);		
			*/
		    }
		    #endif
		}
		//to_Screen(next_kruhobot_RDDs);
		//getchar();
		
		//sDouble build_end_time = sStatistics::get_CPU_Seconds();				

		sASSERT(next_makespan_bound > 0.0);
	
		if (next_iteration)
		{
		    RealModel next_real_sat_Model;

		    if (solver != NULL)
		    {
			delete solver;
		    }		
		    solver = new Glucose::Solver;
		    solver->setIncrementalMode();
		    solver->s_Glucose_timeout = m_timeout;		    
		    
		    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
		    {
			kruhobot_RDDs[kruhobot_id] = next_kruhobot_RDDs[kruhobot_id];
			kruhobot_RDD_Mappings[kruhobot_id] = next_kruhobot_RDD_Mappings[kruhobot_id];
		    }
		    kruhobot_Schedules.clear();

		    bool finding_result = find_NextNonconflictingSchedules(solver,
									   real_context,
									   real_Instance,
									   kruhobot_RDDs,
									   kruhobot_RDD_Mappings,
									   kruhobot_Collisions,
									   /*next_kruhobot_Collisions,*/
									   next_real_sat_Model,
									   kruhobot_Schedules);
	    
		    if (!finding_result)
		    {
			sASSERT(next_makespan_bound > makespan_bound + s_EPSILON);
						
                        #ifdef sVERBOSE	    
			{
			    sDouble end_time = sStatistics::get_CPU_Seconds();
			    
			    printf("%sUnable to solve newly built instance (elapsed time [seconds]: %.3f, current makespan: %.3f, next makespan: %.3f)...\n",
				   s_INDENT.c_str(),
				   (end_time - start_time),
				   makespan_bound,
				   next_makespan_bound);
			}
	                #endif
			bool individual_increase = false;
			    
			for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			{
			    if (kruhobot_next_makespan_Bounds[kruhobot_id] > 0.0)
			    {
				sDouble next_kruhobot_makespan_bound = sMIN(kruhobot_next_makespan_Bounds[kruhobot_id], makespan_bound);				
				if (kruhobot_makespan_lower_Bounds[kruhobot_id] < next_kruhobot_makespan_bound)
				{
				    kruhobot_makespan_lower_Bounds[kruhobot_id] = next_kruhobot_makespan_bound;
				    individual_increase = true;
				}
			    }
			}
			if (!individual_increase)
			{
			    if (fingerprint_limit < 0)
			    {
				for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
				{
				    if (sABS(kruhobot_makespan_lower_Bounds[kruhobot_id] - makespan_bound) <= s_EPSILON)
				    {
					kruhobot_makespan_lower_Bounds[kruhobot_id] = next_makespan_bound;
				    }				
				}
				real_context.m_makespan_bound = makespan_bound = (next_makespan_bound < 0.0) ? makespan_bound : next_makespan_bound;
				fingerprint_limit = 1;
			    }
			    else
			    {
				fingerprint_limit = -1;
			    }
			}
			real_sat_Model = next_real_sat_Model;
			continue;
		    }
		    else
		    {
			real_sat_Model = next_real_sat_Model;
		    }
		}
		else
		{
		    kruhobot_Schedules.clear();

		    bool finding_result = find_NearNonconflictingSchedules(solver,
									   real_context,
									   real_Instance,
									   kruhobot_RDDs,
									   kruhobot_RDD_Mappings,
									   kruhobot_Collisions,
									   next_kruhobot_Collisions,
									   real_sat_Model,
									   kruhobot_Schedules);
//		    sRealCBSBase::to_Screen(kruhobot_Schedules);		    

		    if (!finding_result)
		    {
//			sASSERT(next_makespan_bound > makespan_bound + s_EPSILON);
			
                        #ifdef sVERBOSE	    
			{
			    sDouble end_time = sStatistics::get_CPU_Seconds();

			    printf("%sUnable to solve augmented instance (elapsed time [seconds]: %.3f, current makespan: %.3f, next makespan: %.3f)...\n",
				   s_INDENT.c_str(),
				   (end_time - start_time),
				   makespan_bound,
				   next_makespan_bound);
			}
	                #endif
			bool individual_increase = false;			

			for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			{
			    if (kruhobot_next_makespan_Bounds[kruhobot_id] > 0.0)
			    {
				sDouble next_kruhobot_makespan_bound = sMIN(kruhobot_next_makespan_Bounds[kruhobot_id], makespan_bound);			
				if (kruhobot_makespan_lower_Bounds[kruhobot_id] < next_kruhobot_makespan_bound)
				{
				    kruhobot_makespan_lower_Bounds[kruhobot_id] = next_kruhobot_makespan_bound;
				    individual_increase = true;
				}
			    }
			}
			if (!individual_increase)
			{
			    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			    {
				if (sABS(kruhobot_makespan_lower_Bounds[kruhobot_id] - makespan_bound) <= s_EPSILON)
				{				
				    kruhobot_makespan_lower_Bounds[kruhobot_id] = next_makespan_bound;
				}
			    }			    
			    real_context.m_makespan_bound = makespan_bound = (next_makespan_bound < 0.0) ? makespan_bound : next_makespan_bound;
			}
			continue;
		    }
		}
		augment_Schedules(real_Instance, kruhobot_Schedules);

		sDouble cumulative_makespan;
		next_kruhobot_Collisions.clear();

		cumulative_makespan = analyze_NonconflictingSchedules_nonprioritized(real_Instance,
										     kruhobot_Schedules,
										     next_kruhobot_Collisions);
		
		if (next_kruhobot_Collisions.empty())
		{
		    printf("COLLISION-FREE solution found !\n");

  	            #ifdef sVERBOSE
		    {
			sDouble end_time = sStatistics::get_CPU_Seconds();
			printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
		    }
 	            #endif
		    
		    return cumulative_makespan;
		}

  	        #ifdef sDEBUG
		{
		    /*  
		    printf("Collisions:\n");
		    for (KruhobotCollisions_mset::const_iterator collision = kruhobot_Collisions.begin(); collision != kruhobot_Collisions.end(); ++collision)
		    {
			collision->to_Screen(s_INDENT);
		    }
		    
		    printf("Next collisions:\n");
		    for (KruhobotCollisions_mset::const_iterator collision = next_kruhobot_Collisions.begin(); collision != next_kruhobot_Collisions.end(); ++collision)
		    {
			collision->to_Screen(s_INDENT);
		    }
		    */
		}
	        #endif

		affected_Kruhobots.clear();
		affected_Kruhobots.resize(N_kruhobots + 1, 0);

		sInt_32 effective_conflicts = 0;

		for (KruhobotCollisions_mset::const_iterator next_collision = next_kruhobot_Collisions.begin(); next_collision != next_kruhobot_Collisions.end(); ++next_collision)
		{		    
		    if (verify_KruhobotCollisionDuplicities(*next_collision, kruhobot_Collisions))
		    {
			KruhobotAffection_pair kruhobot_affection = resolve_KruhobotCollision(real_Instance,
											      next_collision->m_traversal_A,
											      next_collision->m_traversal_B,
											      kruhobot_location_Conflicts,
											      kruhobot_linear_Conflicts,
											      last_conflict_id);
			
			affected_Kruhobots[sABS(next_collision->m_traversal_A.m_kruhobot_id)] += kruhobot_affection.first;
			affected_Kruhobots[sABS(next_collision->m_traversal_B.m_kruhobot_id)] += kruhobot_affection.second;
			
			sInt_32 total_affection = kruhobot_affection.first + kruhobot_affection.second;
			effective_conflicts += total_affection;
			
			if (total_affection > 0)
			{
			    kruhobot_Collisions.insert(*next_collision);
			}
		    }
		/*
                #ifdef sDEBUG		
		{
		    printf("Affection [%ld]: ", next_kruhobot_Collisions.size());
		    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
		    {
			printf("%d ", affected_Kruhobots[kruhobot_id]);			
		    }
		    printf("\n");
		}
		#endif
		*/
		}
		
		if (effective_conflicts <= 0)
		{
		    printf("COLLISION-FREE solution found !\n");

  	            #ifdef sVERBOSE
		    {
			sDouble end_time = sStatistics::get_CPU_Seconds();
			printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
		    }
 	            #endif
		    
		    return cumulative_makespan;
		}		
	    }	    
	    }
	}
	    
	return -1.0;
    }

    
/*----------------------------------------------------------------------------*/

    sDouble sRealSMTCBS::find_KruhobotIgnoringSchedule(const sKruhobot &kruhobot,
						       const s2DMap    &map,
						       sInt_32          source_loc_id,
						       sInt_32          sink_loc_id,
						       sDouble          makespan_limit,
						       sDouble          sUNUSED(extra_makespan),
						       Schedule_vector &Schedule) const
    {
	Transitions_mmap transition_Queue;
	Transitions_map explored_Transitions;
	Transitions_vector transition_Store;

	Transition initial_transition(transition_Store.size(), 0.0, 0.0, 0.0, source_loc_id, -1);	
	transition_Queue.insert(Transitions_mmap::value_type(0.0, initial_transition));
	transition_Store.push_back(initial_transition);

	explored_Transitions.insert(Transitions_map::value_type(0.0, LocationIDs_uset()));
	explored_Transitions[0.0].insert(source_loc_id);

	LocationIDs_uset bound_explored_Transitions;

	while (!transition_Queue.empty())
	{
	    const Transition &front_transition = transition_Queue.begin()->second;
	    
	    if (front_transition.m_location_id == sink_loc_id)
	    {
		sInt_32 transition_id = front_transition.m_trans_id;
		    
		Transitions_vector reversed_Schedule;
		while (transition_id >= 0)
		{
		    reversed_Schedule.push_back(transition_Store[transition_id]);
		    transition_id = transition_Store[transition_id].m_prev_trans_id;
		}
		Schedule_vector raw_Schedule;
		sInt_32 N_Events = reversed_Schedule.size();			
		
		for (sInt_32 i = N_Events - 1; i >= 1; --i)
		{	    
		    raw_Schedule.push_back(Event(reversed_Schedule[i].m_location_id, reversed_Schedule[i - 1].m_location_id,
						 reversed_Schedule[i].m_time, reversed_Schedule[i -1].m_time));
		}
		Schedule.clear();
		smooth_Schedule(raw_Schedule, Schedule);
		
		return front_transition.m_time;
	    }
	    Transitions_map::const_iterator explored_transition = explored_Transitions.find(front_transition.m_time);

	    if (explored_transition == explored_Transitions.end())
	    {
		explored_Transitions.insert(Transitions_map::value_type(front_transition.m_time, LocationIDs_uset()));
	    }
	        
	    if (front_transition.m_time + (map.m_straight_Distances[sink_loc_id][front_transition.m_location_id] / kruhobot.m_properties.m_linear_velo) <= makespan_limit)
	    {
		const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[front_transition.m_location_id].m_Neighbors;
		for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
		{
		    sInt_32 neighbor_location_id = (*neighbor)->m_target->m_id;

		    sDouble transition_distance = map.m_straight_Distances[front_transition.m_location_id][neighbor_location_id];
		    if (transition_distance <= s_EPSILON)
		    {
			continue;
		    }			    							    
		    sDouble transition_delta_time = transition_distance / kruhobot.m_properties.m_linear_velo;
		    sDouble transition_finish_time = front_transition.m_time + transition_delta_time;
		    sDouble transition_finish_cost = front_transition.m_cost + transition_delta_time;		    
		    sDouble transition_finish_makespan = front_transition.m_makespan + transition_delta_time;
			
		    LocationIDs_uset *next_explored_Transitions;
		    
		    Transitions_map::const_iterator explored_transition = explored_Transitions.find(transition_finish_time);			
		    if (explored_transition == explored_Transitions.end())
		    {
			explored_Transitions.insert(Transitions_map::value_type(transition_finish_time, LocationIDs_uset()));
		    }
		    next_explored_Transitions = &explored_Transitions[transition_finish_time];
		    
		    if (next_explored_Transitions->find(neighbor_location_id) == next_explored_Transitions->end())
		    {
			Transition neighbor_transition(transition_Store.size(), transition_finish_time, transition_finish_cost, transition_finish_makespan, neighbor_location_id, front_transition.m_trans_id);
			transition_Store.push_back(neighbor_transition);
			
			next_explored_Transitions->insert(neighbor_location_id);
//			transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_time, neighbor_transition));
			transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_makespan, neighbor_transition));			
		    }
		}				
	    }
	    transition_Queue.erase(transition_Queue.begin());
	}
	return -1.0;	
    }


    sDouble sRealSMTCBS::find_KruhobotIgnoringSchedule_strong(const sKruhobot &kruhobot,
							      const s2DMap    &map,
							      sInt_32          source_loc_id,
							      sInt_32          sink_loc_id,
							      sDouble          makespan_limit,
							      sDouble          sUNUSED(extra_makespan),
							      Schedule_vector &Schedule) const
    {
	Transitions_mmap transition_Queue;
	
	Transitions_map explored_Transitions;
	Transitions_vector transition_Store;

	Transition initial_transition(transition_Store.size(), 0.0, 0.0, 0.0, source_loc_id, -1);	
	transition_Queue.insert(Transitions_mmap::value_type(0.0, initial_transition));
	transition_Store.push_back(initial_transition);

/*
	explored_Transitions.insert(Transitions_map::value_type(0.0, LocationIDs_uset()));
	explored_Transitions[0.0].insert(source_loc_id);
	LocationIDs_uset bound_explored_Transitions;
*/

	while (!transition_Queue.empty())
	{
	    const Transition &front_transition = transition_Queue.begin()->second;
	    
	    if (front_transition.m_location_id == sink_loc_id)
	    {
		sInt_32 transition_id = front_transition.m_trans_id;
		    
		Transitions_vector reversed_Schedule;
		while (transition_id >= 0)
		{
		    reversed_Schedule.push_back(transition_Store[transition_id]);
		    transition_id = transition_Store[transition_id].m_prev_trans_id;
		}
		Schedule_vector raw_Schedule;
		sInt_32 N_Events = reversed_Schedule.size();			
		
		for (sInt_32 i = N_Events - 1; i >= 1; --i)
		{	    
		    raw_Schedule.push_back(Event(reversed_Schedule[i].m_location_id, reversed_Schedule[i - 1].m_location_id,
						 reversed_Schedule[i].m_time, reversed_Schedule[i -1].m_time));
		}
		Schedule.clear();
		smooth_Schedule(raw_Schedule, Schedule);
		
		return front_transition.m_time;
	    }

	    /*
	    Transitions_map::const_iterator explored_transition = explored_Transitions.find(front_transition.m_time);

	    if (explored_transition == explored_Transitions.end())
	    {
		explored_Transitions.insert(Transitions_map::value_type(front_transition.m_time, LocationIDs_uset()));
	    }
	    */
	    if (front_transition.m_time + (map.m_shortest_Distances[sink_loc_id][front_transition.m_location_id] / kruhobot.m_properties.m_linear_velo) <= makespan_limit)
	    {
		const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[front_transition.m_location_id].m_Neighbors;
		for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
		{
		    sInt_32 neighbor_location_id = (*neighbor)->m_target->m_id;

		    sDouble transition_distance = map.m_straight_Distances[front_transition.m_location_id][neighbor_location_id];
		    if (transition_distance <= s_EPSILON)
		    {
			continue;
		    }			    							    		    
		    sDouble transition_delta_time = transition_distance / kruhobot.m_properties.m_linear_velo;
		    sDouble transition_finish_time = front_transition.m_time + transition_delta_time;
		    sDouble transition_finish_cost = front_transition.m_cost + transition_delta_time;
		    sDouble transition_finish_makespan = front_transition.m_makespan + transition_delta_time;		    
			
		    LocationIDs_uset *next_explored_Transitions = &explored_Transitions[0.0];
/*
		    Transitions_map::const_iterator explored_transition = explored_Transitions.find(transition_finish_time);
		    if (explored_transition == explored_Transitions.end())
		    {
			explored_Transitions.insert(Transitions_map::value_type(transition_finish_time, LocationIDs_uset()));
		    }
		    next_explored_Transitions = &explored_Transitions[transition_finish_time];
*/
		    if (next_explored_Transitions->find(neighbor_location_id) == next_explored_Transitions->end())
		    {
			Transition neighbor_transition(transition_Store.size(), transition_finish_time, transition_finish_cost, transition_finish_makespan, neighbor_location_id, front_transition.m_trans_id);
			transition_Store.push_back(neighbor_transition);
			
			next_explored_Transitions->insert(neighbor_location_id);
//			transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_time, neighbor_transition));
			transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_makespan, neighbor_transition));			
		    }
		}				
	    }
	    transition_Queue.erase(transition_Queue.begin());
	}
	return -1.0;	
    }


    sDouble sRealSMTCBS::find_KruhobotIgnoringSchedule_superStrong(const sKruhobot &kruhobot,
								   const s2DMap    &map,
								   sInt_32          source_loc_id,
								   sInt_32          sink_loc_id,
								   sDouble          makespan_limit,
								   sDouble          sUNUSED(extra_makespan),
								   Schedule_vector &Schedule) const
    {
	struct Arrival
	{
	    Arrival()
		: m_time(-1.0)
		, m_node_id(-1) {}

	    Arrival(sDouble time, sInt_32 node_id)
		: m_time(time)
		, m_node_id(node_id) {}
	    
	    bool operator<(const Arrival &arrival) const
	    {
		return (m_time < arrival.m_time || (m_time == arrival.m_time && m_node_id < arrival.m_node_id));
	    }
	    
	    sDouble m_time;
	    sInt_32 m_node_id;
	};

	typedef std::set<Arrival, std::less<Arrival> > Queue_set;
	Queue_set open_Queue;	

	/*
	typedef std::multimap<sDouble, sInt_32> Queue_mmap;
	Queue_mmap open_Queue;
	*/
	
	struct GraphNode
	{
	    GraphNode()
		: m_best_time(-1.0)
		, m_prev_node_id(-1) {}
	    
	    sDouble m_best_time;
	    sInt_32 m_prev_node_id;
//	    Queue_mmap::iterator m_queue_element;
	    Queue_set::iterator m_queue_element;
	};

	struct PathNode
	{
	    PathNode()
		: m_time(-1.0)
		, m_node_id(-1) {}

	    PathNode(sDouble time, sInt_32 node_id)
		: m_time(time)
		, m_node_id(node_id) {}	    
	    
	    sDouble m_time;
	    sInt_32 m_node_id;
	};
	    
	std::vector<GraphNode> best_Arrivals;
	best_Arrivals.resize(map.m_Locations.size());
	
	//Queue_mmap::iterator queue_element = open_Queue.insert(Queue_mmap::value_type(0.0, source_loc_id));
	Queue_set::iterator queue_element = open_Queue.insert(Arrival(0.0, source_loc_id)).first;
	
	best_Arrivals[source_loc_id].m_best_time = 0.0;
	best_Arrivals[source_loc_id].m_prev_node_id = -1;
	best_Arrivals[source_loc_id].m_queue_element = queue_element;

	while (!open_Queue.empty())
	{
//	    Queue_mmap::iterator queue_front = open_Queue.begin();
	    Queue_set::iterator queue_front = open_Queue.begin();	    
//	    if (queue_front->second == sink_loc_id)
	    if (queue_front->m_node_id == sink_loc_id)	    
	    {
//		sInt_32 node_id = queue_front->second;
		sInt_32 node_id = queue_front->m_node_id;		
		    
		std::vector<PathNode> reversed_Schedule;
		while (node_id >= 0)
		{
		    reversed_Schedule.push_back(PathNode(best_Arrivals[node_id].m_best_time, node_id));
		    node_id = best_Arrivals[node_id].m_prev_node_id;
		}
	
		Schedule_vector raw_Schedule;
		sInt_32 N_Events = reversed_Schedule.size();			
		
		for (sInt_32 i = N_Events - 1; i >= 1; --i)
		{	    
		    raw_Schedule.push_back(Event(reversed_Schedule[i].m_node_id, reversed_Schedule[i - 1].m_node_id,
						 reversed_Schedule[i].m_time, reversed_Schedule[i -1].m_time));
		}
		Schedule.clear();
		smooth_Schedule(raw_Schedule, Schedule);
		
//		return queue_front->first;
		return queue_front->m_time;		
	    }

//	    if (queue_front->first + (map.m_shortest_Distances[sink_loc_id][queue_front->second] / kruhobot.m_properties.m_linear_velo) <= makespan_limit)
	    if (queue_front->m_time + (map.m_shortest_Distances[sink_loc_id][queue_front->m_node_id] / kruhobot.m_properties.m_linear_velo) <= makespan_limit)	    
	    {
		//const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[queue_front->second].m_Neighbors;	
		const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[queue_front->m_node_id].m_Neighbors;
		
		
		for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
		{
		    sInt_32 neighbor_location_id = (*neighbor)->m_target->m_id;
//		    sDouble transition_distance = map.m_straight_Distances[queue_front->second][neighbor_location_id];
		    sDouble transition_distance = map.m_straight_Distances[queue_front->m_node_id][neighbor_location_id];		    

		    if (transition_distance <= s_EPSILON)
		    {
			continue;
		    }			    							    		    
		    sDouble transition_delta_time = transition_distance / kruhobot.m_properties.m_linear_velo;
//		    sDouble transition_finish_time = queue_front->first + transition_delta_time;
		    sDouble transition_finish_time = queue_front->m_time + transition_delta_time;		    

		    if (best_Arrivals[neighbor_location_id].m_best_time < 0)
		    {
//			Queue_mmap::iterator queue_element = open_Queue.insert(Queue_mmap::value_type(transition_finish_time, neighbor_location_id));
			Queue_set::iterator queue_element = open_Queue.insert(Arrival(transition_finish_time, neighbor_location_id)).first;
			best_Arrivals[neighbor_location_id].m_best_time = transition_finish_time;
//			best_Arrivals[neighbor_location_id].m_prev_node_id = queue_front->second;
			best_Arrivals[neighbor_location_id].m_prev_node_id = queue_front->m_node_id;			
			best_Arrivals[neighbor_location_id].m_queue_element = queue_element;					
		    }
		    else
		    {
			if (best_Arrivals[neighbor_location_id].m_best_time > transition_finish_time)
			{
			    best_Arrivals[neighbor_location_id].m_best_time = transition_finish_time;
//			    best_Arrivals[neighbor_location_id].m_prev_node_id = queue_front->second;
			    best_Arrivals[neighbor_location_id].m_prev_node_id = queue_front->m_node_id;			    

			    open_Queue.erase(best_Arrivals[neighbor_location_id].m_queue_element);
//			    Queue_mmap::iterator queue_element = open_Queue.insert(Queue_mmap::value_type(transition_finish_time, neighbor_location_id));
			    Queue_set::iterator queue_element = open_Queue.insert(Arrival(transition_finish_time, neighbor_location_id)).first;
			    best_Arrivals[neighbor_location_id].m_queue_element = queue_element;
			}
		    }
		}
	    }
	    open_Queue.erase(queue_front);
	}
	return -1.0;	
    }            


/*----------------------------------------------------------------------------*/
    
    void sRealSMTCBS::reflect_KruhobotCollision(const KruhobotCollision          &kruhobot_collision,
						KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
						KruhobotUlinearConflicts_vector   &kruhobot_linear_Conflicts)
    {
	introduce_KruhobotConflict(kruhobot_collision.m_traversal_A, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, (kruhobot_collision.m_traversal_A.m_kruhobot_id < 0));
	introduce_KruhobotConflict(kruhobot_collision.m_traversal_B, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, (kruhobot_collision.m_traversal_B.m_kruhobot_id < 0));	
    }
    

    void sRealSMTCBS::reflect_KruhobotCollision(const KruhobotCollision                &kruhobot_collision,
						KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
						KruhobotUlinearConflicts_upper_vector   &kruhobot_linear_Conflicts)
    {
	introduce_KruhobotConflict(kruhobot_collision.m_traversal_A, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, (kruhobot_collision.m_traversal_A.m_kruhobot_id < 0));
	introduce_KruhobotConflict(kruhobot_collision.m_traversal_B, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, (kruhobot_collision.m_traversal_B.m_kruhobot_id < 0));
    }


/*----------------------------------------------------------------------------*/
    
    void sRealSMTCBS::reflect_KruhobotCollisions(const KruhobotCollisions_mset    &kruhobot_Collisions,
						 KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
						 KruhobotUlinearConflicts_vector   &kruhobot_linear_Conflicts)
    {
	for (KruhobotCollisions_mset::const_iterator collision = kruhobot_Collisions.begin(); collision != kruhobot_Collisions.end(); ++collision)
	{
	    introduce_KruhobotConflict(collision->m_traversal_A, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, (collision->m_traversal_A.m_kruhobot_id < 0));
	    introduce_KruhobotConflict(collision->m_traversal_B, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, (collision->m_traversal_B.m_kruhobot_id < 0));
	}
    }

    
    void sRealSMTCBS::reflect_KruhobotCollisions(const KruhobotCollisions_mset          &kruhobot_Collisions,
						 KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
						 KruhobotUlinearConflicts_upper_vector   &kruhobot_linear_Conflicts)
    {
	for (KruhobotCollisions_mset::const_iterator collision = kruhobot_Collisions.begin(); collision != kruhobot_Collisions.end(); ++collision)
	{
	    introduce_KruhobotConflict(collision->m_traversal_A, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, (collision->m_traversal_A.m_kruhobot_id < 0));
	    introduce_KruhobotConflict(collision->m_traversal_B, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, (collision->m_traversal_B.m_kruhobot_id < 0));
	}
    }    


/*----------------------------------------------------------------------------*/
    
    void sRealSMTCBS::reflect_KruhobotCollision(const KruhobotCollision          &kruhobot_collision,
						KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
						KruhobotUlinearConflicts_vector   &kruhobot_linear_Conflicts,
						sInt_32                          &last_conflict_id)
    {
	introduce_KruhobotConflict(kruhobot_collision.m_traversal_A, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, last_conflict_id, (kruhobot_collision.m_traversal_A.m_kruhobot_id < 0));
	introduce_KruhobotConflict(kruhobot_collision.m_traversal_B, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, last_conflict_id, (kruhobot_collision.m_traversal_B.m_kruhobot_id < 0));	
    }
    

    void sRealSMTCBS::reflect_KruhobotCollision(const KruhobotCollision                &kruhobot_collision,
						KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
						KruhobotUlinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
						sInt_32                                &last_conflict_id)
    {
	introduce_KruhobotConflict(kruhobot_collision.m_traversal_A, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, last_conflict_id, (kruhobot_collision.m_traversal_A.m_kruhobot_id < 0));
	introduce_KruhobotConflict(kruhobot_collision.m_traversal_B, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, last_conflict_id, (kruhobot_collision.m_traversal_B.m_kruhobot_id < 0));
    }


/*----------------------------------------------------------------------------*/
    
    void sRealSMTCBS::reflect_KruhobotCollisions(const KruhobotCollisions_mset    &kruhobot_Collisions,
						 KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
						 KruhobotUlinearConflicts_vector   &kruhobot_linear_Conflicts,
						 sInt_32                          &last_conflict_id)
    {
	for (KruhobotCollisions_mset::const_iterator collision = kruhobot_Collisions.begin(); collision != kruhobot_Collisions.end(); ++collision)
	{
	    introduce_KruhobotConflict(collision->m_traversal_A, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, last_conflict_id, (collision->m_traversal_A.m_kruhobot_id < 0));
	    introduce_KruhobotConflict(collision->m_traversal_B, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, last_conflict_id, (collision->m_traversal_B.m_kruhobot_id < 0));
	}
    }

    
    void sRealSMTCBS::reflect_KruhobotCollisions(const KruhobotCollisions_mset          &kruhobot_Collisions,
						 KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
						 KruhobotUlinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
						 sInt_32                                &last_conflict_id)
    {
	for (KruhobotCollisions_mset::const_iterator collision = kruhobot_Collisions.begin(); collision != kruhobot_Collisions.end(); ++collision)
	{
	    introduce_KruhobotConflict(collision->m_traversal_A, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, last_conflict_id, (collision->m_traversal_A.m_kruhobot_id < 0));
	    introduce_KruhobotConflict(collision->m_traversal_B, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, last_conflict_id, (collision->m_traversal_B.m_kruhobot_id < 0));
	}
    }


/*----------------------------------------------------------------------------*/
    
    void sRealSMTCBS::reflect_KruhobotCollision(const KruhobotCollision          &kruhobot_collision,
						KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
						KruhobotUlinearConflicts_vector   &kruhobot_linear_Conflicts,
						KruhobotAffections_vector        &affected_Kruhobots,
						sInt_32                          &last_conflict_id)
    {
	introduce_KruhobotConflict(kruhobot_collision.m_traversal_A, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, last_conflict_id, (kruhobot_collision.m_traversal_A.m_kruhobot_id < 0));
	introduce_KruhobotConflict(kruhobot_collision.m_traversal_B, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, last_conflict_id, (kruhobot_collision.m_traversal_B.m_kruhobot_id < 0));

	++affected_Kruhobots[sABS(kruhobot_collision.m_traversal_A.m_kruhobot_id)];
	++affected_Kruhobots[sABS(kruhobot_collision.m_traversal_B.m_kruhobot_id)];	
    }
    

    void sRealSMTCBS::reflect_KruhobotCollision(const KruhobotCollision                &kruhobot_collision,
						KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
						KruhobotUlinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
						KruhobotAffections_vector              &affected_Kruhobots,						
						sInt_32                                &last_conflict_id)
    {
	introduce_KruhobotConflict(kruhobot_collision.m_traversal_A, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, last_conflict_id, (kruhobot_collision.m_traversal_A.m_kruhobot_id < 0));
	introduce_KruhobotConflict(kruhobot_collision.m_traversal_B, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, last_conflict_id, (kruhobot_collision.m_traversal_B.m_kruhobot_id < 0));

	++affected_Kruhobots[sABS(kruhobot_collision.m_traversal_A.m_kruhobot_id)];
	++affected_Kruhobots[sABS(kruhobot_collision.m_traversal_B.m_kruhobot_id)];		
    }


/*----------------------------------------------------------------------------*/
    
    void sRealSMTCBS::reflect_KruhobotCollisions(const KruhobotCollisions_mset    &kruhobot_Collisions,
						 KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
						 KruhobotUlinearConflicts_vector   &kruhobot_linear_Conflicts,
						 KruhobotAffections_vector        &affected_Kruhobots,
						 sInt_32                          &last_conflict_id)
    {
	for (KruhobotCollisions_mset::const_iterator collision = kruhobot_Collisions.begin(); collision != kruhobot_Collisions.end(); ++collision)
	{
	    introduce_KruhobotConflict(collision->m_traversal_A, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, last_conflict_id, (collision->m_traversal_A.m_kruhobot_id < 0));
	    introduce_KruhobotConflict(collision->m_traversal_B, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, last_conflict_id, (collision->m_traversal_B.m_kruhobot_id < 0));

	    ++affected_Kruhobots[sABS(collision->m_traversal_A.m_kruhobot_id)];
	    ++affected_Kruhobots[sABS(collision->m_traversal_B.m_kruhobot_id)];		    
	}
    }

    
    void sRealSMTCBS::reflect_KruhobotCollisions(const KruhobotCollisions_mset          &kruhobot_Collisions,
						 KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
						 KruhobotUlinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
						 KruhobotAffections_vector              &affected_Kruhobots,
						 sInt_32                                &last_conflict_id)
    {
	for (KruhobotCollisions_mset::const_iterator collision = kruhobot_Collisions.begin(); collision != kruhobot_Collisions.end(); ++collision)
	{
	    introduce_KruhobotConflict(collision->m_traversal_A, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, last_conflict_id, (collision->m_traversal_A.m_kruhobot_id < 0));
	    introduce_KruhobotConflict(collision->m_traversal_B, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, last_conflict_id, (collision->m_traversal_B.m_kruhobot_id < 0));

	    ++affected_Kruhobots[sABS(collision->m_traversal_A.m_kruhobot_id)];
	    ++affected_Kruhobots[sABS(collision->m_traversal_B.m_kruhobot_id)];		    
	}
    }

    
/*----------------------------------------------------------------------------*/
    
    bool sRealSMTCBS::find_InitialNonconflictingSchedules(Glucose::Solver                       *solver,
							  RealContext                           &context,					 
							  const sRealInstance                   &real_Instance,
							  KruhobotDecisionDiagrams_vector       &kruhobot_RDDs,
							  const KruhobotDecisionMappings_vector &kruhobot_RDD_Mappings, 
							  RealModel                             &real_sat_Model,
							  KruhobotSchedules_vector              &kruhobot_Schedules) const
    {
	sInt_32 variable_ID = build_RealModelVariables(solver, context, real_Instance, kruhobot_RDDs, kruhobot_RDD_Mappings, real_sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);
	build_RealModelConstraints(solver, context, real_Instance, kruhobot_RDDs, kruhobot_RDD_Mappings, real_sat_Model);
		
	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif
	    return false;
	}
	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
	decode_PathModel(solver, real_Instance, kruhobot_RDDs, real_sat_Model, kruhobot_Schedules);
	sASSERT(verify_KruhobotSchedules(real_Instance, kruhobot_Schedules));

	return true;
    }


    bool sRealSMTCBS::find_InitialNonconflictingSchedules(Glucose::Solver                       *solver,
							  RealContext                           &context,					 
							  const sRealInstance                   &real_Instance,
							  KruhobotDecisionDiagrams_vector       &kruhobot_RDDs,
							  const KruhobotDecisionMappings_vector &kruhobot_RDD_Mappings,
							  sDouble                                sUNUSED(cost_bound),
							  const std::vector<sDouble>            &sUNUSED(kruhobot_lower_cost_Bounds),
							  RealModel                             &real_sat_Model,
							  KruhobotSchedules_vector              &kruhobot_Schedules) const
    {
	sInt_32 variable_ID = build_CostRealModelVariables(solver, context, real_Instance, kruhobot_RDDs, kruhobot_RDD_Mappings, real_sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);
	build_RealModelConstraints(solver, context, real_Instance, kruhobot_RDDs, kruhobot_RDD_Mappings, real_sat_Model);
		
	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif
	    return false;
	}
	Glucose::lbool result = solver->solve_();

	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
	decode_PathModel(solver, real_Instance, kruhobot_RDDs, real_sat_Model, kruhobot_Schedules);
	sASSERT(verify_KruhobotSchedules(real_Instance, kruhobot_Schedules));

	return true;
    }    


    bool sRealSMTCBS::find_NextNonconflictingSchedules(Glucose::Solver                       *solver,
						       RealContext                           &context,					 
						       const sRealInstance                   &real_Instance,
						       KruhobotDecisionDiagrams_vector       &kruhobot_RDDs,
						       const KruhobotDecisionMappings_vector &kruhobot_RDD_Mappings,						       
						       const KruhobotCollisions_mset         &kruhobot_Collisions,						       
						       RealModel                             &real_sat_Model,
						       KruhobotSchedules_vector              &kruhobot_Schedules) const
    {
	sInt_32 variable_ID = build_RealModelVariables(solver, context, real_Instance, kruhobot_RDDs, kruhobot_RDD_Mappings, real_sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_RealModelConstraints(solver, context, real_Instance, kruhobot_RDDs, kruhobot_RDD_Mappings, real_sat_Model);

	refine_RealModelConstraints(solver,
				    context,
				    real_Instance,
				    kruhobot_RDDs,
				    kruhobot_RDD_Mappings,
				    kruhobot_Collisions,
				    real_sat_Model);

	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif
	    return false;
	}
	Glucose::lbool result = solver->solve_();
	
	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
	decode_PathModel(solver, real_Instance, kruhobot_RDDs, real_sat_Model, kruhobot_Schedules);
	sASSERT(verify_KruhobotSchedules(real_Instance, kruhobot_Schedules));
	
	return true;	
    }

    bool sRealSMTCBS::find_NextNonconflictingSchedules(Glucose::Solver                       *solver,
						       RealContext                           &context,					 
						       const sRealInstance                   &real_Instance,
						       KruhobotDecisionDiagrams_vector       &kruhobot_RDDs,
						       const KruhobotDecisionMappings_vector &kruhobot_RDD_Mappings,
						       const KruhobotCollisions_mset         &kruhobot_Collisions,
						       sDouble                                cost_bound,
						       const std::vector<sDouble>            &kruhobot_lower_cost_Bounds,
						       RealModel                             &real_sat_Model,
						       KruhobotSchedules_vector              &kruhobot_Schedules,
						       KruhobotExtraVariables_vector         &kruhobot_set_extra_Variables,
						       KruhobotExtraVariables_vector         &kruhobot_all_extra_Variables) const
    {
	sInt_32 variable_ID = build_CostRealModelVariables(solver, context, real_Instance, kruhobot_RDDs, kruhobot_RDD_Mappings, real_sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_RealModelConstraints(solver, context, real_Instance, kruhobot_RDDs, kruhobot_RDD_Mappings, real_sat_Model);

	refine_RealModelConstraints(solver,
				    context,
				    real_Instance,
				    kruhobot_RDDs,
				    kruhobot_RDD_Mappings,
				    kruhobot_Collisions,
				    real_sat_Model);

	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif
	    return false;
	}
	Glucose::lbool result = solver->solve_();
	
	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif	    
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif	    	    
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
	decode_PathModel(solver,
			 real_Instance,
			 kruhobot_RDDs,
			 real_sat_Model,
			 cost_bound,
			 kruhobot_lower_cost_Bounds,			 
			 kruhobot_Schedules,
			 kruhobot_set_extra_Variables,
			 kruhobot_all_extra_Variables);
	
	sASSERT(verify_KruhobotSchedules(real_Instance, kruhobot_Schedules));
	
	return true;	
    }


    bool sRealSMTCBS::find_NearNonconflictingSchedules(Glucose::Solver                       *solver,
						       RealContext                           &context,
						       const sRealInstance                   &real_Instance,
						       KruhobotDecisionDiagrams_vector       &kruhobot_RDDs,
						       const KruhobotDecisionMappings_vector &kruhobot_RDD_Mappings,
						       const KruhobotCollisions_mset         &sUNUSED(kruhobot_Collisions),
						       const KruhobotCollisions_mset         &next_kruhobot_Collisions,
						       RealModel                             &real_sat_Model,
						       KruhobotSchedules_vector              &kruhobot_Schedules) const
    {
	/*
	sInt_32 variable_ID = build_RealModelVariables(solver, context, real_Instance, kruhobot_RDDs, kruhobot_RDD_Mappings, real_sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_RealModelConstraints(solver, context, real_Instance, kruhobot_RDDs, kruhobot_RDD_Mappings, real_sat_Model);
	*/

	refine_RealModelConstraints(solver,
				    context,
				    real_Instance,
				    kruhobot_RDDs,
				    kruhobot_RDD_Mappings,
				    next_kruhobot_Collisions,				    
				    real_sat_Model);
	
	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif
	    return false;
	}
	
	Glucose::lbool result = solver->solve_();
	
	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
	decode_PathModel(solver, real_Instance, kruhobot_RDDs, real_sat_Model, kruhobot_Schedules);
	sASSERT(verify_KruhobotSchedules(real_Instance, kruhobot_Schedules));
	
	return true;	
    }


    bool sRealSMTCBS::find_NearNonconflictingSchedules(Glucose::Solver                       *solver,
						       RealContext                           &context,
						       const sRealInstance                   &real_Instance,
						       KruhobotDecisionDiagrams_vector       &kruhobot_RDDs,
						       const KruhobotDecisionMappings_vector &kruhobot_RDD_Mappings,
						       const KruhobotCollisions_mset         &sUNUSED(kruhobot_Collisions),
						       const KruhobotCollisions_mset         &next_kruhobot_Collisions,
						       sDouble                                cost_bound,					      
						       const std::vector<sDouble>            &kruhobot_lower_cost_Bounds,
						       RealModel                             &real_sat_Model,
						       KruhobotSchedules_vector              &kruhobot_Schedules,
						       KruhobotExtraVariables_vector         &kruhobot_set_extra_Variables,
						       KruhobotExtraVariables_vector         &kruhobot_all_extra_Variables) const
    {
	/*
	sInt_32 variable_ID = build_RealModelVariables(solver, context, real_Instance, kruhobot_RDDs, kruhobot_RDD_Mappings, real_sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_RealModelConstraints(solver, context, real_Instance, kruhobot_RDDs, kruhobot_RDD_Mappings, real_sat_Model);
	*/
	refine_RealModelConstraints(solver,
				    context,
				    real_Instance,
				    kruhobot_RDDs,
				    kruhobot_RDD_Mappings,
				    next_kruhobot_Collisions,				    
				    real_sat_Model);
	
	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif
	    return false;
	}
	
	Glucose::lbool result = solver->solve_();
	
	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
	decode_PathModel(solver,
			 real_Instance,
			 kruhobot_RDDs,
			 real_sat_Model,
			 cost_bound,
			 kruhobot_lower_cost_Bounds,			 
			 kruhobot_Schedules,
			 kruhobot_set_extra_Variables,
			 kruhobot_all_extra_Variables);	
	sASSERT(verify_KruhobotSchedules(real_Instance, kruhobot_Schedules));
	
	return true;	
    }


    bool sRealSMTCBS::find_BetterNonconflictingSchedules(Glucose::Solver                       *solver,
							 RealContext                           &context,
							 const sRealInstance                   &real_Instance,
							 KruhobotDecisionDiagrams_vector       &kruhobot_RDDs,
							 const KruhobotDecisionMappings_vector &kruhobot_RDD_Mappings,
							 const KruhobotCollisions_mset         &sUNUSED(kruhobot_Collisions),
							 const KruhobotCollisions_mset         &sUNUSED(next_kruhobot_Collisions),
							 sDouble                                cost_bound,					      
							 const std::vector<sDouble>            &kruhobot_lower_cost_Bounds,
							 RealModel                             &real_sat_Model,
							 KruhobotSchedules_vector              &kruhobot_Schedules,
							 const KruhobotExtraVariables_vector   &kruhobot_envelope_Variables,
							 KruhobotExtraVariables_vector         &kruhobot_set_extra_Variables,
							 KruhobotExtraVariables_vector         &kruhobot_all_extra_Variables) const
    {
	/*
	sInt_32 variable_ID = build_RealModelVariables(solver, context, real_Instance, kruhobot_RDDs, kruhobot_RDD_Mappings, real_sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_RealModelConstraints(solver, context, real_Instance, kruhobot_RDDs, kruhobot_RDD_Mappings, real_sat_Model);

	refine_RealModelConstraints(solver,
				    context,
				    real_Instance,
				    kruhobot_RDDs,
				    kruhobot_RDD_Mappings,
				    next_kruhobot_Collisions,				    
				    real_sat_Model);
	*/

	introduce_RealModelCostCardinalities(solver,
					     context,
					     real_Instance,
					     kruhobot_RDDs,
					     kruhobot_RDD_Mappings,
					     cost_bound,
					     kruhobot_lower_cost_Bounds,					 
					     real_sat_Model,
					     kruhobot_envelope_Variables);	
	
	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif
	    return false;
	}
	
	Glucose::lbool result = solver->solve_();
	
	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
	decode_PathModel(solver,
			 real_Instance,
			 kruhobot_RDDs,
			 real_sat_Model,
			 cost_bound,
			 kruhobot_lower_cost_Bounds,			 
			 kruhobot_Schedules,
			 kruhobot_set_extra_Variables,
			 kruhobot_all_extra_Variables);
	
	sASSERT(verify_KruhobotSchedules(real_Instance, kruhobot_Schedules));
	
	return true;	
    }


    bool sRealSMTCBS::find_BetterNonconflictingSchedules(Glucose::Solver                       *solver,
							 RealContext                           &context,
							 const sRealInstance                   &real_Instance,
							 KruhobotDecisionDiagrams_vector       &kruhobot_RDDs,
							 const KruhobotDecisionMappings_vector &kruhobot_RDD_Mappings,
							 const KruhobotCollisions_mset         &sUNUSED(kruhobot_Collisions),
							 const KruhobotCollisions_mset         &sUNUSED(next_kruhobot_Collisions),
							 sDouble                                cost_bound,					      
							 const std::vector<sDouble>            &kruhobot_lower_cost_Bounds,
							 RealModel                             &real_sat_Model,
							 KruhobotSchedules_vector              &kruhobot_Schedules,
							 const KruhobotExtraVariables_vector   &kruhobot_envelope_Variables,
							 const KruhobotExtraVariables_vector   &kruhobot_domus_Variables,							 
							 KruhobotExtraVariables_vector         &kruhobot_set_extra_Variables,
							 KruhobotExtraVariables_vector         &kruhobot_all_extra_Variables) const
    {
	/*
	sInt_32 variable_ID = build_RealModelVariables(solver, context, real_Instance, kruhobot_RDDs, kruhobot_RDD_Mappings, real_sat_Model);
	m_solver_Encoder->set_LastVariableID(variable_ID);

	build_RealModelConstraints(solver, context, real_Instance, kruhobot_RDDs, kruhobot_RDD_Mappings, real_sat_Model);

	refine_RealModelConstraints(solver,
				    context,
				    real_Instance,
				    kruhobot_RDDs,
				    kruhobot_RDD_Mappings,
				    next_kruhobot_Collisions,				    
				    real_sat_Model);
	*/

	introduce_RealModelCostCardinalities(solver,
					     context,
					     real_Instance,
					     kruhobot_RDDs,
					     kruhobot_RDD_Mappings,
					     cost_bound,
					     kruhobot_lower_cost_Bounds,
					     real_sat_Model,
					     kruhobot_envelope_Variables,
					     kruhobot_domus_Variables);	
	
	if (!solver->simplify())
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif
	    return false;
	}
	
	Glucose::lbool result = solver->solve_();
	
	if (result == l_True)
	{
    	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_satisfiable_SAT_solver_Calls;
	    }
	    #endif
	}
	else if (result == l_False)
	{
  	    #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_unsatisfiable_SAT_solver_Calls;
	    }
	    #endif
	    return false;
	}
	else if (result == l_Undef)
	{
	    return false;
	}
	else
	{
	    sASSERT(false);
	}
	decode_PathModel(solver,
			 real_Instance,
			 kruhobot_RDDs,
			 real_sat_Model,
			 cost_bound,
			 kruhobot_lower_cost_Bounds,			 
			 kruhobot_Schedules,
			 kruhobot_set_extra_Variables,
			 kruhobot_all_extra_Variables);
	
	sASSERT(verify_KruhobotSchedules(real_Instance, kruhobot_Schedules));
	
	return true;	
    }                    
   

/*----------------------------------------------------------------------------*/
    
    void sRealSMTCBS::collect_CharacteristicMakespans(const sRealInstance &real_Instance, const KruhobotDecisionDiagrams_vector &kruhobot_RDDs, Makespans_vector &characteristic_Makespans) const
    {
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();
	characteristic_Makespans.resize(N_kruhobots + 1, -1.0);
		
	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
	    for (KruhobotDecisionDiagram_vector::const_iterator decision = kruhobot_RDDs[kruhobot_id].begin(); decision != kruhobot_RDDs[kruhobot_id].end(); ++decision)
	    {	    
		if (real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id] == decision->m_location_id)
		{
		    characteristic_Makespans[kruhobot_id] = sMAX(characteristic_Makespans[kruhobot_id], decision->m_time);
		}
	    }
	}
    }


    bool sRealSMTCBS::compare_CharacteristicMakespans(const sRealInstance &real_Instance, const Makespans_vector &characteristic_Makespans, const Makespans_vector &next_characteristic_Makespans) const
    {
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();
	
	sASSERT(characteristic_Makespans.size() == N_kruhobots + 1);
	sASSERT(next_characteristic_Makespans.size() == N_kruhobots + 1);	

	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
	    if (sABS(characteristic_Makespans[kruhobot_id] - next_characteristic_Makespans[kruhobot_id]) > s_EPSILON)
	    {
		return false;
	    }
	}
	return true;
    }
    
    
    bool sRealSMTCBS::compare_KruhobotRealDecisionDiagrams(const s2DMap &sUNUSED(map), const KruhobotDecisionDiagram_vector &kruhobot_RDD, const KruhobotDecisionDiagram_vector &next_kruhobot_RDD) const
    {
	#ifdef sDEBUG
	{
	    /*
	    printf("Kruhobot RDD: %ld\n", kruhobot_RDD.size());
	    for (KruhobotDecisionDiagram_vector::const_iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	    {
		printf("  l: %d, t: %.3f\n", decision->m_location_id, decision->m_time); 
	    }
	    printf("Next kruhobot RDD: %ld\n", next_kruhobot_RDD.size());
	    for (KruhobotDecisionDiagram_vector::const_iterator next_decision = next_kruhobot_RDD.begin(); next_decision != next_kruhobot_RDD.end(); ++next_decision)
	    {
		printf("  l: %d, t: %.3f\n", next_decision->m_location_id, next_decision->m_time); 
	    }
	    */	    
	}
	#endif
	
	if (kruhobot_RDD.size() == next_kruhobot_RDD.size())
	{	    
	    for (KruhobotDecisionDiagram_vector::const_iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	    {
		bool matched = false;
		
		for (KruhobotDecisionDiagram_vector::const_iterator next_decision = next_kruhobot_RDD.begin(); next_decision != next_kruhobot_RDD.end(); ++next_decision)
		{
		    if (decision->m_location_id == next_decision->m_location_id && sABS(decision->m_time - next_decision->m_time) < s_EPSILON)
		    {
			matched = true;
			break;
		    }
		}
		if (!matched)
		{
		    return false;
		}
	    }
	    return true;
	}
	else
	{
	    return false;
	}
    }


    bool sRealSMTCBS::compare_KruhobotRealDecisionDiagrams_smart(const s2DMap &map, const KruhobotDecisionDiagram_vector &kruhobot_RDD, const KruhobotDecisionDiagram_vector &next_kruhobot_RDD) const
    {
	#ifdef sDEBUG
	{
	    /*
	    printf("Kruhobot RDD: %ld\n", kruhobot_RDD.size());
	    for (KruhobotDecisionDiagram_vector::const_iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	    {
		printf("  l: %d, t: %.3f\n", decision->m_location_id, decision->m_time); 
	    }
	    printf("Next kruhobot RDD: %ld\n", next_kruhobot_RDD.size());
	    for (KruhobotDecisionDiagram_vector::const_iterator next_decision = next_kruhobot_RDD.begin(); next_decision != next_kruhobot_RDD.end(); ++next_decision)
	    {
		printf("  l: %d, t: %.3f\n", next_decision->m_location_id, next_decision->m_time); 
	    }
	    */	    
	}
	#endif	
	if (kruhobot_RDD.size() == next_kruhobot_RDD.size())
	{
	    LocationDecisionTimes_vector location_decision_Times;
	    location_decision_Times.resize(map.m_Network.get_VertexCount());
	    
	    for (KruhobotDecisionDiagram_vector::const_iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	    {
		represent_DecisionTime(location_decision_Times[decision->m_location_id], decision->m_time);
	    }	    
		
	    for (KruhobotDecisionDiagram_vector::const_iterator next_decision = next_kruhobot_RDD.begin(); next_decision != next_kruhobot_RDD.end(); ++next_decision)
	    {
		if (obtain_DecisionTime(location_decision_Times[next_decision->m_location_id], next_decision->m_time) == location_decision_Times[next_decision->m_location_id].end())
		{
		    return false;
		}
	    }
	    return true;
	}
	else
	{
	    return false;
	}
    }	


    void sRealSMTCBS::represent_DecisionTime(DecisionTimes_set &decision_Times, sDouble time) const
    {
	DecisionTimes_set::const_iterator decision_time = obtain_DecisionTime(decision_Times, time);

	if (decision_time == decision_Times.end())
	{
	    decision_Times.insert(time);
	}	
    }

    
    sRealSMTCBS::DecisionTimes_set::const_iterator sRealSMTCBS::obtain_DecisionTime(DecisionTimes_set &decision_Times, sDouble time) const
    {
	DecisionTimes_set::const_iterator decision_time = decision_Times.lower_bound(time - s_EPSILON);

	while (decision_time != decision_Times.end())
	{
	    if (sABS(*decision_time - time) < s_EPSILON)
	    {
		return decision_time;
	    }
	    else
	    {
		sASSERT(*decision_time > time + s_EPSILON)
		break;
	    }
	    ++decision_time;
	}
	return decision_Times.end();
    }    

    
/*----------------------------------------------------------------------------*/
    
    sDouble sRealSMTCBS::calc_MakespanBound(const sRealInstance                    &real_Instance,
					    const KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
					    const KruhobotUlinearConflicts_vector   &kruhobot_linear_Conflicts) const
    {
	sDouble makespan_bound = 0.0;	
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();

	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
	    sDouble kruhobot_makespan_bound = calc_KruhobotMakespanBound(real_Instance.m_Kruhobots[kruhobot_id],
									 kruhobot_location_Conflicts[kruhobot_id],
									 kruhobot_linear_Conflicts[kruhobot_id]);
	    makespan_bound = sMAX(makespan_bound, kruhobot_makespan_bound);
	}
	return makespan_bound;
    }

    
    sDouble sRealSMTCBS::calc_MakespanBound(const sRealInstance                          &real_Instance,
					    const KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
					    const KruhobotUlinearConflicts_upper_vector   &kruhobot_linear_Conflicts) const
    {
	sDouble makespan_bound = 0.0;	
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();

	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
	    sDouble kruhobot_makespan_bound = calc_KruhobotMakespanBound(real_Instance.m_Kruhobots[kruhobot_id],
									 kruhobot_location_Conflicts[kruhobot_id],
									 kruhobot_linear_Conflicts[kruhobot_id]);
	    makespan_bound = sMAX(makespan_bound, kruhobot_makespan_bound);
	}
	return makespan_bound;
    }
    
    
    sDouble sRealSMTCBS::calc_KruhobotMakespanBound(const sKruhobot &sUNUSED(kruhobot), const LocationConflicts__umap &location_Conflicts, const UlinearConflicts__map &linear_Conflicts) const
    {
	sDouble makespan_bound = 0.0;
	
	for (LocationConflicts__umap::const_iterator location_Conflict = location_Conflicts.begin(); location_Conflict != location_Conflicts.end(); ++location_Conflict)
	{
	    for (LocationConflicts_map::const_iterator location_conflict = location_Conflict->second.begin(); location_conflict != location_Conflict->second.end(); ++location_conflict)
	    {
		makespan_bound = sMAX(makespan_bound, location_conflict->first.m_upper);
	    }
	}

	for (UlinearConflicts__map::const_iterator linear_Conflict = linear_Conflicts.begin(); linear_Conflict != linear_Conflicts.end(); ++linear_Conflict)
	{
	    for (UlinearConflicts_map::const_iterator linear_conflict = linear_Conflict->second.begin(); linear_conflict != linear_Conflict->second.end(); ++linear_conflict)
	    {
		makespan_bound = sMAX(makespan_bound, linear_conflict->first.m_upper);
	    }
	}	

	return makespan_bound;
    }


    sDouble sRealSMTCBS::calc_KruhobotMakespanBound(const sKruhobot &sUNUSED(kruhobot), const LocationConflicts_upper__umap &location_Conflicts, const UlinearConflicts_upper__map &linear_Conflicts) const
    {
	sDouble makespan_bound = 0.0;
	
	for (LocationConflicts_upper__umap::const_iterator location_Conflict = location_Conflicts.begin(); location_Conflict != location_Conflicts.end(); ++location_Conflict)
	{
	    for (LocationConflicts_upper_map::const_iterator location_conflict = location_Conflict->second.begin(); location_conflict != location_Conflict->second.end(); ++location_conflict)
	    {
		makespan_bound = sMAX(makespan_bound, location_conflict->first.m_upper);
	    }
	}

	for (UlinearConflicts_upper__map::const_iterator linear_Conflict = linear_Conflicts.begin(); linear_Conflict != linear_Conflicts.end(); ++linear_Conflict)
	{
	    for (UlinearConflicts_upper_map::const_iterator linear_conflict = linear_Conflict->second.begin(); linear_conflict != linear_Conflict->second.end(); ++linear_conflict)
	    {
		makespan_bound = sMAX(makespan_bound, linear_conflict->first.m_upper);
	    }
	}	

	return makespan_bound;
    }    


/*----------------------------------------------------------------------------*/
    
    sDouble sRealSMTCBS::calc_NextMakespanBound(sDouble                                 prev_makespan_bound,
						const sRealInstance                    &real_Instance,
						const KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
						const KruhobotUlinearConflicts_vector   &kruhobot_linear_Conflicts) const
    {
	sDouble next_makespan_bound = -1.0;	
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();

	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
	    sDouble kruhobot_next_makespan_bound = calc_KruhobotNextMakespanBound(prev_makespan_bound,
										  real_Instance.m_Kruhobots[kruhobot_id],
										  kruhobot_location_Conflicts[kruhobot_id],
										  kruhobot_linear_Conflicts[kruhobot_id]);
	    
	    next_makespan_bound = (next_makespan_bound < 0.0) ? kruhobot_next_makespan_bound : sMIN(next_makespan_bound, kruhobot_next_makespan_bound);
	}
	return next_makespan_bound;
    }

    
    sDouble sRealSMTCBS::calc_NextMakespanBound(sDouble                                       prev_makespan_bound,
						const sRealInstance                          &real_Instance,
						const KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
						const KruhobotUlinearConflicts_upper_vector   &kruhobot_linear_Conflicts) const
    {
	sDouble next_makespan_bound = 0.0;	
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();

	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
	    sDouble kruhobot_next_makespan_bound = calc_KruhobotNextMakespanBound(prev_makespan_bound,
										  real_Instance.m_Kruhobots[kruhobot_id],
										  kruhobot_location_Conflicts[kruhobot_id],
										  kruhobot_linear_Conflicts[kruhobot_id]);
	    
	    next_makespan_bound = (next_makespan_bound < 0.0) ? kruhobot_next_makespan_bound : sMIN(next_makespan_bound, kruhobot_next_makespan_bound);
	}
	return next_makespan_bound;
    }
    
    
    sDouble sRealSMTCBS::calc_KruhobotNextMakespanBound(sDouble                        prev_makespan_bound,
							const sKruhobot               &sUNUSED(kruhobot),
							const LocationConflicts__umap &location_Conflicts,
							const UlinearConflicts__map    &linear_Conflicts) const
    {
	sDouble next_makespan_bound = -1.0;
	
	for (LocationConflicts__umap::const_iterator location_Conflict = location_Conflicts.begin(); location_Conflict != location_Conflicts.end(); ++location_Conflict)
	{
	    for (LocationConflicts_map::const_iterator location_conflict = location_Conflict->second.begin(); location_conflict != location_Conflict->second.end(); ++location_conflict)
	    {
		if (next_makespan_bound < 0.0)
		{
		    if (location_conflict->first.m_upper > prev_makespan_bound + s_EPSILON)
		    {
			next_makespan_bound = location_conflict->first.m_upper;
		    }
		}
		else
		{
		    if (location_conflict->first.m_upper > prev_makespan_bound + s_EPSILON && next_makespan_bound > location_conflict->first.m_upper + s_EPSILON)
		    {
			next_makespan_bound = location_conflict->first.m_upper;
		    }
		}
	    }
	}

	for (UlinearConflicts__map::const_iterator linear_Conflict = linear_Conflicts.begin(); linear_Conflict != linear_Conflicts.end(); ++linear_Conflict)
	{
	    for (UlinearConflicts_map::const_iterator linear_conflict = linear_Conflict->second.begin(); linear_conflict != linear_Conflict->second.end(); ++linear_conflict)
	    {
		if (next_makespan_bound < 0.0)
		{
		    if (linear_conflict->first.m_upper > prev_makespan_bound + s_EPSILON)
		    {
			next_makespan_bound = linear_conflict->first.m_upper;
		    }
		}
		else
		{
		    if (linear_conflict->first.m_upper > prev_makespan_bound + s_EPSILON && next_makespan_bound > linear_conflict->first.m_upper + s_EPSILON)
		    {
			next_makespan_bound = linear_conflict->first.m_upper;
		    }
		}		
	    }
	}	

	return next_makespan_bound;
    }


    sDouble sRealSMTCBS::calc_KruhobotNextMakespanBound(sDouble                              prev_makespan_bound,
							const sKruhobot                     &sUNUSED(kruhobot),
							const LocationConflicts_upper__umap &location_Conflicts,
							const UlinearConflicts_upper__map    &linear_Conflicts) const
    {
	sDouble next_makespan_bound = -1.0;
	
	for (LocationConflicts_upper__umap::const_iterator location_Conflict = location_Conflicts.begin(); location_Conflict != location_Conflicts.end(); ++location_Conflict)
	{
	    for (LocationConflicts_upper_map::const_iterator location_conflict = location_Conflict->second.begin(); location_conflict != location_Conflict->second.end(); ++location_conflict)
	    {
		if (next_makespan_bound < 0.0)
		{
		    if (location_conflict->first.m_upper > prev_makespan_bound + s_EPSILON)
		    {
			next_makespan_bound = location_conflict->first.m_upper;
		    }
		}
		else
		{
		    if (location_conflict->first.m_upper > prev_makespan_bound + s_EPSILON && next_makespan_bound > location_conflict->first.m_upper + s_EPSILON)
		    {
			next_makespan_bound = location_conflict->first.m_upper;
		    }
		}
	    }
	}

	for (UlinearConflicts_upper__map::const_iterator linear_Conflict = linear_Conflicts.begin(); linear_Conflict != linear_Conflicts.end(); ++linear_Conflict)
	{
	    for (UlinearConflicts_upper_map::const_iterator linear_conflict = linear_Conflict->second.begin(); linear_conflict != linear_Conflict->second.end(); ++linear_conflict)
	    {
		if (next_makespan_bound < 0.0)
		{
		    if (linear_conflict->first.m_upper > prev_makespan_bound + s_EPSILON)
		    {
			next_makespan_bound = linear_conflict->first.m_upper;
		    }
		}
		else
		{
		    if (linear_conflict->first.m_upper > prev_makespan_bound + s_EPSILON && next_makespan_bound > linear_conflict->first.m_upper + s_EPSILON)
		    {
			next_makespan_bound = linear_conflict->first.m_upper;
		    }
		}		
	    }
	}	

	return next_makespan_bound;
    }    

    
/*----------------------------------------------------------------------------*/

    bool sRealSMTCBS::verify_KruhobotSchedules(const sRealInstance &real_Instance, const KruhobotSchedules_vector &kruhobot_Schedules) const
    {
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();
	
	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
	    sDouble current_time = 0.0;
	    sInt_32 current_location_id = real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id];
	    
	    kruhobot_Schedules[kruhobot_id];
	    for (Schedule_vector::const_iterator event = kruhobot_Schedules[kruhobot_id].begin(); event != kruhobot_Schedules[kruhobot_id].end(); ++event)
	    {
		if (sABS(current_time - event->m_start_time) > s_EPSILON)
		{
		    return false;
		}		
		if (current_location_id != event->m_from_loc_id)
		{
		    return false;
		}

		if (event->m_from_loc_id != event->m_to_loc_id)
		{
		    sDouble distance = real_Instance.m_start_conjunction.m_Map->m_straight_Distances[event->m_from_loc_id][event->m_to_loc_id];
		    sDouble travel_time = distance / real_Instance.m_Kruhobots[kruhobot_id].m_properties.m_linear_velo;
		
		    sDouble expected_time = current_time + travel_time;

		    if (sABS(expected_time - event->m_finish_time) > s_EPSILON)
		    {
			return false;
		    }
		}
		current_time = event->m_finish_time;
		current_location_id = event->m_to_loc_id;
	    }
	}

	return true;
    }


    bool sRealSMTCBS::verify_KruhobotCollisionDuplicities(const KruhobotCollision &next_kruhobot_collision, const KruhobotCollisions_mset &kruhobot_Collisions) const
    {
	for (KruhobotCollisions_mset::const_iterator kruhobot_collision = kruhobot_Collisions.begin(); kruhobot_collision != kruhobot_Collisions.end(); ++kruhobot_collision)
	{
	    if (next_kruhobot_collision == *kruhobot_collision)
	    {
		return false;
	    }
	}
	return true;
    }

    
    bool sRealSMTCBS::verify_KruhobotCollisionDuplicities(const KruhobotCollisions_mset next_kruhobot_Collisions, const KruhobotCollisions_mset &kruhobot_Collisions) const
    {
	for (KruhobotCollisions_mset::const_iterator next_kruhobot_collision = next_kruhobot_Collisions.begin(); next_kruhobot_collision != next_kruhobot_Collisions.end(); ++next_kruhobot_collision)
	{
	    for (KruhobotCollisions_mset::const_iterator kruhobot_collision = kruhobot_Collisions.begin(); kruhobot_collision != kruhobot_Collisions.end(); ++kruhobot_collision)
	    {
		if (*next_kruhobot_collision == *kruhobot_collision)
		{
		    return false;
		}
	    }
	}
	return true;
    }    


    

/*----------------------------------------------------------------------------*/

    void sRealSMTCBS::to_Screen(const KruhobotDecisionDiagram_vector &kruhobot_RDD, const sString &indent)
    {
	to_Stream(stdout, kruhobot_RDD, indent);	
    }

    
    void sRealSMTCBS::to_Stream(FILE *fw, const KruhobotDecisionDiagram_vector &kruhobot_RDD, const sString &indent)
    {
	fprintf(fw, "%sKruhobot decision diagram [\n", indent.c_str());	

	for (KruhobotDecisionDiagram_vector::const_iterator kruhobot_decision = kruhobot_RDD.begin(); kruhobot_decision != kruhobot_RDD.end(); ++kruhobot_decision)
	{
	    kruhobot_decision->to_Stream(fw, indent + s_INDENT);
	}	
	fprintf(fw, "%s]\n", indent.c_str());	
    }


    void sRealSMTCBS::to_Screen(const KruhobotDecisionDiagrams_vector &kruhobot_RDDs, const sString &indent)
    {
	to_Stream(stdout, kruhobot_RDDs, indent);	
    }

    
    void sRealSMTCBS::to_Stream(FILE *fw, const KruhobotDecisionDiagrams_vector &kruhobot_RDDs, const sString &indent)
    {
	sInt_32 N_kruhobots_1 = kruhobot_RDDs.size();	
	fprintf(fw, "%sKruhobot decision diagrams {\n", indent.c_str());	

	for (sInt_32 kruhobot_id = 1; kruhobot_id < N_kruhobots_1; ++kruhobot_id)
	{
	    fprintf(fw, "%s%s %d: [\n", indent.c_str(), s_INDENT.c_str(), kruhobot_id);
	    to_Stream(fw, kruhobot_RDDs[kruhobot_id], indent + s2_INDENT);
	    fprintf(fw, "%s%s ]\n", indent.c_str(), s_INDENT.c_str());	    
	}
	fprintf(fw, "%s}\n", indent.c_str());	
    }
    
    
/*----------------------------------------------------------------------------*/

} // namespace boOX


