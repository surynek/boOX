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
/* smtcbsR.cpp / 1-224_leibniz                                                */
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

    sDouble sRealSMTCBS::find_ExactShortestNonconflictingSchedules_individualizedConflictRespectful(sRealSolution &real_Solution, sDouble cost_limit)
    {
	return find_ExactShortestNonconflictingSchedules_individualizedConflictRespectful(*m_real_Instance, real_Solution, cost_limit);	
    }

  
    sDouble sRealSMTCBS::find_ExactShortestNonconflictingSchedules_individualizedConflictRespectful(const sRealInstance &real_Instance,
												    sRealSolution       &sUNUSED(real_Solution),
												    sDouble              makespan_limit)
    {
	sInt_32 cost;
	KruhobotSchedules_vector kruhobot_Schedules;

	if ((cost = find_ExactShortestNonconflictingSchedules_individualizedConflictRespectful(real_Instance, kruhobot_Schedules, makespan_limit)) < 0)
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


    sDouble sRealSMTCBS::find_ExactShortestNonconflictingSchedules_individualizedConflictRespectful(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit)
    {
	return find_ExactShortestNonconflictingSchedules_individualizedConflictRespectful(*m_real_Instance, kruhobot_Schedules, makespan_limit);	
    }


    sDouble sRealSMTCBS::find_ExactShortestNonconflictingSchedules_individualizedConflictRespectful(const sRealInstance      &real_Instance,
												    KruhobotSchedules_vector &kruhobot_Schedules,
												    sDouble                   makespan_limit)
    {
	return find_ExactShortestNonconflictingSchedules_individualizedConflictRespectful(real_Instance, kruhobot_Schedules, makespan_limit, 0.0);
    }    

    
    sDouble sRealSMTCBS::find_ExactShortestNonconflictingSchedules_individualizedConflictRespectful(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit, sDouble extra_makespan)
    {
	return find_ExactShortestNonconflictingSchedules_individualizedConflictRespectful(*m_real_Instance, kruhobot_Schedules, makespan_limit, extra_makespan);	
    }

    
    sDouble sRealSMTCBS::find_ExactShortestNonconflictingSchedules_individualizedConflictRespectful(const sRealInstance      &real_Instance,
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

	    if ((solution_makespan = find_ExactNonconflictingSchedules_individualizedConflictRespectful(real_Instance,
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
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
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
//			makespan_bound =;
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
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
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
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
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
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
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
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
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
	    sRealCBSBase::to_Screen(kruhobot_Schedules);

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
	    to_Screen(kruhobot_RDDs);
		    
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
	    sRealCBSBase::to_Screen(kruhobot_Schedules);

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
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
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
		    printf("mappo: %ld (%.3f)\n", kruhobot_RDD_Mappings[kruhobot_id].size(), kruhobot_makespan_lower_Bounds[kruhobot_id]);
		    
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
		to_Screen(next_kruhobot_RDDs);
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
		    sRealCBSBase::to_Screen(kruhobot_Schedules);
	    
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
		    sRealCBSBase::to_Screen(kruhobot_Schedules);		    

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


    sDouble sRealSMTCBS::find_ExactNonconflictingSchedules_individualizedConflictRespectful(const sRealInstance                    &real_Instance,
											    KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
											    KruhobotUlinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
											    KruhobotSchedules_vector               &kruhobot_Schedules,
											    sDouble                                 makespan_limit,
											    sDouble                                 extra_makespan)
    {
	sInt_32 last_conflict_id = 0;
	
	sDouble makespan_bound = 0.0;
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();

	//bool same_conflict_check = true;

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
	    sRealCBSBase::to_Screen(kruhobot_Schedules);	    
	    real_context.m_makespan_bound = makespan_bound = makespan_lower_bound;
	    
            #ifdef sVERBOSE	    
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		
		printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current makespan: %.3f)...\n",
		       (end_time - start_time),
		       makespan_bound);
	    }
	    #endif

//	    printf("1: ");
	    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	    {
		sDouble nx_makespan_bound;
//		printf("{%.3f} ", kruhobot_next_makespan_Bounds[kruhobot_id]);
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
//		printf("[%.3f] %.3f (%.3f)  ", kruhobot_next_makespan_Bounds[kruhobot_id], nx_makespan_bound, kruhobot_makespan_lower_Bounds[kruhobot_id]);
		    
		if (nx_makespan_bound > 0.0)
		{
		    next_makespan_bound = (next_makespan_bound < 0.0) ? nx_makespan_bound : sMIN(next_makespan_bound, nx_makespan_bound);
		}
	    }
	    to_Screen(kruhobot_RDDs);
	    
//	    printf("\n");
	    sASSERT(next_makespan_bound > 0.0);
	    
            #ifdef sVERBOSE	    
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		
		printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current makespan: %.3f)...\n",
		       (end_time - start_time),
		       makespan_bound);
//		to_Screen(kruhobot_RDDs);		
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
	    sRealCBSBase::to_Screen(kruhobot_Schedules);	    

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
	KruhobotCollisions_mset next_kruhobot_Collisions, save_next_kruhobot_Collisions;
	KruhobotCollisions_mset effective_next_kruhobot_Collisions;

	cumulative_makespan = analyze_NonconflictingSchedules_exactNonprioritized(real_Instance,
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
		    next_collision->to_Screen();
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
	    printf("All collisions 1:\n");
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
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
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
//		    printf("{%.3f} (%ld,%ld) ", kruhobot_next_makespan_Bounds[kruhobot_id], kruhobot_location_Conflicts[kruhobot_id].size(), kruhobot_linear_Conflicts[kruhobot_id].size());		    
		    
		    #ifdef sVERBOSE
		    {
			/*
			printf("Kruho: %d\n", kruhobot_id);
		    
			for (LocationConflicts_upper__umap::const_iterator loc_conflict = kruhobot_location_Conflicts[kruhobot_id].begin(); loc_conflict != kruhobot_location_Conflicts[kruhobot_id].end(); ++loc_conflict)
			{
			    for (LocationConflicts_upper_map::const_iterator lc_conflict = loc_conflict->second.begin(); lc_conflict != loc_conflict->second.end(); ++lc_conflict)
			    {			
				lc_conflict->second.to_Screen();
			    }
			}
			for (LinearConflicts_upper__map::const_iterator loc_conflict = kruhobot_linear_Conflicts[kruhobot_id].begin(); loc_conflict != kruhobot_linear_Conflicts[kruhobot_id].end(); ++loc_conflict)
			{
			    for (LinearConflicts_upper_map::const_iterator lc_conflict = loc_conflict->second.begin(); lc_conflict != loc_conflict->second.end(); ++lc_conflict)
			    {			
				lc_conflict->second.to_Screen();
			    }
			}
			*/
		    }
		    #endif

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
//		    printf("[%.3f] %.3f (%.3f)  ", kruhobot_next_makespan_Bounds[kruhobot_id], nx_makespan_bound, kruhobot_makespan_lower_Bounds[kruhobot_id]);

		    /*
		    printf("mappo: %ld (%.3f)\n", kruhobot_RDD_Mappings[kruhobot_id].size(), kruhobot_makespan_lower_Bounds[kruhobot_id]);
		    
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
		to_Screen(next_kruhobot_RDDs);		

		/*
		if (next_makespan_bound < 0.0)
		{
		    printf("3: ");
		    kruhobot_next_makespan_Bounds = kruhobot_makespan_lower_Bounds;
	
		    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
		    {
		        #ifdef sDEBUG
			{		    
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
			}
		        #endif
		 	
			sDouble nx_makespan_bound;
			printf("{%.3f} ", kruhobot_next_makespan_Bounds[kruhobot_id]);		    
			
			nx_makespan_bound = build_KruhobotRealDecisionDiagram_individualizedConflictRespectfulBucketing(real_Instance.m_Kruhobots[kruhobot_id],
															*real_Instance.m_start_conjunction.m_Map,
															real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
															real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],	    
															kruhobot_location_Conflicts[kruhobot_id],
															kruhobot_linear_Conflicts[kruhobot_id],
															makespan_bound,
															kruhobot_next_makespan_Bounds[kruhobot_id],
															-1,
															next_kruhobot_RDDs[kruhobot_id],
															next_kruhobot_RDD_Mappings[kruhobot_id]);
			printf("[%.3f] %.3f (%.3f)  ", kruhobot_next_makespan_Bounds[kruhobot_id], nx_makespan_bound, kruhobot_makespan_lower_Bounds[kruhobot_id]);

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
		    }
		    printf("\n");
		    getchar();
		}
		*/
		    
		#ifdef sVERBOSE
		{
		    /*
		    printf("New real decisiondiagram constructed A:\n");
		    to_Screen(next_kruhobot_RDDs);		    
		    */
		}
		#endif
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
		    sRealCBSBase::to_Screen(kruhobot_Schedules);

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
				    //			    same_conflict_check = false;
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
		    sRealCBSBase::to_Screen(kruhobot_Schedules);		    

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
				    //			    same_conflict_check = false;				    
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

//		sRealCBSBase::to_Screen(kruhobot_Schedules);

		cumulative_makespan = analyze_NonconflictingSchedules_exactNonprioritized(real_Instance,
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
		if (!save_next_kruhobot_Collisions.empty() && same_conflict_check)
		{
		    if (save_next_kruhobot_Collisions.size() == next_kruhobot_Collisions.size())
		    {
			bool same_collisions = true;
			
			KruhobotCollisions_mset::const_iterator save_next_collision = save_next_kruhobot_Collisions.begin();
			for (KruhobotCollisions_mset::const_iterator next_collision = next_kruhobot_Collisions.begin(); next_collision != next_kruhobot_Collisions.end(); ++next_collision)
			{
			    if (*next_collision == *save_next_collision)
			    {
				// nothing
			    }
			    else
			    {
				same_collisions = false;
				break;
			    }
			    ++save_next_collision;
			}
			same_conflict_check = true;
			
			if (same_collisions)
			{
			    printf("COLLISION-FREE solution found [*] !\n");

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
		same_conflict_check = true;
		save_next_kruhobot_Collisions = next_kruhobot_Collisions;
		*/
		/*
  	        #ifdef sDEBUG
		{
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
		    getchar();
		}
	        #endif
		*/

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
			    next_collision->to_Screen();
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

    
/*----------------------------------------------------------------------------*/
    
    sDouble sRealSMTCBS::build_KruhobotRealDecisionDiagram(const sKruhobot                &kruhobot,
							   const s2DMap                   &map,
							   sInt_32                         source_loc_id,
							   sInt_32                         sink_loc_id,
							   const LocationConflicts__umap  &location_Conflicts,
							   const UlinearConflicts__map     &linear_Conflicts,
							   sDouble                         makespan_bound,
							   KruhobotDecisionDiagram_vector &kruhobot_RDD,
							   KruhobotDecisionMapping_map    &kruhobot_RDD_mapping) const
    {
	sDouble next_makespan_bound = -1.0;
	
	Transitions_mmap transition_Queue;
	TransitionExplorations_map explored_Transitions;
	Explorations_umap bound_explored_Transitions;	
	Transitions_vector transition_Store;

	Transition initial_transition(transition_Store.size(), 0.0, 0.0, 0.0, source_loc_id, -1);	
	transition_Queue.insert(Transitions_mmap::value_type(0.0, initial_transition));
	transition_Store.push_back(initial_transition);
	
	kruhobot_RDD.push_back(KruhobotDecision(initial_transition.m_trans_id,
						initial_transition.m_time,
						initial_transition.m_location_id,
						initial_transition.m_prev_trans_id));
	kruhobot_RDD_mapping[initial_transition.m_location_id].insert(KruhobotDecisionIDs_mmap::value_type(initial_transition.m_time, initial_transition.m_trans_id));
	    
	if (0.0 <= makespan_bound)
	{
	    Explorations_umap initial_explorations;
	    initial_explorations.insert(Explorations_umap::value_type(initial_transition.m_location_id, initial_transition.m_trans_id));
	    
	    explored_Transitions.insert(TransitionExplorations_map::value_type(0.0, initial_explorations));
	}
	else
	{
	    bound_explored_Transitions.insert(Explorations_umap::value_type(initial_transition.m_location_id, initial_transition.m_trans_id));
	}
//	explored_Transitions[0.0].insert(source_loc_id);

	while (!transition_Queue.empty())
	{
	    const Transition &front_transition = transition_Queue.begin()->second;
	    
	    if (next_makespan_bound < 0.0)
	    {
		if (front_transition.m_time > makespan_bound + s_EPSILON)
		{
		    next_makespan_bound = front_transition.m_time;
		}
	    }
	    else
	    {
		if (front_transition.m_time > makespan_bound + s_EPSILON && front_transition.m_time + s_EPSILON < next_makespan_bound)
		{
		    next_makespan_bound = front_transition.m_time;
		}
	    }
	    
	    /*  
	    if (front_transition.m_location_id == sink_loc_id)
	    {
		bool sink_free_to_enter = true;
		
		LocationConflicts__umap::const_iterator sink_Conflict = location_Conflicts.find(sink_loc_id);

		if (sink_Conflict != location_Conflicts.end())
		{
		    LocationConflicts_map::const_iterator lower_sink_conflict = sink_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				
		    if (lower_sink_conflict != sink_Conflict->second.end())
		    {
			sink_free_to_enter = false;
		    }			    
		}

		if (sink_free_to_enter)
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
	    }
	    */
	    if (front_transition.m_time <= makespan_bound)
	    {
		TransitionExplorations_map::const_iterator explored_transition = explored_Transitions.find(front_transition.m_time);

		if (explored_transition == explored_Transitions.end())
		{
		    explored_Transitions.insert(TransitionExplorations_map::value_type(front_transition.m_time, Explorations_umap()));
		}
	    }

	    if (front_transition.m_time + (map.m_straight_Distances[sink_loc_id][front_transition.m_location_id] / kruhobot.m_properties.m_linear_velo) <= makespan_bound)
	    {
		for (s2DMap::Locations_vector::const_iterator location = map.m_Locations.begin(); location != map.m_Locations.end(); ++location)
		{
		    sInt_32 neighbor_location_id = location->m_id;

		    if (neighbor_location_id != front_transition.m_location_id && map.m_Network.is_Adjacent(neighbor_location_id, front_transition.m_location_id))
		    {
			sDouble transition_distance = map.m_straight_Distances[front_transition.m_location_id][neighbor_location_id];
			sDouble transition_delta_time = transition_distance / kruhobot.m_properties.m_linear_velo;
			sDouble transition_finish_time = front_transition.m_time + transition_delta_time;
			sDouble transition_finish_cost = front_transition.m_cost + transition_delta_time;
			sDouble transition_finish_makespan = front_transition.m_makespan + transition_delta_time;

			Explorations_umap *next_explored_Transitions;
			
//			if (front_transition.m_time <= makespan_bound)
			if (transition_finish_time <= makespan_bound)			
			{
			    TransitionExplorations_map::const_iterator explored_transition = explored_Transitions.find(transition_finish_time);			
			    if (explored_transition == explored_Transitions.end())
			    {
				explored_Transitions.insert(TransitionExplorations_map::value_type(transition_finish_time, Explorations_umap()));
			    }
			    next_explored_Transitions = &explored_Transitions[transition_finish_time];
			}
			else
			{
			    next_explored_Transitions = &bound_explored_Transitions;
			}
			Explorations_umap::const_iterator next_explored_transition = next_explored_Transitions->find(neighbor_location_id);
	
			if (next_explored_transition == next_explored_Transitions->end())
			{
			    Transition neighbor_transition(transition_Store.size(), transition_finish_time, transition_finish_cost, transition_finish_makespan, neighbor_location_id, front_transition.m_trans_id);
			    transition_Store.push_back(neighbor_transition);
			    
			    kruhobot_RDD.push_back(KruhobotDecision(neighbor_transition.m_trans_id,
								    neighbor_transition.m_time,
								    neighbor_transition.m_location_id,
								    neighbor_transition.m_prev_trans_id));
			    kruhobot_RDD_mapping[neighbor_transition.m_location_id].insert(KruhobotDecisionIDs_mmap::value_type(neighbor_transition.m_time, neighbor_transition.m_trans_id));
			    
			    next_explored_Transitions->insert(Explorations_umap::value_type(neighbor_location_id, front_transition.m_trans_id));
//			    transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_cost, neighbor_transition));
			    transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_makespan, neighbor_transition));			    
			}
			/*
			else
			{
			    kruhobot_RDD[next_explored_transition->second].m_prev_dec_IDs.insert(front_transition.m_trans_id);
			}
			*/
		    }
		}
		{
		    sDouble wait_location_finish_time = -1.0;
		    sDouble wait_linear_finish_time = -1.0;

		    for (s2DMap::Locations_vector::const_iterator interacting_loc = map.m_Locations.begin(); interacting_loc != map.m_Locations.end(); ++interacting_loc)
		    {
			sInt_32 neighbor_location_id = interacting_loc->m_id;
			
			if (neighbor_location_id == front_transition.m_location_id || map.m_Network.is_Adjacent(neighbor_location_id, front_transition.m_location_id))
			{
			    /*
			    sDouble transition_distance = map.m_Distances[front_transition.m_location_id][neighbor_location_id];
			    sDouble transition_delta_time = transition_distance / kruhobot.m_properties.m_linear_velo;
			    sDouble transition_finish_time = front_transition.m_time + transition_delta_time;
			    */

			    LocationConflicts__umap::const_iterator location_Conflict = location_Conflicts.find(neighbor_location_id);
			    sDouble first_non_conf_location_time = -1.0;
			    
			    if (location_Conflict != location_Conflicts.end())
			    {
				/*
				LocationConflicts_map::const_iterator lower_location_conflict = location_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));

				if (lower_location_conflict != location_Conflict->second.end())
				{
				    first_non_conf_location_time = lower_location_conflict->second.m_interval.m_upper;
				}
				*/

				LocationConflicts_map::const_iterator lower_location_conflict = location_Conflict->second.end();

				for (LocationConflicts_map::const_iterator lower2_location_conflict = location_Conflict->second.begin(); lower2_location_conflict != location_Conflict->second.end(); ++lower2_location_conflict)
				{
				    if (lower2_location_conflict->second.m_interval.m_upper > front_transition.m_time)
				    {
					if (lower_location_conflict != location_Conflict->second.end())
					{
					    if (lower2_location_conflict->second.m_interval.m_upper < lower_location_conflict->second.m_interval.m_upper)
					    {
						lower_location_conflict = lower2_location_conflict;
						first_non_conf_location_time = lower_location_conflict->second.m_interval.m_upper;
					    }
					}
					else
					{
					    lower_location_conflict = lower2_location_conflict;
					    first_non_conf_location_time = lower_location_conflict->second.m_interval.m_upper;					
					}
				    }
				}
			    }
			    
			    if (first_non_conf_location_time >= 0.0)
			    {
				if (wait_location_finish_time < 0.0)
				{
				    wait_location_finish_time = first_non_conf_location_time;
				}			    
				else
				{
				    if (wait_location_finish_time > first_non_conf_location_time)
				    {
					wait_location_finish_time = first_non_conf_location_time;
				    }
				}
			    }
			    
			    UlinearConflicts__map::const_iterator linear_Conflict = linear_Conflicts.find(Uline(front_transition.m_location_id, neighbor_location_id));
			    sDouble first_non_conf_linear_time = -1.0;
			    
			    if (linear_Conflict != linear_Conflicts.end())
			    {
				/*
				UlinearConflicts_map::const_iterator lower_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				if (lower_linear_conflict != linear_Conflict->second.end())
				{
				    first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
				}

				for (UlinearConflicts_map::const_iterator lower2_linear_conflict = lower_linear_conflict; lower2_linear_conflict != linear_Conflict->second.end(); ++lower2_linear_conflict)
				{
				    if (lower2_linear_conflict->second.m_interval.m_upper < lower_linear_conflict->second.m_interval.m_upper)
				    {
					lower_linear_conflict = lower2_linear_conflict;
					first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
				    }
				}
				*/
				UlinearConflicts_map::const_iterator lower_linear_conflict = linear_Conflict->second.end();

				for (UlinearConflicts_map::const_iterator lower2_linear_conflict = linear_Conflict->second.begin(); lower2_linear_conflict != linear_Conflict->second.end(); ++lower2_linear_conflict)
				{
				    if (lower2_linear_conflict->second.m_interval.m_upper > front_transition.m_time)
				    {
					if (lower_linear_conflict != linear_Conflict->second.end())
					{
					    if (lower2_linear_conflict->second.m_interval.m_upper < lower_linear_conflict->second.m_interval.m_upper)
					    {
						lower_linear_conflict = lower2_linear_conflict;
						first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
					    }
					}
					else
					{
					    lower_linear_conflict = lower2_linear_conflict;
					    first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;					
					}
				    }
				}				
			    }

			    if (first_non_conf_linear_time >= 0.0)
			    {
				if (wait_linear_finish_time < 0.0)
				{
				    wait_linear_finish_time = first_non_conf_linear_time;
				}			    
				else
				{
				    if (wait_linear_finish_time > first_non_conf_linear_time)
				    {
					wait_linear_finish_time = first_non_conf_linear_time;
				    }
				}
			    }
			}
		    }
		    sDouble wait_finish_time;
		    
		    if (wait_location_finish_time >= 0.0)
		    {
			if (wait_linear_finish_time >= 0.0)
			{
			    wait_finish_time = sMIN(wait_location_finish_time, wait_linear_finish_time);
			}
			else
			{
			    wait_finish_time = wait_location_finish_time;
			}
		    }
		    else
		    {
			if (wait_linear_finish_time >= 0.0)
			{
			    wait_finish_time = wait_linear_finish_time;
			}
			else
			{
			    wait_finish_time = -1.0;
			}
		    }
		    
		    if (wait_finish_time > front_transition.m_time + s_EPSILON)
		    {
			Explorations_umap *wait_explored_Transitions;

			if (wait_finish_time <= makespan_bound)
			{
			    TransitionExplorations_map::const_iterator explored_transition = explored_Transitions.find(wait_finish_time);
			    
			    if (explored_transition == explored_Transitions.end())
			    {
				explored_Transitions.insert(TransitionExplorations_map::value_type(wait_finish_time, Explorations_umap()));
			    }
			    wait_explored_Transitions = &explored_Transitions[wait_finish_time];
			}
			else
			{
			    wait_explored_Transitions = &bound_explored_Transitions;				
			}
			if (wait_explored_Transitions->find(front_transition.m_location_id) == wait_explored_Transitions->end())
			{
			    sDouble wait_cost = (wait_finish_time - front_transition.m_time) * kruhobot.m_properties.m_wait_factor;
			    Transition wait_transition(transition_Store.size(), wait_finish_time, front_transition.m_cost + wait_cost, wait_finish_time, front_transition.m_location_id, front_transition.m_trans_id);
			    transition_Store.push_back(wait_transition);
			    
			    kruhobot_RDD.push_back(KruhobotDecision(wait_transition.m_trans_id,
								    wait_transition.m_time,
								    wait_transition.m_location_id,
								    wait_transition.m_prev_trans_id));
			    kruhobot_RDD_mapping[wait_transition.m_location_id].insert(KruhobotDecisionIDs_mmap::value_type(wait_transition.m_time, wait_transition.m_trans_id));
			    
			    wait_explored_Transitions->insert(Explorations_umap::value_type(front_transition.m_location_id, front_transition.m_trans_id));		
//			    transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_cost, wait_transition));
			    transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_makespan, wait_transition));
			}
		    }
		}				
	    }
	    transition_Queue.erase(transition_Queue.begin());
	}
	augment_KruhobotRealDecisionDiagram(kruhobot, source_loc_id, sink_loc_id, makespan_bound, kruhobot_RDD, kruhobot_RDD_mapping);	
	interconnect_KruhobotRealDecisionDiagram(kruhobot, map, kruhobot_RDD, kruhobot_RDD_mapping);

	return next_makespan_bound;
    }


    #define sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(time)                                    \
    {                                                                                         \
	if (next_makespan_bound < 0.0)                                                        \
	{                                                                                     \
	    if (time > makespan_bound + s_EPSILON)                                            \
	    {                                                                                 \
		next_makespan_bound = time;                                                   \
	    }                                                                                 \
	}                                                                                     \
	else                                                                                  \
	{                                                                                     \
	    if (time > makespan_bound + s_EPSILON && time + s_EPSILON < next_makespan_bound)  \
	    {                                                                                 \
		next_makespan_bound = time;                                                   \
	    }                                                                                 \
	}                                                                                     \
    }


    #define sREAL_SMT_CBS_UPDATE_NEXT_INDIVIDUAL_MAKESPAN_BOUND(time)                         \
    {                                                                                         \
	if (time >= individual_makespan_bound + s_EPSILON)                                    \
	{					                                              \
	    if (next_individual_makespan_bound >= individual_makespan_bound + s_EPSILON)      \
	    {                                                                                 \
		if (next_individual_makespan_bound > time)                                    \
		{                                                                             \
		    next_individual_makespan_bound = time;                                    \
		}                                                                             \
	    }                                                                                 \
	    else                                                                              \
	    {                                                                                 \
		next_individual_makespan_bound = time;                                        \
	    }                                                                                 \
	}                                                                                     \
    }
    
    
    sDouble sRealSMTCBS::build_KruhobotRealDecisionDiagram(const sKruhobot                      &kruhobot,
							   const s2DMap                         &map,
							   sInt_32                               source_loc_id,
							   sInt_32                               sink_loc_id,
							   const LocationConflicts_upper__umap  &location_Conflicts,
							   const UlinearConflicts_upper__map     &linear_Conflicts,
							   sDouble                               makespan_bound,
							   KruhobotDecisionDiagram_vector       &kruhobot_RDD,
							   KruhobotDecisionMapping_map          &kruhobot_RDD_mapping) const
    {
	sDouble next_makespan_bound = -1.0;
	
	Transitions_mmap transition_Queue;
	TransitionExplorations_map explored_Transitions;
	Explorations_umap bound_explored_Transitions;	
	Transitions_vector transition_Store;

	Transition initial_transition(transition_Store.size(), 0.0, 0.0, 0.0, source_loc_id, -1);
	initial_transition.m_corr_dec_id = 0;
	transition_Queue.insert(Transitions_mmap::value_type(0.0, initial_transition));
	transition_Store.push_back(initial_transition);
	
	kruhobot_RDD.push_back(KruhobotDecision(kruhobot_RDD.size(),
						initial_transition.m_time,
						initial_transition.m_location_id,
						-1));
	sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(initial_transition.m_time);
	kruhobot_RDD_mapping[initial_transition.m_location_id].insert(KruhobotDecisionIDs_mmap::value_type(initial_transition.m_time, initial_transition.m_trans_id));
	    
	{
	    Explorations_umap initial_explorations;
	    initial_explorations.insert(Explorations_umap::value_type(initial_transition.m_location_id, initial_transition.m_trans_id));
	    
	    explored_Transitions.insert(TransitionExplorations_map::value_type(0.0, initial_explorations));
	}

	while (!transition_Queue.empty())
	{
	    const Transition &front_transition = transition_Queue.begin()->second;	    
	    TransitionExplorations_map::const_iterator explored_transition = explored_Transitions.find(front_transition.m_time);
	    
	    if (explored_transition == explored_Transitions.end())
	    {
		explored_Transitions.insert(TransitionExplorations_map::value_type(front_transition.m_time, Explorations_umap()));
	    }

	    if (front_transition.m_time + (map.m_straight_Distances[sink_loc_id][front_transition.m_location_id] / kruhobot.m_properties.m_linear_velo) <= makespan_bound)
	    {
		for (s2DMap::Locations_vector::const_iterator location = map.m_Locations.begin(); location != map.m_Locations.end(); ++location)
		{
		    sInt_32 neighbor_location_id = location->m_id;

		    if (neighbor_location_id != front_transition.m_location_id && map.m_Network.is_Adjacent(neighbor_location_id, front_transition.m_location_id))
		    {
			sDouble transition_distance = map.m_straight_Distances[front_transition.m_location_id][neighbor_location_id];
			sDouble transition_delta_time = transition_distance / kruhobot.m_properties.m_linear_velo;
			sDouble transition_finish_time = front_transition.m_time + transition_delta_time;
			sDouble transition_finish_cost = front_transition.m_cost + transition_delta_time;
			sDouble transition_finish_makespan = front_transition.m_makespan + transition_delta_time;			

			Explorations_umap *next_explored_Transitions;
			
			TransitionExplorations_map::const_iterator explored_transition = explored_Transitions.find(transition_finish_time);			
			if (explored_transition == explored_Transitions.end())
			{
			    explored_Transitions.insert(TransitionExplorations_map::value_type(transition_finish_time, Explorations_umap()));
			}
			next_explored_Transitions = &explored_Transitions[transition_finish_time];
			
			Explorations_umap::const_iterator next_explored_transition = next_explored_Transitions->find(neighbor_location_id);
	
			if (next_explored_transition == next_explored_Transitions->end())
			{
			    Transition neighbor_transition(transition_Store.size(), transition_finish_time, transition_finish_cost, transition_finish_makespan, neighbor_location_id, front_transition.m_trans_id);		    
			    sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(neighbor_transition.m_time);			   

			    next_explored_Transitions->insert(Explorations_umap::value_type(neighbor_location_id, front_transition.m_trans_id));
			    
			    if (neighbor_transition.m_time <= makespan_bound + s_EPSILON)
			    {
				neighbor_transition.m_corr_dec_id = kruhobot_RDD.size();
				
				kruhobot_RDD.push_back(KruhobotDecision(kruhobot_RDD.size(),
									neighbor_transition.m_time,
									neighbor_transition.m_location_id,								    
									front_transition.m_corr_dec_id));
				kruhobot_RDD_mapping[neighbor_transition.m_location_id].insert(KruhobotDecisionIDs_mmap::value_type(neighbor_transition.m_time, neighbor_transition.m_corr_dec_id));
//				transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_cost, neighbor_transition));
				transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_makespan, neighbor_transition));								
			    }
			    else
			    {
				neighbor_transition.m_corr_dec_id = -1;				
			    }
			    transition_Store.push_back(neighbor_transition);			    
			}
		    }
		}
		{
		    sDouble wait_location_finish_time = -1.0;
		    sDouble wait_linear_finish_time = -1.0;

		    for (s2DMap::Locations_vector::const_iterator interacting_loc = map.m_Locations.begin(); interacting_loc != map.m_Locations.end(); ++interacting_loc)
		    {
			sInt_32 neighbor_location_id = interacting_loc->m_id;
			
			if (neighbor_location_id == front_transition.m_location_id || map.m_Network.is_Adjacent(neighbor_location_id, front_transition.m_location_id))
			{
			    LocationConflicts_upper__umap::const_iterator location_Conflict = location_Conflicts.find(neighbor_location_id);
			    sDouble first_non_conf_location_time = -1.0;
			    
			    if (location_Conflict != location_Conflicts.end())
			    {
				LocationConflicts_upper_map::const_iterator lower_location_conflict = location_Conflict->second.end();

				LocationConflicts_upper_map::const_iterator lower2_location_conflict = location_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				if (lower2_location_conflict != location_Conflict->second.end())
				{
				    for (LocationConflicts_map::const_iterator lower2_location_conflict = location_Conflict->second.begin(); lower2_location_conflict != location_Conflict->second.end(); ++lower2_location_conflict)
				    {
					if (lower2_location_conflict->second.m_interval.m_upper > front_transition.m_time)
					{
					    lower_location_conflict = lower2_location_conflict;
					    first_non_conf_location_time = lower_location_conflict->second.m_interval.m_upper;
					    break;
					}
				    }
				}
			    }
			    
			    if (first_non_conf_location_time >= 0.0)
			    {
				if (wait_location_finish_time < 0.0)
				{
				    wait_location_finish_time = first_non_conf_location_time;
				}
				else
				{
				    if (wait_location_finish_time > first_non_conf_location_time)
				    {
					wait_location_finish_time = first_non_conf_location_time;
				    }
				}
			    }
			    
			    UlinearConflicts_upper__map::const_iterator linear_Conflict = linear_Conflicts.find(Uline(front_transition.m_location_id, neighbor_location_id));
			    sDouble first_non_conf_linear_time = -1.0;
			    
			    if (linear_Conflict != linear_Conflicts.end())
			    {
				UlinearConflicts_upper_map::const_iterator lower_linear_conflict = linear_Conflict->second.end();

				UlinearConflicts_upper_map::const_iterator lower2_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				if (lower2_linear_conflict != linear_Conflict->second.end())
				{
				    for (UlinearConflicts_map::const_iterator lower2_linear_conflict = linear_Conflict->second.begin(); lower2_linear_conflict != linear_Conflict->second.end(); ++lower2_linear_conflict)
				    {
					if (lower2_linear_conflict->second.m_interval.m_upper > front_transition.m_time)
					{
					    lower_linear_conflict = lower2_linear_conflict;
					    first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
					    break;
					}
				    }
				}				
			    }

			    if (first_non_conf_linear_time >= 0.0)
			    {
				if (wait_linear_finish_time < 0.0)
				{
				    wait_linear_finish_time = first_non_conf_linear_time;
				}			    
				else
				{
				    if (wait_linear_finish_time > first_non_conf_linear_time)
				    {
					wait_linear_finish_time = first_non_conf_linear_time;
				    }
				}
			    }
			}
		    }
		    sDouble wait_finish_time;
		    
		    if (wait_location_finish_time >= 0.0)
		    {
			if (wait_linear_finish_time >= 0.0)
			{
			    wait_finish_time = sMIN(wait_location_finish_time, wait_linear_finish_time);
			}
			else
			{
			    wait_finish_time = wait_location_finish_time;
			}
		    }
		    else
		    {
			if (wait_linear_finish_time >= 0.0)
			{
			    wait_finish_time = wait_linear_finish_time;
			}
			else
			{
			    wait_finish_time = -1.0;
			}
		    }
		    
		    if (wait_finish_time > front_transition.m_time + s_EPSILON)
		    {
			Explorations_umap *wait_explored_Transitions;

//			if (wait_finish_time <= makespan_bound)
			{
			    TransitionExplorations_map::const_iterator explored_transition = explored_Transitions.find(wait_finish_time);
			    
			    if (explored_transition == explored_Transitions.end())
			    {
				explored_Transitions.insert(TransitionExplorations_map::value_type(wait_finish_time, Explorations_umap()));
			    }
			    wait_explored_Transitions = &explored_Transitions[wait_finish_time];
			}
			/*
			else
			{
			    wait_explored_Transitions = &bound_explored_Transitions;				
			}
			*/
			if (wait_explored_Transitions->find(front_transition.m_location_id) == wait_explored_Transitions->end())
			{
			    sDouble wait_cost = (wait_finish_time - front_transition.m_time) * kruhobot.m_properties.m_wait_factor;
			    Transition wait_transition(transition_Store.size(), wait_finish_time, front_transition.m_cost + wait_cost, wait_finish_time, front_transition.m_location_id, front_transition.m_trans_id);
			    sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(wait_transition.m_time);			    
			    
			    wait_explored_Transitions->insert(Explorations_umap::value_type(front_transition.m_location_id, front_transition.m_trans_id));

 			    if (wait_transition.m_time <= makespan_bound + s_EPSILON)
			    {
				wait_transition.m_corr_dec_id = kruhobot_RDD.size();
				
				kruhobot_RDD.push_back(KruhobotDecision(kruhobot_RDD.size(),
									wait_transition.m_time,
									wait_transition.m_location_id,
									front_transition.m_corr_dec_id));
				kruhobot_RDD_mapping[wait_transition.m_location_id].insert(KruhobotDecisionIDs_mmap::value_type(wait_transition.m_time, wait_transition.m_corr_dec_id));
//				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_cost, wait_transition));
				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_makespan, wait_transition));
			    }
			    else
			    {
				wait_transition.m_corr_dec_id = -1;
			    }
			    transition_Store.push_back(wait_transition);
			}
		    }
		}				
	    }
	    transition_Queue.erase(transition_Queue.begin());
	}
	augment_KruhobotRealDecisionDiagram(kruhobot, source_loc_id, sink_loc_id, makespan_bound, kruhobot_RDD, kruhobot_RDD_mapping);	
	interconnect_KruhobotRealDecisionDiagram(kruhobot, map, kruhobot_RDD, kruhobot_RDD_mapping);

	return next_makespan_bound;
    }


    sDouble sRealSMTCBS::build_KruhobotRealDecisionDiagram_pruning(const sKruhobot                      &kruhobot,
								   const s2DMap                         &map,
								   sInt_32                               source_loc_id,
								   sInt_32                               sink_loc_id,
								   const LocationConflicts_upper__umap  &location_Conflicts,
								   const UlinearConflicts_upper__map     &linear_Conflicts,
								   sDouble                               makespan_bound,
								   KruhobotDecisionDiagram_vector       &kruhobot_RDD,
								   KruhobotDecisionMapping_map          &kruhobot_RDD_mapping) const
    {	
	sDouble next_makespan_bound = -1.0;
	
	Transitions_mmap transition_Queue;
	TransitionExplorations_map explored_Transitions;
	Explorations_umap bound_explored_Transitions;	
	Transitions_vector transition_Store;

	Transition initial_transition(transition_Store.size(), 0.0, 0.0, 0.0, source_loc_id, -1);
	initial_transition.m_corr_dec_id = 0;
	transition_Queue.insert(Transitions_mmap::value_type(0.0, initial_transition));
	transition_Store.push_back(initial_transition);
	
	kruhobot_RDD.push_back(KruhobotDecision(kruhobot_RDD.size(),
						initial_transition.m_time,
						initial_transition.m_location_id,
						-1));
	sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(initial_transition.m_time);
	kruhobot_RDD_mapping[initial_transition.m_location_id].insert(KruhobotDecisionIDs_mmap::value_type(initial_transition.m_time, initial_transition.m_trans_id));
	    
	{
	    Explorations_umap initial_explorations;
	    initial_explorations.insert(Explorations_umap::value_type(initial_transition.m_location_id, initial_transition.m_trans_id));
	    
	    explored_Transitions.insert(TransitionExplorations_map::value_type(0.0, initial_explorations));
	}

	while (!transition_Queue.empty())
	{
	    const Transition &front_transition = transition_Queue.begin()->second;	    
	    TransitionExplorations_map::const_iterator explored_transition = explored_Transitions.find(front_transition.m_time);
	    
	    if (explored_transition == explored_Transitions.end())
	    {
		explored_Transitions.insert(TransitionExplorations_map::value_type(front_transition.m_time, Explorations_umap()));
	    }

	    if (front_transition.m_time + (map.m_straight_Distances[sink_loc_id][front_transition.m_location_id] / kruhobot.m_properties.m_linear_velo) <= makespan_bound)
	    {		
		const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[front_transition.m_location_id].m_Neighbors;
		for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
		{
		    sInt_32 neighbor_location_id = (*neighbor)->m_target->m_id;
		    
		    {
			sDouble transition_distance = map.m_straight_Distances[front_transition.m_location_id][neighbor_location_id];
			sDouble transition_delta_time = transition_distance / kruhobot.m_properties.m_linear_velo;
			sDouble transition_finish_time = front_transition.m_time + transition_delta_time;
			sDouble transition_finish_cost = front_transition.m_cost + transition_delta_time;
			sDouble transition_finish_makespan = front_transition.m_makespan + transition_delta_time;			

			Explorations_umap *next_explored_Transitions;
			
			TransitionExplorations_map::const_iterator explored_transition = explored_Transitions.find(transition_finish_time);			
			if (explored_transition == explored_Transitions.end())
			{
			    explored_Transitions.insert(TransitionExplorations_map::value_type(transition_finish_time, Explorations_umap()));
			}
			next_explored_Transitions = &explored_Transitions[transition_finish_time];
			
			Explorations_umap::const_iterator next_explored_transition = next_explored_Transitions->find(neighbor_location_id);
	
			if (next_explored_transition == next_explored_Transitions->end())
			{
			    Transition neighbor_transition(transition_Store.size(), transition_finish_time, transition_finish_cost, transition_finish_makespan, neighbor_location_id, front_transition.m_trans_id);		    
			    sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(neighbor_transition.m_time);			   

			    next_explored_Transitions->insert(Explorations_umap::value_type(neighbor_location_id, front_transition.m_trans_id));
			    sDouble estimated_remaining =  map.m_straight_Distances[sink_loc_id][neighbor_location_id] / kruhobot.m_properties.m_linear_velo;
			    
			    if (neighbor_transition.m_time + estimated_remaining <= makespan_bound + s_EPSILON)
			    {				
				neighbor_transition.m_corr_dec_id = kruhobot_RDD.size();
				
				kruhobot_RDD.push_back(KruhobotDecision(kruhobot_RDD.size(),
									neighbor_transition.m_time,
									neighbor_transition.m_location_id,
									front_transition.m_corr_dec_id));
				kruhobot_RDD_mapping[neighbor_transition.m_location_id].insert(KruhobotDecisionIDs_mmap::value_type(neighbor_transition.m_time, neighbor_transition.m_corr_dec_id));
//				transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_cost, neighbor_transition));
				transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_makespan, neighbor_transition));				
			    }
			    else
			    {
				neighbor_transition.m_corr_dec_id = -1;				
			    }
			    transition_Store.push_back(neighbor_transition);			    
			}
		    }
		}
		{
		    sDouble wait_location_finish_time = -1.0;
		    sDouble wait_linear_finish_time = -1.0;

		    const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[front_transition.m_location_id].m_Neighbors;
		    for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
		    {
			sInt_32 neighbor_location_id = (*neighbor)->m_target->m_id;
			{
			    LocationConflicts_upper__umap::const_iterator location_Conflict = location_Conflicts.find(neighbor_location_id);
			    sDouble first_non_conf_location_time = -1.0;
			    
			    if (location_Conflict != location_Conflicts.end())
			    {
				LocationConflicts_upper_map::const_iterator lower_location_conflict = location_Conflict->second.end();

				LocationConflicts_upper_map::const_iterator lower2_location_conflict = location_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				if (lower2_location_conflict != location_Conflict->second.end())
				{
				    for (LocationConflicts_map::const_iterator lower2_location_conflict = location_Conflict->second.begin(); lower2_location_conflict != location_Conflict->second.end(); ++lower2_location_conflict)
				    {
					if (lower2_location_conflict->second.m_interval.m_upper > front_transition.m_time)
					{
					    lower_location_conflict = lower2_location_conflict;
					    first_non_conf_location_time = lower_location_conflict->second.m_interval.m_upper;
					    break;
					}
				    }
				}
			    }
			    
			    if (first_non_conf_location_time >= 0.0)
			    {
				if (wait_location_finish_time < 0.0)
				{
				    wait_location_finish_time = first_non_conf_location_time;
				}			    
				else
				{
				    if (wait_location_finish_time > first_non_conf_location_time)
				    {
					wait_location_finish_time = first_non_conf_location_time;
				    }
				}
			    }
			    
			    UlinearConflicts_upper__map::const_iterator linear_Conflict = linear_Conflicts.find(Uline(front_transition.m_location_id, neighbor_location_id));
			    sDouble first_non_conf_linear_time = -1.0;
			    
			    if (linear_Conflict != linear_Conflicts.end())
			    {
				UlinearConflicts_upper_map::const_iterator lower_linear_conflict = linear_Conflict->second.end();

				UlinearConflicts_upper_map::const_iterator lower2_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				if (lower2_linear_conflict != linear_Conflict->second.end())
				{
				    for (UlinearConflicts_map::const_iterator lower2_linear_conflict = linear_Conflict->second.begin(); lower2_linear_conflict != linear_Conflict->second.end(); ++lower2_linear_conflict)
				    {
					if (lower2_linear_conflict->second.m_interval.m_upper > front_transition.m_time)
					{
					    lower_linear_conflict = lower2_linear_conflict;
					    first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
					    break;
					}
				    }
				}				
			    }

			    if (first_non_conf_linear_time >= 0.0)
			    {
				if (wait_linear_finish_time < 0.0)
				{
				    wait_linear_finish_time = first_non_conf_linear_time;
				}			    
				else
				{
				    if (wait_linear_finish_time > first_non_conf_linear_time)
				    {
					wait_linear_finish_time = first_non_conf_linear_time;
				    }
				}
			    }
			}
		    }
		    {
			sInt_32 neighbor_location_id = front_transition.m_location_id;
			
			{
			    LocationConflicts_upper__umap::const_iterator location_Conflict = location_Conflicts.find(neighbor_location_id);
			    sDouble first_non_conf_location_time = -1.0;
			    
			    if (location_Conflict != location_Conflicts.end())
			    {
				LocationConflicts_upper_map::const_iterator lower_location_conflict = location_Conflict->second.end();

				LocationConflicts_upper_map::const_iterator lower2_location_conflict = location_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				if (lower2_location_conflict != location_Conflict->second.end())
				{
				    for (LocationConflicts_map::const_iterator lower2_location_conflict = location_Conflict->second.begin(); lower2_location_conflict != location_Conflict->second.end(); ++lower2_location_conflict)
				    {
					if (lower2_location_conflict->second.m_interval.m_upper > front_transition.m_time)
					{
					    lower_location_conflict = lower2_location_conflict;
					    first_non_conf_location_time = lower_location_conflict->second.m_interval.m_upper;
					    break;
					}
				    }
				}
			    }
			    
			    if (first_non_conf_location_time >= 0.0)
			    {
				if (wait_location_finish_time < 0.0)
				{
				    wait_location_finish_time = first_non_conf_location_time;
				}			    
				else
				{
				    if (wait_location_finish_time > first_non_conf_location_time)
				    {
					wait_location_finish_time = first_non_conf_location_time;
				    }
				}
			    }
			    
			    UlinearConflicts_upper__map::const_iterator linear_Conflict = linear_Conflicts.find(Uline(front_transition.m_location_id, neighbor_location_id));
			    sDouble first_non_conf_linear_time = -1.0;
			    
			    if (linear_Conflict != linear_Conflicts.end())
			    {
				UlinearConflicts_upper_map::const_iterator lower_linear_conflict = linear_Conflict->second.end();

				UlinearConflicts_upper_map::const_iterator lower2_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				if (lower2_linear_conflict != linear_Conflict->second.end())
				{
				    for (UlinearConflicts_map::const_iterator lower2_linear_conflict = linear_Conflict->second.begin(); lower2_linear_conflict != linear_Conflict->second.end(); ++lower2_linear_conflict)
				    {
					if (lower2_linear_conflict->second.m_interval.m_upper > front_transition.m_time)
					{
					    lower_linear_conflict = lower2_linear_conflict;
					    first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
					    break;
					}
				    }
				}				
			    }

			    if (first_non_conf_linear_time >= 0.0)
			    {
				if (wait_linear_finish_time < 0.0)
				{
				    wait_linear_finish_time = first_non_conf_linear_time;
				}			    
				else
				{
				    if (wait_linear_finish_time > first_non_conf_linear_time)
				    {
					wait_linear_finish_time = first_non_conf_linear_time;
				    }
				}
			    }
			}
		    }
		    
		    sDouble wait_finish_time;
		    
		    if (wait_location_finish_time >= 0.0)
		    {
			if (wait_linear_finish_time >= 0.0)
			{
			    wait_finish_time = sMIN(wait_location_finish_time, wait_linear_finish_time);
			}
			else
			{
			    wait_finish_time = wait_location_finish_time;
			}
		    }
		    else
		    {
			if (wait_linear_finish_time >= 0.0)
			{
			    wait_finish_time = wait_linear_finish_time;
			}
			else
			{
			    wait_finish_time = -1.0;
			}
		    }
		    
		    if (wait_finish_time > front_transition.m_time + s_EPSILON)
		    {
			Explorations_umap *wait_explored_Transitions;
			TransitionExplorations_map::const_iterator explored_transition = explored_Transitions.find(wait_finish_time);
			if (explored_transition == explored_Transitions.end())
			{
			    explored_Transitions.insert(TransitionExplorations_map::value_type(wait_finish_time, Explorations_umap()));
			}
			wait_explored_Transitions = &explored_Transitions[wait_finish_time];
			
			if (wait_explored_Transitions->find(front_transition.m_location_id) == wait_explored_Transitions->end())
			{
			    sDouble wait_cost = (wait_finish_time - front_transition.m_time) * kruhobot.m_properties.m_wait_factor;
			    Transition wait_transition(transition_Store.size(), wait_finish_time, front_transition.m_cost + wait_cost, front_transition.m_makespan, front_transition.m_location_id, front_transition.m_trans_id);
			    sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(wait_transition.m_time);
			    
			    wait_explored_Transitions->insert(Explorations_umap::value_type(front_transition.m_location_id, front_transition.m_trans_id));
			    sDouble estimated_remaining = map.m_straight_Distances[sink_loc_id][front_transition.m_location_id] / kruhobot.m_properties.m_linear_velo;

 			    if (wait_transition.m_time + estimated_remaining <= makespan_bound + s_EPSILON)
			    {
				wait_transition.m_corr_dec_id = kruhobot_RDD.size();
				
				kruhobot_RDD.push_back(KruhobotDecision(kruhobot_RDD.size(),
									wait_transition.m_time,
									wait_transition.m_location_id,
									front_transition.m_corr_dec_id));
				kruhobot_RDD_mapping[wait_transition.m_location_id].insert(KruhobotDecisionIDs_mmap::value_type(wait_transition.m_time, wait_transition.m_corr_dec_id));
//				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_cost, wait_transition));
				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_makespan, wait_transition));
			    }
			    else
			    {
				wait_transition.m_corr_dec_id = -1;
			    }
			    transition_Store.push_back(wait_transition);
			}
		    }
		}				
	    }
	    transition_Queue.erase(transition_Queue.begin());
	}
	augment_KruhobotRealDecisionDiagram(kruhobot, source_loc_id, sink_loc_id, makespan_bound, kruhobot_RDD, kruhobot_RDD_mapping);	
	interconnect_KruhobotRealDecisionDiagram(kruhobot, map, kruhobot_RDD, kruhobot_RDD_mapping);

	return next_makespan_bound;
    }


    sDouble sRealSMTCBS::build_KruhobotRealDecisionDiagram_pruningSmart(const sKruhobot                      &kruhobot,
									const s2DMap                         &map,
									sInt_32                               source_loc_id,
									sInt_32                               sink_loc_id,
									const LocationConflicts_upper__umap  &location_Conflicts,
									const UlinearConflicts_upper__map     &linear_Conflicts,
									sDouble                               makespan_bound,
									KruhobotDecisionDiagram_vector       &kruhobot_RDD,
									KruhobotDecisionMapping_map          &kruhobot_RDD_mapping) const
    {
	sDouble next_makespan_bound = -1.0;
	
	Transitions_mmap transition_Queue;
	TransitionExplorations_map explored_Transitions;
	Explorations_umap bound_explored_Transitions;	
	Transitions_vector transition_Store;

	Transition initial_transition(transition_Store.size(), 0.0, 0.0, 0.0, source_loc_id, -1);
	initial_transition.m_corr_dec_id = 0;
	transition_Queue.insert(Transitions_mmap::value_type(0.0, initial_transition));
	transition_Store.push_back(initial_transition);
	
	kruhobot_RDD.push_back(KruhobotDecision(kruhobot_RDD.size(),
						initial_transition.m_time,
						initial_transition.m_location_id,
						-1));
	sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(initial_transition.m_time);
	kruhobot_RDD_mapping[initial_transition.m_location_id].insert(KruhobotDecisionIDs_mmap::value_type(initial_transition.m_time, initial_transition.m_trans_id));
	    
	{
	    Explorations_umap initial_explorations;
	    initial_explorations.insert(Explorations_umap::value_type(initial_transition.m_location_id, initial_transition.m_trans_id));
	    explored_Transitions.insert(TransitionExplorations_map::value_type(0.0, initial_explorations));
	}

	while (!transition_Queue.empty())
	{
	    const Transition &front_transition = transition_Queue.begin()->second;
	    /*
	    TransitionExplorations_map::const_iterator explored_transition = explored_Transitions.find(front_transition.m_time);	    
	    if (explored_transition == explored_Transitions.end())
	    {
		explored_Transitions.insert(TransitionExplorations_map::value_type(front_transition.m_time, Explorations_umap()));
	    }
	    */
	    if (front_transition.m_time + (map.m_straight_Distances[sink_loc_id][front_transition.m_location_id] / kruhobot.m_properties.m_linear_velo) <= makespan_bound + s_EPSILON)
	    {		
		const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[front_transition.m_location_id].m_Neighbors;
		for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
		{
		    sInt_32 neighbor_location_id = (*neighbor)->m_target->m_id;		    
		    {
			sDouble transition_distance = map.m_straight_Distances[front_transition.m_location_id][neighbor_location_id];
			sDouble transition_delta_time = transition_distance / kruhobot.m_properties.m_linear_velo;
			sDouble transition_finish_time = front_transition.m_time + transition_delta_time;
			sDouble transition_finish_cost = front_transition.m_cost + transition_delta_time;
			sDouble transition_finish_makespan = front_transition.m_makespan + transition_delta_time;			

			Explorations_umap *next_explored_Transitions = obtain_ExploredTransitions(explored_Transitions, transition_finish_time);
/*
			TransitionExplorations_map::const_iterator explored_transition = explored_Transitions.find(transition_finish_time);			
			if (explored_transition == explored_Transitions.end())
			{
			    explored_Transitions.insert(TransitionExplorations_map::value_type(transition_finish_time, Explorations_umap()));
			}
			next_explored_Transitions = &explored_Transitions[transition_finish_time];
*/
			
			Explorations_umap::const_iterator next_explored_transition = next_explored_Transitions->find(neighbor_location_id);
	
			if (next_explored_transition == next_explored_Transitions->end())
			{
			    Transition neighbor_transition(transition_Store.size(), transition_finish_time, transition_finish_cost, transition_finish_makespan, neighbor_location_id, front_transition.m_trans_id);		    
			    sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(neighbor_transition.m_time);			   

			    next_explored_Transitions->insert(Explorations_umap::value_type(neighbor_location_id, front_transition.m_trans_id));
			    sDouble estimated_remaining =  map.m_straight_Distances[sink_loc_id][neighbor_location_id] / kruhobot.m_properties.m_linear_velo;
			    
			    if (neighbor_transition.m_time + estimated_remaining <= makespan_bound + s_EPSILON)
			    {				
				neighbor_transition.m_corr_dec_id = kruhobot_RDD.size();
				
				kruhobot_RDD.push_back(KruhobotDecision(kruhobot_RDD.size(),
									neighbor_transition.m_time,
									neighbor_transition.m_location_id,
									front_transition.m_corr_dec_id));
				kruhobot_RDD_mapping[neighbor_transition.m_location_id].insert(KruhobotDecisionIDs_mmap::value_type(neighbor_transition.m_time, neighbor_transition.m_corr_dec_id));
//				transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_cost, neighbor_transition));
				transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_makespan, neighbor_transition));
			    }
			    else
			    {
				/*
				if (sABS(neighbor_transition.m_time + estimated_remaining - (makespan_bound + s_EPSILON)) < 0.01)				    
				{
				    printf("Cut off %d: %.24lf,%.24lf : %.24lf (%d)\n", kruhobot.m_id, neighbor_transition.m_time + estimated_remaining, makespan_bound + s_EPSILON, sABS(neighbor_transition.m_time + estimated_remaining - (makespan_bound + s_EPSILON)), neighbor_transition.m_location_id);
				}
				*/
				neighbor_transition.m_corr_dec_id = -1;				
			    }
			    transition_Store.push_back(neighbor_transition);			    
			}
		    }
		}
		{
		    sDouble wait_location_finish_time = -1.0;
		    sDouble wait_linear_finish_time = -1.0;

		    const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[front_transition.m_location_id].m_Neighbors;
		    for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
		    {
			sInt_32 neighbor_location_id = (*neighbor)->m_target->m_id;
			{
			    LocationConflicts_upper__umap::const_iterator location_Conflict = location_Conflicts.find(neighbor_location_id);
			    sDouble first_non_conf_location_time = -1.0;
			    
			    if (location_Conflict != location_Conflicts.end())
			    {
				LocationConflicts_upper_map::const_iterator lower_location_conflict = location_Conflict->second.end();

				LocationConflicts_upper_map::const_iterator lower2_location_conflict = location_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				if (lower2_location_conflict != location_Conflict->second.end())
				{
				    for (LocationConflicts_map::const_iterator lower2_location_conflict = location_Conflict->second.begin(); lower2_location_conflict != location_Conflict->second.end(); ++lower2_location_conflict)
				    {
					if (lower2_location_conflict->second.m_interval.m_upper > front_transition.m_time)
					{
					    lower_location_conflict = lower2_location_conflict;
					    first_non_conf_location_time = lower_location_conflict->second.m_interval.m_upper;
					    break;
					}
				    }
				}
			    }
			    
			    if (first_non_conf_location_time >= 0.0)
			    {
				if (wait_location_finish_time < 0.0)
				{
				    wait_location_finish_time = first_non_conf_location_time;
				}			    
				else
				{
				    if (wait_location_finish_time > first_non_conf_location_time)
				    {
					wait_location_finish_time = first_non_conf_location_time;
				    }
				}
			    }
			    
			    UlinearConflicts_upper__map::const_iterator linear_Conflict = linear_Conflicts.find(Uline(front_transition.m_location_id, neighbor_location_id));
			    sDouble first_non_conf_linear_time = -1.0;
			    
			    if (linear_Conflict != linear_Conflicts.end())
			    {
				UlinearConflicts_upper_map::const_iterator lower_linear_conflict = linear_Conflict->second.end();

				UlinearConflicts_upper_map::const_iterator lower2_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				if (lower2_linear_conflict != linear_Conflict->second.end())
				{
				    for (UlinearConflicts_map::const_iterator lower2_linear_conflict = linear_Conflict->second.begin(); lower2_linear_conflict != linear_Conflict->second.end(); ++lower2_linear_conflict)
				    {
					if (lower2_linear_conflict->second.m_interval.m_upper > front_transition.m_time)
					{
					    lower_linear_conflict = lower2_linear_conflict;
					    first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
					    break;
					}
				    }
				}				
			    }

			    if (first_non_conf_linear_time >= 0.0)
			    {
				if (wait_linear_finish_time < 0.0)
				{
				    wait_linear_finish_time = first_non_conf_linear_time;
				}			    
				else
				{
				    if (wait_linear_finish_time > first_non_conf_linear_time)
				    {
					wait_linear_finish_time = first_non_conf_linear_time;
				    }
				}
			    }
			}
		    }
		    {
			sInt_32 neighbor_location_id = front_transition.m_location_id;
			
			{
			    LocationConflicts_upper__umap::const_iterator location_Conflict = location_Conflicts.find(neighbor_location_id);
			    sDouble first_non_conf_location_time = -1.0;
			    
			    if (location_Conflict != location_Conflicts.end())
			    {
				LocationConflicts_upper_map::const_iterator lower_location_conflict = location_Conflict->second.end();

				LocationConflicts_upper_map::const_iterator lower2_location_conflict = location_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				if (lower2_location_conflict != location_Conflict->second.end())
				{
				    for (LocationConflicts_map::const_iterator lower2_location_conflict = location_Conflict->second.begin(); lower2_location_conflict != location_Conflict->second.end(); ++lower2_location_conflict)
				    {
					if (lower2_location_conflict->second.m_interval.m_upper > front_transition.m_time)
					{
					    lower_location_conflict = lower2_location_conflict;
					    first_non_conf_location_time = lower_location_conflict->second.m_interval.m_upper;
					    break;
					}
				    }
				}
			    }
			    
			    if (first_non_conf_location_time >= 0.0)
			    {
				if (wait_location_finish_time < 0.0)
				{
				    wait_location_finish_time = first_non_conf_location_time;
				}			    
				else
				{
				    if (wait_location_finish_time > first_non_conf_location_time)
				    {
					wait_location_finish_time = first_non_conf_location_time;
				    }
				}
			    }
			    
			    UlinearConflicts_upper__map::const_iterator linear_Conflict = linear_Conflicts.find(Uline(front_transition.m_location_id, neighbor_location_id));
			    sDouble first_non_conf_linear_time = -1.0;
			    
			    if (linear_Conflict != linear_Conflicts.end())
			    {
				UlinearConflicts_upper_map::const_iterator lower_linear_conflict = linear_Conflict->second.end();

				UlinearConflicts_upper_map::const_iterator lower2_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				if (lower2_linear_conflict != linear_Conflict->second.end())
				{
				    for (UlinearConflicts_map::const_iterator lower2_linear_conflict = linear_Conflict->second.begin(); lower2_linear_conflict != linear_Conflict->second.end(); ++lower2_linear_conflict)
				    {
					if (lower2_linear_conflict->second.m_interval.m_upper > front_transition.m_time)
					{
					    lower_linear_conflict = lower2_linear_conflict;
					    first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
					    break;
					}
				    }
				}				
			    }

			    if (first_non_conf_linear_time >= 0.0)
			    {
				if (wait_linear_finish_time < 0.0)
				{
				    wait_linear_finish_time = first_non_conf_linear_time;
				}			    
				else
				{
				    if (wait_linear_finish_time > first_non_conf_linear_time)
				    {
					wait_linear_finish_time = first_non_conf_linear_time;
				    }
				}
			    }
			}
		    }
		    
		    sDouble wait_finish_time;
		    
		    if (wait_location_finish_time >= 0.0)
		    {
			if (wait_linear_finish_time >= 0.0)
			{
			    wait_finish_time = sMIN(wait_location_finish_time, wait_linear_finish_time);
			}
			else
			{
			    wait_finish_time = wait_location_finish_time;
			}
		    }
		    else
		    {
			if (wait_linear_finish_time >= 0.0)
			{
			    wait_finish_time = wait_linear_finish_time;
			}
			else
			{
			    wait_finish_time = -1.0;
			}
		    }
		    
		    if (wait_finish_time > front_transition.m_time + s_EPSILON)
		    {
			
//			Explorations_umap *wait_explored_Transitions;
			Explorations_umap *wait_explored_Transitions = obtain_ExploredTransitions(explored_Transitions, wait_finish_time);
/*
			TransitionExplorations_map::const_iterator explored_transition = explored_Transitions.find(wait_finish_time);
			if (explored_transition == explored_Transitions.end())
			{
			    explored_Transitions.insert(TransitionExplorations_map::value_type(wait_finish_time, Explorations_umap()));
			}
			wait_explored_Transitions = &explored_Transitions[wait_finish_time];
*/
			
			if (wait_explored_Transitions->find(front_transition.m_location_id) == wait_explored_Transitions->end())
			{
			    sDouble wait_cost = (wait_finish_time - front_transition.m_time) * kruhobot.m_properties.m_wait_factor;
			    Transition wait_transition(transition_Store.size(), wait_finish_time, front_transition.m_cost + wait_cost, wait_finish_time, front_transition.m_location_id, front_transition.m_trans_id);
			    sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(wait_transition.m_time);
			    
			    wait_explored_Transitions->insert(Explorations_umap::value_type(front_transition.m_location_id, front_transition.m_trans_id));
			    sDouble estimated_remaining = map.m_straight_Distances[sink_loc_id][front_transition.m_location_id] / kruhobot.m_properties.m_linear_velo;

 			    if (wait_transition.m_time + estimated_remaining <= makespan_bound + s_EPSILON)
			    {
				wait_transition.m_corr_dec_id = kruhobot_RDD.size();
				
				kruhobot_RDD.push_back(KruhobotDecision(kruhobot_RDD.size(),
									wait_transition.m_time,
									wait_transition.m_location_id,
									front_transition.m_corr_dec_id));
				kruhobot_RDD_mapping[wait_transition.m_location_id].insert(KruhobotDecisionIDs_mmap::value_type(wait_transition.m_time, wait_transition.m_corr_dec_id));
//				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_cost, wait_transition));
				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_makespan, wait_transition));				
			    }
			    else
			    {
				wait_transition.m_corr_dec_id = -1;
			    }
			    transition_Store.push_back(wait_transition);
			}
		    }
		}				
	    }
	    transition_Queue.erase(transition_Queue.begin());
	}
	augment_KruhobotRealDecisionDiagram(kruhobot, source_loc_id, sink_loc_id, makespan_bound, kruhobot_RDD, kruhobot_RDD_mapping);	
	interconnect_KruhobotRealDecisionDiagram_smart(kruhobot, map, kruhobot_RDD, kruhobot_RDD_mapping);

	return next_makespan_bound;
    }


    sDouble sRealSMTCBS::build_KruhobotRealDecisionDiagram_pruningStrong(const sKruhobot                      &kruhobot,
									 const s2DMap                         &map,
									 sInt_32                               source_loc_id,
									 sInt_32                               sink_loc_id,
									 const LocationConflicts_upper__umap  &location_Conflicts,
									 const UlinearConflicts_upper__map     &linear_Conflicts,
									 sDouble                               makespan_bound,
									 KruhobotDecisionDiagram_vector       &kruhobot_RDD,
									 KruhobotDecisionMapping_map          &kruhobot_RDD_mapping) const
    {
	sDouble next_makespan_bound = -1.0;
	
	Transitions_mmap transition_Queue;
	TransitionExplorations_map explored_Transitions;
	Explorations_umap bound_explored_Transitions;	
	Transitions_vector transition_Store;

	Transition initial_transition(transition_Store.size(), 0.0, 0.0, 0.0, source_loc_id, -1);
	initial_transition.m_corr_dec_id = 0;
	transition_Queue.insert(Transitions_mmap::value_type(0.0, initial_transition));
	transition_Store.push_back(initial_transition);
	
	kruhobot_RDD.push_back(KruhobotDecision(kruhobot_RDD.size(),
						initial_transition.m_time,
						initial_transition.m_location_id,
						-1));
	sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(initial_transition.m_time);
	kruhobot_RDD_mapping[initial_transition.m_location_id].insert(KruhobotDecisionIDs_mmap::value_type(initial_transition.m_time, initial_transition.m_trans_id));
	    
	{
	    Explorations_umap initial_explorations;
	    initial_explorations.insert(Explorations_umap::value_type(initial_transition.m_location_id, initial_transition.m_trans_id));
	    explored_Transitions.insert(TransitionExplorations_map::value_type(0.0, initial_explorations));
	}

	while (!transition_Queue.empty())
	{
	    const Transition &front_transition = transition_Queue.begin()->second;
	    /*
	    TransitionExplorations_map::const_iterator explored_transition = explored_Transitions.find(front_transition.m_time);	    
	    if (explored_transition == explored_Transitions.end())
	    {
		explored_Transitions.insert(TransitionExplorations_map::value_type(front_transition.m_time, Explorations_umap()));
	    }
	    */
	    if (front_transition.m_time + (map.m_shortest_Distances[sink_loc_id][front_transition.m_location_id] / kruhobot.m_properties.m_linear_velo) <= makespan_bound + s_EPSILON)
	    {		
		const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[front_transition.m_location_id].m_Neighbors;
		for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
		{
		    sInt_32 neighbor_location_id = (*neighbor)->m_target->m_id;		    
		    {
			sDouble transition_distance = map.m_straight_Distances[front_transition.m_location_id][neighbor_location_id];
			sDouble transition_delta_time = transition_distance / kruhobot.m_properties.m_linear_velo;
			sDouble transition_finish_time = front_transition.m_time + transition_delta_time;
			sDouble transition_finish_cost = front_transition.m_cost + transition_delta_time;			
			sDouble transition_finish_makespan = front_transition.m_makespan + transition_delta_time;

			Explorations_umap *next_explored_Transitions = obtain_ExploredTransitions(explored_Transitions, transition_finish_time);
/*
			TransitionExplorations_map::const_iterator explored_transition = explored_Transitions.find(transition_finish_time);			
			if (explored_transition == explored_Transitions.end())
			{
			    explored_Transitions.insert(TransitionExplorations_map::value_type(transition_finish_time, Explorations_umap()));
			}
			next_explored_Transitions = &explored_Transitions[transition_finish_time];
*/
			
			Explorations_umap::const_iterator next_explored_transition = next_explored_Transitions->find(neighbor_location_id);
	
			if (next_explored_transition == next_explored_Transitions->end())
			{
			    Transition neighbor_transition(transition_Store.size(), transition_finish_time, transition_finish_cost, transition_finish_makespan, neighbor_location_id, front_transition.m_trans_id);		    
			    sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(neighbor_transition.m_time);			   

			    next_explored_Transitions->insert(Explorations_umap::value_type(neighbor_location_id, front_transition.m_trans_id));
			    sDouble estimated_remaining =  map.m_shortest_Distances[sink_loc_id][neighbor_location_id] / kruhobot.m_properties.m_linear_velo;
			    
			    if (neighbor_transition.m_time + estimated_remaining <= makespan_bound + s_EPSILON)
			    {				
				neighbor_transition.m_corr_dec_id = kruhobot_RDD.size();
				
				kruhobot_RDD.push_back(KruhobotDecision(kruhobot_RDD.size(),
									neighbor_transition.m_time,
									neighbor_transition.m_location_id,
									front_transition.m_corr_dec_id));
				kruhobot_RDD_mapping[neighbor_transition.m_location_id].insert(KruhobotDecisionIDs_mmap::value_type(neighbor_transition.m_time, neighbor_transition.m_corr_dec_id));
				transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_cost, neighbor_transition));				
			    }
			    else
			    {
				/*
				if (sABS(neighbor_transition.m_time + estimated_remaining - (makespan_bound + s_EPSILON)) < 0.01)				    
				{
				    printf("Cut off %d: %.24lf,%.24lf : %.24lf (%d)\n", kruhobot.m_id, neighbor_transition.m_time + estimated_remaining, makespan_bound + s_EPSILON, sABS(neighbor_transition.m_time + estimated_remaining - (makespan_bound + s_EPSILON)), neighbor_transition.m_location_id);
				}
				*/
				neighbor_transition.m_corr_dec_id = -1;				
			    }
			    transition_Store.push_back(neighbor_transition);			    
			}
		    }
		}
		{
		    sDouble wait_location_finish_time = -1.0;
		    sDouble wait_linear_finish_time = -1.0;

		    const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[front_transition.m_location_id].m_Neighbors;
		    for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
		    {
			sInt_32 neighbor_location_id = (*neighbor)->m_target->m_id;
			{
			    LocationConflicts_upper__umap::const_iterator location_Conflict = location_Conflicts.find(neighbor_location_id);
			    sDouble first_non_conf_location_time = -1.0;
			    
			    if (location_Conflict != location_Conflicts.end())
			    {
				LocationConflicts_upper_map::const_iterator lower_location_conflict = location_Conflict->second.end();

				for (LocationConflicts_upper_map::const_iterator lower2_location_conflict = location_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				     lower2_location_conflict != location_Conflict->second.end(); ++lower2_location_conflict)
				{
				    if (lower2_location_conflict->second.m_interval.m_upper > front_transition.m_time)
				    {
					lower_location_conflict = lower2_location_conflict;
					first_non_conf_location_time = lower_location_conflict->second.m_interval.m_upper;
					break;
				    }				    
				}
/*				
				LocationConflicts_upper_map::const_iterator lower2_location_conflict = location_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));

				if (lower2_location_conflict != location_Conflict->second.end())
				{
				    for (LocationConflicts_map::const_iterator lower2_location_conflict = location_Conflict->second.begin(); lower2_location_conflict != location_Conflict->second.end(); ++lower2_location_conflict)
				    {
					if (lower2_location_conflict->second.m_interval.m_upper > front_transition.m_time)
					{
					    lower_location_conflict = lower2_location_conflict;
					    first_non_conf_location_time = lower_location_conflict->second.m_interval.m_upper;
					    break;
					}
				    }
				}
*/
				
			    }
			    
			    if (first_non_conf_location_time >= 0.0)
			    {
				if (wait_location_finish_time < 0.0)
				{
				    wait_location_finish_time = first_non_conf_location_time;
				}			    
				else
				{
				    if (wait_location_finish_time > first_non_conf_location_time)
				    {
					wait_location_finish_time = first_non_conf_location_time;
				    }
				}
			    }
			    
			    UlinearConflicts_upper__map::const_iterator linear_Conflict = linear_Conflicts.find(Uline(front_transition.m_location_id, neighbor_location_id));
			    sDouble first_non_conf_linear_time = -1.0;
			    
			    if (linear_Conflict != linear_Conflicts.end())
			    {
				UlinearConflicts_upper_map::const_iterator lower_linear_conflict = linear_Conflict->second.end();

				for (UlinearConflicts_upper_map::const_iterator lower2_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				     lower2_linear_conflict != linear_Conflict->second.end(); ++lower2_linear_conflict)
				{
				    if (lower2_linear_conflict->second.m_interval.m_upper > front_transition.m_time)
				    {
					lower_linear_conflict = lower2_linear_conflict;
					first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
					break;
				    }
				}
				
				/*
				UlinearConflicts_upper_map::const_iterator lower2_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				if (lower2_linear_conflict != linear_Conflict->second.end())
				{
				    for (UlinearConflicts_map::const_iterator lower2_linear_conflict = linear_Conflict->second.begin(); lower2_linear_conflict != linear_Conflict->second.end(); ++lower2_linear_conflict)
				    {
					if (lower2_linear_conflict->second.m_interval.m_upper > front_transition.m_time)
					{
					    lower_linear_conflict = lower2_linear_conflict;
					    first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
					    break;
					}
				    }
				}
				*/				
			    }

			    if (first_non_conf_linear_time >= 0.0)
			    {
				if (wait_linear_finish_time < 0.0)
				{
				    wait_linear_finish_time = first_non_conf_linear_time;
				}			    
				else
				{
				    if (wait_linear_finish_time > first_non_conf_linear_time)
				    {
					wait_linear_finish_time = first_non_conf_linear_time;
				    }
				}
			    }
			}
		    }
		    {
			sInt_32 neighbor_location_id = front_transition.m_location_id;
			
			{
			    LocationConflicts_upper__umap::const_iterator location_Conflict = location_Conflicts.find(neighbor_location_id);
			    sDouble first_non_conf_location_time = -1.0;
			    
			    if (location_Conflict != location_Conflicts.end())
			    {
				LocationConflicts_upper_map::const_iterator lower_location_conflict = location_Conflict->second.end();

				for (LocationConflicts_upper_map::const_iterator lower2_location_conflict = location_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				     lower2_location_conflict != location_Conflict->second.end(); ++lower2_location_conflict)
				{
				    if (lower2_location_conflict->second.m_interval.m_upper > front_transition.m_time)
				    {
					lower_location_conflict = lower2_location_conflict;
					first_non_conf_location_time = lower_location_conflict->second.m_interval.m_upper;
					break;
				    }
				}				

				/*
				LocationConflicts_upper_map::const_iterator lower2_location_conflict = location_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				if (lower2_location_conflict != location_Conflict->second.end())
				{
				    for (LocationConflicts_map::const_iterator lower2_location_conflict = location_Conflict->second.begin(); lower2_location_conflict != location_Conflict->second.end(); ++lower2_location_conflict)
				    {
					if (lower2_location_conflict->second.m_interval.m_upper > front_transition.m_time)
					{
					    lower_location_conflict = lower2_location_conflict;
					    first_non_conf_location_time = lower_location_conflict->second.m_interval.m_upper;
					    break;
					}
				    }
				}
				*/
			    }
			    
			    if (first_non_conf_location_time >= 0.0)
			    {
				if (wait_location_finish_time < 0.0)
				{
				    wait_location_finish_time = first_non_conf_location_time;
				}			    
				else
				{
				    if (wait_location_finish_time > first_non_conf_location_time)
				    {
					wait_location_finish_time = first_non_conf_location_time;
				    }
				}
			    }
			    
			    UlinearConflicts_upper__map::const_iterator linear_Conflict = linear_Conflicts.find(Uline(front_transition.m_location_id, neighbor_location_id));
			    sDouble first_non_conf_linear_time = -1.0;
			    
			    if (linear_Conflict != linear_Conflicts.end())
			    {
				UlinearConflicts_upper_map::const_iterator lower_linear_conflict = linear_Conflict->second.end();

				for (UlinearConflicts_upper_map::const_iterator lower2_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				     lower2_linear_conflict != linear_Conflict->second.end(); ++lower2_linear_conflict)
				{
				    if (lower2_linear_conflict->second.m_interval.m_upper > front_transition.m_time)
				    {
					lower_linear_conflict = lower2_linear_conflict;
					first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
					break;
				    }
				}

/*
				UlinearConflicts_upper_map::const_iterator lower2_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				if (lower2_linear_conflict != linear_Conflict->second.end())
				{
				    for (UlinearConflicts_map::const_iterator lower2_linear_conflict = linear_Conflict->second.begin(); lower2_linear_conflict != linear_Conflict->second.end(); ++lower2_linear_conflict)
				    {
					if (lower2_linear_conflict->second.m_interval.m_upper > front_transition.m_time)
					{
					    lower_linear_conflict = lower2_linear_conflict;
					    first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
					    break;
					}
				    }
				}
*/
			    }

			    if (first_non_conf_linear_time >= 0.0)
			    {
				if (wait_linear_finish_time < 0.0)
				{
				    wait_linear_finish_time = first_non_conf_linear_time;
				}			    
				else
				{
				    if (wait_linear_finish_time > first_non_conf_linear_time)
				    {
					wait_linear_finish_time = first_non_conf_linear_time;
				    }
				}
			    }
			}
		    }
		    
		    sDouble wait_finish_time;
		    
		    if (wait_location_finish_time >= 0.0)
		    {
			if (wait_linear_finish_time >= 0.0)
			{
			    wait_finish_time = sMIN(wait_location_finish_time, wait_linear_finish_time);
			}
			else
			{
			    wait_finish_time = wait_location_finish_time;
			}
		    }
		    else
		    {
			if (wait_linear_finish_time >= 0.0)
			{
			    wait_finish_time = wait_linear_finish_time;
			}
			else
			{
			    wait_finish_time = -1.0;
			}
		    }
		    
		    if (wait_finish_time > front_transition.m_time + s_EPSILON)
		    {
			
//			Explorations_umap *wait_explored_Transitions;
			Explorations_umap *wait_explored_Transitions = obtain_ExploredTransitions(explored_Transitions, wait_finish_time);
/*
			TransitionExplorations_map::const_iterator explored_transition = explored_Transitions.find(wait_finish_time);
			if (explored_transition == explored_Transitions.end())
			{
			    explored_Transitions.insert(TransitionExplorations_map::value_type(wait_finish_time, Explorations_umap()));
			}
			wait_explored_Transitions = &explored_Transitions[wait_finish_time];
*/
			
			if (wait_explored_Transitions->find(front_transition.m_location_id) == wait_explored_Transitions->end())
			{
			    sDouble wait_cost = (wait_finish_time - front_transition.m_time) * kruhobot.m_properties.m_wait_factor;
			    Transition wait_transition(transition_Store.size(), wait_finish_time, front_transition.m_cost + wait_cost, wait_finish_time, front_transition.m_location_id, front_transition.m_trans_id);
			    sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(wait_transition.m_time);
			    
			    wait_explored_Transitions->insert(Explorations_umap::value_type(front_transition.m_location_id, front_transition.m_trans_id));
			    sDouble estimated_remaining = map.m_shortest_Distances[sink_loc_id][front_transition.m_location_id] / kruhobot.m_properties.m_linear_velo;

 			    if (wait_transition.m_time + estimated_remaining <= makespan_bound + s_EPSILON)
			    {
				wait_transition.m_corr_dec_id = kruhobot_RDD.size();
				
				kruhobot_RDD.push_back(KruhobotDecision(kruhobot_RDD.size(),
									wait_transition.m_time,
									wait_transition.m_location_id,
									front_transition.m_corr_dec_id));
				kruhobot_RDD_mapping[wait_transition.m_location_id].insert(KruhobotDecisionIDs_mmap::value_type(wait_transition.m_time, wait_transition.m_corr_dec_id));
//				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_cost, wait_transition));
				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_makespan, wait_transition));				
			    }
			    else
			    {
				wait_transition.m_corr_dec_id = -1;
			    }
			    transition_Store.push_back(wait_transition);
			}
		    }
		}				
	    }
	    transition_Queue.erase(transition_Queue.begin());
	}
	augment_KruhobotRealDecisionDiagram(kruhobot, source_loc_id, sink_loc_id, makespan_bound, kruhobot_RDD, kruhobot_RDD_mapping);	
	interconnect_KruhobotRealDecisionDiagram_smart(kruhobot, map, kruhobot_RDD, kruhobot_RDD_mapping);

	return next_makespan_bound;
    }


    sDouble sRealSMTCBS::build_KruhobotRealDecisionDiagram_conflictRespectful(const sKruhobot                      &kruhobot,
									      const s2DMap                         &map,
									      sInt_32                               source_loc_id,
									      sInt_32                               sink_loc_id,
									      const LocationConflicts_upper__umap  &location_Conflicts,
									      const UlinearConflicts_upper__map     &linear_Conflicts,
									      sDouble                               makespan_bound,
									      KruhobotDecisionDiagram_vector       &kruhobot_RDD,
									      KruhobotDecisionMapping_map          &kruhobot_RDD_mapping) const
    {
        #ifdef sDEBUG
	bool sink_reached = false;
        #endif	
	
	sInt_32 last_transition_id = 0;
	sDouble next_makespan_bound = -1.0;
	
	RespectfulTransitions_mmap respectful_transition_Queue;
	RespectfulExplorations_map respectful_Explorations;

	RespectfulTransition initial_transition(last_transition_id++, 0.0, 0.0, 0.0, source_loc_id, -1);
	initial_transition.m_prev_corr_dec_id = -1;

	UnifiedVisits_umap unified_Visits;
	
	respectful_transition_Queue.insert(RespectfulTransitions_mmap::value_type(0.0, initial_transition));    
	{
	    RespectfulVisits_umap empty_conflict_Visits;
	    RespectfulVisit respectful_visit(0.0, initial_transition.m_trans_id);
	    respectful_visit.m_queue_iter = respectful_transition_Queue.begin();
		
	    empty_conflict_Visits.insert(RespectfulVisits_umap::value_type(initial_transition.m_location_id, respectful_visit));
	    respectful_Explorations.insert(RespectfulExplorations_map::value_type(initial_transition.m_conflict_fingerprint, empty_conflict_Visits));

	    //unified_Visits[initial_transition.m_location_id].insert(0.0);
	}

	while (!respectful_transition_Queue.empty())
	{
	    const RespectfulTransition &front_respectful_transition = respectful_transition_Queue.begin()->second;
	    	    
	    if (front_respectful_transition.m_time + (map.m_shortest_Distances[sink_loc_id][front_respectful_transition.m_location_id] / kruhobot.m_properties.m_linear_velo) <= makespan_bound + s_EPSILON)
	    {
		if (sink_loc_id == front_respectful_transition.m_location_id)
		{
		    #ifdef sDEBUG
		    {
			sink_reached = true;
		    }
		    #endif
		}
		
		sInt_32 front_kruhobot_decision_id = kruhobot_RDD.size();

		if (!is_UnifiedlyVisited(front_respectful_transition.m_location_id, front_respectful_transition.m_time, unified_Visits))
		{
		    kruhobot_RDD.push_back(KruhobotDecision(front_kruhobot_decision_id,
							    front_respectful_transition.m_time,
							    front_respectful_transition.m_location_id,
							    front_respectful_transition.m_prev_corr_dec_id));	      
		    kruhobot_RDD_mapping[front_respectful_transition.m_location_id].insert(KruhobotDecisionIDs_mmap::value_type(front_respectful_transition.m_time, front_kruhobot_decision_id));
		    unified_Visits[front_respectful_transition.m_location_id].insert(front_respectful_transition.m_time);		    
		}				    
		const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[front_respectful_transition.m_location_id].m_Neighbors;
		for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
		{		    
		    sInt_32 neighbor_location_id = (*neighbor)->m_target->m_id;
		    {
			sDouble transition_distance = map.m_straight_Distances[front_respectful_transition.m_location_id][neighbor_location_id];
			sDouble transition_delta_time = transition_distance / kruhobot.m_properties.m_linear_velo;
			sDouble transition_finish_time = front_respectful_transition.m_time + transition_delta_time;
			sDouble transition_finish_cost = front_respectful_transition.m_cost + transition_delta_time;			
			sDouble transition_finish_makespan = front_respectful_transition.m_makespan + transition_delta_time;

			if (!is_UnifiedlyVisited(neighbor_location_id, transition_finish_time, unified_Visits))
			{
			    RespectfulExplorations_map::iterator respectful_exploration = respectful_Explorations.find(front_respectful_transition.m_conflict_fingerprint);
			    sASSERT(respectful_exploration != respectful_Explorations.end());

			    RespectfulVisits_umap::iterator next_respectful_visit = respectful_exploration->second.find(neighbor_location_id);

			    RespectfulTransition neighbor_respectful_transition(last_transition_id++,
										transition_finish_time,
										transition_finish_cost,
										transition_finish_makespan,
										neighbor_location_id,
										front_respectful_transition.m_trans_id);
			    neighbor_respectful_transition.m_prev_corr_dec_id = front_kruhobot_decision_id;
			    neighbor_respectful_transition.m_conflict_fingerprint = front_respectful_transition.m_conflict_fingerprint;
			    sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(neighbor_respectful_transition.m_time);
			    
			    if (next_respectful_visit == respectful_exploration->second.end()) /* visiting for the first time in a given fingerprint */
			    {
				sDouble estimated_remaining = map.m_shortest_Distances[sink_loc_id][neighbor_location_id] / kruhobot.m_properties.m_linear_velo;
			    
				if (neighbor_respectful_transition.m_time + estimated_remaining <= makespan_bound + s_EPSILON)
				{
				    RespectfulVisit neighbor_respectful_visit(neighbor_respectful_transition.m_time, neighbor_respectful_transition.m_trans_id);
				    RespectfulVisits_umap::iterator neighbor_respectful_visit_iter = respectful_exploration->second.insert(RespectfulVisits_umap::value_type(neighbor_location_id,
																					     neighbor_respectful_visit)).first;
				    /*
				    RespectfulTransitions_mmap::iterator queue_iter = respectful_transition_Queue.insert(RespectfulTransitions_mmap::value_type(neighbor_respectful_transition.m_cost,
																				neighbor_respectful_transition));
				    */				    
				    RespectfulTransitions_mmap::iterator queue_iter = respectful_transition_Queue.insert(RespectfulTransitions_mmap::value_type(neighbor_respectful_transition.m_makespan,
																				neighbor_respectful_transition));
				    neighbor_respectful_visit_iter->second.m_queue_iter = queue_iter;
				}
				else
				{
				    sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(neighbor_respectful_transition.m_time + estimated_remaining);
				}
			    }
			    else /* visiting for the next time in a given fingerprint */
			    {
				sASSERT(next_respectful_visit != respectful_exploration->second.end());
				
				if (next_respectful_visit->second.m_time > neighbor_respectful_transition.m_time) /* update neighbor */
				{
				    #ifdef sDEBUG
				    {
					sDouble estimated_remaining = map.m_shortest_Distances[sink_loc_id][neighbor_location_id] / kruhobot.m_properties.m_linear_velo;
					sASSERT(neighbor_respectful_transition.m_time + estimated_remaining <= makespan_bound + s_EPSILON);
				    }
				    #endif
				
				    next_respectful_visit->second.m_time = neighbor_respectful_transition.m_time;
				    next_respectful_visit->second.m_trans_id = neighbor_respectful_transition.m_trans_id;
				
				    respectful_transition_Queue.erase(next_respectful_visit->second.m_queue_iter);

				    /*
				    RespectfulTransitions_mmap::iterator queue_iter = respectful_transition_Queue.insert(RespectfulTransitions_mmap::value_type(neighbor_respectful_transition.m_cost,
																				neighbor_respectful_transition));
				    */				    
				    RespectfulTransitions_mmap::iterator queue_iter = respectful_transition_Queue.insert(RespectfulTransitions_mmap::value_type(neighbor_respectful_transition.m_makespan,
																				neighbor_respectful_transition));
				    next_respectful_visit->second.m_queue_iter = queue_iter;
				}
			    }
//			    unified_Visits[neighbor_respectful_transition.m_location_id].insert(neighbor_respectful_transition.m_time);
			}
			else
			{
			    sASSERT(false);
			    #ifdef sDEBUG
			    {
				/*
				for (UnifiedVisits_umap::const_iterator unified_visit = unified_Visits.begin(); unified_visit != unified_Visits.end(); ++unified_visit)
				{
				    printf("%d: ", unified_visit->first);
				    for (VisitTimes_set::const_iterator visit_time = unified_visit->second.begin(); visit_time != unified_visit->second.end(); ++visit_time)
				    {
					printf("%.3f ", *visit_time);
				    }
				    printf("\n");
				}
				printf("Queue: %ld (%ld)\n", respectful_transition_Queue.size(), unified_Visits.size());
				*/
			    }
			    #endif
			}
		    }
		}
		{
		    sDouble wait_location_finish_time = -1.0;
		    sInt_32 wait_location_culprit_conflict_id = -1;
		    
		    sDouble wait_linear_finish_time = -1.0;
		    sInt_32 wait_linear_culprit_conflict_id = -1;		    

		    const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[front_respectful_transition.m_location_id].m_Neighbors;
		    for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
		    {
			sInt_32 neighbor_location_id = (*neighbor)->m_target->m_id;
			{
			    LocationConflicts_upper__umap::const_iterator location_Conflict = location_Conflicts.find(neighbor_location_id);
			    sDouble first_non_conf_location_time = -1.0;
			    sInt_32 first_location_culprit_conflict_id = -1;
			    
			    if (location_Conflict != location_Conflicts.end())
			    {
				LocationConflicts_upper_map::const_iterator lower_location_conflict = location_Conflict->second.end();

				for (LocationConflicts_upper_map::const_iterator lower2_location_conflict = location_Conflict->second.lower_bound(Interval(front_respectful_transition.m_time,
																			   front_respectful_transition.m_time));
				     lower2_location_conflict != location_Conflict->second.end(); ++lower2_location_conflict)
				{
				    if (lower2_location_conflict->second.m_interval.m_upper > front_respectful_transition.m_time)
				    {
					lower_location_conflict = lower2_location_conflict;
					first_non_conf_location_time = lower_location_conflict->second.m_interval.m_upper;
					first_location_culprit_conflict_id = lower_location_conflict->second.m_conflict_id;
					break;
				    }				    
				}
/*				
				LocationConflicts_upper_map::const_iterator lower2_location_conflict = location_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));

				if (lower2_location_conflict != location_Conflict->second.end())
				{
				    for (LocationConflicts_map::const_iterator lower2_location_conflict = location_Conflict->second.begin(); lower2_location_conflict != location_Conflict->second.end(); ++lower2_location_conflict)
				    {
					if (lower2_location_conflict->second.m_interval.m_upper > front_transition.m_time)
					{
					    lower_location_conflict = lower2_location_conflict;
					    first_non_conf_location_time = lower_location_conflict->second.m_interval.m_upper;
					    break;
					}
				    }
				}
*/
				
			    }
			    
			    if (first_non_conf_location_time >= 0.0)
			    {
				if (wait_location_finish_time < 0.0)
				{
				    wait_location_finish_time = first_non_conf_location_time;
				    wait_location_culprit_conflict_id = first_location_culprit_conflict_id;
				}			    
				else
				{
				    if (wait_location_finish_time > first_non_conf_location_time)
				    {
					wait_location_finish_time = first_non_conf_location_time;
					wait_location_culprit_conflict_id = first_location_culprit_conflict_id;
				    }
				}
			    }
			    
			    UlinearConflicts_upper__map::const_iterator linear_Conflict = linear_Conflicts.find(Uline(front_respectful_transition.m_location_id, neighbor_location_id));
			    sDouble first_non_conf_linear_time = -1.0;
			    sInt_32 first_linear_culprit_conflict_id = -1;			    
			    
			    if (linear_Conflict != linear_Conflicts.end())
			    {
				UlinearConflicts_upper_map::const_iterator lower_linear_conflict = linear_Conflict->second.end();

				for (UlinearConflicts_upper_map::const_iterator lower2_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_respectful_transition.m_time,
																		     front_respectful_transition.m_time));
				     lower2_linear_conflict != linear_Conflict->second.end(); ++lower2_linear_conflict)
				{
				    if (lower2_linear_conflict->second.m_interval.m_upper > front_respectful_transition.m_time)
				    {
					lower_linear_conflict = lower2_linear_conflict;
					first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
					first_linear_culprit_conflict_id = lower_linear_conflict->second.m_conflict_id;					
					break;
				    }
				}
				
				/*
				UlinearConflicts_upper_map::const_iterator lower2_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				if (lower2_linear_conflict != linear_Conflict->second.end())
				{
				    for (UlinearConflicts_map::const_iterator lower2_linear_conflict = linear_Conflict->second.begin(); lower2_linear_conflict != linear_Conflict->second.end(); ++lower2_linear_conflict)
				    {
					if (lower2_linear_conflict->second.m_interval.m_upper > front_transition.m_time)
					{
					    lower_linear_conflict = lower2_linear_conflict;
					    first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
					    break;
					}
				    }
				}
				*/				
			    }

			    if (first_non_conf_linear_time >= 0.0)
			    {
				if (wait_linear_finish_time < 0.0)
				{
				    wait_linear_finish_time = first_non_conf_linear_time;
				    wait_linear_culprit_conflict_id = first_linear_culprit_conflict_id;
				}			    
				else
				{
				    if (wait_linear_finish_time > first_non_conf_linear_time)
				    {
					wait_linear_finish_time = first_non_conf_linear_time;
					wait_linear_culprit_conflict_id = first_linear_culprit_conflict_id;					
				    }
				}
			    }
			}
		    }
		    {
			sInt_32 neighbor_location_id = front_respectful_transition.m_location_id;			
			{
			    LocationConflicts_upper__umap::const_iterator location_Conflict = location_Conflicts.find(neighbor_location_id);
			    sDouble first_non_conf_location_time = -1.0;
			    sInt_32 first_location_culprit_conflict_id = -1;			    
			    
			    if (location_Conflict != location_Conflicts.end())
			    {
				LocationConflicts_upper_map::const_iterator lower_location_conflict = location_Conflict->second.end();

				for (LocationConflicts_upper_map::const_iterator lower2_location_conflict = location_Conflict->second.lower_bound(Interval(front_respectful_transition.m_time,
																			   front_respectful_transition.m_time));
				     lower2_location_conflict != location_Conflict->second.end(); ++lower2_location_conflict)
				{
				    if (lower2_location_conflict->second.m_interval.m_upper > front_respectful_transition.m_time)
				    {
					lower_location_conflict = lower2_location_conflict;
					first_non_conf_location_time = lower_location_conflict->second.m_interval.m_upper;
					first_location_culprit_conflict_id = lower_location_conflict->second.m_conflict_id;					
					break;
				    }
				}				

				/*
				LocationConflicts_upper_map::const_iterator lower2_location_conflict = location_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				if (lower2_location_conflict != location_Conflict->second.end())
				{
				    for (LocationConflicts_map::const_iterator lower2_location_conflict = location_Conflict->second.begin(); lower2_location_conflict != location_Conflict->second.end(); ++lower2_location_conflict)
				    {
					if (lower2_location_conflict->second.m_interval.m_upper > front_transition.m_time)
					{
					    lower_location_conflict = lower2_location_conflict;
					    first_non_conf_location_time = lower_location_conflict->second.m_interval.m_upper;
					    break;
					}
				    }
				}
				*/
			    }
			    
			    if (first_non_conf_location_time >= 0.0)
			    {
				if (wait_location_finish_time < 0.0)
				{
				    wait_location_finish_time = first_non_conf_location_time;
				    wait_location_culprit_conflict_id = first_location_culprit_conflict_id;				    
				}			    
				else
				{
				    if (wait_location_finish_time > first_non_conf_location_time)
				    {
					wait_location_finish_time = first_non_conf_location_time;
					wait_location_culprit_conflict_id = first_location_culprit_conflict_id;
				    }
				}
			    }
			    
			    UlinearConflicts_upper__map::const_iterator linear_Conflict = linear_Conflicts.find(Uline(front_respectful_transition.m_location_id, neighbor_location_id));
			    sDouble first_non_conf_linear_time = -1.0;
			    sInt_32 first_linear_culprit_conflict_id = -1;			    
			    
			    if (linear_Conflict != linear_Conflicts.end())
			    {
				UlinearConflicts_upper_map::const_iterator lower_linear_conflict = linear_Conflict->second.end();

				for (UlinearConflicts_upper_map::const_iterator lower2_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_respectful_transition.m_time,
																		     front_respectful_transition.m_time));
				     lower2_linear_conflict != linear_Conflict->second.end(); ++lower2_linear_conflict)
				{
				    if (lower2_linear_conflict->second.m_interval.m_upper > front_respectful_transition.m_time)
				    {
					lower_linear_conflict = lower2_linear_conflict;
					first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
					first_linear_culprit_conflict_id = lower_linear_conflict->second.m_conflict_id;					
					break;
				    }
				}

/*
				UlinearConflicts_upper_map::const_iterator lower2_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				if (lower2_linear_conflict != linear_Conflict->second.end())
				{
				    for (UlinearConflicts_map::const_iterator lower2_linear_conflict = linear_Conflict->second.begin(); lower2_linear_conflict != linear_Conflict->second.end(); ++lower2_linear_conflict)
				    {
					if (lower2_linear_conflict->second.m_interval.m_upper > front_transition.m_time)
					{
					    lower_linear_conflict = lower2_linear_conflict;
					    first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
					    break;
					}
				    }
				}
*/
			    }

			    if (first_non_conf_linear_time >= 0.0)
			    {
				if (wait_linear_finish_time < 0.0)
				{
				    wait_linear_finish_time = first_non_conf_linear_time;
				    wait_linear_culprit_conflict_id = first_linear_culprit_conflict_id;				    
				}			    
				else
				{
				    if (wait_linear_finish_time > first_non_conf_linear_time)
				    {
					wait_linear_finish_time = first_non_conf_linear_time;
					wait_linear_culprit_conflict_id = first_linear_culprit_conflict_id;					
				    }
				}
			    }
			}
		    }
		    
		    sDouble wait_finish_time;
		    sInt_32 wait_culprit_conflict_id;
		    
		    if (wait_location_finish_time >= 0.0)
		    {
			if (wait_linear_finish_time >= 0.0)
			{
			    if (wait_location_finish_time < wait_linear_finish_time)
			    {
				wait_finish_time = wait_location_finish_time;
				wait_culprit_conflict_id = wait_location_culprit_conflict_id;				
			    }
			    else
			    {
				wait_finish_time = wait_linear_finish_time;
				wait_culprit_conflict_id = wait_linear_culprit_conflict_id;				
			    }
			}
			else
			{
			    wait_finish_time = wait_location_finish_time;
			    wait_culprit_conflict_id = wait_location_culprit_conflict_id;			    
			}
		    }
		    else
		    {
			if (wait_linear_finish_time >= 0.0)
			{
			    wait_finish_time = wait_linear_finish_time;
			    wait_culprit_conflict_id = wait_linear_culprit_conflict_id;
			}
			else
			{
			    wait_finish_time = -1.0;
			    wait_culprit_conflict_id = -1;			    
			}
		    }
		    
		    if (wait_finish_time > front_respectful_transition.m_time + s_EPSILON)
		    {
			if (!is_UnifiedlyVisited(front_respectful_transition.m_location_id, wait_finish_time, unified_Visits))
			{
			    sDouble wait_cost = (wait_finish_time - front_respectful_transition.m_time) * kruhobot.m_properties.m_wait_factor;
			    
			    RespectfulTransition wait_respectful_transition(last_transition_id++,
									    wait_finish_time,
									    front_respectful_transition.m_cost + wait_cost,
									    wait_finish_time,
									    front_respectful_transition.m_location_id,
									    front_respectful_transition.m_trans_id);			    
			    sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(wait_respectful_transition.m_time);
			
			    wait_respectful_transition.m_conflict_fingerprint = front_respectful_transition.m_conflict_fingerprint;			
			    wait_respectful_transition.m_conflict_fingerprint.m_conflict_IDs.insert(wait_culprit_conflict_id);
			
			    RespectfulExplorations_map::iterator respectful_exploration = respectful_Explorations.find(wait_respectful_transition.m_conflict_fingerprint);
			
			    if (respectful_exploration == respectful_Explorations.end()) /* non-existent fingerprint */
			    {
				sDouble estimated_remaining = map.m_shortest_Distances[sink_loc_id][wait_respectful_transition.m_location_id] / kruhobot.m_properties.m_linear_velo;
				
				if (wait_respectful_transition.m_time + estimated_remaining <= makespan_bound + s_EPSILON)
				{			    
				    RespectfulVisits_umap extended_conflict_Visits;
				    RespectfulVisit wait_respectful_visit(wait_respectful_transition.m_time, wait_respectful_transition.m_trans_id);
				    
				    RespectfulVisits_umap::iterator wait_respectful_visit_iter = extended_conflict_Visits.insert(RespectfulVisits_umap::value_type(wait_respectful_transition.m_location_id,
																				   wait_respectful_visit)).first;
				    respectful_Explorations.insert(RespectfulExplorations_map::value_type(wait_respectful_transition.m_conflict_fingerprint, extended_conflict_Visits));
				    /*
				    RespectfulTransitions_mmap::iterator queue_iter = respectful_transition_Queue.insert(RespectfulTransitions_mmap::value_type(wait_respectful_transition.m_cost,
																				wait_respectful_transition));
				    */
				    RespectfulTransitions_mmap::iterator queue_iter = respectful_transition_Queue.insert(RespectfulTransitions_mmap::value_type(wait_respectful_transition.m_makespan,
																				wait_respectful_transition));
				    wait_respectful_visit_iter->second.m_queue_iter = queue_iter;
				}
				else
				{
				    sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(wait_respectful_transition.m_time + estimated_remaining);
				}
			    }
			    else /* existent fingerprint */
			    {			    
				RespectfulVisits_umap::iterator wait_respectful_visit = respectful_exploration->second.find(front_respectful_transition.m_location_id);
				
				if (wait_respectful_visit == respectful_exploration->second.end()) /* visiting for the first time */
				{
				    sDouble estimated_remaining = map.m_shortest_Distances[sink_loc_id][wait_respectful_transition.m_location_id] / kruhobot.m_properties.m_linear_velo;
				    
				    if (wait_respectful_transition.m_time + estimated_remaining <= makespan_bound + s_EPSILON)
				    {				
					RespectfulVisit wait_respectful_visit(wait_respectful_transition.m_time, wait_respectful_transition.m_trans_id);
					RespectfulVisits_umap::iterator wait_respectful_visit_iter = respectful_exploration->second.insert(RespectfulVisits_umap::value_type(wait_respectful_transition.m_location_id,
																					     wait_respectful_visit)).first;
					/*
					RespectfulTransitions_mmap::iterator queue_iter = respectful_transition_Queue.insert(RespectfulTransitions_mmap::value_type(wait_respectful_transition.m_cost,
																				    wait_respectful_transition));
					*/					
					RespectfulTransitions_mmap::iterator queue_iter = respectful_transition_Queue.insert(RespectfulTransitions_mmap::value_type(wait_respectful_transition.m_makespan,
																				    wait_respectful_transition));
					wait_respectful_visit_iter->second.m_queue_iter = queue_iter;
				    }
				    else
				    {
					sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(wait_respectful_transition.m_time + estimated_remaining);
				    }
				}
				else /* visiting for the next time */
				{
				    sASSERT(wait_respectful_visit != respectful_exploration->second.end());
				    
				    if (wait_respectful_visit->second.m_time > wait_respectful_transition.m_time) /* update neighbor */
				    {
                                        #ifdef sDEBUG
					{
					    sDouble estimated_remaining = map.m_shortest_Distances[sink_loc_id][wait_respectful_transition.m_location_id] / kruhobot.m_properties.m_linear_velo;
					    sASSERT(wait_respectful_transition.m_time + estimated_remaining <= makespan_bound + s_EPSILON);
					}
				        #endif
				
					wait_respectful_visit->second.m_time = wait_respectful_transition.m_time;
					wait_respectful_visit->second.m_trans_id = wait_respectful_transition.m_trans_id;
				    
					respectful_transition_Queue.erase(wait_respectful_visit->second.m_queue_iter);
					/*
					RespectfulTransitions_mmap::iterator queue_iter = respectful_transition_Queue.insert(RespectfulTransitions_mmap::value_type(wait_respectful_transition.m_cost,
																				    wait_respectful_transition));
					*/					
					RespectfulTransitions_mmap::iterator queue_iter = respectful_transition_Queue.insert(RespectfulTransitions_mmap::value_type(wait_respectful_transition.m_makespan,
																				    wait_respectful_transition));
					
					wait_respectful_visit->second.m_queue_iter = queue_iter;
				    }
				}			    		    
			    }
//			    unified_Visits[wait_respectful_transition.m_location_id].insert(wait_respectful_transition.m_time);
			}
		    }
		}
	    }
	    else
	    {
		sASSERT(false);
	    }
	    respectful_transition_Queue.erase(respectful_transition_Queue.begin());
	}
	
        #ifdef sDEBUG
	{
	    /*
	    for (UnifiedVisits_umap::const_iterator unified_visit = unified_Visits.begin(); unified_visit != unified_Visits.end(); ++unified_visit)
	    {
		printf("%d: ", unified_visit->first);
		for (VisitTimes_set::const_iterator visit_time = unified_visit->second.begin(); visit_time != unified_visit->second.end(); ++visit_time)
		{
		    printf("%.3f ", *visit_time);
		}
		printf("\n");
	    }
	    printf("%d: Fino queue: %ld (%ld)\n", kruhobot.m_id, respectful_transition_Queue.size(), unified_Visits.size());
	    */
//	    printf("RDD size: %ld\n", kruhobot_RDD.size());
	}
        #endif

	#ifdef sDEBUG
	{
	    sASSERT(sink_reached);
	}
        #endif

	augment_KruhobotRealDecisionDiagram(kruhobot, source_loc_id, sink_loc_id, makespan_bound, kruhobot_RDD, kruhobot_RDD_mapping);	
	interconnect_KruhobotRealDecisionDiagram_smart(kruhobot, map, kruhobot_RDD, kruhobot_RDD_mapping);	

	return next_makespan_bound;
    }


    sDouble sRealSMTCBS::build_KruhobotRealDecisionDiagram_conflictRespectfulBucketing(const sKruhobot                      &kruhobot,
										       const s2DMap                         &map,
										       sInt_32                               source_loc_id,
										       sInt_32                               sink_loc_id,
										       const LocationConflicts_upper__umap  &location_Conflicts,
										       const UlinearConflicts_upper__map     &linear_Conflicts,
										       sDouble                               makespan_bound,
										       KruhobotDecisionDiagram_vector       &kruhobot_RDD,
										       KruhobotDecisionMapping_map          &kruhobot_RDD_mapping) const
    {
        #ifdef sDEBUG
	bool sink_reached = false;
        #endif
	
	sInt_32 last_transition_id = 0;
	sDouble next_makespan_bound = -1.0;

	RespectfulExplorations_map respectful_Explorations;
	RespectfulExplorations_map respectful_Bypasses;	

	BucketedRespectfulTransitions_mmap bucketed_respectful_transition_Queues;

	RespectfulTransition initial_transition(last_transition_id++, 0.0, 0.0, 0.0, 0.0, source_loc_id, -1);
	initial_transition.m_prev_corr_dec_id = -1;

	UnifiedVisits_umap unified_Visits;
	SinkReachabilities_mmap sink_Reachabilities;
	
	bucketed_respectful_transition_Queues[initial_transition.m_conflict_fingerprint].insert(RespectfulTransitions_mmap::value_type(0.0, initial_transition));    
	{
	    RespectfulVisits_umap empty_conflict_Visits;
	    RespectfulVisit respectful_visit(0.0, initial_transition.m_trans_id);
	    respectful_visit.m_queue_iter = bucketed_respectful_transition_Queues[initial_transition.m_conflict_fingerprint].begin();
		
	    empty_conflict_Visits.insert(RespectfulVisits_umap::value_type(initial_transition.m_location_id, respectful_visit));
	    respectful_Explorations.insert(RespectfulExplorations_map::value_type(initial_transition.m_conflict_fingerprint, empty_conflict_Visits));
	    
	    //unified_Visits[initial_transition.m_location_id].insert(0.0);

	    if (sink_Reachabilities.find(initial_transition.m_conflict_fingerprint) == sink_Reachabilities.end())
	    {
		sink_Reachabilities[initial_transition.m_conflict_fingerprint] = -1.0;
	    }
	}

	while (!bucketed_respectful_transition_Queues.empty())
	{
	    RespectfulTransitions_mmap &front_respectful_transition_Queue = bucketed_respectful_transition_Queues.begin()->second;

            #if defined(sDEBUG) && defined(sVERBOSE)
	    {
		sInt_32 cumulative_queue_size = 0;

		for (BucketedRespectfulTransitions_mmap::const_iterator bucketed_Queue = bucketed_respectful_transition_Queues.begin(); bucketed_Queue != bucketed_respectful_transition_Queues.end(); ++bucketed_Queue)
		{
		    cumulative_queue_size +=  bucketed_Queue->second.size();
		}		
	    }
	    #endif
	    
            #if defined(sDEBUG) && defined(sVERBOSE)
	    {
		/*
		for (BucketedRespectfulTransitions_mmap::const_iterator bucketed_Queue = bucketed_respectful_transition_Queues.begin(); bucketed_Queue != bucketed_respectful_transition_Queues.end(); ++bucketed_Queue)
		{
		    bucketed_Queue->first.to_Screen();
		    printf(" - size: %ld\n", bucketed_Queue->second.size());
		}
		*/
	    }
	    #endif

	    bool clear_after = false;
	    ConflictFingerprint clear_conflict_fingerprint;
	    if (!front_respectful_transition_Queue.empty())
	    {
		clear_conflict_fingerprint = front_respectful_transition_Queue.begin()->second.m_conflict_fingerprint;
		clear_after = true;
	    }	    
	    while (!front_respectful_transition_Queue.empty())
	    {
		const RespectfulTransition &front_respectful_transition = front_respectful_transition_Queue.begin()->second;
		sDouble effective_makespan_bound = makespan_bound;

		if (front_respectful_transition.m_time + (map.m_shortest_Distances[sink_loc_id][front_respectful_transition.m_location_id] / kruhobot.m_properties.m_linear_velo) <= effective_makespan_bound + s_EPSILON)
		{
		    if (sink_loc_id == front_respectful_transition.m_location_id)
		    {
                        #ifdef sDEBUG
			{
			    sink_reached = true;
			}
			#endif

			SinkReachabilities_mmap::iterator sink_reachability;
			if ((sink_reachability = sink_Reachabilities.find(front_respectful_transition.m_conflict_fingerprint)) != sink_Reachabilities.end())
			{
			    if (sink_reachability->second < 0.0)
			    {
				sink_reachability->second = front_respectful_transition.m_time;
			    }
			    else
			    {
				sink_reachability->second = (front_respectful_transition.m_time < sink_reachability->second) ? sink_reachability->second : front_respectful_transition.m_time;
			    }
			}
			else
			{
			    sink_Reachabilities[front_respectful_transition.m_conflict_fingerprint] = front_respectful_transition.m_time;
			}
		    }
		
		    sInt_32 front_kruhobot_decision_id = kruhobot_RDD.size();
		    
		    if (!is_UnifiedlyVisited(front_respectful_transition.m_location_id, front_respectful_transition.m_time, unified_Visits))
		    {
			kruhobot_RDD.push_back(KruhobotDecision(front_kruhobot_decision_id,
								front_respectful_transition.m_time,
								front_respectful_transition.m_location_id,
								front_respectful_transition.m_prev_corr_dec_id));	      
			kruhobot_RDD_mapping[front_respectful_transition.m_location_id].insert(KruhobotDecisionIDs_mmap::value_type(front_respectful_transition.m_time, front_kruhobot_decision_id));
			unified_Visits[front_respectful_transition.m_location_id].insert(front_respectful_transition.m_time);		    
		    }
		    const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[front_respectful_transition.m_location_id].m_Neighbors;
		    for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
		    {
			sInt_32 neighbor_location_id = (*neighbor)->m_target->m_id;
			{
			    sDouble transition_distance = map.m_straight_Distances[front_respectful_transition.m_location_id][neighbor_location_id];
			    sDouble transition_delta_time = transition_distance / kruhobot.m_properties.m_linear_velo;
			    sDouble transition_finish_time = front_respectful_transition.m_time + transition_delta_time;
			    sDouble transition_finish_cost = front_respectful_transition.m_cost + transition_delta_time;		
			    sDouble transition_finish_makespan = front_respectful_transition.m_makespan + transition_delta_time;
			    sDouble transition_waited = front_respectful_transition.m_waited;

			    if (!is_TransitionConflicting(front_respectful_transition.m_location_id,
							 neighbor_location_id,
							 front_respectful_transition.m_time,
							 transition_finish_time,
							 location_Conflicts,
							 linear_Conflicts,
							 front_respectful_transition.m_conflict_fingerprint))
			    {
				if (!is_UnifiedlyVisited(neighbor_location_id, transition_finish_time, unified_Visits))
				{
				    RespectfulExplorations_map::iterator respectful_exploration = respectful_Explorations.find(front_respectful_transition.m_conflict_fingerprint);
				    sASSERT(respectful_exploration != respectful_Explorations.end());
				    
				    RespectfulVisits_umap::iterator next_respectful_visit = respectful_exploration->second.find(neighbor_location_id);
				    
				    RespectfulTransition neighbor_respectful_transition(last_transition_id++,
											transition_finish_time,
											transition_finish_cost,
											transition_finish_makespan,
											transition_waited,
											neighbor_location_id,
											front_respectful_transition.m_trans_id);
				    neighbor_respectful_transition.m_prev_corr_dec_id = front_kruhobot_decision_id;
				    neighbor_respectful_transition.m_conflict_fingerprint = front_respectful_transition.m_conflict_fingerprint;
				    sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(neighbor_respectful_transition.m_time);

				    if (next_respectful_visit == respectful_exploration->second.end()) /* visiting for the first time in a given fingerprint */
				    {
					sDouble estimated_remaining = map.m_shortest_Distances[sink_loc_id][neighbor_location_id] / kruhobot.m_properties.m_linear_velo;
					sDouble effective_makespan_bound = makespan_bound;

					if (neighbor_respectful_transition.m_time + estimated_remaining <= effective_makespan_bound + s_EPSILON)
					{
					    RespectfulVisit neighbor_respectful_visit(neighbor_respectful_transition.m_time, neighbor_respectful_transition.m_trans_id);

					    RespectfulVisits_umap::iterator neighbor_respectful_visit_iter = respectful_exploration->second.insert(RespectfulVisits_umap::value_type(neighbor_location_id,
																						     neighbor_respectful_visit)).first;
					    RespectfulTransitions_mmap::iterator queue_iter = bucketed_respectful_transition_Queues[neighbor_respectful_transition.m_conflict_fingerprint].insert(RespectfulTransitions_mmap::value_type(neighbor_respectful_transition.m_makespan,
																													 neighbor_respectful_transition));
					    neighbor_respectful_visit_iter->second.m_queue_iter = queue_iter;
					}
					else
					{
					    sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(neighbor_respectful_transition.m_time + estimated_remaining);
					}
				    }
				    else /* visiting for the next time in a given fingerprint */
				    {
					sASSERT(next_respectful_visit != respectful_exploration->second.end());
					
					if (next_respectful_visit->second.m_time > neighbor_respectful_transition.m_time + s_EPSILON) /* update neighbor */
					{
					    next_respectful_visit->second.m_time = neighbor_respectful_transition.m_time;
					    next_respectful_visit->second.m_trans_id = neighbor_respectful_transition.m_trans_id;
					    
					    bucketed_respectful_transition_Queues[neighbor_respectful_transition.m_conflict_fingerprint].erase(next_respectful_visit->second.m_queue_iter);
					    RespectfulTransitions_mmap::iterator queue_iter = bucketed_respectful_transition_Queues[neighbor_respectful_transition.m_conflict_fingerprint].insert(RespectfulTransitions_mmap::value_type(neighbor_respectful_transition.m_makespan,
																													 neighbor_respectful_transition));
					    next_respectful_visit->second.m_queue_iter = queue_iter;
					}
				    }
				}
			    }
			}				
		    }
		    {
			sDouble wait_location_finish_time = -1.0;
			sInt_32 wait_location_culprit_conflict_id = -1;
			
			sDouble wait_linear_finish_time = -1.0;
			sInt_32 wait_linear_culprit_conflict_id = -1;		    

			const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[front_respectful_transition.m_location_id].m_Neighbors;
			for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
			{
			    sInt_32 neighbor_location_id = (*neighbor)->m_target->m_id;
			    {
				LocationConflicts_upper__umap::const_iterator location_Conflict = location_Conflicts.find(neighbor_location_id);
				sDouble first_non_conf_location_time = -1.0;
				sInt_32 first_location_culprit_conflict_id = -1;
				
				if (location_Conflict != location_Conflicts.end())
				{
				    LocationConflicts_upper_map::const_iterator lower_location_conflict = location_Conflict->second.end();
				    
				    for (LocationConflicts_upper_map::const_iterator lower2_location_conflict = location_Conflict->second.lower_bound(Interval(front_respectful_transition.m_time,
																			       front_respectful_transition.m_time));
					 lower2_location_conflict != location_Conflict->second.end(); ++lower2_location_conflict)
				    {
					if (lower2_location_conflict->second.m_interval.m_upper > front_respectful_transition.m_time)
					{
					    lower_location_conflict = lower2_location_conflict;
					    first_non_conf_location_time = lower_location_conflict->second.m_interval.m_upper;
					    first_location_culprit_conflict_id = lower_location_conflict->second.m_conflict_id;
					    break;
					}				    
				    }
				}
				
				if (first_non_conf_location_time >= 0.0)
				{
				    if (wait_location_finish_time < 0.0)
				    {
					wait_location_finish_time = first_non_conf_location_time;
					wait_location_culprit_conflict_id = first_location_culprit_conflict_id;
				    }			    
				    else
				    {
					if (wait_location_finish_time > first_non_conf_location_time)
					{
					    wait_location_finish_time = first_non_conf_location_time;
					    wait_location_culprit_conflict_id = first_location_culprit_conflict_id;
					}
				    }
				}
				
				UlinearConflicts_upper__map::const_iterator linear_Conflict = linear_Conflicts.find(Uline(front_respectful_transition.m_location_id, neighbor_location_id));
				sDouble first_non_conf_linear_time = -1.0;
				sInt_32 first_linear_culprit_conflict_id = -1;
				
				if (linear_Conflict != linear_Conflicts.end())
				{
				    UlinearConflicts_upper_map::const_iterator lower_linear_conflict = linear_Conflict->second.end();
				    
				    for (UlinearConflicts_upper_map::const_iterator lower2_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_respectful_transition.m_time,
																			 front_respectful_transition.m_time));
					 lower2_linear_conflict != linear_Conflict->second.end(); ++lower2_linear_conflict)
				    {
					if (lower2_linear_conflict->second.m_interval.m_upper > front_respectful_transition.m_time)
					{
					    lower_linear_conflict = lower2_linear_conflict;
					    first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
					    first_linear_culprit_conflict_id = lower_linear_conflict->second.m_conflict_id;					
					    break;
					}
				    }
				}

				if (first_non_conf_linear_time >= 0.0)
				{
				    if (wait_linear_finish_time < 0.0)
				    {
					wait_linear_finish_time = first_non_conf_linear_time;
					wait_linear_culprit_conflict_id = first_linear_culprit_conflict_id;
				    }			    
				    else
				    {
					if (wait_linear_finish_time > first_non_conf_linear_time)
					{
					    wait_linear_finish_time = first_non_conf_linear_time;
					    wait_linear_culprit_conflict_id = first_linear_culprit_conflict_id;					
					}
				    }
				}
			    }
			}
			{
			    sInt_32 neighbor_location_id = front_respectful_transition.m_location_id;			
			    {
				LocationConflicts_upper__umap::const_iterator location_Conflict = location_Conflicts.find(neighbor_location_id);
				sDouble first_non_conf_location_time = -1.0;
				sInt_32 first_location_culprit_conflict_id = -1;			    
				
				if (location_Conflict != location_Conflicts.end())
				{
				    LocationConflicts_upper_map::const_iterator lower_location_conflict = location_Conflict->second.end();

				    for (LocationConflicts_upper_map::const_iterator lower2_location_conflict = location_Conflict->second.lower_bound(Interval(front_respectful_transition.m_time,
																			       front_respectful_transition.m_time));
					 lower2_location_conflict != location_Conflict->second.end(); ++lower2_location_conflict)
				    {
					if (lower2_location_conflict->second.m_interval.m_upper > front_respectful_transition.m_time)
					{
					    lower_location_conflict = lower2_location_conflict;
					    first_non_conf_location_time = lower_location_conflict->second.m_interval.m_upper;
					    first_location_culprit_conflict_id = lower_location_conflict->second.m_conflict_id;					
					    break;
					}
				    }				
				}
				
				if (first_non_conf_location_time >= 0.0)
				{
				    if (wait_location_finish_time < 0.0)
				    {
					wait_location_finish_time = first_non_conf_location_time;
					wait_location_culprit_conflict_id = first_location_culprit_conflict_id;				    
				    }			    
				    else
				    {
					if (wait_location_finish_time > first_non_conf_location_time)
					{
					    wait_location_finish_time = first_non_conf_location_time;
					    wait_location_culprit_conflict_id = first_location_culprit_conflict_id;
					}
				    }
				}

				UlinearConflicts_upper__map::const_iterator linear_Conflict = linear_Conflicts.find(Uline(front_respectful_transition.m_location_id, neighbor_location_id));
				sDouble first_non_conf_linear_time = -1.0;
				sInt_32 first_linear_culprit_conflict_id = -1;			    
			    
				if (linear_Conflict != linear_Conflicts.end())
				{
				    UlinearConflicts_upper_map::const_iterator lower_linear_conflict = linear_Conflict->second.end();
				    
				    for (UlinearConflicts_upper_map::const_iterator lower2_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_respectful_transition.m_time,
																			 front_respectful_transition.m_time));
					 lower2_linear_conflict != linear_Conflict->second.end(); ++lower2_linear_conflict)
				    {
					if (lower2_linear_conflict->second.m_interval.m_upper > front_respectful_transition.m_time)
					{
					    lower_linear_conflict = lower2_linear_conflict;
					    first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
					    first_linear_culprit_conflict_id = lower_linear_conflict->second.m_conflict_id;					
					    break;
					}
				    }
				}
				
				if (first_non_conf_linear_time >= 0.0)
				{
				    if (wait_linear_finish_time < 0.0)
				    {
					wait_linear_finish_time = first_non_conf_linear_time;
					wait_linear_culprit_conflict_id = first_linear_culprit_conflict_id;				    
				    }			    
				    else
				    {
					if (wait_linear_finish_time > first_non_conf_linear_time)
					{
					    wait_linear_finish_time = first_non_conf_linear_time;
					    wait_linear_culprit_conflict_id = first_linear_culprit_conflict_id;					
					}
				    }
				}
			    }
			}
			sDouble wait_finish_time;
			sInt_32 wait_culprit_conflict_id;
			
			if (wait_location_finish_time >= 0.0)
			{
			    if (wait_linear_finish_time >= 0.0)
			    {
				if (wait_location_finish_time < wait_linear_finish_time)
				{
				    wait_finish_time = wait_location_finish_time;
				    wait_culprit_conflict_id = wait_location_culprit_conflict_id;				
				}
				else
				{
				    wait_finish_time = wait_linear_finish_time;
				    wait_culprit_conflict_id = wait_linear_culprit_conflict_id;				
				}
			    }
			    else
			    {
				wait_finish_time = wait_location_finish_time;
				wait_culprit_conflict_id = wait_location_culprit_conflict_id;			    
			    }
			}
			else
			{
			    if (wait_linear_finish_time >= 0.0)
			    {
				wait_finish_time = wait_linear_finish_time;
				wait_culprit_conflict_id = wait_linear_culprit_conflict_id;
			    }
			    else
			    {
				wait_finish_time = -1.0;
				wait_culprit_conflict_id = -1;			    
			    }
			}
			if (wait_finish_time > front_respectful_transition.m_time + s_EPSILON)
			{			    
			    /* bypass */
			    const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[front_respectful_transition.m_location_id].m_Neighbors;
			    for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
			    {
				sInt_32 neighbor_location_id = (*neighbor)->m_target->m_id;
				{
				    sDouble bypass_transition_distance = map.m_straight_Distances[front_respectful_transition.m_location_id][neighbor_location_id];
				    sDouble bypass_transition_delta_time = bypass_transition_distance / kruhobot.m_properties.m_linear_velo;
				    sDouble bypass_transition_finish_time = front_respectful_transition.m_time + bypass_transition_delta_time;
				    sDouble bypass_transition_finish_cost = front_respectful_transition.m_cost + bypass_transition_delta_time;		
				    sDouble bypass_transition_finish_makespan = front_respectful_transition.m_makespan + bypass_transition_delta_time;
				    sDouble bypass_transition_waited = front_respectful_transition.m_waited;

				    RespectfulTransition bypass_respectful_transition(last_transition_id++,
										      bypass_transition_finish_time,
										      bypass_transition_finish_cost,
										      bypass_transition_finish_makespan,
										      bypass_transition_waited,
										      neighbor_location_id,
										      front_respectful_transition.m_trans_id);
				    bypass_respectful_transition.m_prev_corr_dec_id = front_kruhobot_decision_id;
				    bypass_respectful_transition.m_conflict_fingerprint = front_respectful_transition.m_conflict_fingerprint;
				    bypass_respectful_transition.m_conflict_fingerprint.m_conflict_IDs.insert(wait_culprit_conflict_id);				    
				    
				    if (!is_TransitionConflicting(front_respectful_transition.m_location_id,
								  neighbor_location_id,
								  front_respectful_transition.m_time,
								  bypass_transition_finish_time,
								  location_Conflicts,
								  linear_Conflicts,
								  bypass_respectful_transition.m_conflict_fingerprint))
				    {
					if (!is_UnifiedlyVisited(neighbor_location_id, bypass_transition_finish_time, unified_Visits))
					{				
					    if (front_respectful_transition.m_conflict_fingerprint.m_conflict_IDs.find(wait_culprit_conflict_id) == front_respectful_transition.m_conflict_fingerprint.m_conflict_IDs.end())
					    {						
						sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(bypass_respectful_transition.m_time);
						
						RespectfulExplorations_map::iterator respectful_exploration = respectful_Explorations.find(bypass_respectful_transition.m_conflict_fingerprint);
						if (respectful_exploration == respectful_Explorations.end())
						{
						    RespectfulVisits_umap extended_conflict_Visits;
						    RespectfulVisit bypass_respectful_visit(bypass_respectful_transition.m_time, bypass_respectful_transition.m_trans_id);
						    
						    RespectfulVisits_umap::iterator bypass_respectful_visit_iter = extended_conflict_Visits.insert(RespectfulVisits_umap::value_type(bypass_respectful_transition.m_location_id,
																						     bypass_respectful_visit)).first;
						    RespectfulTransitions_mmap::iterator queue_iter = bucketed_respectful_transition_Queues[bypass_respectful_transition.m_conflict_fingerprint].insert(RespectfulTransitions_mmap::value_type(bypass_respectful_transition.m_makespan, bypass_respectful_transition));
						    bypass_respectful_visit_iter->second.m_queue_iter = queue_iter;
						    
						    respectful_Explorations.insert(RespectfulExplorations_map::value_type(bypass_respectful_transition.m_conflict_fingerprint, extended_conflict_Visits));
						    
						    if (sink_Reachabilities.find(bypass_respectful_transition.m_conflict_fingerprint) == sink_Reachabilities.end())
						    {
							sink_Reachabilities[bypass_respectful_transition.m_conflict_fingerprint] = -1.0;
						    }											
						}
						else
						{
						    RespectfulVisits_umap::iterator next_respectful_visit = respectful_exploration->second.find(bypass_respectful_transition.m_location_id);
						    
						    if (next_respectful_visit == respectful_exploration->second.end()) /* visiting for the first time in a given fingerprint */
						    {
							RespectfulVisit bypass_respectful_visit(bypass_respectful_transition.m_time, bypass_respectful_transition.m_trans_id);
							
							RespectfulVisits_umap::iterator bypass_respectful_visit_iter = respectful_exploration->second.insert(RespectfulVisits_umap::value_type(bypass_respectful_transition.m_location_id,
																							       bypass_respectful_visit)).first;
							RespectfulTransitions_mmap::iterator queue_iter = bucketed_respectful_transition_Queues[bypass_respectful_transition.m_conflict_fingerprint].insert(RespectfulTransitions_mmap::value_type(bypass_respectful_transition.m_makespan,
																														   bypass_respectful_transition));
							bypass_respectful_visit_iter->second.m_queue_iter = queue_iter;
							
							if (sink_Reachabilities.find(bypass_respectful_transition.m_conflict_fingerprint) == sink_Reachabilities.end())
							{
							    sink_Reachabilities[bypass_respectful_transition.m_conflict_fingerprint] = -1.0;
							}											
						    }
						    else /* visiting next time */
						    {
							sASSERT(next_respectful_visit != respectful_exploration->second.end());
														
							if (next_respectful_visit->second.m_time > bypass_respectful_transition.m_time)
							{
							    next_respectful_visit->second.m_time = bypass_respectful_transition.m_time;
							    next_respectful_visit->second.m_trans_id = bypass_respectful_transition.m_trans_id;
							    
							    bucketed_respectful_transition_Queues[bypass_respectful_transition.m_conflict_fingerprint].erase(next_respectful_visit->second.m_queue_iter);
							    RespectfulTransitions_mmap::iterator queue_iter = bucketed_respectful_transition_Queues[bypass_respectful_transition.m_conflict_fingerprint].insert(RespectfulTransitions_mmap::value_type(bypass_respectful_transition.m_makespan,
																														       bypass_respectful_transition));
							    
							    next_respectful_visit->second.m_queue_iter = queue_iter;
							}
						    }
						}
					    }
					}
				    }
				}
			    }
			    /* waiting */
			    if (!is_UnifiedlyVisited(front_respectful_transition.m_location_id, wait_finish_time, unified_Visits))
			    {
				if (front_respectful_transition.m_conflict_fingerprint.m_conflict_IDs.find(wait_culprit_conflict_id) == front_respectful_transition.m_conflict_fingerprint.m_conflict_IDs.end())
				{					
				    sDouble waited = wait_finish_time - front_respectful_transition.m_time;
				    sDouble wait_cost = waited * kruhobot.m_properties.m_wait_factor;
				
				    RespectfulTransition wait_respectful_transition(last_transition_id++,
										    wait_finish_time,
										    front_respectful_transition.m_cost + wait_cost,
										    wait_finish_time,
										    front_respectful_transition.m_waited + waited,
										    front_respectful_transition.m_location_id,
										    front_respectful_transition.m_trans_id);
				    sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(wait_respectful_transition.m_time);
				    
				    wait_respectful_transition.m_conflict_fingerprint = front_respectful_transition.m_conflict_fingerprint;			
				    wait_respectful_transition.m_conflict_fingerprint.m_conflict_IDs.insert(wait_culprit_conflict_id);
				    
				    RespectfulExplorations_map::iterator respectful_exploration = respectful_Explorations.find(wait_respectful_transition.m_conflict_fingerprint);

				    if (respectful_exploration == respectful_Explorations.end()) /* non-existent fingerprint */
				    {
					sDouble estimated_remaining = map.m_shortest_Distances[sink_loc_id][wait_respectful_transition.m_location_id] / kruhobot.m_properties.m_linear_velo;
					sDouble effective_makespan_bound = makespan_bound;
					
					if (wait_respectful_transition.m_time + estimated_remaining <= effective_makespan_bound + s_EPSILON)
					{
					    RespectfulVisits_umap extended_conflict_Visits;
					    RespectfulVisit wait_respectful_visit(wait_respectful_transition.m_time, wait_respectful_transition.m_trans_id);
					    
					    RespectfulVisits_umap::iterator wait_respectful_visit_iter = extended_conflict_Visits.insert(RespectfulVisits_umap::value_type(wait_respectful_transition.m_location_id,
																					   wait_respectful_visit)).first;
					    RespectfulTransitions_mmap::iterator queue_iter = bucketed_respectful_transition_Queues[wait_respectful_transition.m_conflict_fingerprint].insert(RespectfulTransitions_mmap::value_type(wait_respectful_transition.m_makespan,
																												     wait_respectful_transition));
					    wait_respectful_visit_iter->second.m_queue_iter = queue_iter;

					    respectful_Explorations.insert(RespectfulExplorations_map::value_type(wait_respectful_transition.m_conflict_fingerprint, extended_conflict_Visits));

					    if (sink_Reachabilities.find(wait_respectful_transition.m_conflict_fingerprint) == sink_Reachabilities.end())
					    {
						sink_Reachabilities[wait_respectful_transition.m_conflict_fingerprint] = -1.0;
					    }					    
					}
					else
					{
					    sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(wait_respectful_transition.m_time + estimated_remaining);
					}
				    }
				    else /* existent fingerprint */
				    {
					RespectfulVisits_umap::iterator wait_respectful_visit = respectful_exploration->second.find(front_respectful_transition.m_location_id);
				    
					if (wait_respectful_visit == respectful_exploration->second.end()) /* visiting for the first time */
					{
					    sDouble estimated_remaining = map.m_shortest_Distances[sink_loc_id][wait_respectful_transition.m_location_id] / kruhobot.m_properties.m_linear_velo;
					    sDouble effective_makespan_bound = makespan_bound;

					    if (wait_respectful_transition.m_time + estimated_remaining <= effective_makespan_bound + s_EPSILON)
					    {
						RespectfulVisit wait_respectful_visit(wait_respectful_transition.m_time, wait_respectful_transition.m_trans_id);
						RespectfulVisits_umap::iterator wait_respectful_visit_iter = respectful_exploration->second.insert(RespectfulVisits_umap::value_type(wait_respectful_transition.m_location_id,
																						     wait_respectful_visit)).first;
						RespectfulTransitions_mmap::iterator queue_iter = bucketed_respectful_transition_Queues[wait_respectful_transition.m_conflict_fingerprint].insert(RespectfulTransitions_mmap::value_type(wait_respectful_transition.m_makespan,
																													 wait_respectful_transition));
						wait_respectful_visit_iter->second.m_queue_iter = queue_iter;

						if (sink_Reachabilities.find(wait_respectful_transition.m_conflict_fingerprint) == sink_Reachabilities.end())
						{
						    sink_Reachabilities[wait_respectful_transition.m_conflict_fingerprint] = -1.0;
						}			
					    }
					    else
					    {
						sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(wait_respectful_transition.m_time + estimated_remaining);
					    }
					}
					else /* visiting for the next time, almost same as first time */
					{
					    sASSERT(wait_respectful_visit != respectful_exploration->second.end());
					    
					    if (wait_respectful_visit->second.m_time > wait_respectful_transition.m_time)
					    {
						wait_respectful_visit->second.m_time = wait_respectful_transition.m_time;
						wait_respectful_visit->second.m_trans_id = wait_respectful_transition.m_trans_id;

						bucketed_respectful_transition_Queues[wait_respectful_transition.m_conflict_fingerprint].erase(wait_respectful_visit->second.m_queue_iter);
						RespectfulTransitions_mmap::iterator queue_iter = bucketed_respectful_transition_Queues[wait_respectful_transition.m_conflict_fingerprint].insert(RespectfulTransitions_mmap::value_type(wait_respectful_transition.m_makespan,
																													 wait_respectful_transition));
						wait_respectful_visit->second.m_queue_iter = queue_iter;

						if (sink_Reachabilities.find(wait_respectful_transition.m_conflict_fingerprint) == sink_Reachabilities.end())
						{
						    sink_Reachabilities[wait_respectful_transition.m_conflict_fingerprint] = -1.0;
						}						
					    }
					}			    		    
				    }
				}
			    }
			}
		    }
		}
		front_respectful_transition_Queue.erase(front_respectful_transition_Queue.begin());
	    }
	    if (clear_after)
	    {
		RespectfulExplorations_map::iterator respectful_exploration = respectful_Explorations.find(clear_conflict_fingerprint);
		respectful_exploration->second.clear();
	    }
	
            #ifdef sDEBUG
	    {
		/*
		for (UnifiedVisits_umap::const_iterator unified_visit = unified_Visits.begin(); unified_visit != unified_Visits.end(); ++unified_visit)
		{
		    printf("%d: ", unified_visit->first);
		    for (VisitTimes_set::const_iterator visit_time = unified_visit->second.begin(); visit_time != unified_visit->second.end(); ++visit_time)
		    {
			printf("%.3f ", *visit_time);
		    }
		    printf("\n");
		}
		printf("%d: Fino queue: %ld (%ld)\n", kruhobot.m_id, front_respectful_transition_Queue.size(), unified_Visits.size());
		*/
		//printf("RDD size: %ld\n", kruhobot_RDD.size());
	    }
            #endif

	    #ifdef sDEBUG
	    {
		sASSERT(sink_reached);
	    }
            #endif

            #if defined(sDEBUG) && defined(sVERBOSE)
	    {
		/*
		for (BucketedRespectfulTransitions_mmap::const_iterator bucketed_Queue = bucketed_respectful_transition_Queues.begin(); bucketed_Queue != bucketed_respectful_transition_Queues.end(); ++bucketed_Queue)
		{
		    bucketed_Queue->first.to_Screen();
		    printf(" - size: %ld\n", bucketed_Queue->second.size());
		}
		*/
	    }
	    #endif	    
    
	    while (!bucketed_respectful_transition_Queues.empty() && bucketed_respectful_transition_Queues.begin()->second.empty())
	    {
		bucketed_respectful_transition_Queues.erase(bucketed_respectful_transition_Queues.begin());
	    }
	}
	augment_KruhobotRealDecisionDiagram(kruhobot, source_loc_id, sink_loc_id, makespan_bound, kruhobot_RDD, kruhobot_RDD_mapping);	
	interconnect_KruhobotRealDecisionDiagram_smart(kruhobot, map, kruhobot_RDD, kruhobot_RDD_mapping);

	#ifdef sDEBUG
	{
	    /*
	    printf("Sink reachability:\n");
	    for (SinkReachabilities_mmap::const_iterator sink_reachability = sink_Reachabilities.begin(); sink_reachability != sink_Reachabilities.end(); ++sink_reachability)
	    {
		sink_reachability->first.to_Screen();
		printf(": %.3f\n", sink_reachability->second);
	    }
	    */
	}
	#endif
	return next_makespan_bound;	
    }


    sDouble sRealSMTCBS::build_KruhobotRealDecisionDiagram_individualizedConflictRespectfulBucketing(const sKruhobot                      &kruhobot,
												     const s2DMap                         &map,
												     sInt_32                               source_loc_id,
												     sInt_32                               sink_loc_id,
												     const LocationConflicts_upper__umap  &location_Conflicts,
												     const UlinearConflicts_upper__map    &linear_Conflicts,
												     sDouble                               makespan_bound,
												     sDouble                              &individual_makespan_bound,
												     sInt_32                               fingerprint_limit,
												     KruhobotDecisionDiagram_vector       &kruhobot_RDD,
												     KruhobotDecisionMapping_map          &kruhobot_RDD_mapping) const
    {
        #ifdef sDEBUG
	bool sink_reached = false;
	sInt_32 processed_nodes = 0;
        #endif	
	
	sInt_32 last_transition_id = 0;
	sDouble next_makespan_bound = -1.0;

	sDouble next_individual_makespan_bound = individual_makespan_bound;

	RespectfulExplorations_map respectful_Explorations;
	RespectfulExplorations_map respectful_Bypasses;	

	BucketedRespectfulTransitions_mmap bucketed_respectful_transition_Queues;

	RespectfulTransition initial_transition(last_transition_id++, 0.0, 0.0, 0.0, 0.0, source_loc_id, -1);
	initial_transition.m_prev_corr_dec_id = -1;

	UnifiedVisits_umap unified_Visits;
	SinkReachabilities_mmap sink_Reachabilities;

	bucketed_respectful_transition_Queues[initial_transition.m_conflict_fingerprint].insert(RespectfulTransitions_mmap::value_type(0.0, initial_transition));    
	{
	    RespectfulVisits_umap empty_conflict_Visits;
	    RespectfulVisit respectful_visit(0.0, initial_transition.m_trans_id);
	    respectful_visit.m_queue_iter = bucketed_respectful_transition_Queues[initial_transition.m_conflict_fingerprint].begin();
		
	    empty_conflict_Visits.insert(RespectfulVisits_umap::value_type(initial_transition.m_location_id, respectful_visit));
	    respectful_Explorations.insert(RespectfulExplorations_map::value_type(initial_transition.m_conflict_fingerprint, empty_conflict_Visits));
	    
	    //unified_Visits[initial_transition.m_location_id].insert(0.0);

	    if (sink_Reachabilities.find(initial_transition.m_conflict_fingerprint) == sink_Reachabilities.end())
	    {
		sink_Reachabilities[initial_transition.m_conflict_fingerprint] = -1.0;
	    }
	}

	while (!bucketed_respectful_transition_Queues.empty())
	{
	    RespectfulTransitions_mmap &front_respectful_transition_Queue = bucketed_respectful_transition_Queues.begin()->second;
/*
            #if defined(sDEBUG) && defined(sVERBOSE)
	    {
		for (BucketedRespectfulTransitions_mmap::const_iterator bucketed_Queue = bucketed_respectful_transition_Queues.begin(); bucketed_Queue != bucketed_respectful_transition_Queues.end(); ++bucketed_Queue)
		{
		    bucketed_Queue->first.to_Screen();
		    printf(" - size: %ld\n", bucketed_Queue->second.size());
		}
	    }
	    #endif
	    
            #if defined(sDEBUG) && defined(sVERBOSE)
	    {
		sInt_32 cumulative_queue_size = 0;

		for (BucketedRespectfulTransitions_mmap::const_iterator bucketed_Queue = bucketed_respectful_transition_Queues.begin(); bucketed_Queue != bucketed_respectful_transition_Queues.end(); ++bucketed_Queue)
		{
		    cumulative_queue_size +=  bucketed_Queue->second.size();
		}
		printf("Cumulative size: %d\n", cumulative_queue_size);
	    }
	    #endif	    
*/
	    bool clear_after = false;
	    ConflictFingerprint clear_conflict_fingerprint;
	    if (!front_respectful_transition_Queue.empty())
	    {
		clear_conflict_fingerprint = front_respectful_transition_Queue.begin()->second.m_conflict_fingerprint;
		clear_after = true;
	    }	    
	    while (!front_respectful_transition_Queue.empty())
	    {
		#ifdef sDEBUG
		{
		    ++processed_nodes;
		}
		#endif

		const RespectfulTransition &front_respectful_transition = front_respectful_transition_Queue.begin()->second;

		sASSERT(individual_makespan_bound >= 0.0);
		sDouble effective_makespan_bound = sMIN(individual_makespan_bound + front_respectful_transition.m_waited, makespan_bound);
		
		if (front_respectful_transition.m_time + (map.m_shortest_Distances[sink_loc_id][front_respectful_transition.m_location_id] / kruhobot.m_properties.m_linear_velo) <= effective_makespan_bound + s_EPSILON)
		{
		    RespectfulExplorations_map::iterator respectful_exploration = respectful_Explorations.find(front_respectful_transition.m_conflict_fingerprint);
		    sASSERT(respectful_exploration != respectful_Explorations.end());
				    
		    if (sink_loc_id == front_respectful_transition.m_location_id)
		    {
                        #ifdef sDEBUG
			{
			    sink_reached = true;
			}
			#endif

			SinkReachabilities_mmap::iterator sink_reachability;
			if ((sink_reachability = sink_Reachabilities.find(front_respectful_transition.m_conflict_fingerprint)) != sink_Reachabilities.end())
			{
			    if (sink_reachability->second < 0.0)
			    {
				sink_reachability->second = front_respectful_transition.m_time;
			    }
			    else
			    {
				sink_reachability->second = (front_respectful_transition.m_time < sink_reachability->second) ? sink_reachability->second : front_respectful_transition.m_time;
			    }
			}
			else
			{
			    sink_Reachabilities[front_respectful_transition.m_conflict_fingerprint] = front_respectful_transition.m_time;
			}
		    }
		
		    sInt_32 front_kruhobot_decision_id = kruhobot_RDD.size();
		    
		    if (!is_UnifiedlyVisited(front_respectful_transition.m_location_id, front_respectful_transition.m_time, unified_Visits))
		    {
			kruhobot_RDD.push_back(KruhobotDecision(front_kruhobot_decision_id,
								front_respectful_transition.m_time,
								front_respectful_transition.m_location_id,
								front_respectful_transition.m_prev_corr_dec_id));	      
			kruhobot_RDD_mapping[front_respectful_transition.m_location_id].insert(KruhobotDecisionIDs_mmap::value_type(front_respectful_transition.m_time, front_kruhobot_decision_id));
			unified_Visits[front_respectful_transition.m_location_id].insert(front_respectful_transition.m_time);
		    }
		    const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[front_respectful_transition.m_location_id].m_Neighbors;
		    for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
		    {
			sInt_32 neighbor_location_id = (*neighbor)->m_target->m_id;
			
			{
			    sDouble transition_distance = map.m_straight_Distances[front_respectful_transition.m_location_id][neighbor_location_id];
			    sDouble transition_delta_time = transition_distance / kruhobot.m_properties.m_linear_velo;
			    sDouble transition_finish_time = front_respectful_transition.m_time + transition_delta_time;
			    sDouble transition_finish_cost = front_respectful_transition.m_cost + transition_delta_time;		
			    sDouble transition_finish_makespan = front_respectful_transition.m_makespan + transition_delta_time;
			    sDouble transition_waited = front_respectful_transition.m_waited;

			    if (!is_TransitionConflicting(front_respectful_transition.m_location_id,
							  neighbor_location_id,
							  front_respectful_transition.m_time,
							  transition_finish_time,
							  location_Conflicts,
							  linear_Conflicts,
							  front_respectful_transition.m_conflict_fingerprint))
			    {
				if (!is_UnifiedlyVisited(neighbor_location_id, transition_finish_time, unified_Visits))
				{
				    /*
				    for (RespectfulVisits_umap::const_iterator re = respectful_exploration->second.begin(); re != respectful_exploration->second.end(); ++re)
				    {
					printf("%d ", re->first);
				    }
				    printf("\n");
				    */
				    RespectfulVisits_umap::iterator next_respectful_visit = respectful_exploration->second.find(neighbor_location_id);
				    
				    RespectfulTransition neighbor_respectful_transition(last_transition_id++,
											transition_finish_time,
											transition_finish_cost,
											transition_finish_makespan,
											transition_waited,
											neighbor_location_id,
											front_respectful_transition.m_trans_id);
				    neighbor_respectful_transition.m_prev_corr_dec_id = front_kruhobot_decision_id;
				    neighbor_respectful_transition.m_conflict_fingerprint = front_respectful_transition.m_conflict_fingerprint;
				    
				    sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(neighbor_respectful_transition.m_time);
//				    unified_Visits[neighbor_respectful_transition.m_location_id].insert(neighbor_respectful_transition.m_time);

				    if (next_respectful_visit == respectful_exploration->second.end()) /* visiting for the first time in a given fingerprint */
				    {
					sDouble estimated_remaining = map.m_shortest_Distances[sink_loc_id][neighbor_location_id] / kruhobot.m_properties.m_linear_velo;
					sASSERT(individual_makespan_bound >= 0.0);					
					sDouble effective_makespan_bound = sMIN(individual_makespan_bound + neighbor_respectful_transition.m_waited, makespan_bound);

					if (neighbor_respectful_transition.m_time + estimated_remaining <= effective_makespan_bound + s_EPSILON)
					{				    
					    RespectfulVisit neighbor_respectful_visit(neighbor_respectful_transition.m_time, neighbor_respectful_transition.m_trans_id);
					    RespectfulVisits_umap::iterator neighbor_respectful_visit_iter = respectful_exploration->second.insert(RespectfulVisits_umap::value_type(neighbor_location_id,
																						     neighbor_respectful_visit)).first;
					    /*
					    RespectfulTransitions_mmap::iterator queue_iter = bucketed_respectful_transition_Queues[neighbor_respectful_transition.m_conflict_fingerprint].insert(RespectfulTransitions_mmap::value_type(neighbor_respectful_transition.m_makespan,

					    neighbor_respectful_transition));
					    */

					    RespectfulTransitions_mmap::iterator queue_iter = front_respectful_transition_Queue.insert(RespectfulTransitions_mmap::value_type(neighbor_respectful_transition.m_makespan,
																					      
																					      neighbor_respectful_transition));
					    neighbor_respectful_visit_iter->second.m_queue_iter = queue_iter;
					}
					else
					{
					    sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(neighbor_respectful_transition.m_time + estimated_remaining);
					    sREAL_SMT_CBS_UPDATE_NEXT_INDIVIDUAL_MAKESPAN_BOUND(neighbor_respectful_transition.m_time + estimated_remaining - neighbor_respectful_transition.m_waited);
					}
				    }
				    else /* visiting for the next time in a given fingerprint */
				    {
					sASSERT(next_respectful_visit != respectful_exploration->second.end());

					if (next_respectful_visit->second.m_time > neighbor_respectful_transition.m_time + s_EPSILON) /* update neighbor */
					{
					    next_respectful_visit->second.m_time = neighbor_respectful_transition.m_time;
					    next_respectful_visit->second.m_trans_id = neighbor_respectful_transition.m_trans_id;

					    /*
					    bucketed_respectful_transition_Queues[neighbor_respectful_transition.m_conflict_fingerprint].erase(next_respectful_visit->second.m_queue_iter);
					    RespectfulTransitions_mmap::iterator queue_iter = bucketed_respectful_transition_Queues[neighbor_respectful_transition.m_conflict_fingerprint].insert(RespectfulTransitions_mmap::value_type(neighbor_respectful_transition.m_makespan,
																													 neighbor_respectful_transition));
					    */
					    front_respectful_transition_Queue.erase(next_respectful_visit->second.m_queue_iter);
					    RespectfulTransitions_mmap::iterator queue_iter = front_respectful_transition_Queue.insert(RespectfulTransitions_mmap::value_type(neighbor_respectful_transition.m_makespan, neighbor_respectful_transition));
					    next_respectful_visit->second.m_queue_iter = queue_iter;
					}
				    }
				}
			    }
			}				
		    }
		    {
			sDouble wait_location_finish_time = -1.0;
			sInt_32 wait_location_culprit_conflict_id = -1;
			
			sDouble wait_linear_finish_time = -1.0;
			sInt_32 wait_linear_culprit_conflict_id = -1;		    

			const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[front_respectful_transition.m_location_id].m_Neighbors;
			for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
			{
			    sInt_32 neighbor_location_id = (*neighbor)->m_target->m_id;
			    {
				LocationConflicts_upper__umap::const_iterator location_Conflict = location_Conflicts.find(neighbor_location_id);
				sDouble first_non_conf_location_time = -1.0;
				sInt_32 first_location_culprit_conflict_id = -1;
				
				if (location_Conflict != location_Conflicts.end())
				{
				    LocationConflicts_upper_map::const_iterator lower_location_conflict = location_Conflict->second.end();
				    
				    for (LocationConflicts_upper_map::const_iterator lower2_location_conflict = location_Conflict->second.lower_bound(Interval(front_respectful_transition.m_time,
																			       front_respectful_transition.m_time));
					 lower2_location_conflict != location_Conflict->second.end(); ++lower2_location_conflict)
				    {
					if (lower2_location_conflict->second.m_interval.m_upper > front_respectful_transition.m_time)
					{
					    lower_location_conflict = lower2_location_conflict;
					    first_non_conf_location_time = lower_location_conflict->second.m_interval.m_upper;
					    first_location_culprit_conflict_id = lower_location_conflict->second.m_conflict_id;
					    break;
					}				    
				    }
				}
				
				if (first_non_conf_location_time >= 0.0)
				{
				    if (wait_location_finish_time < 0.0)
				    {
					wait_location_finish_time = first_non_conf_location_time;
					wait_location_culprit_conflict_id = first_location_culprit_conflict_id;
				    }			    
				    else
				    {
					if (wait_location_finish_time > first_non_conf_location_time)
					{
					    wait_location_finish_time = first_non_conf_location_time;
					    wait_location_culprit_conflict_id = first_location_culprit_conflict_id;
					}
				    }
				}
				
				UlinearConflicts_upper__map::const_iterator linear_Conflict = linear_Conflicts.find(Uline(front_respectful_transition.m_location_id, neighbor_location_id));
				sDouble first_non_conf_linear_time = -1.0;
				sInt_32 first_linear_culprit_conflict_id = -1;
				
				if (linear_Conflict != linear_Conflicts.end())
				{
				    UlinearConflicts_upper_map::const_iterator lower_linear_conflict = linear_Conflict->second.end();
				    
				    for (UlinearConflicts_upper_map::const_iterator lower2_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_respectful_transition.m_time,
																			 front_respectful_transition.m_time));
					 lower2_linear_conflict != linear_Conflict->second.end(); ++lower2_linear_conflict)
				    {
					if (lower2_linear_conflict->second.m_interval.m_upper > front_respectful_transition.m_time)
					{
					    lower_linear_conflict = lower2_linear_conflict;
					    first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
					    first_linear_culprit_conflict_id = lower_linear_conflict->second.m_conflict_id;					
					    break;
					}
				    }
				}

				if (first_non_conf_linear_time >= 0.0)
				{
				    if (wait_linear_finish_time < 0.0)
				    {
					wait_linear_finish_time = first_non_conf_linear_time;
					wait_linear_culprit_conflict_id = first_linear_culprit_conflict_id;
				    }			    
				    else
				    {
					if (wait_linear_finish_time > first_non_conf_linear_time)
					{
					    wait_linear_finish_time = first_non_conf_linear_time;
					    wait_linear_culprit_conflict_id = first_linear_culprit_conflict_id;					
					}
				    }
				}
			    }
			}
			{
			    sInt_32 neighbor_location_id = front_respectful_transition.m_location_id;			
			    {
				LocationConflicts_upper__umap::const_iterator location_Conflict = location_Conflicts.find(neighbor_location_id);
				sDouble first_non_conf_location_time = -1.0;
				sInt_32 first_location_culprit_conflict_id = -1;			    
				
				if (location_Conflict != location_Conflicts.end())
				{
				    LocationConflicts_upper_map::const_iterator lower_location_conflict = location_Conflict->second.end();

				    for (LocationConflicts_upper_map::const_iterator lower2_location_conflict = location_Conflict->second.lower_bound(Interval(front_respectful_transition.m_time,
																			       front_respectful_transition.m_time));
					 lower2_location_conflict != location_Conflict->second.end(); ++lower2_location_conflict)
				    {
					if (lower2_location_conflict->second.m_interval.m_upper > front_respectful_transition.m_time)
					{
					    lower_location_conflict = lower2_location_conflict;
					    first_non_conf_location_time = lower_location_conflict->second.m_interval.m_upper;
					    first_location_culprit_conflict_id = lower_location_conflict->second.m_conflict_id;					
					    break;
					}
				    }				
				}
				
				if (first_non_conf_location_time >= 0.0)
				{
				    if (wait_location_finish_time < 0.0)
				    {
					wait_location_finish_time = first_non_conf_location_time;
					wait_location_culprit_conflict_id = first_location_culprit_conflict_id;				    
				    }			    
				    else
				    {
					if (wait_location_finish_time > first_non_conf_location_time)
					{
					    wait_location_finish_time = first_non_conf_location_time;
					    wait_location_culprit_conflict_id = first_location_culprit_conflict_id;
					}
				    }
				}

				UlinearConflicts_upper__map::const_iterator linear_Conflict = linear_Conflicts.find(Uline(front_respectful_transition.m_location_id, neighbor_location_id));
				sDouble first_non_conf_linear_time = -1.0;
				sInt_32 first_linear_culprit_conflict_id = -1;			    
			    
				if (linear_Conflict != linear_Conflicts.end())
				{
				    UlinearConflicts_upper_map::const_iterator lower_linear_conflict = linear_Conflict->second.end();
				    
				    for (UlinearConflicts_upper_map::const_iterator lower2_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_respectful_transition.m_time,
																			 front_respectful_transition.m_time));
					 lower2_linear_conflict != linear_Conflict->second.end(); ++lower2_linear_conflict)
				    {
					if (lower2_linear_conflict->second.m_interval.m_upper > front_respectful_transition.m_time)
					{
					    lower_linear_conflict = lower2_linear_conflict;
					    first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
					    first_linear_culprit_conflict_id = lower_linear_conflict->second.m_conflict_id;					
					    break;
					}
				    }
				}
				
				if (first_non_conf_linear_time >= 0.0)
				{
				    if (wait_linear_finish_time < 0.0)
				    {
					wait_linear_finish_time = first_non_conf_linear_time;
					wait_linear_culprit_conflict_id = first_linear_culprit_conflict_id;				    
				    }			    
				    else
				    {
					if (wait_linear_finish_time > first_non_conf_linear_time)
					{
					    wait_linear_finish_time = first_non_conf_linear_time;
					    wait_linear_culprit_conflict_id = first_linear_culprit_conflict_id;					
					}
				    }
				}
			    }
			}
			sDouble wait_finish_time;
			sInt_32 wait_culprit_conflict_id;
			
			if (wait_location_finish_time >= 0.0)
			{
			    if (wait_linear_finish_time >= 0.0)
			    {
				if (wait_location_finish_time < wait_linear_finish_time)
				{
				    wait_finish_time = wait_location_finish_time;
				    wait_culprit_conflict_id = wait_location_culprit_conflict_id;				
				}
				else
				{
				    wait_finish_time = wait_linear_finish_time;
				    wait_culprit_conflict_id = wait_linear_culprit_conflict_id;				
				}
			    }
			    else
			    {
				wait_finish_time = wait_location_finish_time;
				wait_culprit_conflict_id = wait_location_culprit_conflict_id;			    
			    }
			}
			else
			{
			    if (wait_linear_finish_time >= 0.0)
			    {
				wait_finish_time = wait_linear_finish_time;
				wait_culprit_conflict_id = wait_linear_culprit_conflict_id;
			    }
			    else
			    {
				wait_finish_time = -1.0;
				wait_culprit_conflict_id = -1;			    
			    }
			}
			if (wait_finish_time > front_respectful_transition.m_time + s_EPSILON)
			{
			    ConflictFingerprint next_conflict_fingerprint(front_respectful_transition.m_conflict_fingerprint);

			    if (fingerprint_limit < 0 || next_conflict_fingerprint.m_conflict_IDs.size() < fingerprint_limit)
			    {				
				next_conflict_fingerprint.m_conflict_IDs.insert(wait_culprit_conflict_id);
			    
				RespectfulExplorations_map::iterator respectful_exploration = respectful_Explorations.find(next_conflict_fingerprint);
				
				if (respectful_exploration == respectful_Explorations.end())
				{
				    if (sink_Reachabilities.find(next_conflict_fingerprint) == sink_Reachabilities.end())
				    {
					sink_Reachabilities[next_conflict_fingerprint] = -1.0;
				    }
				}
						
			        /* bypass */
				const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[front_respectful_transition.m_location_id].m_Neighbors;
				for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
				{
				    sInt_32 neighbor_location_id = (*neighbor)->m_target->m_id;
				    {
					sDouble bypass_transition_distance = map.m_straight_Distances[front_respectful_transition.m_location_id][neighbor_location_id];
					sDouble bypass_transition_delta_time = bypass_transition_distance / kruhobot.m_properties.m_linear_velo;
					sDouble bypass_transition_finish_time = front_respectful_transition.m_time + bypass_transition_delta_time;
					sDouble bypass_transition_finish_cost = front_respectful_transition.m_cost + bypass_transition_delta_time;		
					sDouble bypass_transition_finish_makespan = front_respectful_transition.m_makespan + bypass_transition_delta_time;
					sDouble bypass_transition_waited = front_respectful_transition.m_waited;

					RespectfulTransition bypass_respectful_transition(last_transition_id++,
											  bypass_transition_finish_time,
											  bypass_transition_finish_cost,
											  bypass_transition_finish_makespan,
											  bypass_transition_waited,
											  neighbor_location_id,
											  front_respectful_transition.m_trans_id);
				
					bypass_respectful_transition.m_prev_corr_dec_id = front_kruhobot_decision_id;
					bypass_respectful_transition.m_conflict_fingerprint = next_conflict_fingerprint;
								    
					if (!is_TransitionConflicting(front_respectful_transition.m_location_id,
								      neighbor_location_id,
								      front_respectful_transition.m_time,
								      bypass_transition_finish_time,
								      location_Conflicts,
								      linear_Conflicts,
								      bypass_respectful_transition.m_conflict_fingerprint))
					{
					    if (!is_UnifiedlyVisited(neighbor_location_id, bypass_transition_finish_time, unified_Visits))
					    {				
						if (front_respectful_transition.m_conflict_fingerprint.m_conflict_IDs.find(wait_culprit_conflict_id) == front_respectful_transition.m_conflict_fingerprint.m_conflict_IDs.end())
						{						
						    sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(bypass_respectful_transition.m_time);
						    
						    if (respectful_exploration == respectful_Explorations.end())
						    {
							RespectfulVisits_umap extended_conflict_Visits;
							RespectfulVisit bypass_respectful_visit(bypass_respectful_transition.m_time, bypass_respectful_transition.m_trans_id);
						    
							RespectfulVisits_umap::iterator bypass_respectful_visit_iter = extended_conflict_Visits.insert(RespectfulVisits_umap::value_type(bypass_respectful_transition.m_location_id,
																							 bypass_respectful_visit)).first;
							RespectfulTransitions_mmap::iterator queue_iter = bucketed_respectful_transition_Queues[bypass_respectful_transition.m_conflict_fingerprint].insert(RespectfulTransitions_mmap::value_type(bypass_respectful_transition.m_makespan, bypass_respectful_transition));
							bypass_respectful_visit_iter->second.m_queue_iter = queue_iter;
						    
							respectful_Explorations.insert(RespectfulExplorations_map::value_type(bypass_respectful_transition.m_conflict_fingerprint, extended_conflict_Visits));
						    }
						    else
						    {
							RespectfulVisits_umap::iterator next_respectful_visit = respectful_exploration->second.find(bypass_respectful_transition.m_location_id);
							
							if (next_respectful_visit == respectful_exploration->second.end()) /* visiting for the first time in a given fingerprint */
							{
							    RespectfulVisit bypass_respectful_visit(bypass_respectful_transition.m_time, bypass_respectful_transition.m_trans_id);
							    RespectfulVisits_umap::iterator bypass_respectful_visit_iter = respectful_exploration->second.insert(RespectfulVisits_umap::value_type(bypass_respectful_transition.m_location_id,
																								   bypass_respectful_visit)).first;
							    RespectfulTransitions_mmap::iterator queue_iter = bucketed_respectful_transition_Queues[bypass_respectful_transition.m_conflict_fingerprint].insert(RespectfulTransitions_mmap::value_type(bypass_respectful_transition.m_makespan,
																														       bypass_respectful_transition));
							    bypass_respectful_visit_iter->second.m_queue_iter = queue_iter;
							}
							else /* visiting next time */
							{
							    sASSERT(next_respectful_visit != respectful_exploration->second.end());
							    
							    if (next_respectful_visit->second.m_time > bypass_respectful_transition.m_time)
							    {
								next_respectful_visit->second.m_time = bypass_respectful_transition.m_time;
								next_respectful_visit->second.m_trans_id = bypass_respectful_transition.m_trans_id;
								
								RespectfulTransitions_mmap &respectful_transition_Queue = bucketed_respectful_transition_Queues[bypass_respectful_transition.m_conflict_fingerprint];
								
								respectful_transition_Queue.erase(next_respectful_visit->second.m_queue_iter);
								RespectfulTransitions_mmap::iterator queue_iter = respectful_transition_Queue.insert(RespectfulTransitions_mmap::value_type(bypass_respectful_transition.m_makespan,
																							    bypass_respectful_transition));
								next_respectful_visit->second.m_queue_iter = queue_iter;
							    }
							}
						    }
						}
					    }
					}
				    }
				}
				/* waiting */
				if (!is_UnifiedlyVisited(front_respectful_transition.m_location_id, wait_finish_time, unified_Visits))
				{
				    if (front_respectful_transition.m_conflict_fingerprint.m_conflict_IDs.find(wait_culprit_conflict_id) == front_respectful_transition.m_conflict_fingerprint.m_conflict_IDs.end())
				    {					
					sDouble waited = wait_finish_time - front_respectful_transition.m_time;
					sDouble wait_cost = waited * kruhobot.m_properties.m_wait_factor;
				
					RespectfulTransition wait_respectful_transition(last_transition_id++,
											wait_finish_time,
											front_respectful_transition.m_cost + wait_cost,
											wait_finish_time,
											front_respectful_transition.m_waited + waited,
											front_respectful_transition.m_location_id,
											front_respectful_transition.m_trans_id);
				
					sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(wait_respectful_transition.m_time);					
					wait_respectful_transition.m_conflict_fingerprint = next_conflict_fingerprint;

					if (respectful_exploration == respectful_Explorations.end()) /* non-existent fingerprint */
					{
					    sDouble estimated_remaining = map.m_shortest_Distances[sink_loc_id][wait_respectful_transition.m_location_id] / kruhobot.m_properties.m_linear_velo;
					    
					    sASSERT(individual_makespan_bound >= 0.0);
					    sDouble effective_makespan_bound = sMIN(individual_makespan_bound + wait_respectful_transition.m_waited, makespan_bound);
					    
					    if (wait_respectful_transition.m_time + estimated_remaining <= effective_makespan_bound + s_EPSILON)
					    {
						RespectfulVisits_umap extended_conflict_Visits;
						RespectfulVisit wait_respectful_visit(wait_respectful_transition.m_time, wait_respectful_transition.m_trans_id);
					    
						RespectfulVisits_umap::iterator wait_respectful_visit_iter = extended_conflict_Visits.insert(RespectfulVisits_umap::value_type(wait_respectful_transition.m_location_id,
																					       wait_respectful_visit)).first;
						RespectfulTransitions_mmap::iterator queue_iter = bucketed_respectful_transition_Queues[wait_respectful_transition.m_conflict_fingerprint].insert(RespectfulTransitions_mmap::value_type(wait_respectful_transition.m_makespan,
																													 wait_respectful_transition));
						wait_respectful_visit_iter->second.m_queue_iter = queue_iter;
						
						respectful_Explorations.insert(RespectfulExplorations_map::value_type(wait_respectful_transition.m_conflict_fingerprint, extended_conflict_Visits));
					    }
					    else
					    {
						sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(wait_respectful_transition.m_time + estimated_remaining);
						sREAL_SMT_CBS_UPDATE_NEXT_INDIVIDUAL_MAKESPAN_BOUND(wait_respectful_transition.m_time + estimated_remaining - wait_respectful_transition.m_waited);
					    }
					}
					else /* existent fingerprint */
					{
					    RespectfulVisits_umap::iterator wait_respectful_visit = respectful_exploration->second.find(front_respectful_transition.m_location_id);
					    
					    if (wait_respectful_visit == respectful_exploration->second.end()) /* visiting for the first time */
					    {
						sDouble estimated_remaining = map.m_shortest_Distances[sink_loc_id][wait_respectful_transition.m_location_id] / kruhobot.m_properties.m_linear_velo;

						sASSERT(individual_makespan_bound >= 0.0);
						sDouble effective_makespan_bound = sMIN(individual_makespan_bound + wait_respectful_transition.m_waited, makespan_bound);
						
						if (wait_respectful_transition.m_time + estimated_remaining <= effective_makespan_bound + s_EPSILON)
						{
						    RespectfulVisit wait_respectful_visit(wait_respectful_transition.m_time, wait_respectful_transition.m_trans_id);						    
						    RespectfulVisits_umap::iterator wait_respectful_visit_iter = respectful_exploration->second.insert(RespectfulVisits_umap::value_type(wait_respectful_transition.m_location_id,
																							 wait_respectful_visit)).first;
						    RespectfulTransitions_mmap::iterator queue_iter = bucketed_respectful_transition_Queues[wait_respectful_transition.m_conflict_fingerprint].insert(RespectfulTransitions_mmap::value_type(wait_respectful_transition.m_makespan,
																													 wait_respectful_transition));
						    wait_respectful_visit_iter->second.m_queue_iter = queue_iter;
						}
						else
						{
						    sREAL_SMT_CBS_UPDATE_NEXT_MAKESPAN_BOUND(wait_respectful_transition.m_time + estimated_remaining);
						    sREAL_SMT_CBS_UPDATE_NEXT_INDIVIDUAL_MAKESPAN_BOUND(wait_respectful_transition.m_time + estimated_remaining - wait_respectful_transition.m_waited);
						}
					    }
					    else /* visiting for the next time, almost same as first time */
					    {
						sASSERT(wait_respectful_visit != respectful_exploration->second.end());
						
						if (wait_respectful_visit->second.m_time > wait_respectful_transition.m_time)
						{
						    wait_respectful_visit->second.m_time = wait_respectful_transition.m_time;
						    wait_respectful_visit->second.m_trans_id = wait_respectful_transition.m_trans_id;
						    
						    RespectfulTransitions_mmap &respectful_transition_Queue = bucketed_respectful_transition_Queues[wait_respectful_transition.m_conflict_fingerprint];
						    
						    respectful_transition_Queue.erase(wait_respectful_visit->second.m_queue_iter);
						    RespectfulTransitions_mmap::iterator queue_iter = respectful_transition_Queue.insert(RespectfulTransitions_mmap::value_type(wait_respectful_transition.m_makespan,
																						wait_respectful_transition));
						    wait_respectful_visit->second.m_queue_iter = queue_iter;
						}
					    }			    		    
					}
				    }
				}
			    }
			}
		    }
		}
		front_respectful_transition_Queue.erase(front_respectful_transition_Queue.begin());
	    }
	    if (clear_after)
	    {
		RespectfulExplorations_map::iterator respectful_exploration = respectful_Explorations.find(clear_conflict_fingerprint);
		respectful_exploration->second.clear();
	    }
	
            #ifdef sDEBUG
	    {
		/*
		for (UnifiedVisits_umap::const_iterator unified_visit = unified_Visits.begin(); unified_visit != unified_Visits.end(); ++unified_visit)
		{
		    printf("%d: ", unified_visit->first);
		    for (VisitTimes_set::const_iterator visit_time = unified_visit->second.begin(); visit_time != unified_visit->second.end(); ++visit_time)
		    {
			printf("%.3f ", *visit_time);
		    }
		    printf("\n");
		}
		printf("%d: Fino queue: %ld (%ld)\n", kruhobot.m_id, front_respectful_transition_Queue.size(), unified_Visits.size());
		printf("RDD size: %ld\n", kruhobot_RDD.size());
		*/
	    }
            #endif

	    #ifdef sDEBUG
	    {
		sASSERT(sink_reached);
	    }
            #endif
    
	    while (!bucketed_respectful_transition_Queues.empty() && bucketed_respectful_transition_Queues.begin()->second.empty())
	    {
		bucketed_respectful_transition_Queues.erase(bucketed_respectful_transition_Queues.begin());
	    }

	}
	augment_KruhobotRealDecisionDiagram(kruhobot, source_loc_id, sink_loc_id, makespan_bound, kruhobot_RDD, kruhobot_RDD_mapping);
	interconnect_KruhobotRealDecisionDiagram_smart(kruhobot, map, kruhobot_RDD, kruhobot_RDD_mapping);

	#ifdef sDEBUG
	{
	    /*
	    printf("Sink reachability:\n");
	    for (SinkReachabilities_mmap::const_iterator sink_reachability = sink_Reachabilities.begin(); sink_reachability != sink_Reachabilities.end(); ++sink_reachability)
	    {
		sink_reachability->first.to_Screen();
		printf(": %.3f\n", sink_reachability->second);
	    }
	    */
	}
	#endif
	
	bool sink_all_reached = true;
	for (SinkReachabilities_mmap::const_iterator sink_reachability = sink_Reachabilities.begin(); sink_reachability != sink_Reachabilities.end(); ++sink_reachability)
	{
	    if (sink_reachability->second < 0.0)
	    {
		sink_all_reached = false;
		break;
	    }
	}
	if (sink_all_reached)
	{
	    next_individual_makespan_bound = -1.0;
	}
	individual_makespan_bound = next_individual_makespan_bound;

	/*
	#ifdef sDEBUG
	{
	    printf("RDD size: %ld [processed: %d, percentage:%.3f]\n", kruhobot_RDD.size(), processed_nodes, (sDouble)kruhobot_RDD.size() / processed_nodes);
	}
	#endif
	*/
	
	return next_makespan_bound;
    }        


    bool sRealSMTCBS::is_UnifiedlyVisited(sInt_32 location_id, sDouble time, const UnifiedVisits_umap &unified_Visits) const
    {
	UnifiedVisits_umap::const_iterator unified_visit = unified_Visits.find(location_id);
	
	if (unified_visit != unified_Visits.end())
	{
	    VisitTimes_set::const_iterator visit_time = unified_visit->second.lower_bound(time - s_EPSILON);

	    if (visit_time != unified_visit->second.end() && *visit_time <= time + s_EPSILON)
	    {
		return true;
	    }
	    /*
	    while (visit_time != unified_visit->second.end() && *visit_time <= time + s_EPSILON)
	    {
		if (sABS(*visit_time - time) <= s_EPSILON)
		{
		    return true;
		}
		++visit_time;
	    }
	    */
	}	
	return false;
    }


    sRealSMTCBS::Explorations_umap* sRealSMTCBS::obtain_ExploredTransitions(TransitionExplorations_map &explored_Transitions, sDouble time) const
    {
	TransitionExplorations_map::iterator explored_transition = explored_Transitions.lower_bound(time - s_EPSILON);

	while (explored_transition != explored_Transitions.end())
	{
	    if (sABS(explored_transition->first - time) < s_EPSILON)
	    {
		return &explored_transition->second;
	    }
	    else
	    {
		sASSERT(explored_transition->first > time + s_EPSILON);
		break;
	    }
	    ++explored_transition;
	}
	return &(explored_Transitions.insert(TransitionExplorations_map::value_type(time, Explorations_umap())).first->second);
    }


    bool sRealSMTCBS::is_TransitionConflicting(sInt_32                              location_u_id,
					       sInt_32                              location_v_id,
					       sDouble                              start_time,
					       sDouble                              finish_time,
					       const LocationConflicts_upper__umap &location_Conflicts,
					       const UlinearConflicts_upper__map    &linear_Conflicts,
					       const ConflictFingerprint           &conflict_Fingerprint) const
    {       
	LocationConflicts_upper__umap::const_iterator location_Conflict = location_Conflicts.find(location_v_id);
	       
	if (location_Conflict != location_Conflicts.end())
	{
	    for (LocationConflicts_upper_map::const_iterator lower_location_conflict = location_Conflict->second.lower_bound(Interval(start_time, start_time));		
		 lower_location_conflict != location_Conflict->second.end(); ++lower_location_conflict)
	    {
		if (lower_location_conflict->second.overlaps(Interval(start_time, finish_time)))
		{		    		    
		    if (conflict_Fingerprint.m_conflict_IDs.find(lower_location_conflict->second.m_conflict_id) != conflict_Fingerprint.m_conflict_IDs.end())
		    {
			return true;			
		    }
		}
		if (lower_location_conflict->second.m_interval.m_lower > finish_time)
		{
		    break;
		}
	    }
	}

	UlinearConflicts_upper__map::const_iterator linear_Conflict = linear_Conflicts.find(Uline(location_u_id, location_v_id));
	
	if (linear_Conflict != linear_Conflicts.end())
	{
	    /*
	    for (UlinearConflicts_upper_map::const_iterator lower_linear_conflict = linear_Conflict->second.begin(); lower_linear_conflict != linear_Conflict->second.end(); ++lower_linear_conflict)
	    {
		lower_linear_conflict->first.to_Screen();
	    }
	    */
	    for (UlinearConflicts_upper_map::const_iterator lower_linear_conflict = linear_Conflict->second.lower_bound(Interval(start_time, start_time));
		 lower_linear_conflict != linear_Conflict->second.end(); ++lower_linear_conflict)
	    {
		if (lower_linear_conflict->second.overlaps(Interval(start_time, finish_time)))
		{
		    if (conflict_Fingerprint.m_conflict_IDs.find(lower_linear_conflict->second.m_conflict_id) != conflict_Fingerprint.m_conflict_IDs.end())
		    {
			return true;
		    }
		}
		if (lower_linear_conflict->second.m_interval.m_lower > finish_time)
		{
		    break;
		}
	    }
	}
	
	return false;
    }


    void sRealSMTCBS::determine_ClimbingStatus(sInt_32                              location_id,
					       sDouble                              time,
					       const LocationConflicts_upper__umap &location_Conflicts,
					       const UlinearConflicts_upper__map    &linear_Conflicts,
					       const ConflictFingerprint           &conflict_Fingerprint) const
    {
	LocationConflicts_upper__umap::const_iterator location_Conflict = location_Conflicts.find(location_id);
	       
	if (location_Conflict != location_Conflicts.end())
	{
	    for (LocationConflicts_upper_map::const_iterator lower_location_conflict = location_Conflict->second.lower_bound(Interval(time, time));
		 lower_location_conflict != location_Conflict->second.end(); ++lower_location_conflict)
	    {
		if (conflict_Fingerprint.m_conflict_IDs.find(lower_location_conflict->second.m_conflict_id) != conflict_Fingerprint.m_conflict_IDs.end())
		{
		    if (lower_location_conflict->first.m_upper <= time + s_EPSILON)
		    {
		    }
		}
	    }
	}

	for (UlinearConflicts_upper__map::const_iterator linear_Conflict = linear_Conflicts.begin(); linear_Conflict != linear_Conflicts.end(); ++linear_Conflict)
	{
	    if (linear_Conflict->first.m_lower_id == location_id || linear_Conflict->first.m_upper_id == location_id)
	    {
		for (UlinearConflicts_upper_map::const_iterator lower_linear_conflict = linear_Conflict->second.lower_bound(Interval(time, time));
		     lower_linear_conflict != linear_Conflict->second.end(); ++lower_linear_conflict)
		{		    
		    if (conflict_Fingerprint.m_conflict_IDs.find(lower_linear_conflict->second.m_conflict_id) != conflict_Fingerprint.m_conflict_IDs.end())
		    {
			if (lower_linear_conflict->first.m_upper <= time + s_EPSILON)
			{
			}
		    }
		}
	    }
	}
    }
    

    void sRealSMTCBS::augment_KruhobotRealDecisionDiagram(const sKruhobot                &sUNUSED(kruhobot),
							  sInt_32                         sUNUSED(source_loc_id),
							  sInt_32                         sink_loc_id,
							  sDouble                         makespan_bound,
							  KruhobotDecisionDiagram_vector &kruhobot_RDD,
							  KruhobotDecisionMapping_map    &kruhobot_RDD_mapping) const
    {
	for (KruhobotDecisionDiagram_vector::const_iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	{
	    if (decision->m_location_id == sink_loc_id)
	    {
		if (sABS(decision->m_time - makespan_bound) < s_EPSILON)
		{
		    return;
		}
	    }
	}
	
	sInt_32 sink_decision_id = kruhobot_RDD.size();
	kruhobot_RDD.push_back(KruhobotDecision(sink_decision_id,
						makespan_bound,
						sink_loc_id,
						-1));
	kruhobot_RDD_mapping[sink_loc_id].insert(KruhobotDecisionIDs_mmap::value_type(makespan_bound, sink_decision_id));

	for (sInt_32 dec_id = 0; dec_id < sink_decision_id; ++dec_id)
	{
	    if (kruhobot_RDD[dec_id].m_location_id == sink_loc_id)
	    {
		kruhobot_RDD[dec_id].m_next_dec_IDs.push_back(sink_decision_id);
		kruhobot_RDD[sink_decision_id].m_prev_dec_IDs.insert(dec_id);
	    }
	}	
    }

    
    void sRealSMTCBS::interconnect_KruhobotRealDecisionDiagram(const sKruhobot                &kruhobot,
							       const s2DMap                   &map,
							       KruhobotDecisionDiagram_vector &kruhobot_RDD,
							       KruhobotDecisionMapping_map    &sUNUSED(kruhobot_RDD_mapping)) const
    {
	for (KruhobotDecisionDiagram_vector::iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	{
	    KruhobotDecisionDiagram_vector::iterator wait_decision = kruhobot_RDD.end();	    
	    for (KruhobotDecisionDiagram_vector::iterator next_decision = kruhobot_RDD.begin(); next_decision != kruhobot_RDD.end(); ++next_decision)
	    {
		if (next_decision != decision)
		{
		    if (decision->m_location_id == next_decision->m_location_id || map.m_Network.is_Adjacent(decision->m_location_id, next_decision->m_location_id))
		    {
			sDouble transition_distance = map.m_straight_Distances[decision->m_location_id][next_decision->m_location_id];
			sDouble transition_delta_time = transition_distance / kruhobot.m_properties.m_linear_velo;
			sDouble transition_finish_time = decision->m_time + transition_delta_time;

			if (decision->m_location_id != next_decision->m_location_id)
			{
			    if (sABS(next_decision->m_time - transition_finish_time) < s_EPSILON)
			    {
				decision->m_next_dec_IDs.push_back(next_decision->m_dec_id);
				next_decision->m_prev_dec_IDs.insert(decision->m_dec_id);
			    }
			}
			else
			{
			    if (next_decision->m_time > decision->m_time + s_EPSILON)
			    {
				if (wait_decision != kruhobot_RDD.end())
				{
				    if (next_decision->m_time < wait_decision->m_time)
				    {
					wait_decision = next_decision;
				    }
				}
				else
				{
				    wait_decision = next_decision;
				}
			    }
			}
		    }
		}
	    }
	    if (wait_decision != kruhobot_RDD.end())	    
	    {
		decision->m_next_dec_IDs.push_back(wait_decision->m_dec_id);
		wait_decision->m_prev_dec_IDs.insert(decision->m_dec_id);
	    }
	}	
    }


    void sRealSMTCBS::interconnect_KruhobotRealDecisionDiagram_smart(const sKruhobot                &kruhobot,
								     const s2DMap                   &map,
								     KruhobotDecisionDiagram_vector &kruhobot_RDD,
								     KruhobotDecisionMapping_map    &sUNUSED(kruhobot_RDD_mapping)) const
    {
//	KruhobotDecisionMapping_map vertex_Decisions;
	KruhobotDecisionMapping_vector vertex_Decisions;
	vertex_Decisions.resize(map.m_Network.get_VertexCount());

	for (KruhobotDecisionDiagram_vector::iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	{
	    vertex_Decisions[decision->m_location_id].insert(KruhobotDecisionIDs_map::value_type(decision->m_time, decision->m_dec_id));
	}

	for (KruhobotDecisionDiagram_vector::iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	{
	    sInt_32 wait_decision_id = -1;

	    const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[decision->m_location_id].m_Neighbors;
	    for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
	    {
		const KruhobotDecisionIDs_mmap &vertex_decisions = vertex_Decisions[(*neighbor)->m_target->m_id];

		sDouble transition_distance = map.m_straight_Distances[decision->m_location_id][(*neighbor)->m_target->m_id];
		sDouble transition_delta_time = transition_distance / kruhobot.m_properties.m_linear_velo;
		sDouble transition_finish_time = decision->m_time + transition_delta_time;		
		
		bool started = false;
//		KruhobotDecisionIDs_mmap::const_iterator vertex_decision = vertex_decisions.begin();
//		KruhobotDecisionIDs_mmap::const_iterator vertex_decision = vertex_decisions.lower_bound(decision->m_time);
		KruhobotDecisionIDs_mmap::const_iterator vertex_decision = vertex_decisions.lower_bound(transition_finish_time - s_EPSILON);

		while (vertex_decision != vertex_decisions.end())
//		for (KruhobotDecisionIDs_mmap::const_iterator vertex_decision = vertex_decisions.begin(); vertex_decision != vertex_decisions.end(); ++vertex_decision)
		{
		    KruhobotDecision &next_decision = kruhobot_RDD[vertex_decision->second];
		    
		    sASSERT(decision->m_location_id != next_decision.m_location_id);
		    
		    if (sABS(next_decision.m_time - transition_finish_time) < s_EPSILON)
		    {
			decision->m_next_dec_IDs.push_back(next_decision.m_dec_id);
			next_decision.m_prev_dec_IDs.insert(decision->m_dec_id);
			started = true;
		    }
		    else
		    {
			if (started)
			{
			    break;
			}
		    }
		    ++vertex_decision;
		}      
	    }
	    {
//		const KruhobotDecisionIDs_vector &vertex_decisions = vertex_Decisions[decision->m_location_id];		
		const KruhobotDecisionIDs_mmap &vertex_decisions = vertex_Decisions[decision->m_location_id];

//		KruhobotDecisionIDs_mmap::const_iterator vertex_decision = vertex_decisions.begin();
		KruhobotDecisionIDs_mmap::const_iterator vertex_decision = vertex_decisions.lower_bound(decision->m_time - s_EPSILON);

		while (vertex_decision != vertex_decisions.end())		
//		for (KruhobotDecisionIDs_mmap::const_iterator vertex_decision = vertex_decisions.begin(); vertex_decision != vertex_decisions.end(); ++vertex_decision)
		{
		    KruhobotDecision &next_decision = kruhobot_RDD[vertex_decision->second];
		    if (next_decision.m_dec_id != decision->m_dec_id)
		    {
			if (next_decision.m_time > decision->m_time + s_EPSILON)
			{
			    if (wait_decision_id >= 0)
			    {
				if (next_decision.m_time < kruhobot_RDD[wait_decision_id].m_time)
				{
				    wait_decision_id = next_decision.m_dec_id;
				}
			    }
			    else
			    {
				wait_decision_id = next_decision.m_dec_id;
			    }
			    break;
			}
		    }
		    ++vertex_decision;
		}
	    }	    
	    if (wait_decision_id >= 0)	    
	    {
		decision->m_next_dec_IDs.push_back(wait_decision_id);
		kruhobot_RDD[wait_decision_id].m_prev_dec_IDs.insert(decision->m_dec_id);
	    }
	}
    }    


    void sRealSMTCBS::trim_KruhobotRealDecisionDiagram(sDouble makespan_bound, KruhobotDecisionDiagram_vector &kruhobot_RDD) const
    {
	for (KruhobotDecisionDiagram_vector::iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	{
	    if (decision->m_time >= makespan_bound + s_EPSILON)
	    {
		decision->m_dec_id = -1;
	    }
	}
    }

    
    void sRealSMTCBS::trim_KruhobotRealDecisionDiagrams(sDouble makespan_bound, KruhobotDecisionDiagrams_vector &kruhobot_RDDs) const
    {
	for (KruhobotDecisionDiagrams_vector::iterator kruhobot_RDD = kruhobot_RDDs.begin(); kruhobot_RDD != kruhobot_RDDs.end(); ++kruhobot_RDD)
	{
	    trim_KruhobotRealDecisionDiagram(makespan_bound, *kruhobot_RDD);
	}
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
    
    sInt_32 sRealSMTCBS::build_RealModelVariables(Glucose::Solver                       *sUNUSED(solver),
						  RealContext                           &sUNUSED(context),
						  const sRealInstance                   &real_Instance,
						  KruhobotDecisionDiagrams_vector       &kruhobot_RDDs,
						  const KruhobotDecisionMappings_vector &sUNUSED(kruhobot_RDD_Mappings),
						  RealModel                             &real_sat_Model) const
    {   
	sASSERT(!kruhobot_RDDs.empty());
	
	sInt_32 variable_ID = 1;	
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();

	real_sat_Model.m_vertex_occupancy.resize(N_kruhobots + 1);
	real_sat_Model.m_goal_sinking.resize(N_kruhobots + 1);
	
	real_sat_Model.m_variable_mapping.push_back(RealCoordinate());

	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
	    DecisionIDs_vector goal_decision_IDs;
	    
	    collect_GoalKruhobotDecisions(real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id], kruhobot_RDDs[kruhobot_id], goal_decision_IDs);
	    
	    real_sat_Model.m_vertex_occupancy[kruhobot_id].resize(kruhobot_RDDs[kruhobot_id].size());
	    real_sat_Model.m_goal_sinking[kruhobot_id].resize(goal_decision_IDs.size());
	    sInt_32 sink_id = 0;
	    
	    for (sInt_32 dec_id = 0; dec_id < kruhobot_RDDs[kruhobot_id].size(); ++dec_id)
	    {
		real_sat_Model.m_vertex_occupancy[kruhobot_id][dec_id] = variable_ID++;		    
		real_sat_Model.m_variable_mapping.push_back(RealCoordinate(kruhobot_id, dec_id));
		
		if (kruhobot_RDDs[kruhobot_id][dec_id].m_location_id == real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id])
		{
		    real_sat_Model.m_goal_sinking[kruhobot_id][sink_id] = variable_ID++;
		    real_sat_Model.m_variable_mapping.push_back(RealCoordinate(kruhobot_id, dec_id, sink_id));
		    
		    kruhobot_RDDs[kruhobot_id][dec_id].m_sink_id = sink_id;
		    ++sink_id;
		}
	    }
	}

	real_sat_Model.m_edge_occupancy.resize(N_kruhobots + 1);

	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
	    real_sat_Model.m_edge_occupancy[kruhobot_id].resize(kruhobot_RDDs[kruhobot_id].size());

	    for (sInt_32 dec_id = 0; dec_id < kruhobot_RDDs[kruhobot_id].size(); ++dec_id)
	    {
		if (kruhobot_RDDs[kruhobot_id][dec_id].m_location_id == real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id])
		{		    
		    real_sat_Model.m_edge_occupancy[kruhobot_id][dec_id].resize(kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.size() + 1);
		    
		    for (sInt_32 next = 0; next < kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.size() + 1; ++next)
		    {
			real_sat_Model.m_edge_occupancy[kruhobot_id][dec_id][next] = variable_ID++;				
		    }
		}
		else
		{
		    real_sat_Model.m_edge_occupancy[kruhobot_id][dec_id].resize(kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.size());
		    
		    for (sInt_32 next = 0; next < kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.size(); ++next)
		    {
			real_sat_Model.m_edge_occupancy[kruhobot_id][dec_id][next] = variable_ID++;				
		    }		    
		}
	    }
	}

	return variable_ID;
    }


    void sRealSMTCBS::build_RealModelConstraints(Glucose::Solver                       *solver,
						 RealContext                           &sUNUSED(context),
						 const sRealInstance                   &real_Instance,
						 const KruhobotDecisionDiagrams_vector &kruhobot_RDDs,
						 const KruhobotDecisionMappings_vector &sUNUSED(kruhobot_RDD_Mappings),
						 RealModel                             &real_sat_Model) const
    {
	sASSERT(!kruhobot_RDDs.empty());
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();

	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
	    sInt_32 sink_id = 0;

	    for (sInt_32 dec_id = 0; dec_id < kruhobot_RDDs[kruhobot_id].size(); ++dec_id)
	    {
		VariableIDs_vector mutex_target_Identifiers;
				    
		if (!kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.empty())
		{
		    for (sInt_32 next_dec = 0; next_dec < kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.size(); ++next_dec)
		    {
			mutex_target_Identifiers.push_back(real_sat_Model.m_edge_occupancy[kruhobot_id][dec_id][next_dec]);
			
			sInt_32 next_dec_id = kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs[next_dec];
			
			m_solver_Encoder->cast_Implication(solver,
							   real_sat_Model.m_edge_occupancy[kruhobot_id][dec_id][next_dec],
							   real_sat_Model.m_vertex_occupancy[kruhobot_id][next_dec_id]);
			m_solver_Encoder->cast_Implication(solver,
							   real_sat_Model.m_edge_occupancy[kruhobot_id][dec_id][next_dec],
							   real_sat_Model.m_vertex_occupancy[kruhobot_id][dec_id]);
		    }
		}
		else
		{
		    sASSERT(kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.empty());
		    if (kruhobot_RDDs[kruhobot_id][dec_id].m_location_id != real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id])
		    {
			m_solver_Encoder->cast_BitUnset(solver, real_sat_Model.m_vertex_occupancy[kruhobot_id][dec_id]);
		    }
		}				
		if (kruhobot_RDDs[kruhobot_id][dec_id].m_location_id == real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id])
		{
		    mutex_target_Identifiers.push_back(real_sat_Model.m_edge_occupancy[kruhobot_id][dec_id][kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.size()]);

		    m_solver_Encoder->cast_Implication(solver,
						       real_sat_Model.m_edge_occupancy[kruhobot_id][dec_id][kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.size()],
						       real_sat_Model.m_goal_sinking[kruhobot_id][sink_id]);		    
		    m_solver_Encoder->cast_Implication(solver,
						       real_sat_Model.m_edge_occupancy[kruhobot_id][dec_id][kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.size()],
						       real_sat_Model.m_vertex_occupancy[kruhobot_id][dec_id]);
		    m_solver_Encoder->cast_Implication(solver,
						       real_sat_Model.m_goal_sinking[kruhobot_id][sink_id],
						       real_sat_Model.m_edge_occupancy[kruhobot_id][dec_id][kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.size()]);
		    ++sink_id;
		}
		
		if (!mutex_target_Identifiers.empty())
		{
		    m_solver_Encoder->cast_MultiImplication(solver,
							    real_sat_Model.m_vertex_occupancy[kruhobot_id][dec_id],
							    mutex_target_Identifiers);
		    m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_target_Identifiers);
		}

		if (!kruhobot_RDDs[kruhobot_id][dec_id].m_prev_dec_IDs.empty())
		{
		    VariableIDs_vector mutex_source_Identifiers;
				    
		    for (LocationIDs_uset::const_iterator prev_dec = kruhobot_RDDs[kruhobot_id][dec_id].m_prev_dec_IDs.begin(); prev_dec != kruhobot_RDDs[kruhobot_id][dec_id].m_prev_dec_IDs.end(); ++prev_dec)
		    {
			if (*prev_dec >= 0)
			{
			    for (sInt_32 prev_next_dec = 0; prev_next_dec < kruhobot_RDDs[kruhobot_id][*prev_dec].m_next_dec_IDs.size(); ++prev_next_dec)
			    {
				if (kruhobot_RDDs[kruhobot_id][*prev_dec].m_next_dec_IDs[prev_next_dec] == dec_id)
				{
				    mutex_source_Identifiers.push_back(real_sat_Model.m_edge_occupancy[kruhobot_id][*prev_dec][prev_next_dec]);
				    break;
				}
			    }
			}
		    }
		    if (!mutex_source_Identifiers.empty())
		    {
			m_solver_Encoder->cast_MultiImplication(solver,
								real_sat_Model.m_vertex_occupancy[kruhobot_id][dec_id],
								mutex_source_Identifiers);
//			m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_source_Identifiers);
		    }
		}
/*
		DecisionIDs_vector mutex_decision_IDs;
		collect_KruhobotExcludedDecision(real_Instance.m_Kruhobots[kruhobot_id],
						 *real_Instance.m_start_conjunction.m_Map,
						 dec_id,
						 kruhobot_RDDs[kruhobot_id],
						 mutex_decision_IDs);

		VariableIDs_vector mutex_vertex_Identifiers;

		printf("mutex %d: ", dec_id);
		for (DecisionIDs_vector::const_iterator mutex_decision = mutex_decision_IDs.begin(); mutex_decision != mutex_decision_IDs.end(); ++mutex_decision)
		{
		    mutex_vertex_Identifiers.push_back(real_sat_Model.m_vertex_occupancy[kruhobot_id][*mutex_decision]);
		    printf(" %d", *mutex_decision);
		}
		printf("\n");
		if (!mutex_vertex_Identifiers.empty())
		{
		    mutex_vertex_Identifiers.push_back(real_sat_Model.m_vertex_occupancy[kruhobot_id][dec_id]);
		    m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_vertex_Identifiers);		    
		}
*/
	    }
	}

	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
	    sInt_32 start_decision_id;
	    start_decision_id = collect_StartKruhobotDecision(real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id], kruhobot_RDDs[kruhobot_id]);
	    sASSERT(start_decision_id != -1);

	    m_solver_Encoder->cast_BitSet(solver, real_sat_Model.m_vertex_occupancy[kruhobot_id][start_decision_id]);
	}
   
	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
//	    sInt_32 goal_decision_id;
	    DecisionIDs_vector goal_decision_IDs;
	    
	    collect_GoalKruhobotDecisions(real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id]/*, context.m_makespan_bound*/, kruhobot_RDDs[kruhobot_id], goal_decision_IDs);
//	    printf("%ld, %ld\n", goal_decision_IDs.size() == real_sat_Model.m_goal_sinking[kruhobot_id].size());
	    sASSERT(!goal_decision_IDs.empty() && goal_decision_IDs.size() == real_sat_Model.m_goal_sinking[kruhobot_id].size());

	    VariableIDs_vector mutex_sink_Identifiers;

	    for (sInt_32 sink_id = 0; sink_id < goal_decision_IDs.size(); ++sink_id)
	    {
		mutex_sink_Identifiers.push_back(real_sat_Model.m_goal_sinking[kruhobot_id][sink_id]);
	    }
	    m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_sink_Identifiers);	    

	    VariableIDs_vector goal_variable_IDs;

	    for (DecisionIDs_vector::const_iterator goal_decision = goal_decision_IDs.begin(); goal_decision != goal_decision_IDs.end(); ++goal_decision)
	    {
		goal_variable_IDs.push_back(real_sat_Model.m_vertex_occupancy[kruhobot_id][*goal_decision]);
	    }
	    m_solver_Encoder->cast_Disjunction(solver, goal_variable_IDs);
	    /*
	    if (goal_variable_IDs.size() == 1)
	    {
		m_solver_Encoder->cast_BitSet(solver, *goal_variable_IDs.begin());
	    }
	    */
	}
    }


    void sRealSMTCBS::refine_RealModelConstraints(Glucose::Solver                       *solver,
						  RealContext                           &sUNUSED(context),
						  const sRealInstance                   &sUNUSED(real_Instance),
						  const KruhobotDecisionDiagrams_vector &kruhobot_RDDs,
						  const KruhobotDecisionMappings_vector &kruhobot_RDD_Mappings,
						  const KruhobotCollisions_mset         &kruhobot_Collisions,
						  RealModel                             &real_sat_Model) const
    {
	for (KruhobotCollisions_mset::const_iterator kruhobot_collision = kruhobot_Collisions.begin(); kruhobot_collision != kruhobot_Collisions.end(); ++kruhobot_collision)
	{
	    std::vector<sInt_32> variable_A_IDs, variable_B_IDs;	    
	    sInt_32 kruhobot_A_id = sABS(kruhobot_collision->m_traversal_A.m_kruhobot_id);

	    {
		DecisionIDs_vector decision_A_IDs;
		std::vector<sInt_32> Neighbors_A;
		
		sInt_32 match_count = match_CorrespondingNeighbors(kruhobot_collision->m_traversal_A,
								   kruhobot_RDDs[kruhobot_A_id],
								   kruhobot_RDD_Mappings[kruhobot_A_id],
								   decision_A_IDs,					     
								   Neighbors_A);	       
		for (sInt_32 match = 0; match < match_count; ++match)
		{
		    variable_A_IDs.push_back(real_sat_Model.m_edge_occupancy[kruhobot_A_id][decision_A_IDs[match]][Neighbors_A[match]]);
		}
	    }
	    sInt_32 kruhobot_B_id = sABS(kruhobot_collision->m_traversal_B.m_kruhobot_id);

	    {
		DecisionIDs_vector decision_B_IDs;
		std::vector<sInt_32> Neighbors_B;
		
		sInt_32 match_count = match_CorrespondingNeighbors(kruhobot_collision->m_traversal_B,
								   kruhobot_RDDs[kruhobot_B_id],
								   kruhobot_RDD_Mappings[kruhobot_B_id],
								   decision_B_IDs,					     
								   Neighbors_B);
//		sASSERT(!decision_B_IDs.empty() && !Neighbors_B.empty());
		
		for (sInt_32 match = 0; match < match_count; ++match)
		{
		    variable_B_IDs.push_back(real_sat_Model.m_edge_occupancy[kruhobot_B_id][decision_B_IDs[match]][Neighbors_B[match]]);
		}
	    }
	    m_solver_Encoder->cast_Mutexes(solver, variable_A_IDs, variable_B_IDs);	   
	}
    }

    
/*----------------------------------------------------------------------------*/
    
    sInt_32 sRealSMTCBS::match_CorrespondingDecision(const Traversal                      &traversal,
						     const KruhobotDecisionDiagram_vector &kruhobot_RDD,
						     const KruhobotDecisionMapping_map    &sUNUSED(kruhobot_RDD_mapping)) const
    {
	for (KruhobotDecisionDiagram_vector::const_iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	{
	    if (decision->m_location_id == traversal.m_u_loc_id && decision->m_time >= traversal.m_interval.m_lower - s_EPSILON && decision->m_time <= traversal.m_interval.m_upper + s_EPSILON)
	    {
		return decision->m_dec_id;
	    }
	}	
	return -1;
    }


    sInt_32 sRealSMTCBS::match_CorrespondingNeighbor(const Traversal                      &traversal,
						     const KruhobotDecisionDiagram_vector &kruhobot_RDD,
						     const KruhobotDecisionMapping_map    &sUNUSED(kruhobot_RDD_mapping),
						     sInt_32                              &neighbor) const
    {
	neighbor = -1;

	for (KruhobotDecisionDiagram_vector::const_iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	{
	    if (decision->m_location_id == traversal.m_u_loc_id && decision->m_time <= traversal.m_interval.m_lower + s_EPSILON)
	    {
		for (sInt_32 neigh = 0; neigh < decision->m_next_dec_IDs.size(); ++neigh)
		{
		    if (kruhobot_RDD[decision->m_next_dec_IDs[neigh]].m_location_id == traversal.m_v_loc_id && kruhobot_RDD[decision->m_next_dec_IDs[neigh]].m_time >= traversal.m_interval.m_upper - s_EPSILON)
		    {
			neighbor = neigh;
			return decision->m_dec_id;
		    }
		}
	    }

	    if (decision->m_location_id == traversal.m_v_loc_id && decision->m_time <= traversal.m_interval.m_lower + s_EPSILON)
	    {
		for (sInt_32 neigh = 0; neigh < decision->m_next_dec_IDs.size(); ++neigh)
		{
		    if (kruhobot_RDD[decision->m_next_dec_IDs[neigh]].m_location_id == traversal.m_u_loc_id && kruhobot_RDD[decision->m_next_dec_IDs[neigh]].m_time >= traversal.m_interval.m_upper - s_EPSILON)
		    {
			neighbor = neigh;
			return decision->m_dec_id;
		    }
		}
	    }	    
	}
	return -1;
    }


    sInt_32 sRealSMTCBS::match_CorrespondingSink(const sRealInstance                  &real_Instance,
						 sInt_32                               kruhobot_id,
						 const Traversal                      &traversal,
						 const KruhobotDecisionDiagram_vector &kruhobot_RDD,
						 const KruhobotDecisionMapping_map    &sUNUSED(kruhobot_RDD_mapping)) const
    {
	sInt_32 sink_id = 0;
	
	for (KruhobotDecisionDiagram_vector::const_iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	{	
	    if (real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id] == decision->m_location_id)
	    {
		if (decision->m_location_id == traversal.m_u_loc_id && decision->m_time < traversal.m_interval.m_upper + s_EPSILON)
		{
		    return sink_id;
		}
		++sink_id;
	    }
	}
	return -1;
    }


    sInt_32 sRealSMTCBS::match_CorrespondingSink(const sRealInstance                  &real_Instance,
						 sInt_32                               kruhobot_id,
						 sDouble                               makespan_bound,
						 const Traversal                      &traversal,
						 const KruhobotDecisionDiagram_vector &kruhobot_RDD,
						 const KruhobotDecisionMapping_map    &sUNUSED(kruhobot_RDD_mapping)) const
    {
	sInt_32 sink_id = 0;
	
	for (KruhobotDecisionDiagram_vector::const_iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	{	
	    if (real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id] == decision->m_location_id && decision->m_time <= makespan_bound + s_EPSILON)
	    {
		if (decision->m_location_id == traversal.m_u_loc_id && decision->m_time < traversal.m_interval.m_upper + s_EPSILON)
		{
		    return sink_id;
		}
		++sink_id;
	    }
	}
	return -1;
    }            


    sInt_32 sRealSMTCBS::match_CorrespondingDecisions(const Traversal                      &traversal,
						      const KruhobotDecisionDiagram_vector &kruhobot_RDD,
						      const KruhobotDecisionMapping_map    &kruhobot_RDD_mapping,
						      DecisionIDs_vector                   &decision_IDs) const
    {
	/*
	for (KruhobotDecisionDiagram_vector::const_iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	{
	    if (decision->m_location_id == traversal.m_u_loc_id && decision->m_time >= traversal.m_interval.m_lower - s_EPSILON && decision->m_time <= traversal.m_interval.m_upper + s_EPSILON)
	    {
		decision_IDs.push_back(decision->m_dec_id);
	    }
	}
	*/
	KruhobotDecisionMapping_map::const_iterator mp_decisions_IDs = kruhobot_RDD_mapping.find(traversal.m_u_loc_id);
	sASSERT(mp_decisions_IDs != kruhobot_RDD_mapping.end());	
	KruhobotDecisionIDs_mmap::const_iterator mp_decision = mp_decisions_IDs->second.lower_bound(traversal.m_interval.m_lower);

	while (mp_decision != mp_decisions_IDs->second.end())
	{
	    if (kruhobot_RDD[mp_decision->second].m_time >= traversal.m_interval.m_lower - s_EPSILON)
	    {
		if (kruhobot_RDD[mp_decision->second].m_time <= traversal.m_interval.m_upper + s_EPSILON)
		{
		    decision_IDs.push_back(mp_decision->second);
		}
		else
		{
		    break;
		}
	    }
	    ++mp_decision;
	}

	/*
	printf("\n");
	printf("dps\n");
	for (DecisionIDs_vector::const_iterator deci = decision_IDs.begin(); deci != decision_IDs.end(); ++deci)
	{
	    printf("%d ", *deci);
	}
	printf("\n");
	getchar();
	*/
	
	return decision_IDs.size();
    }


    sInt_32 sRealSMTCBS::match_CorrespondingSinks(const sRealInstance                  &real_Instance,
						  sInt_32                               kruhobot_id,
						  const Traversal                      &traversal,
						  const KruhobotDecisionDiagram_vector &kruhobot_RDD,
						  const KruhobotDecisionMapping_map    &kruhobot_RDD_mapping,
						  std::vector<sInt_32>                 &sink_IDs) const
    {
/*
	sInt_32 sink_id = 0;

	for (KruhobotDecisionDiagram_vector::const_iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	{
	    if (real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id] == decision->m_location_id)
	    {
		if (decision->m_location_id == traversal.m_u_loc_id && decision->m_time < traversal.m_interval.m_upper + s_EPSILON)
		{
		    sink_IDs.push_back(sink_id);
		}
		++sink_id;
	    }
	}
*/
	if (real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id] == traversal.m_u_loc_id)
	{
	    KruhobotDecisionMapping_map::const_iterator mp_decisions_IDs = kruhobot_RDD_mapping.find(traversal.m_u_loc_id);
	    sASSERT(mp_decisions_IDs != kruhobot_RDD_mapping.end());
	    KruhobotDecisionIDs_mmap::const_iterator mp_decision = mp_decisions_IDs->second.begin();
	
	    while (mp_decision != mp_decisions_IDs->second.end())
	    {
		if (kruhobot_RDD[mp_decision->second].m_time < traversal.m_interval.m_upper + s_EPSILON)
		{
	   	    sink_IDs.push_back(kruhobot_RDD[mp_decision->second].m_sink_id);
		}
		++mp_decision;
	    }
	}
	return sink_IDs.size();
    }    


    sInt_32 sRealSMTCBS::match_CorrespondingSinks(const sRealInstance                  &real_Instance,
						  sInt_32                               kruhobot_id,
						  sDouble                               makespan_bound,
						  const Traversal                      &traversal,
						  const KruhobotDecisionDiagram_vector &kruhobot_RDD,
						  const KruhobotDecisionMapping_map    &kruhobot_RDD_mapping,
						  std::vector<sInt_32>                 &sink_IDs) const
    {
/*
	sInt_32 sink_id = 0;

	for (KruhobotDecisionDiagram_vector::const_iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	{
	    if (real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id] == decision->m_location_id)
	    {
		if (decision->m_location_id == traversal.m_u_loc_id && decision->m_time < traversal.m_interval.m_upper + s_EPSILON)
		{
		    sink_IDs.push_back(sink_id);
		}
		++sink_id;
	    }
	}
*/
	if (real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id] == traversal.m_u_loc_id && traversal.m_interval.m_lower <= makespan_bound + s_EPSILON)
	{
	    KruhobotDecisionMapping_map::const_iterator mp_decisions_IDs = kruhobot_RDD_mapping.find(traversal.m_u_loc_id);
	    sASSERT(mp_decisions_IDs != kruhobot_RDD_mapping.end());
	    KruhobotDecisionIDs_mmap::const_iterator mp_decision = mp_decisions_IDs->second.begin();
	
	    while (mp_decision != mp_decisions_IDs->second.end())
	    {
		if (kruhobot_RDD[mp_decision->second].m_time < traversal.m_interval.m_upper + s_EPSILON)
		{
	   	    sink_IDs.push_back(kruhobot_RDD[mp_decision->second].m_sink_id);
		}
		++mp_decision;
	    }
	}
	return sink_IDs.size();
    }    
    
    
    sInt_32 sRealSMTCBS::match_CorrespondingNeighbors(const Traversal                      &traversal,
						      const KruhobotDecisionDiagram_vector &kruhobot_RDD,
						      const KruhobotDecisionMapping_map    &kruhobot_RDD_mapping,
						      DecisionIDs_vector                   &decision_IDs,
						      std::vector<sInt_32>                 &Neighbors) const
    {
	/*
	for (KruhobotDecisionDiagram_vector::const_iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	{	    
	    if (decision->m_location_id == traversal.m_u_loc_id && decision->m_time <= traversal.m_interval.m_lower + s_EPSILON)
	    {
		for (sInt_32 neigh = 0; neigh < decision->m_next_dec_IDs.size(); ++neigh)
		{
		    if (kruhobot_RDD[decision->m_next_dec_IDs[neigh]].m_location_id == traversal.m_v_loc_id && kruhobot_RDD[decision->m_next_dec_IDs[neigh]].m_time >= traversal.m_interval.m_upper - s_EPSILON)
		    {
//			printf("  --> %d <--> %d: %.3f %.3f\n", decision->m_location_id, kruhobot_RDD[decision->m_next_dec_IDs[neigh]].m_location_id, decision->m_time, kruhobot_RDD[decision->m_next_dec_IDs[neigh]].m_time);
			Neighbors.push_back(neigh);
			decision_IDs.push_back(decision->m_dec_id);

//			printf("[%d,%d] ", neigh, decision->m_dec_id);
		    }
		}
	    }

	    if (decision->m_location_id == traversal.m_v_loc_id && decision->m_time <= traversal.m_interval.m_lower + s_EPSILON)
	    {
		for (sInt_32 neigh = 0; neigh < decision->m_next_dec_IDs.size(); ++neigh)
		{
		    if (kruhobot_RDD[decision->m_next_dec_IDs[neigh]].m_location_id == traversal.m_u_loc_id && kruhobot_RDD[decision->m_next_dec_IDs[neigh]].m_time >= traversal.m_interval.m_upper - s_EPSILON)
		    {
			Neighbors.push_back(neigh);
			decision_IDs.push_back(decision->m_dec_id);

//			printf("[%d,%d] ", neigh, decision->m_dec_id);
		    }
		}
	    }	    
	}
//	printf("\n");
*/
	{
	    KruhobotDecisionMapping_map::const_iterator mp_decisions_IDs = kruhobot_RDD_mapping.find(traversal.m_u_loc_id);
	    sASSERT(mp_decisions_IDs != kruhobot_RDD_mapping.end());
	    
	    KruhobotDecisionIDs_mmap::const_iterator mp_decision = mp_decisions_IDs->second.begin();
	    
	    while (mp_decision != mp_decisions_IDs->second.end())
	    {
		if (sABS(kruhobot_RDD[mp_decision->second].m_time - traversal.m_interval.m_lower) < s_EPSILON)
		{
		    for (sInt_32 neigh = 0; neigh < kruhobot_RDD[mp_decision->second].m_next_dec_IDs.size(); ++neigh)
		    {
			if (   kruhobot_RDD[kruhobot_RDD[mp_decision->second].m_next_dec_IDs[neigh]].m_location_id == traversal.m_v_loc_id
			    && sABS(kruhobot_RDD[kruhobot_RDD[mp_decision->second].m_next_dec_IDs[neigh]].m_time - traversal.m_interval.m_upper) < s_EPSILON)
			{
			    Neighbors.push_back(neigh);
			    decision_IDs.push_back(kruhobot_RDD[mp_decision->second].m_dec_id);
			}
		    }
		}
		++mp_decision;
	    }
	}	
	if (traversal.m_v_loc_id != traversal.m_u_loc_id)
	{
	    KruhobotDecisionMapping_map::const_iterator mp_decisions_IDs = kruhobot_RDD_mapping.find(traversal.m_v_loc_id);
	    sASSERT(mp_decisions_IDs != kruhobot_RDD_mapping.end());
	    KruhobotDecisionIDs_mmap::const_iterator mp_decision = mp_decisions_IDs->second.begin();
	    
	    while (mp_decision != mp_decisions_IDs->second.end())
	    {
		if (sABS(kruhobot_RDD[mp_decision->second].m_time - traversal.m_interval.m_lower) < s_EPSILON)
		{
		    for (sInt_32 neigh = 0; neigh < kruhobot_RDD[mp_decision->second].m_next_dec_IDs.size(); ++neigh)
		    {
			if (   kruhobot_RDD[kruhobot_RDD[mp_decision->second].m_next_dec_IDs[neigh]].m_location_id == traversal.m_u_loc_id
			    && sABS(kruhobot_RDD[kruhobot_RDD[mp_decision->second].m_next_dec_IDs[neigh]].m_time - traversal.m_interval.m_upper) < s_EPSILON)
			{
			    Neighbors.push_back(neigh);
			    decision_IDs.push_back(kruhobot_RDD[mp_decision->second].m_dec_id);
			}
		    }
		}
		++mp_decision;
	    }	    
	}
	
	return decision_IDs.size();
    }

    
/*----------------------------------------------------------------------------*/

    void sRealSMTCBS::decode_PathModel(Glucose::Solver                       *solver,
				       const sRealInstance                   &real_Instance,
				       const KruhobotDecisionDiagrams_vector &kruhobot_RDDs,
				       const RealModel                       &real_sat_Model,
				       KruhobotSchedules_vector              &kruhobot_Schedules) const
    {
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();
	kruhobot_Schedules.resize(N_kruhobots + 1);

	KruhobotDesisions_vector kruhobot_Decisions;
	kruhobot_Decisions.resize(N_kruhobots + 1);

	for (sInt_32 i = 0; i < solver->nVars(); i++)
	{
	    sInt_32 literal;
	    
	    if (solver->model[i] != l_Undef)
	    {
		literal = (solver->model[i] == l_True) ? i + 1 : -(i+1);
	    }
	    else
	    {
		sASSERT(false);
	    }

	    if (literal > 0)
	    {
		sInt_32 variable_ID = sABS(literal);
		
		if (variable_ID < real_sat_Model.m_variable_mapping.size())
		{
		    const RealCoordinate &coordinate = real_sat_Model.m_variable_mapping[variable_ID];

		    sInt_32 kruhobot_id = coordinate.m_kruhobot_id;
		    sInt_32 decision_id = coordinate.m_decision_id;
		    
		    sInt_32 location_id = kruhobot_RDDs[kruhobot_id][decision_id].m_location_id;
		    sDouble time = kruhobot_RDDs[kruhobot_id][decision_id].m_time;
		    
		    if (coordinate.m_sink_id < 0)
		    {
//		    sASSERT(kruhobot_Decisions[kruhobot_id].find(time) == kruhobot_Decisions[kruhobot_id].end());
			kruhobot_Decisions[kruhobot_id][time] = location_id;

			/*
  		        #ifdef sDEBUG
			{
			    printf("Extratracted from satisfying kruhobot:%d, loc:%d, time:%.3f, dec:%d\n", kruhobot_id, location_id, time, decision_id);
			}
		        #endif
			*/
		    }
		    else
		    {
			/*
			{
  		            #ifdef sDEBUG
			    {
			        printf("---__ Sinked into goal kruhobot:%d, loc:%d, time:%.3f, dec:%d\n", kruhobot_id, location_id, time, decision_id);
			    }
		            #endif			
                        }
			*/
		    }
		}
	    }
	}

	/*
	#ifdef sDEBUG
	{
	    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	    {
		printf("%d:\n", kruhobot_id);
		for (Desisions_map::const_iterator next_decision = kruhobot_Decisions[kruhobot_id].begin(); next_decision != kruhobot_Decisions[kruhobot_id].end(); ++next_decision)
		{
		    printf("  %.3f, %d\n", next_decision->first, next_decision->second);
		}
	    }
	}
	#endif
	*/

	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
	    if (kruhobot_Decisions[kruhobot_id].size() > 1)
	    {
		Desisions_map::const_iterator prev_decision = kruhobot_Decisions[kruhobot_id].begin();
		Desisions_map::const_iterator next_decision = prev_decision;
		
		for (++next_decision; next_decision != kruhobot_Decisions[kruhobot_id].end(); prev_decision = next_decision++)
		{
		    kruhobot_Schedules[kruhobot_id].push_back(Event(prev_decision->second,
								    next_decision->second,
								    prev_decision->first,
								    next_decision->first));
		}
	    }
	    else
	    {
		Desisions_map::const_iterator decision = kruhobot_Decisions[kruhobot_id].begin();

		kruhobot_Schedules[kruhobot_id].push_back(Event(decision->second,
								decision->second,
								decision->first,
								decision->first));
	    }
	}
    }


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


    sInt_32 sRealSMTCBS::collect_StartKruhobotDecision(sInt_32 start_location_id, const KruhobotDecisionDiagram_vector &kruhobot_RDD) const
    {
	for (KruhobotDecisionDiagram_vector::const_iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	{
	    if (decision->m_location_id == start_location_id)
	    {
		return decision->m_dec_id;
	    }
	}
	return -1;
    }

    
    sInt_32 sRealSMTCBS::collect_GoalKruhobotDecision(sInt_32 goal_location_id, const KruhobotDecisionDiagram_vector &kruhobot_RDD) const
    {
	for (KruhobotDecisionDiagram_vector::const_reverse_iterator decision = kruhobot_RDD.rbegin(); decision != kruhobot_RDD.rend(); ++decision)
	{
	    if (decision->m_location_id == goal_location_id)
	    {
		return decision->m_dec_id;
	    }
	}
	return -1;
    }


    sInt_32 sRealSMTCBS::collect_GoalKruhobotDecision(sInt_32 goal_location_id, sDouble makespan_bound, const KruhobotDecisionDiagram_vector &kruhobot_RDD) const
    {
	for (KruhobotDecisionDiagram_vector::const_reverse_iterator decision = kruhobot_RDD.rbegin(); decision != kruhobot_RDD.rend(); ++decision)
	{
	    if (decision->m_location_id == goal_location_id && decision->m_time <= makespan_bound + s_EPSILON)
	    {
		return decision->m_dec_id;
	    }
	}
	return -1;
    }    


    void sRealSMTCBS::collect_StartKruhobotDecisions(sInt_32 start_location_id, const KruhobotDecisionDiagram_vector &kruhobot_RDD, DecisionIDs_vector &decision_IDs) const
    {
	for (KruhobotDecisionDiagram_vector::const_iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	{
	    if (decision->m_location_id == start_location_id)
	    {
		decision_IDs.push_back(decision->m_dec_id);
	    }
	}
    }

    
    void sRealSMTCBS::collect_GoalKruhobotDecisions(sInt_32 goal_location_id, const KruhobotDecisionDiagram_vector &kruhobot_RDD, DecisionIDs_vector &decision_IDs) const
    {
	for (KruhobotDecisionDiagram_vector::const_iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	{
	    if (decision->m_location_id == goal_location_id)
	    {
		decision_IDs.push_back(decision->m_dec_id);
	    }
	}	
    }


    void sRealSMTCBS::collect_GoalKruhobotDecisions(sInt_32 goal_location_id, sDouble makespan_bound, const KruhobotDecisionDiagram_vector &kruhobot_RDD, DecisionIDs_vector &decision_IDs) const
    {
	for (KruhobotDecisionDiagram_vector::const_iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	{
	    if (decision->m_location_id == goal_location_id && decision->m_time <= makespan_bound + s_EPSILON)
	    {
		decision_IDs.push_back(decision->m_dec_id);
	    }
	}	
    }    


    void sRealSMTCBS::collect_KruhobotExcludedDecision(const sKruhobot                      &kruhobot,
						       const s2DMap                         &map,
						       sInt_32                               decision_id,
						       const KruhobotDecisionDiagram_vector &kruhobot_RDD,
						       DecisionIDs_vector                   &decision_IDs) const
    {   
//	for (KruhobotDecisionDiagram_vector::const_iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	for (sInt_32 next_decision_id = decision_id + 1; next_decision_id < kruhobot_RDD.size(); ++next_decision_id)
	{
//	    if (decision_id != decision->m_dec_id)
	    if (decision_id != next_decision_id)			    
	    {
//		sDouble distance = map.m_Distances[kruhobot_RDD[decision_id].m_location_id][kruhobot_RDD[decision->m_dec_id].m_location_id];
//		sDouble diff_time = sABS(kruhobot_RDD[decision_id].m_time - kruhobot_RDD[decision->m_dec_id].m_time);

		sDouble distance = map.m_straight_Distances[kruhobot_RDD[decision_id].m_location_id][kruhobot_RDD[next_decision_id].m_location_id];
		sDouble diff_time = sABS(kruhobot_RDD[decision_id].m_time - kruhobot_RDD[next_decision_id].m_time);		
		
		sDouble travel_time = distance / kruhobot.m_properties.m_linear_velo;

//		printf("mut check: %d x %d ---- %.3f x %.3f\n", decision_id, next_decision_id, diff_time, travel_time);
		if (travel_time > diff_time + s_EPSILON)
		{
//		    decision_IDs.push_back(decision->m_dec_id);
		    decision_IDs.push_back(next_decision_id);		    
		}
	    }
	}	
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


