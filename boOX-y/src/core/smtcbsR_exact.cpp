/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             boOX 2-211_planck                              */
/*                                                                            */
/*                  (C) Copyright 2018 - 2022 Pavel Surynek                   */
/*                                                                            */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* smtcbsR_exact.cpp / 2-211_planck                                           */
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

    sDouble sRealSMTCBS::find_CostExactShortestNonconflictingSchedules_individualizedConflictRespectful(sRealSolution &real_Solution, sDouble makespan_limit, sDouble cost_limit)
    {
	return find_CostExactShortestNonconflictingSchedules_individualizedConflictRespectful(*m_real_Instance, real_Solution, makespan_limit, cost_limit);	
    }

  
    sDouble sRealSMTCBS::find_CostExactShortestNonconflictingSchedules_individualizedConflictRespectful(const sRealInstance &real_Instance,
													sRealSolution       &sUNUSED(real_Solution),
													sDouble              makespan_limit,
													sDouble              cost_limit)
    {
	sDouble cost;
	KruhobotSchedules_vector kruhobot_Schedules;

	if ((cost = find_CostExactShortestNonconflictingSchedules_individualizedConflictRespectful(real_Instance, kruhobot_Schedules, makespan_limit, cost_limit)) < 0)
	{
	    return cost;
	}
	
	return -1;
    }


    sDouble sRealSMTCBS::find_CostExactShortestNonconflictingSchedules_individualizedConflictRespectful(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit, sDouble cost_limit)
    {
	return find_CostExactShortestNonconflictingSchedules_individualizedConflictRespectful(*m_real_Instance, kruhobot_Schedules, makespan_limit, cost_limit);	
    }


    sDouble sRealSMTCBS::find_CostExactShortestNonconflictingSchedules_individualizedConflictRespectful(const sRealInstance      &real_Instance,
													KruhobotSchedules_vector &kruhobot_Schedules,
													sDouble                   makespan_limit,
													sDouble                   cost_limit)
    {
	return find_CostExactShortestNonconflictingSchedules_individualizedConflictRespectful(real_Instance, kruhobot_Schedules, makespan_limit, cost_limit, 0.0);
    }    

    
    sDouble sRealSMTCBS::find_CostExactShortestNonconflictingSchedules_individualizedConflictRespectful(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit, sDouble cost_limit, sDouble extra_cost)
    {
	return find_CostExactShortestNonconflictingSchedules_individualizedConflictRespectful(*m_real_Instance, kruhobot_Schedules, makespan_limit, cost_limit, extra_cost);	
    }

    
    sDouble sRealSMTCBS::find_CostExactShortestNonconflictingSchedules_individualizedConflictRespectful(const sRealInstance      &real_Instance,
													KruhobotSchedules_vector &kruhobot_Schedules,
													sDouble                   makespan_limit,
													sDouble                   cost_limit,
													sDouble                   extra_cost)
    {
	sDouble solution_cost;
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

	    if ((solution_cost = find_CostExactNonconflictingSchedules_individualizedConflictRespectful(real_Instance,
													kruhobot_location_Conflicts,
													kruhobot_linear_Conflicts,
													kruhobot_Schedules,
													makespan_limit,
													cost_limit,
													extra_cost)) >= 0.0)
	    {
		return solution_cost;
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

    sDouble sRealSMTCBS::find_DomularCostExactShortestNonconflictingSchedules_individualizedConflictRespectful(sRealSolution &real_Solution, sDouble makespan_limit, sDouble cost_limit)
    {
	return find_DomularCostExactShortestNonconflictingSchedules_individualizedConflictRespectful(*m_real_Instance, real_Solution, makespan_limit, cost_limit);	
    }

  
    sDouble sRealSMTCBS::find_DomularCostExactShortestNonconflictingSchedules_individualizedConflictRespectful(const sRealInstance &real_Instance,
													       sRealSolution       &sUNUSED(real_Solution),
													       sDouble              makespan_limit,
													       sDouble              cost_limit)
    {
	sDouble cost;
	KruhobotSchedules_vector kruhobot_Schedules;

	if ((cost = find_DomularCostExactShortestNonconflictingSchedules_individualizedConflictRespectful(real_Instance, kruhobot_Schedules, makespan_limit, cost_limit)) < 0)
	{
	    return cost;
	}
	
	return -1;
    }


    sDouble sRealSMTCBS::find_DomularCostExactShortestNonconflictingSchedules_individualizedConflictRespectful(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit, sDouble cost_limit)
    {
	return find_DomularCostExactShortestNonconflictingSchedules_individualizedConflictRespectful(*m_real_Instance, kruhobot_Schedules, makespan_limit, cost_limit);	
    }


    sDouble sRealSMTCBS::find_DomularCostExactShortestNonconflictingSchedules_individualizedConflictRespectful(const sRealInstance      &real_Instance,
													       KruhobotSchedules_vector &kruhobot_Schedules,
													       sDouble                   makespan_limit,
													       sDouble                   cost_limit)
    {
	return find_DomularCostExactShortestNonconflictingSchedules_individualizedConflictRespectful(real_Instance, kruhobot_Schedules, makespan_limit, cost_limit, 0.0);
    }    

    
    sDouble sRealSMTCBS::find_DomularCostExactShortestNonconflictingSchedules_individualizedConflictRespectful(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit, sDouble cost_limit, sDouble extra_cost)
    {
	return find_DomularCostExactShortestNonconflictingSchedules_individualizedConflictRespectful(*m_real_Instance, kruhobot_Schedules, makespan_limit, cost_limit, extra_cost);	
    }

    
    sDouble sRealSMTCBS::find_DomularCostExactShortestNonconflictingSchedules_individualizedConflictRespectful(const sRealInstance      &real_Instance,
													       KruhobotSchedules_vector &kruhobot_Schedules,
													       sDouble                   makespan_limit,
													       sDouble                   cost_limit,
													       sDouble                   extra_cost)
    {
	sDouble solution_cost;
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

	    if ((solution_cost = find_DomularCostExactNonconflictingSchedules_individualizedConflictRespectful(real_Instance,
													       kruhobot_location_Conflicts,
													       kruhobot_linear_Conflicts,
													       kruhobot_Schedules,
													       makespan_limit,
													       cost_limit,
													       extra_cost)) >= 0.0)
	    {
		return solution_cost;
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

//	bool same_conflict_check = true;

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

		if ((kruhobot_makespan_lower_bound = find_KruhobotIgnoringSchedule_superStrong(real_Instance.m_Kruhobots[kruhobot_id],
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

	    /*
	    printf("***************** Initial *****************\n");
	    sRealCBSBase::to_Screen(kruhobot_Schedules);
	    printf("***************************************************\n");	    
	    */	    
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
	    sASSERT(!kruhobot_Schedules.empty());
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
	KruhobotCollisions_mset next_kruhobot_Collisions, save_next_kruhobot_Collisions;
	KruhobotCollisions_mset effective_next_kruhobot_Collisions;

	cumulative_makespan = analyze_NonconflictingSchedules_exactNonprioritized(real_Instance,
										  kruhobot_Schedules,
										  next_kruhobot_Collisions);
	if (next_kruhobot_Collisions.empty())
	{
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

	/*
  	#ifdef sDEBUG
	{
	    printf("Next collisions (initial):\n");
	    for (KruhobotCollisions_mset::const_iterator collision = next_kruhobot_Collisions.begin(); collision != next_kruhobot_Collisions.end(); ++collision)
	    {
		collision->to_Screen(s_INDENT);
	    }
	    //getchar();
	}
        #endif	
	*/

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
		    effective_next_kruhobot_Collisions.insert(*next_collision);		    
		}
	    }
	}

        #ifdef sVERBOSE
	{
	    /*
	    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	    {
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
	    }
	    */
	}
        #endif

	if (effective_conflicts <= 0)
	{	    
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

		/*
		#ifdef sDEBUG
		{
		    printf("Bound: %.3f\n", makespan_bound);
		    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
		    {
			printf("%d: %.3f\n", kruhobot_id, kruhobot_next_makespan_Bounds[kruhobot_id]);
		    }
		}
		#endif
		*/

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
			printf("RDD: %d\n", kruhobot_id);
			to_Screen(kruhobot_RDDs[kruhobot_id]);

			printf("next RDD: %d\n", kruhobot_id);
			to_Screen(next_kruhobot_RDDs[kruhobot_id]);			
			*/
                        /*
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
		    //getchar();
		    #endif
		}
		sASSERT(next_makespan_bound > 0.0 || fingerprint_limit >= 0);
	
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

		    /*
		    printf("****************************** Schedules A ******************************\n");
		    sRealCBSBase::to_Screen(kruhobot_Schedules);
		    printf("*************************----- Schedules A -----*************************\n");
		    */
		    
		    if (!finding_result)
		    {
			sASSERT(next_makespan_bound > makespan_bound + s_EPSILON || fingerprint_limit >= 0);
						
                        #ifdef sVERBOSE	    
			{
			    sDouble end_time = sStatistics::get_CPU_Seconds();
			    
			    printf("%sProving non-existence of solution at the current stage [fresh] (elapsed time [seconds]: %.3f, current makespan: %.3f, next makespan: %.3f)...\n",
				   s_INDENT.c_str(),
				   (end_time - start_time),
				   makespan_bound,
				   next_makespan_bound > 0 ? next_makespan_bound : makespan_bound);
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

			#ifdef sDEBUG
			{
			    /*
			    printf("Finger: %d\n", fingerprint_limit);
			    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			    {
				printf("LB %d: %.3f\n", kruhobot_id, kruhobot_makespan_lower_Bounds[kruhobot_id]);
			    }
			    */
			}
			#endif
			
			if (!individual_increase)
			{
			    if (fingerprint_limit < 0)
			    {		       	
				sASSERT(next_makespan_bound > makespan_bound + s_EPSILON);

				bool all_on_bound = true;				
				for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
				{
				    if (sABS(kruhobot_makespan_lower_Bounds[kruhobot_id] - makespan_bound) > s_EPSILON)
				    {
					all_on_bound = false;
				    }
				}

				if (all_on_bound)
				{				    
				    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
				    {
					kruhobot_makespan_lower_Bounds[kruhobot_id] = next_makespan_bound;
				    }
				    real_context.m_makespan_bound = makespan_bound = next_makespan_bound;
				}
				else
				{
				    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)					
				    {
					if (sABS(kruhobot_makespan_lower_Bounds[kruhobot_id] - kruhobot_next_makespan_Bounds[kruhobot_id]) > s_EPSILON)
					{
					    kruhobot_makespan_lower_Bounds[kruhobot_id] = kruhobot_next_makespan_Bounds[kruhobot_id] < makespan_bound ? kruhobot_next_makespan_Bounds[kruhobot_id] : makespan_bound;
					}
				    }
				}
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
									   effective_next_kruhobot_Collisions,
									   real_sat_Model,
									   kruhobot_Schedules);

		    /*
		    printf("****************************** Schedules B (%d) ******************************\n", finding_result);
		    sRealCBSBase::to_Screen(kruhobot_Schedules);
		    printf("*************************----- Schedules B (%d) -----*************************\n", finding_result);
		    */

  	            #ifdef sDEBUG
		    {
			/*
			printf("Collisions:\n");
			for (KruhobotCollisions_mset::const_iterator collision = kruhobot_Collisions.begin(); collision != kruhobot_Collisions.end(); ++collision)
			{
			    collision->to_Screen(s_INDENT);
			}
			
			printf("Next effective collisions:\n");
			for (KruhobotCollisions_mset::const_iterator collision = effective_next_kruhobot_Collisions.begin(); collision != effective_next_kruhobot_Collisions.end(); ++collision)
			{
			    collision->to_Screen(s_INDENT);
			}
			*/
		    }
	            #endif
		    
		    if (!finding_result)
		    {
//			sASSERT(next_makespan_bound > makespan_bound + s_EPSILON);
			
                        #ifdef sVERBOSE	    
			{
			    sDouble end_time = sStatistics::get_CPU_Seconds();

			    printf("%sProving non-existence of solution at the current stage [augmented] (elapsed time [seconds]: %.3f, current makespan: %.3f, next makespan: %.3f)...\n",
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
				sASSERT(next_makespan_bound > makespan_bound + s_EPSILON);

				bool all_on_bound = true;				
				for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
				{
				    if (sABS(kruhobot_makespan_lower_Bounds[kruhobot_id] - makespan_bound) > s_EPSILON)
				    {
					all_on_bound = false;
				    }
				}

				if (all_on_bound)
				{				    
				    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
				    {
					kruhobot_makespan_lower_Bounds[kruhobot_id] = next_makespan_bound;
				    }
				    real_context.m_makespan_bound = makespan_bound = next_makespan_bound;
				}
				else
				{
				    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)					
				    {
					if (sABS(kruhobot_makespan_lower_Bounds[kruhobot_id] - kruhobot_next_makespan_Bounds[kruhobot_id]) > s_EPSILON)
					{
					    kruhobot_makespan_lower_Bounds[kruhobot_id] = kruhobot_next_makespan_Bounds[kruhobot_id] < makespan_bound ? kruhobot_next_makespan_Bounds[kruhobot_id] : makespan_bound;
					}
				    }
				}
				fingerprint_limit = 1;
			    }
			    else
			    {
				fingerprint_limit = -1;
			    }
			}
			continue;
		    }		    
		}
		sDouble cumulative_makespan;
		next_kruhobot_Collisions.clear();

		cumulative_makespan = analyze_NonconflictingSchedules_exactNonprioritized(real_Instance,
											  kruhobot_Schedules,
											  next_kruhobot_Collisions);

		if (next_kruhobot_Collisions.empty())
		{
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
		effective_next_kruhobot_Collisions.clear();		

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
			    effective_next_kruhobot_Collisions.insert(*next_collision);
			}
		    }
		}
		
		#ifdef sVERBOSE
		{
		    /*
		    printf("  Linear conflicts\n");
		    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
		    {
			printf("    Kruhobot: %d\n", kruhobot_id);
			for (UlinearConflicts_upper__map::const_iterator conflict = kruhobot_linear_Conflicts[kruhobot_id].begin(); conflict != kruhobot_linear_Conflicts[kruhobot_id].end(); ++conflict)
			{
OA			    for (UlinearConflicts_upper_map::const_iterator _conflict = conflict->second.begin(); _conflict != conflict->second.end(); ++_conflict)
			    {
				_conflict->second.to_Screen("      ");
			    }
			}
		    }
		    */
		}
		#endif

		if (effective_conflicts <= 0)
		{
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


    sDouble sRealSMTCBS::find_CostExactNonconflictingSchedules_individualizedConflictRespectful(const sRealInstance                    &real_Instance,
												KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
												KruhobotUlinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
												KruhobotSchedules_vector               &kruhobot_Schedules,
												sDouble                                 makespan_limit,
												sDouble                                 cost_limit,
												sDouble                                 extra_cost)
    {	
	sInt_32 last_conflict_id = 0;
	
	sDouble makespan_bound = 0.0;
	sDouble cost_bound = 0.0;
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();

	#ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif
    
	std::vector<sDouble> kruhobot_cost_lower_Bounds;
	kruhobot_cost_lower_Bounds.resize(N_kruhobots + 1);

	std::vector<sDouble> kruhobot_next_cost_Bounds;	
	kruhobot_next_cost_Bounds.resize(N_kruhobots + 1);

	std::vector<sDouble> kruhobot_super_next_cost_Bounds;	
	kruhobot_super_next_cost_Bounds.resize(N_kruhobots + 1);		

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
	    sDouble makespan_lower_bound = 0.0;
	    
	    sDouble cost_lower_bound = 0.0;
	    sDouble next_cost_bound = -1.0;

	    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	    {
		sDouble kruhobot_makespan_lower_bound;
		
		if ((kruhobot_makespan_lower_bound = find_KruhobotIgnoringSchedule_superStrong(real_Instance.m_Kruhobots[kruhobot_id],
											       *real_Instance.m_start_conjunction.m_Map,
											       real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
											       real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],
											       makespan_limit,
											       0.0,
											       kruhobot_Schedules[kruhobot_id])) < 0.0)
		{
		    return -1.0;
		}
		makespan_lower_bound = (makespan_lower_bound < kruhobot_makespan_lower_bound) ? kruhobot_makespan_lower_bound : makespan_lower_bound;
		cost_lower_bound += kruhobot_makespan_lower_bound;
		kruhobot_cost_lower_Bounds[kruhobot_id] = kruhobot_next_cost_Bounds[kruhobot_id] = kruhobot_makespan_lower_bound;
	    }
	    
	    #ifdef sDEBUG
	    {
		printf("Lower makespan: %.3f\n", makespan_lower_bound);
		printf("Lower cost: %.3f\n", cost_lower_bound);
	    }
	    #endif
    
	    real_context.m_makespan_bound = makespan_bound = makespan_lower_bound;
	    real_context.m_cost_bound = cost_bound = cost_lower_bound;
	    
            #ifdef sVERBOSE	    
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		
		printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current cost: %.3f [makespan: %.3f])...\n",
		       (end_time - start_time),
		       cost_bound, makespan_bound);
	    }
	    #endif

	    sDouble next_extra_cost = 0.0;
	    sDouble next_sink_extra_cost = -1.0;


	    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	    {
		sDouble nx_cost_bound, individual_extra_cost;
		sDouble sink_individual_extra = -1.0;

		SinkATTs_set sink_ATTs;
		nx_cost_bound = build_KruhobotCostRealDecisionDiagram_individualizedConflictRespectfulBucketingLeaping(real_Instance.m_Kruhobots[kruhobot_id],
														       *real_Instance.m_start_conjunction.m_Map,
														       real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
														       real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],
														       kruhobot_location_Conflicts[kruhobot_id],
														       kruhobot_linear_Conflicts[kruhobot_id],
														       makespan_bound,
														       kruhobot_cost_lower_Bounds[kruhobot_id],
														       -1,
														       kruhobot_RDDs[kruhobot_id],
														       kruhobot_RDD_Mappings[kruhobot_id],
														       0.0,
														       sink_ATTs);
/*
		#ifdef sDEBUG
		{
		    printf("Sink ATTs 1\n");
		    for (SinkATTs_set::const_iterator sink_att = sink_ATTs.begin(); sink_att != sink_ATTs.end(); ++sink_att)
		    {
			printf("%.3f ", *sink_att);
		    }
		    printf("\n");
		}
                #endif

		to_Screen(kruhobot_RDDs[kruhobot_id]);
*/		    
		if (nx_cost_bound > 0.0)
		{
		    individual_extra_cost = nx_cost_bound - kruhobot_cost_lower_Bounds[kruhobot_id];
		    next_cost_bound = (next_cost_bound < 0.0) ? nx_cost_bound : sMIN(next_cost_bound, nx_cost_bound);
		}
//		printf("Individual extra: %.3f\n", individual_extra_cost);

		if(next_extra_cost <= 0.0)
		{
		    next_extra_cost = individual_extra_cost;
		}
		else
		{
		    next_extra_cost = sMIN(next_extra_cost, individual_extra_cost);
		}

		if (sink_ATTs.size() >= 2)
		{
		    SinkATTs_set::const_iterator sink_att = sink_ATTs.begin();
		    sink_individual_extra = -(*sink_att - *(++sink_att));
		}
		else
		{
		    sink_individual_extra = 1.0;
		}
		kruhobot_super_next_cost_Bounds[kruhobot_id] = kruhobot_next_cost_Bounds[kruhobot_id] + sink_individual_extra;
		
		if (next_sink_extra_cost <= -1.0)
		{
		    next_sink_extra_cost = sink_individual_extra;
		}
		else
		{
		    next_sink_extra_cost = sMIN(next_sink_extra_cost, sink_individual_extra);
		}			
	    }	    	    
	    sASSERT(next_extra_cost > 0.0);
    
            #ifdef sVERBOSE	    
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();

		printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current cost: %.3f [makespan: %.3f])...\n",
		       (end_time - start_time),
		       cost_bound, makespan_bound);
		
//		to_Screen(kruhobot_RDDs);		
	    }
	    #endif

	    kruhobot_Schedules.clear();
	    find_InitialNonconflictingSchedules(solver,
						real_context,					 
						real_Instance,
						kruhobot_RDDs,
						kruhobot_RDD_Mappings,
						cost_bound,
						kruhobot_cost_lower_Bounds,						
						real_sat_Model,
						kruhobot_Schedules);
	    sASSERT(!kruhobot_Schedules.empty());
	    
	    //sRealCBSBase::to_Screen(kruhobot_Schedules);	    

	    if (m_timeout >= 0.0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		
		if (end_time - start_time > m_timeout)
		{
		    return -2.0;
		}
	    }
	    if (makespan_bound > makespan_limit || cost_bound > cost_limit + extra_cost)
	    {
		return -3.0;
	    }
	}

	KruhobotCollisions_mset kruhobot_Collisions;
	KruhobotCollisions_mset next_kruhobot_Collisions, save_next_kruhobot_Collisions;
	KruhobotCollisions_mset effective_next_kruhobot_Collisions;

	analyze_NonconflictingSchedules_exactNonprioritized(real_Instance,
							    kruhobot_Schedules,
							    next_kruhobot_Collisions);
	if (next_kruhobot_Collisions.empty())
	{
	    printf("COLLISION-FREE solution found !\n");
	    
            #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Elapsed time [seconds]: %.3f, current cost: %.3f [makespan: %.3f])...\n", (end_time - start_time),
		       cost_bound, makespan_bound);		
	    }
            #endif
	    
	    return cost_bound;
	}

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
	}

	if (effective_conflicts <= 0)
	{
	    printf("COLLISION-FREE solution found !\n");
	    
            #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		
		printf("Elapsed time [seconds]: %.3f, current cost: %.3f [makespan: %.3f])...\n", (end_time - start_time),
		       cost_bound, makespan_bound);
	    }
            #endif
	    
	    return cost_bound;
	}

        #ifdef sVERBOSE	    
	{
	    sDouble end_time = sStatistics::get_CPU_Seconds();

	    printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current cost: %.3f [makespan: %.3f])...\n",
		   (end_time - start_time),
		   cost_bound, makespan_bound);
	}
	#endif

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
	    if (makespan_bound > makespan_limit || cost_bound > cost_limit + extra_cost)
	    {
		return -3.0;
	    }
	    
	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current cost: %.3f [makespan: %.3f])...\n",
			   (end_time - start_time),
			   cost_bound, makespan_bound);
		    
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

		sDouble next_extra_cost = 0.0;
		sDouble next_sink_extra_cost = 0.0;
		bool next_iteration = false;		
		{
		    sDouble next_cost_bound = -1.0;
//		    kruhobot_next_cost_Bounds = kruhobot_cost_lower_Bounds;

		    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
		    {
			sDouble individual_extra_cost;			
			sDouble nx_cost_bound;

			sDouble sink_individual_extra = -1.0;

			SinkATTs_set sink_ATTs;			
			nx_cost_bound = build_KruhobotCostRealDecisionDiagram_individualizedConflictRespectfulBucketingLeaping(real_Instance.m_Kruhobots[kruhobot_id],
															       *real_Instance.m_start_conjunction.m_Map,
															       real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
															       real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],	    
															       kruhobot_location_Conflicts[kruhobot_id],
															       kruhobot_linear_Conflicts[kruhobot_id],
															       makespan_bound,
															       kruhobot_next_cost_Bounds[kruhobot_id],
															       fingerprint_limit,
															       next_kruhobot_RDDs[kruhobot_id],
															       next_kruhobot_RDD_Mappings[kruhobot_id],
															       0.0,
															       sink_ATTs);
/*
                        #ifdef sDEBUG
			{
			    printf("Sink ATTs 1: %.3f\n", kruhobot_next_cost_Bounds[kruhobot_id]);
			    for (SinkATTs_set::const_iterator sink_att = sink_ATTs.begin(); sink_att != sink_ATTs.end(); ++sink_att)
			    {
				printf("%.3f ", *sink_att);
			    }
			    printf("\n");
			}
                        #endif
*/

			if (sink_ATTs.size() >= 2)
			{
			    SinkATTs_set::const_iterator sink_att = sink_ATTs.begin();
			    sink_individual_extra = -(*sink_att - *(++sink_att));
			}
			else
			{
			    sink_individual_extra = 1.0;
			}
			kruhobot_super_next_cost_Bounds[kruhobot_id] = kruhobot_next_cost_Bounds[kruhobot_id] + sink_individual_extra;
			
			if (nx_cost_bound > 0.0)
			{
			    individual_extra_cost = nx_cost_bound - kruhobot_next_cost_Bounds[kruhobot_id];
			    next_cost_bound = (next_cost_bound < 0.0) ? nx_cost_bound : sMIN(next_cost_bound, nx_cost_bound);
			}
			/*
			printf("Individual extra: %.3f\n", individual_extra_cost);
			printf("Sink individual: %.3f\n", sink_individual_extra);
			*/

			if (next_extra_cost <= 0.0)
			{
			    next_extra_cost = individual_extra_cost;
			}
			else
			{
			    next_extra_cost = sMIN(next_extra_cost, individual_extra_cost);
			}

			if (next_sink_extra_cost <= 0.0)
			{
			    next_sink_extra_cost = sink_individual_extra;
			}
			else
			{
			    next_sink_extra_cost = sMIN(next_sink_extra_cost, sink_individual_extra);
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
		}
		//to_Screen(kruhobot_RDDs);		
		//sASSERT(next_extra_cost > 0.0);

		KruhobotExtraVariables_vector kruhobot_set_extra_Variables;
		kruhobot_set_extra_Variables.resize(N_kruhobots + 1);
		KruhobotExtraVariables_vector kruhobot_all_extra_Variables;
		kruhobot_all_extra_Variables.resize(N_kruhobots + 1);
		KruhobotExtraVariables_vector kruhobot_envelope_Variables;
		kruhobot_envelope_Variables.resize(N_kruhobots + 1);
		KruhobotExtraVariables_vector kruhobot_domus_Variables;
		kruhobot_domus_Variables.resize(N_kruhobots + 1);						

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
									   cost_bound,
									   kruhobot_cost_lower_Bounds,
									   next_real_sat_Model,
									   kruhobot_Schedules,
									   kruhobot_set_extra_Variables,
									   kruhobot_all_extra_Variables);
		    
		    /*
		    #ifdef sDEBUG
		    {
			printf("All extra variables\n");
			for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			{
			    printf("  Kruhobot: %d [%.3f]\n", kruhobot_id, kruhobot_cost_lower_Bounds[kruhobot_id]);
			    printf("    ");
			    for (ExtraVariables_map::const_iterator extra_variable = kruhobot_all_extra_Variables[kruhobot_id].begin(); extra_variable != kruhobot_all_extra_Variables[kruhobot_id].end(); ++extra_variable)
			    {
				printf("%.3f,%d ", extra_variable->first, extra_variable->second);
			    }
			    printf("\n");
			}

			printf("SET (to TRUE) extra variables\n");
			for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			{
			    printf("  Kruhobot: %d [%.3f]\n", kruhobot_id, kruhobot_cost_lower_Bounds[kruhobot_id]);
			    printf("    ");
			    for (ExtraVariables_map::const_iterator extra_variable = kruhobot_set_extra_Variables[kruhobot_id].begin(); extra_variable != kruhobot_set_extra_Variables[kruhobot_id].end(); ++extra_variable)
			    {
				printf("%.3f,%d ", extra_variable->first, extra_variable->second);
			    }
			    printf("\n");
			}			
		    }
		    #endif
		    */
	    
		    if (!finding_result)
		    {
			sASSERT(next_extra_cost > s_EPSILON);
		    
                        #ifdef sVERBOSE	    
			{
			    sDouble end_time = sStatistics::get_CPU_Seconds();
			    
			    printf("%sUnable (1) to solve newly built instance (elapsed time [seconds]: %.3f, current cost: %.3f, [makespan: %.3f])...\n",
				   s_INDENT.c_str(),
				   (end_time - start_time),
				   cost_bound, makespan_bound);
			}
	                #endif
		    
			if (fingerprint_limit < 0)
			{
			    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			    {
				kruhobot_next_cost_Bounds[kruhobot_id] += next_extra_cost;//kruhobot_super_next_cost_Bounds[kruhobot_id];
			    }
			    real_context.m_makespan_bound = makespan_bound = makespan_bound + next_extra_cost;
			    real_context.m_cost_bound = cost_bound = cost_bound + next_extra_cost;
			    
			    /*
			    real_context.m_makespan_bound = makespan_bound = makespan_bound + next_sink_extra_cost;
			    real_context.m_cost_bound = cost_bound = cost_bound + next_sink_extra_cost;			    
			    */
			    
			    fingerprint_limit = 1;
			}
			else
			{
			    fingerprint_limit = -1;
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
									   cost_bound,
									   kruhobot_cost_lower_Bounds,
									   real_sat_Model,
									   kruhobot_Schedules,
									   kruhobot_set_extra_Variables,
									   kruhobot_all_extra_Variables);

		    /*
		    #ifdef sDEBUG
		    {
			printf("All extra variables\n");
			for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			{
			    printf("  Kruhobot: %d [%.3f]\n", kruhobot_id, kruhobot_cost_lower_Bounds[kruhobot_id]);
			    printf("    ");
			    for (ExtraVariables_map::const_iterator extra_variable = kruhobot_all_extra_Variables[kruhobot_id].begin(); extra_variable != kruhobot_all_extra_Variables[kruhobot_id].end(); ++extra_variable)
			    {
				printf("%.3f,%d ", extra_variable->first, extra_variable->second);
			    }
			    printf("\n");
			}

			printf("SET (to TRUE) extra variables\n");
			for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			{
			    printf("  Kruhobot: %d [%.3f]\n", kruhobot_id, kruhobot_cost_lower_Bounds[kruhobot_id]);
			    printf("    ");
			    for (ExtraVariables_map::const_iterator extra_variable = kruhobot_set_extra_Variables[kruhobot_id].begin(); extra_variable != kruhobot_set_extra_Variables[kruhobot_id].end(); ++extra_variable)
			    {
				printf("%.3f,%d ", extra_variable->first, extra_variable->second);
			    }
			    printf("\n");
			}			
		    }
		    #endif		    
		    */
		    
		    if (!finding_result)
		    {
                        #ifdef sVERBOSE	    
			{
			    sDouble end_time = sStatistics::get_CPU_Seconds();

			    printf("%sUnable (2) to solve newly built instance (elapsed time [seconds]: %.3f, current cost: %.3f, [makespan: %.3f])...\n",
				   s_INDENT.c_str(),
				   (end_time - start_time),
				   cost_bound, makespan_bound);
			}
	                #endif

			if (fingerprint_limit < 0)
			{
			    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			    {
				kruhobot_next_cost_Bounds[kruhobot_id] += next_extra_cost;//next_sink_extra_cost;//kruhobot_super_next_cost_Bounds[kruhobot_id];
			    }
			    /*
			    real_context.m_makespan_bound = makespan_bound = makespan_bound + next_extra_cost;
			    real_context.m_cost_bound = cost_bound = cost_bound + next_extra_cost;
			    */
			    real_context.m_makespan_bound = makespan_bound = makespan_bound + next_extra_cost;//next_sink_extra_cost;
			    real_context.m_cost_bound = cost_bound = cost_bound + next_extra_cost;//next_sink_extra_cost;
			    
			    fingerprint_limit = 1;
			}
			else
			{
			    fingerprint_limit = -1;
			}
			continue;
		    }
		}
		/*
		augment_Schedules(real_Instance, kruhobot_Schedules);
		*/
		
		next_kruhobot_Collisions.clear();
	
		analyze_NonconflictingSchedules_exactNonprioritized(real_Instance,
								    kruhobot_Schedules,
								    next_kruhobot_Collisions);
		if (next_kruhobot_Collisions.empty())
		{
		    printf("COLLISION-FREE solution found !\n");

  	            #ifdef sVERBOSE
		    {
			sDouble end_time = sStatistics::get_CPU_Seconds();

			printf("Elapsed time [seconds]: %.3f, current cost: %.3f [makespan: %.3f])...\n", (end_time - start_time),
			       cost_bound, makespan_bound);			
		    }
 	            #endif

		    sASSERT(false);
		    return cost_bound;
		}

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
		}

		if (effective_conflicts <= 0)
		{		    
                    #ifdef sVERBOSE	    
		    {
			sDouble end_time = sStatistics::get_CPU_Seconds();
			
			printf("Optimizing cost (elapsed time [seconds]: %.3f)...\n",
			       (end_time - start_time));			    
		    }
	            #endif

		    //return cost_bound;
		    		    
		    while (true)
		    {
			kruhobot_envelope_Variables.clear();
			kruhobot_envelope_Variables.resize(N_kruhobots + 1);
			kruhobot_domus_Variables.clear();
			kruhobot_domus_Variables.resize(N_kruhobots + 1);
			
			sDouble total_cost = analyze_NonconflictingSchedulesCosts(real_Instance,
										  kruhobot_Schedules,
										  cost_bound,
										  kruhobot_cost_lower_Bounds,
										  kruhobot_set_extra_Variables,
										  kruhobot_all_extra_Variables,
										  kruhobot_envelope_Variables,
										  kruhobot_domus_Variables);
			//sRealCBSBase::to_Screen(kruhobot_Schedules);			
			//printf("Tot vs. cost: %.3f, %.3f\n", total_cost, cost_bound);

			{
			    /*
			    printf("Extra ALL variables\n");
			    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			    {
				printf("  Kruhobot: %d [%.3f]\n", kruhobot_id, kruhobot_cost_lower_Bounds[kruhobot_id]);
				printf("    ");
				for (ExtraVariables_mmap::const_iterator extra_variable = kruhobot_all_extra_Variables[kruhobot_id].begin(); extra_variable != kruhobot_all_extra_Variables[kruhobot_id].end(); ++extra_variable)
				{
				    printf("%.3f,%d ", extra_variable->first, extra_variable->second);
				}
				printf("\n");
			    }
			    
			    printf("Extra SET variables\n");
			    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			    {
				printf("  Kruhobot: %d [%.3f]\n", kruhobot_id, kruhobot_cost_lower_Bounds[kruhobot_id]);
				printf("    ");
				for (ExtraVariables_mmap::const_iterator extra_variable = kruhobot_set_extra_Variables[kruhobot_id].begin(); extra_variable != kruhobot_set_extra_Variables[kruhobot_id].end(); ++extra_variable)
				{
				    printf("%.3f,%d ", extra_variable->first, extra_variable->second);
				}
				printf("\n");
			    }
			    
			    printf("Extra ENVELOPE variables\n");
			    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			    {
				printf("  Kruhobot: %d [%.3f]\n", kruhobot_id, kruhobot_cost_lower_Bounds[kruhobot_id]);
				printf("    ");
				for (ExtraVariables_mmap::const_iterator extra_variable = kruhobot_envelope_Variables[kruhobot_id].begin(); extra_variable != kruhobot_envelope_Variables[kruhobot_id].end(); ++extra_variable)
				{
				    printf("%.3f,%d ", extra_variable->first, extra_variable->second);
				}
				printf("\n");
			    }

			    printf("Extra DOMUS variables\n");
			    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			    {
				printf("  Kruhobot: %d [%.3f]\n", kruhobot_id, kruhobot_cost_lower_Bounds[kruhobot_id]);
				printf("    ");
				for (ExtraVariables_mmap::const_iterator extra_variable = kruhobot_domus_Variables[kruhobot_id].begin(); extra_variable != kruhobot_domus_Variables[kruhobot_id].end(); ++extra_variable)
				{
				    printf("%.3f,%d ", extra_variable->first, extra_variable->second);
				}
				printf("\n");
			    }
			    */			    
			}
			
			if (total_cost > cost_bound)
			{
			    /*
			    printf("Extra ENVELOPE variables\n");
			    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			    {
				printf("  Kruhobot: %d [%.3f]\n", kruhobot_id, kruhobot_cost_lower_Bounds[kruhobot_id]);
				printf("    ");
				for (ExtraVariables_mmap::const_iterator extra_variable = kruhobot_envelope_Variables[kruhobot_id].begin(); extra_variable != kruhobot_envelope_Variables[kruhobot_id].end(); ++extra_variable)
				{
				    printf("%.3f,%d ", extra_variable->first, extra_variable->second);
				}
				printf("\n");
			    }
			    
			    printf("Extra DOMUS variables\n");
			    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			    {
				printf("  Kruhobot: %d [%.3f]\n", kruhobot_id, kruhobot_cost_lower_Bounds[kruhobot_id]);
				printf("    ");
				for (ExtraVariables_mmap::const_iterator extra_variable = kruhobot_domus_Variables[kruhobot_id].begin(); extra_variable != kruhobot_domus_Variables[kruhobot_id].end(); ++extra_variable)
				{
				    printf("%.3f,%d ", extra_variable->first, extra_variable->second);
				}
				printf("\n");
			    }
			    getchar();
			    */
			    kruhobot_Schedules.clear();

			    kruhobot_set_extra_Variables.clear();
			    kruhobot_set_extra_Variables.resize(N_kruhobots + 1);
			    kruhobot_all_extra_Variables.clear();
			    kruhobot_all_extra_Variables.resize(N_kruhobots + 1);
			    
			    bool finding_result = find_BetterNonconflictingSchedules(solver,
										     real_context,
										     real_Instance,
										     kruhobot_RDDs,
										     kruhobot_RDD_Mappings,
										     kruhobot_Collisions,
										     next_kruhobot_Collisions,
										     cost_bound,
										     kruhobot_cost_lower_Bounds,
										     real_sat_Model,
										     kruhobot_Schedules,
										     kruhobot_envelope_Variables,
										     kruhobot_set_extra_Variables,
										     kruhobot_all_extra_Variables);

			    if (!finding_result)
			    {
				sDouble end_time = sStatistics::get_CPU_Seconds();
				
				printf("%sUnable (3-cost) to solve newly built instance (elapsed time [seconds]: %.3f, current cost: %.3f, [makespan: %.3f])...\n",
				       s_INDENT.c_str(),
				       (end_time - start_time),
				       cost_bound, makespan_bound);			    

				if (fingerprint_limit < 0)
				{
				    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
				    {
					kruhobot_next_cost_Bounds[kruhobot_id] += next_extra_cost;//next_sink_extra_cost;//kruhobot_super_next_cost_Bounds[kruhobot_id];
				    }
				    
				    real_context.m_makespan_bound = makespan_bound = makespan_bound + next_extra_cost; //next_sink_extra_cost;
				    real_context.m_cost_bound = cost_bound = cost_bound + next_extra_cost; //next_sink_extra_cost;

				    fingerprint_limit = 1;
				}
				else
				{
				    fingerprint_limit = -1;				    
				}
				break;
			    }
			    else
			    {				
				analyze_NonconflictingSchedules_exactNonprioritized(real_Instance,
										    kruhobot_Schedules,
										    next_kruhobot_Collisions);
				if (next_kruhobot_Collisions.empty())
				{
				    continue;
				}
				
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
				}
				if (effective_conflicts <= 0)
				{
				    continue;
				}
				else
				{
				    break;
				}
			    }
			}
			else
			{
			    analyze_NonconflictingSchedules_exactNonprioritized(real_Instance,
										kruhobot_Schedules,
										next_kruhobot_Collisions);
			    if (next_kruhobot_Collisions.empty())
			    {
				printf("COST OPTIMAL COLLISION-FREE solution found (alpha)!\n");
			    
                                #ifdef sVERBOSE
				{
				    sDouble end_time = sStatistics::get_CPU_Seconds();
				    printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
				}
                                #endif
				
				return cost_bound;				    
			    }
				
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
			    }
			    if (effective_conflicts <= 0)
			    {
				printf("COST OPTIMAL COLLISION-FREE solution found (beta)!\n");
			    
                                #ifdef sVERBOSE
				{
				    sDouble end_time = sStatistics::get_CPU_Seconds();
				    printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
				}
                                #endif
		    
				return cost_bound;				    
			    }
			    else
			    {
				break;
			    }
			}
		    }
		}
	    }
	}	    
	return -1.0;
    }

    
    sDouble sRealSMTCBS::find_DomularCostExactNonconflictingSchedules_individualizedConflictRespectful(const sRealInstance                    &real_Instance,
												       KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
												       KruhobotUlinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
												       KruhobotSchedules_vector               &kruhobot_Schedules,
												       sDouble                                 makespan_limit,
												       sDouble                                 cost_limit,
												       sDouble                                 extra_cost)
    {
	sInt_32 last_conflict_id = 0;
	
	sDouble makespan_bound = 0.0;
	sDouble cost_bound = 0.0;
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();

//	bool same_conflict_check = true;

	#ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif
    
	std::vector<sDouble> kruhobot_cost_lower_Bounds;
	kruhobot_cost_lower_Bounds.resize(N_kruhobots + 1);

	std::vector<sDouble> kruhobot_next_cost_Bounds;	
	kruhobot_next_cost_Bounds.resize(N_kruhobots + 1);

	std::vector<sDouble> kruhobot_super_next_cost_Bounds;	
	kruhobot_super_next_cost_Bounds.resize(N_kruhobots + 1);		

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
//	    sDouble next_makespan_bound = -1.0;
	    sDouble makespan_lower_bound = 0.0;
	    
	    sDouble cost_lower_bound = 0.0;
	    sDouble next_cost_bound = -1.0;

	    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	    {
		sDouble kruhobot_makespan_lower_bound;
		
		if ((kruhobot_makespan_lower_bound = find_KruhobotIgnoringSchedule_superStrong(real_Instance.m_Kruhobots[kruhobot_id],
											       *real_Instance.m_start_conjunction.m_Map,
											       real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
											       real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],
											       makespan_limit,
											       0.0,
											       kruhobot_Schedules[kruhobot_id])) < 0.0)
		{
		    return -1.0;
		}
		makespan_lower_bound = (makespan_lower_bound < kruhobot_makespan_lower_bound) ? kruhobot_makespan_lower_bound : makespan_lower_bound;
		cost_lower_bound += kruhobot_makespan_lower_bound;
		kruhobot_cost_lower_Bounds[kruhobot_id] = kruhobot_next_cost_Bounds[kruhobot_id] = kruhobot_makespan_lower_bound;
	    }
	    
	    sRealCBSBase::to_Screen(kruhobot_Schedules);
	    #ifdef sDEBUG
	    {
		printf("Lower makespan: %.3f\n", makespan_lower_bound);
		printf("Lower cost: %.3f\n", cost_lower_bound);
	    }
	    #endif
    
	    real_context.m_makespan_bound = makespan_bound = makespan_lower_bound;
	    real_context.m_cost_bound = cost_bound = cost_lower_bound;
	    
            #ifdef sVERBOSE	    
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		
		printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current cost: %.3f [makespan: %.3f])...\n",
		       (end_time - start_time),
		       cost_bound, makespan_bound);
	    }
	    #endif

	    sDouble next_extra_cost = 0.0;
	    sDouble next_sink_extra_cost = -1.0;


	    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	    {
		sDouble nx_cost_bound, individual_extra_cost;
		sDouble sink_individual_extra = -1.0;

		SinkATTs_set sink_ATTs;
		nx_cost_bound = build_KruhobotCostRealDecisionDiagram_individualizedConflictRespectfulBucketingLeaping(real_Instance.m_Kruhobots[kruhobot_id],
														       *real_Instance.m_start_conjunction.m_Map,
														       real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
														       real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],
														       kruhobot_location_Conflicts[kruhobot_id],
														       kruhobot_linear_Conflicts[kruhobot_id],
														       makespan_bound,
														       kruhobot_cost_lower_Bounds[kruhobot_id],
														       -1,
														       kruhobot_RDDs[kruhobot_id],
														       kruhobot_RDD_Mappings[kruhobot_id],
														       0.0,
														       sink_ATTs);
/*
		#ifdef sDEBUG
		{
		    printf("Sink ATTs 1\n");
		    for (SinkATTs_set::const_iterator sink_att = sink_ATTs.begin(); sink_att != sink_ATTs.end(); ++sink_att)
		    {
			printf("%.3f ", *sink_att);
		    }
		    printf("\n");
		}
                #endif

		to_Screen(kruhobot_RDDs[kruhobot_id]);
*/		    
		if (nx_cost_bound > 0.0)
		{
		    individual_extra_cost = nx_cost_bound - kruhobot_cost_lower_Bounds[kruhobot_id];
		    next_cost_bound = (next_cost_bound < 0.0) ? nx_cost_bound : sMIN(next_cost_bound, nx_cost_bound);
		}
//		printf("Individual extra: %.3f\n", individual_extra_cost);

		if(next_extra_cost <= 0.0)
		{
		    next_extra_cost = individual_extra_cost;
		}
		else
		{
		    next_extra_cost = sMIN(next_extra_cost, individual_extra_cost);
		}

		if (sink_ATTs.size() >= 2)
		{
		    SinkATTs_set::const_iterator sink_att = sink_ATTs.begin();
		    sink_individual_extra = -(*sink_att - *(++sink_att));
		}
		else
		{
		    sink_individual_extra = 1.0;
		}
		kruhobot_super_next_cost_Bounds[kruhobot_id] = kruhobot_next_cost_Bounds[kruhobot_id] + sink_individual_extra;
		
		if (next_sink_extra_cost <= -1.0)
		{
		    next_sink_extra_cost = sink_individual_extra;
		}
		else
		{
		    next_sink_extra_cost = sMIN(next_sink_extra_cost, sink_individual_extra);
		}			
	    }	    	    
	    sASSERT(next_extra_cost > 0.0);
    
            #ifdef sVERBOSE	    
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();

		printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current cost: %.3f [makespan: %.3f])...\n",
		       (end_time - start_time),
		       cost_bound, makespan_bound);
		
//		to_Screen(kruhobot_RDDs);		
	    }
	    #endif

	    kruhobot_Schedules.clear();
	    find_InitialNonconflictingSchedules(solver,
						real_context,					 
						real_Instance,
						kruhobot_RDDs,
						kruhobot_RDD_Mappings,
						cost_bound,
						kruhobot_cost_lower_Bounds,						
						real_sat_Model,
						kruhobot_Schedules);
	    sASSERT(!kruhobot_Schedules.empty());
	    
//	    delete solver;
//	    sRealCBSBase::to_Screen(kruhobot_Schedules);	    

	    if (m_timeout >= 0.0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		
		if (end_time - start_time > m_timeout)
		{
		    return -2.0;
		}
	    }
	    if (makespan_bound > makespan_limit || cost_bound > cost_limit + extra_cost)
	    {
		return -3.0;
	    }
	}

	KruhobotCollisions_mset kruhobot_Collisions;
	KruhobotCollisions_mset next_kruhobot_Collisions, save_next_kruhobot_Collisions;
	KruhobotCollisions_mset effective_next_kruhobot_Collisions;

	analyze_NonconflictingSchedules_exactNonprioritized(real_Instance,
							    kruhobot_Schedules,
							    next_kruhobot_Collisions);
	if (next_kruhobot_Collisions.empty())
	{
	    printf("COLLISION-FREE solution found !\n");
	    
            #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Elapsed time [seconds]: %.3f, current cost: %.3f [makespan: %.3f])...\n", (end_time - start_time),
		       cost_bound, makespan_bound);		
	    }
            #endif
	    
	    return cost_bound;
	}

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
	}

	if (effective_conflicts <= 0)
	{
	    printf("COLLISION-FREE solution found !\n");
	    
            #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		
		printf("Elapsed time [seconds]: %.3f, current cost: %.3f [makespan: %.3f])...\n", (end_time - start_time),
		       cost_bound, makespan_bound);
	    }
            #endif
	    
	    return cost_bound;
	}

        #ifdef sVERBOSE	    
	{
	    sDouble end_time = sStatistics::get_CPU_Seconds();

	    printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current cost: %.3f [makespan: %.3f])...\n",
		   (end_time - start_time),
		   cost_bound, makespan_bound);
	}
	#endif

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
	    if (makespan_bound > makespan_limit || cost_bound > cost_limit + extra_cost)
	    {
		return -3.0;
	    }
	    
	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Solving MAPF-R (elapsed time [seconds]: %.3f, current cost: %.3f [makespan: %.3f])...\n",
			   (end_time - start_time),
			   cost_bound, makespan_bound);
		    
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

		sDouble next_extra_cost = 0.0;
		sDouble next_sink_extra_cost = 0.0;
		bool next_iteration = false;		
		{
		    sDouble next_cost_bound = -1.0;
//		    kruhobot_next_cost_Bounds = kruhobot_cost_lower_Bounds;

		    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
		    {
			sDouble individual_extra_cost;			
			sDouble nx_cost_bound;

			sDouble sink_individual_extra = -1.0;

			SinkATTs_set sink_ATTs;			
			nx_cost_bound = build_KruhobotCostRealDecisionDiagram_individualizedConflictRespectfulBucketingLeaping(real_Instance.m_Kruhobots[kruhobot_id],
															       *real_Instance.m_start_conjunction.m_Map,
															       real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
															       real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],	    
															       kruhobot_location_Conflicts[kruhobot_id],
															       kruhobot_linear_Conflicts[kruhobot_id],
															       makespan_bound,
															       kruhobot_next_cost_Bounds[kruhobot_id],
															       fingerprint_limit,
															       next_kruhobot_RDDs[kruhobot_id],
															       next_kruhobot_RDD_Mappings[kruhobot_id],
															       0.0,
															       sink_ATTs);
/*
                        #ifdef sDEBUG
			{
			    printf("Sink ATTs 1: %.3f\n", kruhobot_next_cost_Bounds[kruhobot_id]);
			    for (SinkATTs_set::const_iterator sink_att = sink_ATTs.begin(); sink_att != sink_ATTs.end(); ++sink_att)
			    {
				printf("%.3f ", *sink_att);
			    }
			    printf("\n");
			}
                        #endif
*/

			if (sink_ATTs.size() >= 2)
			{
			    SinkATTs_set::const_iterator sink_att = sink_ATTs.begin();
			    sink_individual_extra = -(*sink_att - *(++sink_att));
			}
			else
			{
			    sink_individual_extra = 1.0;
			}
			kruhobot_super_next_cost_Bounds[kruhobot_id] = kruhobot_next_cost_Bounds[kruhobot_id] + sink_individual_extra;
			
			if (nx_cost_bound > 0.0)
			{
			    individual_extra_cost = nx_cost_bound - kruhobot_next_cost_Bounds[kruhobot_id];
			    next_cost_bound = (next_cost_bound < 0.0) ? nx_cost_bound : sMIN(next_cost_bound, nx_cost_bound);
			}
			/*
			printf("Individual extra: %.3f\n", individual_extra_cost);
			printf("Sink individual: %.3f\n", sink_individual_extra);
			*/

			if (next_extra_cost <= 0.0)
			{
			    next_extra_cost = individual_extra_cost;
			}
			else
			{
			    next_extra_cost = sMIN(next_extra_cost, individual_extra_cost);
			}

			if (next_sink_extra_cost <= 0.0)
			{
			    next_sink_extra_cost = sink_individual_extra;
			}
			else
			{
			    next_sink_extra_cost = sMIN(next_sink_extra_cost, sink_individual_extra);
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
		}
		sASSERT(next_extra_cost > 0.0);

		KruhobotExtraVariables_vector kruhobot_set_extra_Variables;
		kruhobot_set_extra_Variables.resize(N_kruhobots + 1);
		KruhobotExtraVariables_vector kruhobot_all_extra_Variables;
		kruhobot_all_extra_Variables.resize(N_kruhobots + 1);
		KruhobotExtraVariables_vector kruhobot_envelope_Variables;
		kruhobot_envelope_Variables.resize(N_kruhobots + 1);
		KruhobotExtraVariables_vector kruhobot_domus_Variables;
		kruhobot_domus_Variables.resize(N_kruhobots + 1);						

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
		    //to_Screen(kruhobot_RDDs);
		    
		    bool finding_result = find_NextNonconflictingSchedules(solver,
									   real_context,
									   real_Instance,
									   kruhobot_RDDs,
									   kruhobot_RDD_Mappings,
									   kruhobot_Collisions,
									   cost_bound,
									   kruhobot_cost_lower_Bounds,
									   next_real_sat_Model,
									   kruhobot_Schedules,
									   kruhobot_set_extra_Variables,
									   kruhobot_all_extra_Variables);
		    
		    /*
		    #ifdef sDEBUG
		    {
			printf("All extra variables\n");
			for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			{
			    printf("  Kruhobot: %d [%.3f]\n", kruhobot_id, kruhobot_cost_lower_Bounds[kruhobot_id]);
			    printf("    ");
			    for (ExtraVariables_map::const_iterator extra_variable = kruhobot_all_extra_Variables[kruhobot_id].begin(); extra_variable != kruhobot_all_extra_Variables[kruhobot_id].end(); ++extra_variable)
			    {
				printf("%.3f,%d ", extra_variable->first, extra_variable->second);
			    }
			    printf("\n");
			}

			printf("SET (to TRUE) extra variables\n");
			for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			{
			    printf("  Kruhobot: %d [%.3f]\n", kruhobot_id, kruhobot_cost_lower_Bounds[kruhobot_id]);
			    printf("    ");
			    for (ExtraVariables_map::const_iterator extra_variable = kruhobot_set_extra_Variables[kruhobot_id].begin(); extra_variable != kruhobot_set_extra_Variables[kruhobot_id].end(); ++extra_variable)
			    {
				printf("%.3f,%d ", extra_variable->first, extra_variable->second);
			    }
			    printf("\n");
			}			
		    }
		    #endif
		    */
	    
		    if (!finding_result)
		    {
			sASSERT(next_extra_cost > s_EPSILON);
		    
                        #ifdef sVERBOSE	    
			{
			    sDouble end_time = sStatistics::get_CPU_Seconds();
			    
			    printf("%sUnable (1) to solve newly built instance (elapsed time [seconds]: %.3f, current cost: %.3f, [makespan: %.3f])...\n",
				   s_INDENT.c_str(),
				   (end_time - start_time),
				   cost_bound, makespan_bound);
			}
	                #endif
		    
			if (fingerprint_limit < 0)
			{
			    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			    {
				kruhobot_next_cost_Bounds[kruhobot_id] += next_extra_cost;//kruhobot_super_next_cost_Bounds[kruhobot_id];
			    }
			    real_context.m_makespan_bound = makespan_bound = makespan_bound + next_extra_cost;
			    real_context.m_cost_bound = cost_bound = cost_bound + next_extra_cost;
			    
			    /*
			    real_context.m_makespan_bound = makespan_bound = makespan_bound + next_sink_extra_cost;
			    real_context.m_cost_bound = cost_bound = cost_bound + next_sink_extra_cost;			    
			    */
			    
			    fingerprint_limit = 1;
			}
			else
			{
			    fingerprint_limit = -1;
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
									   cost_bound,
									   kruhobot_cost_lower_Bounds,
									   real_sat_Model,
									   kruhobot_Schedules,
									   kruhobot_set_extra_Variables,
									   kruhobot_all_extra_Variables);

		    /*
		    #ifdef sDEBUG
		    {
			printf("All extra variables\n");
			for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			{
			    printf("  Kruhobot: %d [%.3f]\n", kruhobot_id, kruhobot_cost_lower_Bounds[kruhobot_id]);
			    printf("    ");
			    for (ExtraVariables_map::const_iterator extra_variable = kruhobot_all_extra_Variables[kruhobot_id].begin(); extra_variable != kruhobot_all_extra_Variables[kruhobot_id].end(); ++extra_variable)
			    {
				printf("%.3f,%d ", extra_variable->first, extra_variable->second);
			    }
			    printf("\n");
			}

			printf("SET (to TRUE) extra variables\n");
			for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			{
			    printf("  Kruhobot: %d [%.3f]\n", kruhobot_id, kruhobot_cost_lower_Bounds[kruhobot_id]);
			    printf("    ");
			    for (ExtraVariables_map::const_iterator extra_variable = kruhobot_set_extra_Variables[kruhobot_id].begin(); extra_variable != kruhobot_set_extra_Variables[kruhobot_id].end(); ++extra_variable)
			    {
				printf("%.3f,%d ", extra_variable->first, extra_variable->second);
			    }
			    printf("\n");
			}			
		    }
		    #endif		    
		    */
		    
		    if (!finding_result)
		    {
                        #ifdef sVERBOSE	    
			{
			    sDouble end_time = sStatistics::get_CPU_Seconds();

			    printf("%sUnable (2) to solve newly built instance (elapsed time [seconds]: %.3f, current cost: %.3f, [makespan: %.3f])...\n",
				   s_INDENT.c_str(),
				   (end_time - start_time),
				   cost_bound, makespan_bound);			    
			}
	                #endif

			if (fingerprint_limit < 0)
			{
			    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			    {
				kruhobot_next_cost_Bounds[kruhobot_id] += next_extra_cost;//next_sink_extra_cost;//kruhobot_super_next_cost_Bounds[kruhobot_id];
			    }
			    /*
			    real_context.m_makespan_bound = makespan_bound = makespan_bound + next_extra_cost;
			    real_context.m_cost_bound = cost_bound = cost_bound + next_extra_cost;
			    */
			    real_context.m_makespan_bound = makespan_bound = makespan_bound + next_extra_cost;//next_sink_extra_cost;
			    real_context.m_cost_bound = cost_bound = cost_bound + next_extra_cost;//next_sink_extra_cost;			    
			    
			    fingerprint_limit = 1;
			}
			else
			{
			    fingerprint_limit = -1;
			}
			continue;
		    }
		}
		//augment_Schedules(real_Instance, kruhobot_Schedules);
		next_kruhobot_Collisions.clear();
	
		analyze_NonconflictingSchedules_exactNonprioritized(real_Instance,
								    kruhobot_Schedules,
								    next_kruhobot_Collisions);

		/*
		if (save_next_kruhobot_Collisions.size() == next_kruhobot_Collisions.size())
		{
		    printf("next colli: %ld, %ld\n", save_next_kruhobot_Collisions.size(), next_kruhobot_Collisions.size());
		    getchar();
		    bool same_collisions = true;
		    
		    KruhobotCollisions_mset::const_iterator save_next_collision = save_next_kruhobot_Collisions.begin();
		    for (KruhobotCollisions_mset::const_iterator next_collision = next_kruhobot_Collisions.begin(); next_collision != next_kruhobot_Collisions.end(); ++next_collision)
		    {
			next_collision->to_Screen();
			save_next_collision->to_Screen();
			printf("----\n");
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
		    if (same_collisions)
		    {
			printf("Maybe done, let us try to reduce...\n");
			getchar();
		    
			while (true)
			{
			    sDouble total_cost = analyze_NonconflictingSchedulesCosts(real_Instance,
										      kruhobot_Schedules,
										      cost_bound,
										      kruhobot_cost_lower_Bounds,
										      kruhobot_set_extra_Variables,
										      kruhobot_all_extra_Variables,
										      kruhobot_envelope_Variables);
			    if (total_cost > cost_bound)
			    {
				kruhobot_Schedules.clear();
				
				kruhobot_set_extra_Variables.clear();
				kruhobot_set_extra_Variables.resize(N_kruhobots + 1);
				kruhobot_all_extra_Variables.clear();
				kruhobot_all_extra_Variables.resize(N_kruhobots + 1);
				kruhobot_envelope_Variables.clear();
				kruhobot_envelope_Variables.resize(N_kruhobots + 1);							    
				
				bool finding_result = find_BetterNonconflictingSchedules(solver,
											 real_context,
											 real_Instance,
											 kruhobot_RDDs,
											 kruhobot_RDD_Mappings,
											 kruhobot_Collisions,
											 next_kruhobot_Collisions,
											 cost_bound,
											 kruhobot_cost_lower_Bounds,
											 real_sat_Model,
											 kruhobot_Schedules,
											 kruhobot_envelope_Variables,
											 kruhobot_set_extra_Variables,
											 kruhobot_all_extra_Variables);
				
				if (!finding_result)
				{
				    sDouble end_time = sStatistics::get_CPU_Seconds();
				    
				    printf("%sUnable (4-cost) to solve newly built instance (elapsed time [seconds]: %.3f, current cost: %.3f, [makespan: %.3f])...\n",
					   s_INDENT.c_str(),
					   (end_time - start_time),
					   cost_bound, makespan_bound);			    
				    
				    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
				    {
					kruhobot_next_cost_Bounds[kruhobot_id] += next_extra_cost;
				    }
				    
				    real_context.m_makespan_bound = makespan_bound = makespan_bound + next_extra_cost;
				    real_context.m_cost_bound = cost_bound = cost_bound + next_extra_cost;
				    
				    break;
				}			
			    }
			    else
			    {
				printf("COST OPTIMAL COLLISION-FREE solution found [*]!\n");
				
                                #ifdef sVERBOSE
				{
				    sDouble end_time = sStatistics::get_CPU_Seconds();
				    printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
				}
 	                        #endif
		    
				return cost_bound;
			    }
			}
			continue;
	            }
		}
		save_next_kruhobot_Collisions = next_kruhobot_Collisions;
		*/
		
		if (next_kruhobot_Collisions.empty())
		{
		    printf("COLLISION-FREE solution found !\n");

  	            #ifdef sVERBOSE
		    {
			sDouble end_time = sStatistics::get_CPU_Seconds();

			printf("Elapsed time [seconds]: %.3f, current cost: %.3f [makespan: %.3f])...\n", (end_time - start_time),
			       cost_bound, makespan_bound);			
		    }
 	            #endif

		    sASSERT(false);
		    printf("  No further check of optim 1\n");
		    
		    return cost_bound;
		}

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
			/*
			next_collision->to_Screen();
			printf("%d,%d\n", kruhobot_affection.first, kruhobot_affection.second);
			*/
			
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
                    #ifdef sVERBOSE	    
		    {
			sDouble end_time = sStatistics::get_CPU_Seconds();
			
			printf("Optimizing cost (elapsed time [seconds]: %.3f)...\n",
			       (end_time - start_time));			    
		    }
	            #endif		    
		    
		    while (true)
		    {
			kruhobot_envelope_Variables.clear();
			kruhobot_envelope_Variables.resize(N_kruhobots + 1);
			kruhobot_domus_Variables.clear();
			kruhobot_domus_Variables.resize(N_kruhobots + 1);							    			    			    			
			
			sDouble total_cost = analyze_NonconflictingSchedulesCosts(real_Instance,
										  kruhobot_Schedules,
										  cost_bound,
										  kruhobot_cost_lower_Bounds,
										  kruhobot_set_extra_Variables,
										  kruhobot_all_extra_Variables,
										  kruhobot_envelope_Variables,
										  kruhobot_domus_Variables);
			/*
			printf("Tot vs. cost: %.3f, %.3f\n", total_cost, cost_bound);
			*/
			
			if (total_cost > cost_bound)
			{
			    /*
			    printf("Extra ENVELOPE variables\n");
			    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			    {
				printf("  Kruhobot: %d [%.3f]\n", kruhobot_id, kruhobot_cost_lower_Bounds[kruhobot_id]);
				printf("    ");
				for (ExtraVariables_mmap::const_iterator extra_variable = kruhobot_envelope_Variables[kruhobot_id].begin(); extra_variable != kruhobot_envelope_Variables[kruhobot_id].end(); ++extra_variable)
				{
				    printf("%.3f,%d ", extra_variable->first, extra_variable->second);
				}
				printf("\n");
			    }

			    printf("Extra DOMUS variables\n");
			    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
			    {
				printf("  Kruhobot: %d [%.3f]\n", kruhobot_id, kruhobot_cost_lower_Bounds[kruhobot_id]);
				printf("    ");
				for (ExtraVariables_mmap::const_iterator extra_variable = kruhobot_domus_Variables[kruhobot_id].begin(); extra_variable != kruhobot_domus_Variables[kruhobot_id].end(); ++extra_variable)
				{
				    printf("%.3f,%d ", extra_variable->first, extra_variable->second);
				}
				printf("\n");
			    }
			    getchar();
			    */
			    
			    kruhobot_Schedules.clear();

			    kruhobot_set_extra_Variables.clear();
			    kruhobot_set_extra_Variables.resize(N_kruhobots + 1);
			    kruhobot_all_extra_Variables.clear();
			    kruhobot_all_extra_Variables.resize(N_kruhobots + 1);
			    
			    bool finding_result = find_BetterNonconflictingSchedules(solver,
										     real_context,
										     real_Instance,
										     kruhobot_RDDs,
										     kruhobot_RDD_Mappings,
										     kruhobot_Collisions,
										     next_kruhobot_Collisions,
										     cost_bound,
										     kruhobot_cost_lower_Bounds,
										     real_sat_Model,
										     kruhobot_Schedules,
										     kruhobot_envelope_Variables,
										     kruhobot_domus_Variables,										     
										     kruhobot_set_extra_Variables,
										     kruhobot_all_extra_Variables);			   

			    if (!finding_result)
			    {
				sDouble end_time = sStatistics::get_CPU_Seconds();
				
				printf("%sUnable (3-cost) to solve newly built instance (elapsed time [seconds]: %.3f, current cost: %.3f, [makespan: %.3f])...\n",
				       s_INDENT.c_str(),
				       (end_time - start_time),
				       cost_bound, makespan_bound);			    

				for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
				{
				    kruhobot_next_cost_Bounds[kruhobot_id] += next_extra_cost;//next_sink_extra_cost;//kruhobot_super_next_cost_Bounds[kruhobot_id];
				}
				
				real_context.m_makespan_bound = makespan_bound = makespan_bound + next_extra_cost; //next_sink_extra_cost;
				real_context.m_cost_bound = cost_bound = cost_bound + next_extra_cost; //next_sink_extra_cost;

				break;
			    }
			    else
			    {
				analyze_NonconflictingSchedules_exactNonprioritized(real_Instance,
										    kruhobot_Schedules,
										    next_kruhobot_Collisions);
				if (next_kruhobot_Collisions.empty())
				{
				    continue;
				}
				
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
				}
				if (effective_conflicts <= 0)
				{
				    continue;
				}
				else
				{
				    break;
				}				
			    }
			}
			else
			{
			    printf("COST OPTIMAL COLLISION-FREE solution found !\n");
			    
                            #ifdef sVERBOSE
			    {
				sDouble end_time = sStatistics::get_CPU_Seconds();
				printf("Elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
			    }
 	                    #endif
		    
			    return cost_bound;
			}
		    }
		}
	    }
	}	    
	return -1.0;
    }                            

            
/*----------------------------------------------------------------------------*/

} // namespace boOX
