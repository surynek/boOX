/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                              boOX 0-279_zenon                              */
/*                                                                            */
/*                  (C) Copyright 2018 - 2019 Pavel Surynek                   */
/*                                                                            */
/*                http://www.surynek.com | <pavel@surynek.com>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* smtcbsR.cpp / 0-279_zenon                                                  */
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
#include "types.h"
#include "result.h"

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
							   sDouble              cost_limit)
    {
	sInt_32 cost;
	KruhobotSchedules_vector kruhobot_Schedules;

	if ((cost = find_ShortestNonconflictingSchedules(real_Instance, kruhobot_Schedules, cost_limit)) < 0)
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


    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules(KruhobotSchedules_vector &kruhobot_Schedules, sDouble cost_limit)
    {
	return find_ShortestNonconflictingSchedules(*m_real_Instance, kruhobot_Schedules, cost_limit);	
    }


    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules(const sRealInstance      &sUNUSED(real_Instance),
							   KruhobotSchedules_vector &sUNUSED(kruhobot_Schedules),
							   sDouble                   sUNUSED(cost_limit))
    {
	return 0.0;
    }    

    
    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules(KruhobotSchedules_vector &kruhobot_Schedules, sDouble cost_limit, sDouble extra_cost)
    {
	return find_ShortestNonconflictingSchedules(*m_real_Instance, kruhobot_Schedules, cost_limit, extra_cost);	
    }

    
    sDouble sRealSMTCBS::find_ShortestNonconflictingSchedules(const sRealInstance      &real_Instance,
							   KruhobotSchedules_vector &kruhobot_Schedules,
							   sDouble                   cost_limit,
							   sDouble                   extra_cost)
    {
	sDouble solution_cost;
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();	
	
	#ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif

	{    
	    #ifdef sVERBOSE
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		printf("Solving MAPF-R (elapsed time [seconds]: %.3f)...\n", (end_time - start_time));
	    }
	    #endif

//	    KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
//	    KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;

	    KruhobotLocationConflicts_upper_vector kruhobot_location_Conflicts;
	    KruhobotLinearConflicts_upper_vector kruhobot_linear_Conflicts;	    

	    kruhobot_location_Conflicts.resize(N_kruhobots + 1);
	    kruhobot_linear_Conflicts.resize(N_kruhobots + 1);
	    kruhobot_Schedules.resize(N_kruhobots + 1);

	    kruhobot_location_Conflicts.resize(N_kruhobots + 1);
	    kruhobot_linear_Conflicts.resize(N_kruhobots + 1);
	    kruhobot_Schedules.resize(N_kruhobots + 1);  	    
	    
	    if ((solution_cost = find_NonconflictingSchedules(real_Instance,
							      kruhobot_location_Conflicts,
							      kruhobot_linear_Conflicts,
							      kruhobot_Schedules,
							      cost_limit,
							      extra_cost)) >= 0.0)
	    {
		return solution_cost;
	    }
	    
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2;
		}
	    }
	}

	return -1;	
    }


/*----------------------------------------------------------------------------*/
    
    sDouble sRealSMTCBS::find_NonconflictingSchedules(const sRealInstance              &real_Instance,
						      KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
						      KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts,
						      KruhobotSchedules_vector         &kruhobot_Schedules,
						      sDouble                           cost_limit,
						      sDouble                           extra_cost)
    {
//	sDouble cummulative;
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();

//	sDouble start_time = sStatistics::get_CPU_Seconds();

	/*
	Node initial_node(0);
	initial_node.m_upper_node_id = -1;

	initial_node.m_update_kruhobot_id = -1;

	initial_node.m_left_node_id = -1;
	initial_node.m_right_node_id = -1;
	
	initial_node.m_kruhobot_location_Conflicts = kruhobot_location_Conflicts;
	initial_node.m_kruhobot_linear_Conflicts = kruhobot_linear_Conflicts;
	initial_node.m_kruhobot_Schedules.resize(N_kruhobots + 1);
	*/

	#ifdef sVERBOSE
	sDouble start_time = sStatistics::get_CPU_Seconds();
	#endif	

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
		sDouble nx_makespan_bound;
		nx_makespan_bound = build_KruhobotRealDecisionDiagram(real_Instance.m_Kruhobots[kruhobot_id],
								      *real_Instance.m_start_conjunction.m_Map,
								      real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
								      real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],
								      cost_limit,
								      extra_cost,					    
								      kruhobot_location_Conflicts[kruhobot_id],
								      kruhobot_linear_Conflicts[kruhobot_id],
								      -1.0,
								      kruhobot_RDDs[kruhobot_id],
								      kruhobot_RDD_Mappings[kruhobot_id]);
		printf("nx makespan bound: %.3f\n", nx_makespan_bound);
//		to_Screen(kruhobot_RDDs[kruhobot_id]);
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
	}
	sDouble cummulative;
	KruhobotCollisions_mset kruhobot_Collisions;

	cummulative = analyze_NonconflictingSchedules(real_Instance,
						      kruhobot_Schedules,
						      kruhobot_Collisions);

	if (kruhobot_Collisions.empty())	
	{
	    printf("COLLISION-FREE solution found !\n");	    
	    return cummulative;
	}
	
	reflect_KruhobotCollisions(kruhobot_Collisions,
				   kruhobot_location_Conflicts,
				   kruhobot_linear_Conflicts);	

  	#ifdef sDEBUG
	{
	    printf("All collisions:\n");
	    for (KruhobotCollisions_mset::const_iterator collision = kruhobot_Collisions.begin(); collision != kruhobot_Collisions.end(); ++collision)
	    {
		collision->to_Screen();
	    }
	}
	#endif		

	sDouble makespan_bound = -1.0;
	sInt_32 ite = 0;

	while (true)
	{
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
									      cost_limit,
									      extra_cost,					    
									      kruhobot_location_Conflicts[kruhobot_id],
									      kruhobot_linear_Conflicts[kruhobot_id],
									      makespan_bound,
									      kruhobot_RDDs[kruhobot_id],
									      kruhobot_RDD_Mappings[kruhobot_id]);
			
			printf("nx makespan bound: %.3f\n", nx_makespan_bound);			
			printf("%d: size %ld\n", kruhobot_id, kruhobot_RDDs[kruhobot_id].size());
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
		    for (LinearConflicts__map::const_iterator linear_Conflict = kruhobot_linear_Conflicts[kruhobot_id].begin(); linear_Conflict != kruhobot_linear_Conflicts[kruhobot_id].end(); ++linear_Conflict)
		    {
			printf("  %ld\n", linear_Conflict->second.size());
			for (LinearConflicts_map::const_iterator linear_conflict = linear_Conflict->second.begin(); linear_conflict != linear_Conflict->second.end(); ++linear_conflict)
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

		    sDouble cummulative;
		    KruhobotCollisions_mset next_kruhobot_Collisions;		

		    cummulative = analyze_NonconflictingSchedules(real_Instance,
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
		    
			return cummulative;
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
		
		    reflect_KruhobotCollisions(next_kruhobot_Collisions,
					       kruhobot_location_Conflicts,
					       kruhobot_linear_Conflicts);
		    
		    for (KruhobotCollisions_mset::const_iterator next_collision = next_kruhobot_Collisions.begin(); next_collision != next_kruhobot_Collisions.end(); ++next_collision)
		    {		    
			if (verify_KruhobotCollisionDuplicities(*next_collision, kruhobot_Collisions))
			{
			    kruhobot_Collisions.insert(*next_collision);
//			break;
			}
		    }
		    printf("Collisions: %ld (%d)\n", kruhobot_Collisions.size(), ite);
		    ++ite;
		}
	    }	    
	}	
	
	return -1.0;
    }


    sDouble sRealSMTCBS::find_NonconflictingSchedules(const sRealInstance                    &real_Instance,
						      KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
						      KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
						      KruhobotSchedules_vector               &kruhobot_Schedules,
						      sDouble                                 cost_limit,
						      sDouble                                 extra_cost)
    {
	sDouble makespan_bound = 0.0;
//	Makespans_vector characteristic_Makespans;
	    
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
										   cost_limit,
										   extra_cost,
										   kruhobot_Schedules[kruhobot_id])) < 0.0)
		{
		    return -1.0;
		}
		makespan_lower_bound = (makespan_lower_bound < kruhobot_makespan_lower_bound) ? kruhobot_makespan_lower_bound : makespan_lower_bound;
		printf("Kruhobot lower bound makespan: %.3f\n", kruhobot_makespan_lower_bound);
	    }
	    real_context.m_makespan_bound = makespan_lower_bound;
	    printf("Lower bound makespan: %.3f\n", makespan_lower_bound);
//	    makespan_lower_bound = 0.0;	    
	    
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
		nx_makespan_bound = build_KruhobotRealDecisionDiagram(real_Instance.m_Kruhobots[kruhobot_id],
								      *real_Instance.m_start_conjunction.m_Map,
								      real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
								      real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],
								      cost_limit,
								      extra_cost,					    
								      kruhobot_location_Conflicts[kruhobot_id],
								      kruhobot_linear_Conflicts[kruhobot_id],
								      makespan_lower_bound,
								      kruhobot_RDDs[kruhobot_id],
								      kruhobot_RDD_Mappings[kruhobot_id]);
		next_makespan_bound = (next_makespan_bound < 0.0) ? nx_makespan_bound : sMIN(next_makespan_bound, nx_makespan_bound);		
	    }
	    printf("next makespan bound: %.3f\n", next_makespan_bound);
	    real_context.m_makespan_bound = makespan_bound = next_makespan_bound;
//	    collect_CharacteristicMakespans(real_Instance, kruhobot_RDDs, characteristic_Makespans);

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

	    /*
	    to_Screen(kruhobot_RDDs);
	    sRealCBSBase::to_Screen(kruhobot_Schedules);
	    */
	    
	    delete solver;
	}
	sDouble cummulative;
	KruhobotCollisions_mset kruhobot_Collisions;

	cummulative = analyze_NonconflictingSchedules(real_Instance,
						      kruhobot_Schedules,
						      kruhobot_Collisions);

	
	if (kruhobot_Collisions.empty())	
	{
	    printf("COLLISION-FREE solution found !\n");	    
	    return cummulative;
	}
	
	reflect_KruhobotCollisions(kruhobot_Collisions,
				   kruhobot_location_Conflicts,
				   kruhobot_linear_Conflicts);	

  	#ifdef sDEBUG
	{
	    printf("All collisions:\n");
	    for (KruhobotCollisions_mset::const_iterator collision = kruhobot_Collisions.begin(); collision != kruhobot_Collisions.end(); ++collision)
	    {
		collision->to_Screen();
	    }
	}
	#endif		
//	sInt_32 ite = 0;

	Glucose::Solver *solver = NULL;

	KruhobotDecisionDiagrams_vector kruhobot_RDDs;
	kruhobot_RDDs.resize(N_kruhobots + 1);
	
	KruhobotDecisionMappings_vector kruhobot_RDD_Mappings;
	kruhobot_RDD_Mappings.resize(N_kruhobots + 1);

	RealModel real_sat_Model;
	KruhobotCollisions_mset next_kruhobot_Collisions;

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

	    /*
	    makespan_bound = calc_MakespanBound(real_Instance, kruhobot_location_Conflicts, kruhobot_linear_Conflicts);
	    printf("Computed bound: %.3f\n", makespan_bound);
	    */
	    
//	    makespan_bound = sMIN(makespan_bound, 5.650);
	    
//	    makespan_bound = 5.7;
//	    makespan_bound = 5.5;

//	    makespan_bound = 9.5;
//	    makespan_bound = 4.65;	    
//	    makespan_bound = 1.0;
//	    makespan_bound = 9.0;

	    {
		bool next_iteration = false;
		sDouble next_makespan_bound = -1.0;		

		for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
		{
		    sDouble nx_makespan_bound;
		    nx_makespan_bound = build_KruhobotRealDecisionDiagram(real_Instance.m_Kruhobots[kruhobot_id],
									  *real_Instance.m_start_conjunction.m_Map,
									  real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
									  real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],
									  cost_limit,
									  extra_cost,					    
									  kruhobot_location_Conflicts[kruhobot_id],
									  kruhobot_linear_Conflicts[kruhobot_id],
									  makespan_bound,
									  next_kruhobot_RDDs[kruhobot_id],
									  next_kruhobot_RDD_Mappings[kruhobot_id]);
		    next_makespan_bound = (next_makespan_bound < 0.0) ? nx_makespan_bound : sMIN(next_makespan_bound, nx_makespan_bound);

//		    printf("%d: size %ld,%ld\n", kruhobot_id,  kruhobot_RDDs[kruhobot_id].size(), next_kruhobot_RDDs[kruhobot_id].size());
		    
		    if (kruhobot_RDDs[kruhobot_id].size() != next_kruhobot_RDDs[kruhobot_id].size())
		    {
//			sASSERT(!compare_KruhobotRealDecisionDiagrams(kruhobot_RDDs[kruhobot_id], next_kruhobot_RDDs[kruhobot_id]));
			next_iteration = true;
		    }
		    else
		    {
			if (!compare_KruhobotRealDecisionDiagrams(kruhobot_RDDs[kruhobot_id], next_kruhobot_RDDs[kruhobot_id]))
			{
			    next_iteration = true;
			}
//			sASSERT(compare_KruhobotRealDecisionDiagrams(kruhobot_RDDs[kruhobot_id], next_kruhobot_RDDs[kruhobot_id]));
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
			for (LinearConflicts_upper__map::const_iterator linear_Conflict = kruhobot_linear_Conflicts[kruhobot_id].begin(); linear_Conflict != kruhobot_linear_Conflicts[kruhobot_id].end(); ++linear_Conflict)
			{
			    printf("  %ld\n", linear_Conflict->second.size());
			    for (LinearConflicts_map::const_iterator linear_conflict = linear_Conflict->second.begin(); linear_conflict != linear_Conflict->second.end(); ++linear_conflict)
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
		    /*
		    Makespans_vector next_characteristic_Makespans;
		    collect_CharacteristicMakespans(real_Instance, next_kruhobot_RDDs, next_characteristic_Makespans);

		    printf("Characteristic:\n");
		    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)		
		    {
			printf("%.3f ", characteristic_Makespans[kruhobot_id]);
		    }
		    printf("\n");
		    printf("Next characteristic:\n");
		    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)		
		    {
			printf("%.3f ", next_characteristic_Makespans[kruhobot_id]);
		    }
		    printf("\n");
		    getchar();
		    */
		    /*
		    if (compare_CharacteristicMakespans(real_Instance, characteristic_Makespans, next_characteristic_Makespans))
		    {			
			real_context.m_makespan_bound = makespan_bound = next_makespan_bound;
			continue;
		    }
		    characteristic_Makespans = next_characteristic_Makespans;
		    */
		    
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
			//sDouble next_makespan_bound = calc_NextMakespanBound(makespan_bound, real_Instance, kruhobot_location_Conflicts, kruhobot_linear_Conflicts);
			//sASSERT(next_makespan_bound > makespan_bound + s_EPSILON);
                        #ifdef sVERBOSE	    
			{
			    sDouble end_time = sStatistics::get_CPU_Seconds();
			    
			    printf("    Unable to solve newly built instance (elapsed time [seconds]: %.3f, current makespan: %.3f, next makespan: %.3f)...\n",
				   (end_time - start_time),
				   makespan_bound,
				   next_makespan_bound);
			}
	                #endif
			real_context.m_makespan_bound = makespan_bound = next_makespan_bound;
						
			continue;
		    }
		    real_sat_Model = next_real_sat_Model;		    
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
//			sDouble next_makespan_bound = calc_NextMakespanBound(makespan_bound, real_Instance, kruhobot_location_Conflicts, kruhobot_linear_Conflicts);
			sASSERT(next_makespan_bound > makespan_bound + s_EPSILON);
			
                        #ifdef sVERBOSE	    
			{
			    sDouble end_time = sStatistics::get_CPU_Seconds();

			    printf("    Unable to solve augmented instance (elapsed time [seconds]: %.3f, current makespan: %.3f, next makespan: %.3f)...\n",
				   (end_time - start_time),
				   makespan_bound,
				   next_makespan_bound);
			}
	                #endif
			real_context.m_makespan_bound = makespan_bound = next_makespan_bound;
			continue;
		    }
		}
		augment_Schedules(real_Instance, kruhobot_Schedules);

		sDouble cummulative;
		next_kruhobot_Collisions.clear();
		cummulative = analyze_NonconflictingSchedules(real_Instance,
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
		    
		    return cummulative;
		}

  	        #ifdef sDEBUG
		{
		    /*
		    printf("Collisions:\n");
		    for (KruhobotCollisions_mset::const_iterator collision = kruhobot_Collisions.begin(); collision != kruhobot_Collisions.end(); ++collision)
		    {
			collision->to_Screen("    ");
		    }
		    
		    printf("Next collisions:\n");
		    for (KruhobotCollisions_mset::const_iterator collision = next_kruhobot_Collisions.begin(); collision != next_kruhobot_Collisions.end(); ++collision)
		    {
			collision->to_Screen("    ");
		    }
		    */
		}
	        #endif

		/*
		reflect_KruhobotCollisions(next_kruhobot_Collisions,
					   kruhobot_location_Conflicts,
					   kruhobot_linear_Conflicts);
		*/

                /*
		for (KruhobotCollisions_mset::const_iterator next_collision = next_kruhobot_Collisions.begin(); next_collision != next_kruhobot_Collisions.end(); ++next_collision)
		{		    
		    if (verify_KruhobotCollisionDuplicities(*next_collision, kruhobot_Collisions))
		    {
			break;
		    }
		    else
		    {
			printf("pre Duplic !!!!!!!!!!\n");
			next_collision->to_Screen();
			printf("****\n");			
		    }		    
		}
		printf("-------->\n");
		*/
		
		for (KruhobotCollisions_mset::const_iterator next_collision = next_kruhobot_Collisions.begin(); next_collision != next_kruhobot_Collisions.end(); ++next_collision)
		{		    
		    if (verify_KruhobotCollisionDuplicities(*next_collision, kruhobot_Collisions))
		    {
			kruhobot_Collisions.insert(*next_collision);

			reflect_KruhobotCollision(*next_collision,
						  kruhobot_location_Conflicts,
						  kruhobot_linear_Conflicts);			
//			break;
		    }
		    /*
		    else
		    {
			printf("Duplic !!!!!!!!!!\n");
		    }		    
		    */
		}
//		printf("Collisions: %ld (%d)\n", kruhobot_Collisions.size(), ite);
//		++ite;
	    }
	    }	    
	}	
	
	return -1.0;
    }    


    sDouble sRealSMTCBS::find_KruhobotIgnoringSchedule(const sKruhobot &kruhobot,
						       const s2DMap    &map,
						       sInt_32          source_loc_id,
						       sInt_32          sink_loc_id,
						       sDouble          cost_limit,
						       sDouble          sUNUSED(extra_cost),
						       Schedule_vector &Schedule) const
    {
	Transitions_mmap transition_Queue;
	Transitions_map explored_Transitions;
	Transitions_vector transition_Store;

	Transition initial_transition(transition_Store.size(), 0.0, 0.0, source_loc_id, -1);	
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
	        
	    if (front_transition.m_time + map.m_straight_Distances[front_transition.m_location_id][sink_loc_id] <= cost_limit)
	    {
		for (s2DMap::Locations_vector::const_iterator location = map.m_Locations.begin(); location != map.m_Locations.end(); ++location) /* regular actions */
		{
		    sInt_32 neighbor_location_id = location->m_id;

		    if (neighbor_location_id != front_transition.m_location_id && map.m_Network.is_Adjacent(neighbor_location_id, front_transition.m_location_id)) /* regular actions only */
		    {
			sDouble transition_distance = map.m_straight_Distances[front_transition.m_location_id][neighbor_location_id];
			sDouble transition_delta_time = transition_distance / kruhobot.m_properties.m_linear_velo;
			sDouble transition_finish_time = front_transition.m_time + transition_delta_time;
			sDouble transition_finish_cost = front_transition.m_cost + transition_delta_time;

			LocationIDs_uset *next_explored_Transitions;
			
			Transitions_map::const_iterator explored_transition = explored_Transitions.find(transition_finish_time);			
			if (explored_transition == explored_Transitions.end())
			{
			    explored_Transitions.insert(Transitions_map::value_type(transition_finish_time, LocationIDs_uset()));
			}
			next_explored_Transitions = &explored_Transitions[transition_finish_time];

			if (next_explored_Transitions->find(neighbor_location_id) == next_explored_Transitions->end())
			{
			    Transition neighbor_transition(transition_Store.size(), transition_finish_time, transition_finish_cost, neighbor_location_id, front_transition.m_trans_id);
			    transition_Store.push_back(neighbor_transition);
				
			    next_explored_Transitions->insert(neighbor_location_id);
			    transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_time, neighbor_transition));
			}
		    }
		}				
	    }
	    transition_Queue.erase(transition_Queue.begin());
	}
	return -1.0;	
    }    

    
    void sRealSMTCBS::reflect_KruhobotCollision(const KruhobotCollision          &kruhobot_collision,
						KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
						KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts)
    {
	introduce_KruhobotConflict(kruhobot_collision.m_traversal_A, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, (kruhobot_collision.m_traversal_A.m_kruhobot_id < 0));
	introduce_KruhobotConflict(kruhobot_collision.m_traversal_B, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, (kruhobot_collision.m_traversal_B.m_kruhobot_id < 0));	
    }
    

    void sRealSMTCBS::reflect_KruhobotCollision(const KruhobotCollision                &kruhobot_collision,
						KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
						KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts)
    {
	introduce_KruhobotConflict(kruhobot_collision.m_traversal_A, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, (kruhobot_collision.m_traversal_A.m_kruhobot_id < 0));
	introduce_KruhobotConflict(kruhobot_collision.m_traversal_B, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, (kruhobot_collision.m_traversal_B.m_kruhobot_id < 0));
    }

    
    void sRealSMTCBS::reflect_KruhobotCollisions(const KruhobotCollisions_mset    &kruhobot_Collisions,
						 KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
						 KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts)
    {
	for (KruhobotCollisions_mset::const_iterator collision = kruhobot_Collisions.begin(); collision != kruhobot_Collisions.end(); ++collision)
	{
	    introduce_KruhobotConflict(collision->m_traversal_A, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, (collision->m_traversal_A.m_kruhobot_id < 0));
	    introduce_KruhobotConflict(collision->m_traversal_B, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, (collision->m_traversal_B.m_kruhobot_id < 0));
	}
    }

    
    void sRealSMTCBS::reflect_KruhobotCollisions(const KruhobotCollisions_mset          &kruhobot_Collisions,
						 KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
						 KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts)
    {
	for (KruhobotCollisions_mset::const_iterator collision = kruhobot_Collisions.begin(); collision != kruhobot_Collisions.end(); ++collision)
	{
	    introduce_KruhobotConflict(collision->m_traversal_A, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, (collision->m_traversal_A.m_kruhobot_id < 0));
	    introduce_KruhobotConflict(collision->m_traversal_B, kruhobot_location_Conflicts, kruhobot_linear_Conflicts, (collision->m_traversal_B.m_kruhobot_id < 0));
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
							   sDouble                         cost_limit,
							   sDouble                         extra_cost,
							   const LocationConflicts__umap  &location_Conflicts,
							   const LinearConflicts__map     &linear_Conflicts,
							   sDouble                         makespan_bound,
							   KruhobotDecisionDiagram_vector &kruhobot_RDD,
							   KruhobotDecisionMapping_map    &kruhobot_RDD_mapping) const
    {
	sDouble next_makespan_bound = -1.0;
	
	Transitions_mmap transition_Queue;
	TransitionExplorations_map explored_Transitions;
	Explorations_umap bound_explored_Transitions;	
	Transitions_vector transition_Store;

	Transition initial_transition(transition_Store.size(), 0.0, 0.0, source_loc_id, -1);	
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
	    #ifdef sDEBUG
	    {
		//printf("Generation queue: %ld (bnd:%ld)\n", transition_Queue.size(), bound_explored_Transitions.size());
	    }
	    #endif
	    
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

	    if (front_transition.m_time + map.m_straight_Distances[front_transition.m_location_id][sink_loc_id] <= cost_limit + extra_cost)
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
			    Transition neighbor_transition(transition_Store.size(), transition_finish_time, transition_finish_cost, neighbor_location_id, front_transition.m_trans_id);
			    transition_Store.push_back(neighbor_transition);
			    
			    kruhobot_RDD.push_back(KruhobotDecision(neighbor_transition.m_trans_id,
								    neighbor_transition.m_time,
								    neighbor_transition.m_location_id,
								    neighbor_transition.m_prev_trans_id));
			    kruhobot_RDD_mapping[neighbor_transition.m_location_id].insert(KruhobotDecisionIDs_mmap::value_type(neighbor_transition.m_time, neighbor_transition.m_trans_id));
			    
			    next_explored_Transitions->insert(Explorations_umap::value_type(neighbor_location_id, front_transition.m_trans_id));
			    transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_cost, neighbor_transition));
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
			    
			    LinearConflicts__map::const_iterator linear_Conflict = linear_Conflicts.find(Uline(front_transition.m_location_id, neighbor_location_id));
			    sDouble first_non_conf_linear_time = -1.0;
			    
			    if (linear_Conflict != linear_Conflicts.end())
			    {
				/*
				LinearConflicts_map::const_iterator lower_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				if (lower_linear_conflict != linear_Conflict->second.end())
				{
				    first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
				}

				for (LinearConflicts_map::const_iterator lower2_linear_conflict = lower_linear_conflict; lower2_linear_conflict != linear_Conflict->second.end(); ++lower2_linear_conflict)
				{
				    if (lower2_linear_conflict->second.m_interval.m_upper < lower_linear_conflict->second.m_interval.m_upper)
				    {
					lower_linear_conflict = lower2_linear_conflict;
					first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
				    }
				}
				*/
				LinearConflicts_map::const_iterator lower_linear_conflict = linear_Conflict->second.end();

				for (LinearConflicts_map::const_iterator lower2_linear_conflict = linear_Conflict->second.begin(); lower2_linear_conflict != linear_Conflict->second.end(); ++lower2_linear_conflict)
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
			    Transition wait_transition(transition_Store.size(), wait_finish_time, front_transition.m_cost + wait_cost, front_transition.m_location_id, front_transition.m_trans_id);
			    transition_Store.push_back(wait_transition);
			    
			    kruhobot_RDD.push_back(KruhobotDecision(wait_transition.m_trans_id,
								    wait_transition.m_time,
								    wait_transition.m_location_id,
								    wait_transition.m_prev_trans_id));
			    kruhobot_RDD_mapping[wait_transition.m_location_id].insert(KruhobotDecisionIDs_mmap::value_type(wait_transition.m_time, wait_transition.m_trans_id));
			    
			    wait_explored_Transitions->insert(Explorations_umap::value_type(front_transition.m_location_id, front_transition.m_trans_id));		
			    transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_cost, wait_transition));
			}
		    }
		}				
	    }
	    transition_Queue.erase(transition_Queue.begin());
	}
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
    
    
    sDouble sRealSMTCBS::build_KruhobotRealDecisionDiagram(const sKruhobot                      &kruhobot,
							   const s2DMap                         &map,
							   sInt_32                               source_loc_id,
							   sInt_32                               sink_loc_id,
							   sDouble                               cost_limit,
							   sDouble                               extra_cost,
							   const LocationConflicts_upper__umap  &location_Conflicts,
							   const LinearConflicts_upper__map     &linear_Conflicts,
							   sDouble                               makespan_bound,
							   KruhobotDecisionDiagram_vector       &kruhobot_RDD,
							   KruhobotDecisionMapping_map          &kruhobot_RDD_mapping) const
    {
	sDouble next_makespan_bound = -1.0;
	
	Transitions_mmap transition_Queue;
	TransitionExplorations_map explored_Transitions;
	Explorations_umap bound_explored_Transitions;	
	Transitions_vector transition_Store;

	Transition initial_transition(transition_Store.size(), 0.0, 0.0, source_loc_id, -1);
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

	    if (front_transition.m_time + map.m_straight_Distances[front_transition.m_location_id][sink_loc_id] <= cost_limit + extra_cost)
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
			    Transition neighbor_transition(transition_Store.size(), transition_finish_time, transition_finish_cost, neighbor_location_id, front_transition.m_trans_id);		    
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
				transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_cost, neighbor_transition));				
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
			    
			    LinearConflicts_upper__map::const_iterator linear_Conflict = linear_Conflicts.find(Uline(front_transition.m_location_id, neighbor_location_id));
			    sDouble first_non_conf_linear_time = -1.0;
			    
			    if (linear_Conflict != linear_Conflicts.end())
			    {
				LinearConflicts_upper_map::const_iterator lower_linear_conflict = linear_Conflict->second.end();

				LinearConflicts_upper_map::const_iterator lower2_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_transition.m_time, front_transition.m_time));
				if (lower2_linear_conflict != linear_Conflict->second.end())
				{
				    for (LinearConflicts_map::const_iterator lower2_linear_conflict = linear_Conflict->second.begin(); lower2_linear_conflict != linear_Conflict->second.end(); ++lower2_linear_conflict)
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
			    Transition wait_transition(transition_Store.size(), wait_finish_time, front_transition.m_cost + wait_cost, front_transition.m_location_id, front_transition.m_trans_id);
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
				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_cost, wait_transition));				
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
	interconnect_KruhobotRealDecisionDiagram(kruhobot, map, kruhobot_RDD, kruhobot_RDD_mapping);

	return next_makespan_bound;
    }
    

    void sRealSMTCBS::interconnect_KruhobotRealDecisionDiagram(const sKruhobot                &kruhobot,
							       const s2DMap                   &map,
							       KruhobotDecisionDiagram_vector &kruhobot_RDD,
							       KruhobotDecisionMapping_map    &sUNUSED(kruhobot_RDD_mapping)) const
    {
	for (KruhobotDecisionDiagram_vector::iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	{
	    KruhobotDecisionDiagram_vector::iterator wait_decision = kruhobot_RDD.end();

//	    KruhobotDecisionDiagram_vector::iterator next_decision = decision;	    
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
//			        sASSERT(next_decision->m_dec_id > decision->m_dec_id);
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
    
    
    bool sRealSMTCBS::compare_KruhobotRealDecisionDiagrams(const KruhobotDecisionDiagram_vector &kruhobot_RDD, const KruhobotDecisionDiagram_vector &next_kruhobot_RDD) const
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

    
/*----------------------------------------------------------------------------*/
    
    sDouble sRealSMTCBS::calc_MakespanBound(const sRealInstance                    &real_Instance,
					    const KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
					    const KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts) const
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
					    const KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts) const
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
    
    
    sDouble sRealSMTCBS::calc_KruhobotMakespanBound(const sKruhobot &sUNUSED(kruhobot), const LocationConflicts__umap &location_Conflicts, const LinearConflicts__map &linear_Conflicts) const
    {
	sDouble makespan_bound = 0.0;
	
	for (LocationConflicts__umap::const_iterator location_Conflict = location_Conflicts.begin(); location_Conflict != location_Conflicts.end(); ++location_Conflict)
	{
	    for (LocationConflicts_map::const_iterator location_conflict = location_Conflict->second.begin(); location_conflict != location_Conflict->second.end(); ++location_conflict)
	    {
		makespan_bound = sMAX(makespan_bound, location_conflict->first.m_upper);
	    }
	}

	for (LinearConflicts__map::const_iterator linear_Conflict = linear_Conflicts.begin(); linear_Conflict != linear_Conflicts.end(); ++linear_Conflict)
	{
	    for (LinearConflicts_map::const_iterator linear_conflict = linear_Conflict->second.begin(); linear_conflict != linear_Conflict->second.end(); ++linear_conflict)
	    {
		makespan_bound = sMAX(makespan_bound, linear_conflict->first.m_upper);
	    }
	}	

	return makespan_bound;
    }


    sDouble sRealSMTCBS::calc_KruhobotMakespanBound(const sKruhobot &sUNUSED(kruhobot), const LocationConflicts_upper__umap &location_Conflicts, const LinearConflicts_upper__map &linear_Conflicts) const
    {
	sDouble makespan_bound = 0.0;
	
	for (LocationConflicts_upper__umap::const_iterator location_Conflict = location_Conflicts.begin(); location_Conflict != location_Conflicts.end(); ++location_Conflict)
	{
	    for (LocationConflicts_upper_map::const_iterator location_conflict = location_Conflict->second.begin(); location_conflict != location_Conflict->second.end(); ++location_conflict)
	    {
		makespan_bound = sMAX(makespan_bound, location_conflict->first.m_upper);
	    }
	}

	for (LinearConflicts_upper__map::const_iterator linear_Conflict = linear_Conflicts.begin(); linear_Conflict != linear_Conflicts.end(); ++linear_Conflict)
	{
	    for (LinearConflicts_upper_map::const_iterator linear_conflict = linear_Conflict->second.begin(); linear_conflict != linear_Conflict->second.end(); ++linear_conflict)
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
						const KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts) const
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
						const KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts) const
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
							const LinearConflicts__map    &linear_Conflicts) const
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

	for (LinearConflicts__map::const_iterator linear_Conflict = linear_Conflicts.begin(); linear_Conflict != linear_Conflicts.end(); ++linear_Conflict)
	{
	    for (LinearConflicts_map::const_iterator linear_conflict = linear_Conflict->second.begin(); linear_conflict != linear_Conflict->second.end(); ++linear_conflict)
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
							const LinearConflicts_upper__map    &linear_Conflicts) const
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

	for (LinearConflicts_upper__map::const_iterator linear_Conflict = linear_Conflicts.begin(); linear_Conflict != linear_Conflicts.end(); ++linear_Conflict)
	{
	    for (LinearConflicts_upper_map::const_iterator linear_conflict = linear_Conflict->second.begin(); linear_conflict != linear_Conflict->second.end(); ++linear_conflict)
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
	    sASSERT(!goal_decision_IDs.empty());
	    
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
				}
			    }
			}
		    }
		    if (!mutex_source_Identifiers.empty())
		    {
			m_solver_Encoder->cast_MultiImplication(solver,
								real_sat_Model.m_vertex_occupancy[kruhobot_id][dec_id],
								mutex_source_Identifiers);
			m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_source_Identifiers);
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
						  const sRealInstance                   &real_Instance,
						  const KruhobotDecisionDiagrams_vector &kruhobot_RDDs,
						  const KruhobotDecisionMappings_vector &kruhobot_RDD_Mappings,
						  const KruhobotCollisions_mset         &kruhobot_Collisions,
						  RealModel                             &real_sat_Model) const
    {
	for (KruhobotCollisions_mset::const_iterator kruhobot_collision = kruhobot_Collisions.begin(); kruhobot_collision != kruhobot_Collisions.end(); ++kruhobot_collision)
	{
	    std::vector<sInt_32> variable_A_IDs, variable_B_IDs;	    
	    sInt_32 kruhobot_A_id = sABS(kruhobot_collision->m_traversal_A.m_kruhobot_id);

	    if (kruhobot_collision->m_traversal_A.m_u_loc_id != kruhobot_collision->m_traversal_A.m_v_loc_id)
	    {
		DecisionIDs_vector decision_A_IDs;
		std::vector<sInt_32> Neighbors_A;
		
		sInt_32 match_count = match_CorrespondingNeighbors(kruhobot_collision->m_traversal_A,
								   kruhobot_RDDs[kruhobot_A_id],
								   kruhobot_RDD_Mappings[kruhobot_A_id],
								   decision_A_IDs,					     
								   Neighbors_A);
//		sASSERT(!decision_A_IDs.empty() && !Neighbors_A.empty());
		for (sInt_32 match = 0; match < match_count; ++match)
		{
//		    printf("A %d: %d --> %d\n", kruhobot_A_id, decision_A_IDs[match], kruhobot_RDDs[kruhobot_A_id][decision_A_IDs[match]].m_next_dec_IDs[Neighbors_A[match]]);
		    variable_A_IDs.push_back(real_sat_Model.m_edge_occupancy[kruhobot_A_id][decision_A_IDs[match]][Neighbors_A[match]]);
		}
	    }
	    else
	    {
		DecisionIDs_vector decision_A_IDs;		
		sInt_32 match_count = match_CorrespondingDecisions(kruhobot_collision->m_traversal_A,
								   kruhobot_RDDs[kruhobot_A_id],
								   kruhobot_RDD_Mappings[kruhobot_A_id],
								   decision_A_IDs);
//		sASSERT(!decision_A_IDs.empty());
		for (sInt_32 match = 0; match < match_count; ++match)
		{
//		    printf("A %d: > %d <\n", kruhobot_A_id, decision_A_IDs[match]);
		    variable_A_IDs.push_back(real_sat_Model.m_vertex_occupancy[kruhobot_A_id][decision_A_IDs[match]]);
		}

		std::vector<sInt_32> sink_A_IDs;
		match_count = match_CorrespondingSinks(real_Instance,
						       kruhobot_A_id,
//						       context.m_makespan_bound,
						       kruhobot_collision->m_traversal_A,
						       kruhobot_RDDs[kruhobot_A_id],
						       kruhobot_RDD_Mappings[kruhobot_A_id],
						       sink_A_IDs);
		for (sInt_32 match = 0; match < match_count; ++match)
		{
//		    printf("A' %d: > %d <\n", kruhobot_A_id, sink_A_IDs[match]);
		    variable_A_IDs.push_back(real_sat_Model.m_goal_sinking[kruhobot_A_id][sink_A_IDs[match]]);
		}
	    }
	    sInt_32 kruhobot_B_id = sABS(kruhobot_collision->m_traversal_B.m_kruhobot_id);
	    
	    if (kruhobot_collision->m_traversal_B.m_u_loc_id != kruhobot_collision->m_traversal_B.m_v_loc_id)
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
//		    printf("B %d: %d --> %d\n", kruhobot_B_id, decision_B_IDs[match], kruhobot_RDDs[kruhobot_B_id][decision_B_IDs[match]].m_next_dec_IDs[Neighbors_B[match]]);
		    variable_B_IDs.push_back(real_sat_Model.m_edge_occupancy[kruhobot_B_id][decision_B_IDs[match]][Neighbors_B[match]]);
		}
	    }
	    else
	    {
		DecisionIDs_vector decision_B_IDs;		
		sInt_32 match_count = match_CorrespondingDecisions(kruhobot_collision->m_traversal_B,
								   kruhobot_RDDs[kruhobot_B_id],
								   kruhobot_RDD_Mappings[kruhobot_B_id],
								   decision_B_IDs);			    
//		sASSERT(!decision_B_IDs.empty());
		for (sInt_32 match = 0; match < match_count; ++match)
		{
//		    printf("B %d: > %d <\n", kruhobot_B_id, decision_B_IDs[match]);
		    variable_B_IDs.push_back(real_sat_Model.m_vertex_occupancy[kruhobot_B_id][decision_B_IDs[match]]);
		}

		std::vector<sInt_32> sink_B_IDs;
		match_count = match_CorrespondingSinks(real_Instance,
						       kruhobot_B_id,
//						       context.m_makespan_bound,
						       kruhobot_collision->m_traversal_B,
						       kruhobot_RDDs[kruhobot_B_id],
						       kruhobot_RDD_Mappings[kruhobot_B_id],
						       sink_B_IDs);
		for (sInt_32 match = 0; match < match_count; ++match)
		{
//		    printf("B' %d: > %d <\n", kruhobot_B_id, sink_B_IDs[match]);
		    variable_B_IDs.push_back(real_sat_Model.m_goal_sinking[kruhobot_B_id][sink_B_IDs[match]]);
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
		if (kruhobot_RDD[mp_decision->second].m_time <= traversal.m_interval.m_lower + s_EPSILON)
		{
		    for (sInt_32 neigh = 0; neigh < kruhobot_RDD[mp_decision->second].m_next_dec_IDs.size(); ++neigh)
		    {
			if (   kruhobot_RDD[kruhobot_RDD[mp_decision->second].m_next_dec_IDs[neigh]].m_location_id == traversal.m_v_loc_id
			    && kruhobot_RDD[kruhobot_RDD[mp_decision->second].m_next_dec_IDs[neigh]].m_time >= traversal.m_interval.m_upper - s_EPSILON)
			{
			    Neighbors.push_back(neigh);
			    decision_IDs.push_back(kruhobot_RDD[mp_decision->second].m_dec_id);

//			    printf("[%d,%d] ", neigh, kruhobot_RDD[mp_decision->second].m_dec_id);			    
			}
		    }
		}
		++mp_decision;
	    }
	}
	{
	    KruhobotDecisionMapping_map::const_iterator mp_decisions_IDs = kruhobot_RDD_mapping.find(traversal.m_v_loc_id);
	    sASSERT(mp_decisions_IDs != kruhobot_RDD_mapping.end());
	    KruhobotDecisionIDs_mmap::const_iterator mp_decision = mp_decisions_IDs->second.begin();
	    
	    while (mp_decision != mp_decisions_IDs->second.end())
	    {
		if (kruhobot_RDD[mp_decision->second].m_time <= traversal.m_interval.m_lower + s_EPSILON)
		{
		    for (sInt_32 neigh = 0; neigh < kruhobot_RDD[mp_decision->second].m_next_dec_IDs.size(); ++neigh)
		    {
			if (   kruhobot_RDD[kruhobot_RDD[mp_decision->second].m_next_dec_IDs[neigh]].m_location_id == traversal.m_u_loc_id
			    && kruhobot_RDD[kruhobot_RDD[mp_decision->second].m_next_dec_IDs[neigh]].m_time >= traversal.m_interval.m_upper - s_EPSILON)
			{
			    Neighbors.push_back(neigh);
			    decision_IDs.push_back(kruhobot_RDD[mp_decision->second].m_dec_id);

//			    printf("[%d,%d] ", neigh, kruhobot_RDD[mp_decision->second].m_dec_id);			    			    
			}
		    }
		}
		++mp_decision;
	    }	    
	}
//	printf("\n");
	
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
	    sASSERT(kruhobot_Decisions[kruhobot_id].size() >= 2);
	    
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


