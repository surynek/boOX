/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             boOX 2-203_planck                              */
/*                                                                            */
/*                  (C) Copyright 2018 - 2022 Pavel Surynek                   */
/*                                                                            */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* smtcbsR_sat.cpp / 2-203_planck                                             */
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
	//real_sat_Model.m_goal_sinking.resize(N_kruhobots + 1);
	
	real_sat_Model.m_variable_mapping.push_back(RealCoordinate());

	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
	    sInt_32 final_goal_decision_ID;	    
	    DecisionIDs_vector goal_decision_IDs;
	    
	    collect_GoalKruhobotDecisions(real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id], kruhobot_RDDs[kruhobot_id], goal_decision_IDs, final_goal_decision_ID);
	    
	    real_sat_Model.m_vertex_occupancy[kruhobot_id].resize(kruhobot_RDDs[kruhobot_id].size());
	    //real_sat_Model.m_goal_sinking[kruhobot_id].resize(goal_decision_IDs.size());
	    //sInt_32 sink_id = 0;
	    
	    for (sInt_32 dec_id = 0; dec_id < kruhobot_RDDs[kruhobot_id].size(); ++dec_id)
	    {
		real_sat_Model.m_vertex_occupancy[kruhobot_id][dec_id] = variable_ID++;		    
		real_sat_Model.m_variable_mapping.push_back(RealCoordinate(kruhobot_id, dec_id));

		/*
		if (kruhobot_RDDs[kruhobot_id][dec_id].m_location_id == real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id])
		{
		    real_sat_Model.m_goal_sinking[kruhobot_id][sink_id] = variable_ID++;
		    real_sat_Model.m_variable_mapping.push_back(RealCoordinate(kruhobot_id, dec_id, sink_id));
		    
		    kruhobot_RDDs[kruhobot_id][dec_id].m_sink_id = sink_id;
		    ++sink_id;
		}
		*/
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
//		    real_sat_Model.m_edge_occupancy[kruhobot_id][dec_id].resize(kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.size() + 1);		    
		    real_sat_Model.m_edge_occupancy[kruhobot_id][dec_id].resize(kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.size());
		    /*
		    for (sInt_32 next = 0; next < kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.size() + 1; ++next)
		    {
			if (next < kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.size())
			{
			    printf("RDDg[%d]: %d --> %d (%d)\n", kruhobot_id, dec_id, kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs[next], variable_ID);
			}
			real_sat_Model.m_edge_occupancy[kruhobot_id][dec_id][next] = variable_ID++;				
		    }
		    */

		    for (sInt_32 next = 0; next < kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.size(); ++next)
		    {
			real_sat_Model.m_edge_occupancy[kruhobot_id][dec_id][next] = variable_ID++;
			/*
			#ifdef sDEBUG
			{
			    if (next < kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.size())
			    {
				printf("RDDg[%d]: %d --> %d (%d)\n", kruhobot_id, dec_id, kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs[next], variable_ID);
			    }
			}
			#endif
			*/
			real_sat_Model.m_transition_mapping.push_back(RealBicoordinate(kruhobot_id, dec_id, kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs[next]));			
		    }		    
		}
		else
		{
		    real_sat_Model.m_edge_occupancy[kruhobot_id][dec_id].resize(kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.size());
		    
		    for (sInt_32 next = 0; next < kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.size(); ++next)
		    {
			real_sat_Model.m_edge_occupancy[kruhobot_id][dec_id][next] = variable_ID++;
                        /*
			#ifdef sDEBUG
			{
			    printf("RDD[%d]: %d --> %d (%d)\n", kruhobot_id, dec_id, kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs[next], variable_ID);			
			}
			#endif
			*/
			real_sat_Model.m_transition_mapping.push_back(RealBicoordinate(kruhobot_id, dec_id, kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs[next]));			
		    }		    
		}
	    }
	}

	return variable_ID;
    }


    sInt_32 sRealSMTCBS::build_CostRealModelVariables(Glucose::Solver                       *sUNUSED(solver),
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
	//real_sat_Model.m_goal_sinking.resize(N_kruhobots + 1);
	
	real_sat_Model.m_variable_mapping.push_back(RealCoordinate());

	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
	    sInt_32 final_goal_decision_ID;
	    DecisionIDs_vector goal_decision_IDs;
	    
	    collect_GoalKruhobotDecisions(real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id], kruhobot_RDDs[kruhobot_id], goal_decision_IDs, final_goal_decision_ID);
	    
	    real_sat_Model.m_vertex_occupancy[kruhobot_id].resize(kruhobot_RDDs[kruhobot_id].size());
	    //real_sat_Model.m_goal_sinking[kruhobot_id].resize(goal_decision_IDs.size());
	    //sInt_32 sink_id = 0;
	    
	    for (sInt_32 dec_id = 0; dec_id < kruhobot_RDDs[kruhobot_id].size(); ++dec_id)
	    {
		real_sat_Model.m_vertex_occupancy[kruhobot_id][dec_id] = variable_ID++;		    
		real_sat_Model.m_variable_mapping.push_back(RealCoordinate(kruhobot_id, dec_id));

		/*
		if (kruhobot_RDDs[kruhobot_id][dec_id].m_location_id == real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id])
		{
		    real_sat_Model.m_goal_sinking[kruhobot_id][sink_id] = variable_ID++;
		    real_sat_Model.m_variable_mapping.push_back(RealCoordinate(kruhobot_id, dec_id, sink_id));
		    
		    kruhobot_RDDs[kruhobot_id][dec_id].m_sink_id = sink_id;
		    ++sink_id;
		}
		*/
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
		    /*
		    
		    for (sInt_32 next = 0; next < kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.size() + 1; ++next)
		    {
			real_sat_Model.m_edge_occupancy[kruhobot_id][dec_id][next] = variable_ID++;
		    }
		    for (sInt_32 next = 0; next < kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.size(); ++next)
		    {
			real_sat_Model.m_transition_mapping.push_back(RealBicoordinate(kruhobot_id, dec_id, kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs[next]));
		    }
		    real_sat_Model.m_transition_mapping.push_back(RealBicoordinate(kruhobot_id, dec_id, -1));
		    */
		    for (sInt_32 next = 0; next < kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.size(); ++next)
		    {
			real_sat_Model.m_edge_occupancy[kruhobot_id][dec_id][next] = variable_ID++;
			real_sat_Model.m_transition_mapping.push_back(RealBicoordinate(kruhobot_id, dec_id, kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs[next]));
		    }
		}
		else
		{
		    real_sat_Model.m_edge_occupancy[kruhobot_id][dec_id].resize(kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.size());
		    
		    for (sInt_32 next = 0; next < kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.size(); ++next)
		    {
			real_sat_Model.m_edge_occupancy[kruhobot_id][dec_id][next] = variable_ID++;
			real_sat_Model.m_transition_mapping.push_back(RealBicoordinate(kruhobot_id, dec_id, kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs[next]));			
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
	    sInt_32 final_goal_decision_ID;
	    DecisionIDs_vector goal_decision_IDs;
	    collect_GoalKruhobotDecisions(real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id]/*, context.m_makespan_bound*/, kruhobot_RDDs[kruhobot_id], goal_decision_IDs, final_goal_decision_ID);
	    
//	    sInt_32 sink_id = 0;

	    for (sInt_32 dec_id = 0; dec_id < kruhobot_RDDs[kruhobot_id].size(); ++dec_id)
	    {
		VariableIDs_vector mutex_target_Identifiers;
				    
		if (!kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.empty())
		{
		    for (sInt_32 next_dec = 0; next_dec < kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs.size(); ++next_dec)
		    {
			sInt_32 next_dec_id = kruhobot_RDDs[kruhobot_id][dec_id].m_next_dec_IDs[next_dec];

			mutex_target_Identifiers.push_back(real_sat_Model.m_edge_occupancy[kruhobot_id][dec_id][next_dec]);		       

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
		/*
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
		*/
		
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
			//m_solver_Encoder->cast_AdaptiveAllMutexConstraint(solver, mutex_source_Identifiers);
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
	    sInt_32 final_goal_decision_ID;
	    DecisionIDs_vector goal_decision_IDs;
	    
	    collect_GoalKruhobotDecisions(real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id]/*, context.m_makespan_bound*/, kruhobot_RDDs[kruhobot_id], goal_decision_IDs, final_goal_decision_ID);
//	    printf("%ld, %ld\n", goal_decision_IDs.size() == real_sat_Model.m_goal_sinking[kruhobot_id].size());
	    /*
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
	    */
	    m_solver_Encoder->cast_BitSet(solver, real_sat_Model.m_vertex_occupancy[kruhobot_id][final_goal_decision_ID]);
	    
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
            /*
	    #ifdef sDEBUG
	    {
		printf("%ld,%ld\n", variable_A_IDs.size(), variable_B_IDs.size());
		for (std::vector<sInt_32>::const_iterator variable_A_id = variable_A_IDs.begin(); variable_A_id != variable_A_IDs.end(); ++variable_A_id)
		{
		    for (std::vector<sInt_32>::const_iterator variable_B_id = variable_B_IDs.begin(); variable_B_id != variable_B_IDs.end(); ++variable_B_id)
		    {		    
			printf("  mutex:%d x %d\n", *variable_A_id, *variable_B_id);
		    }
		}

	    
		for (std::vector<sInt_32>::const_iterator variable_A_id = variable_A_IDs.begin(); variable_A_id != variable_A_IDs.end(); ++variable_A_id)
		{
		    const RealBicoordinate &bicoordinate = real_sat_Model.m_transition_mapping[*variable_A_id - real_sat_Model.m_variable_mapping.size()];
		    
		    sInt_32 kruhobot_id = bicoordinate.m_kruhobot_id;
		    sInt_32 decision_id = bicoordinate.m_decision_id;
		    sInt_32 next_id = bicoordinate.m_next_id;
		    
		    sInt_32 u_id = kruhobot_RDDs[kruhobot_id][decision_id].m_location_id;
		    sInt_32 v_id = kruhobot_RDDs[kruhobot_id][next_id].m_location_id;
		    
		    sDouble u_time = kruhobot_RDDs[kruhobot_id][decision_id].m_time;
		    sDouble v_time = kruhobot_RDDs[kruhobot_id][next_id].m_time;
		
		    printf("A: [%d] %d --> %d (%.3f,%.3f) %d\n", kruhobot_id, u_id, v_id, u_time, v_time, *variable_A_IDs.begin());
		    
		    sASSERT(   kruhobot_id == kruhobot_A_id
			    && kruhobot_collision->m_traversal_A.m_u_loc_id == u_id
			    && kruhobot_collision->m_traversal_A.m_v_loc_id == v_id
			    && sABS(kruhobot_collision->m_traversal_A.m_interval.m_lower - u_time) < s_EPSILON
			    && sABS(kruhobot_collision->m_traversal_A.m_interval.m_upper - v_time) < s_EPSILON);
		}
		for (std::vector<sInt_32>::const_iterator variable_B_id = variable_B_IDs.begin(); variable_B_id != variable_B_IDs.end(); ++variable_B_id)
		{
		    const RealBicoordinate &bicoordinate = real_sat_Model.m_transition_mapping[*variable_B_id - real_sat_Model.m_variable_mapping.size()];
		    
		    sInt_32 kruhobot_id = bicoordinate.m_kruhobot_id;
		    sInt_32 decision_id = bicoordinate.m_decision_id;
		    sInt_32 next_id = bicoordinate.m_next_id;		    		    
		    
		    sInt_32 u_id = kruhobot_RDDs[kruhobot_id][decision_id].m_location_id;
		    sInt_32 v_id = kruhobot_RDDs[kruhobot_id][next_id].m_location_id;
		    
		    sDouble u_time = kruhobot_RDDs[kruhobot_id][decision_id].m_time;
		    sDouble v_time = kruhobot_RDDs[kruhobot_id][next_id].m_time;
		
		    printf("B: [%d] %d --> %d (%.3f,%.3f) %d\n", kruhobot_id, u_id, v_id, u_time, v_time, *variable_B_IDs.begin());

		    sASSERT(   kruhobot_id == kruhobot_B_id
			    && kruhobot_collision->m_traversal_B.m_u_loc_id == u_id
			    && kruhobot_collision->m_traversal_B.m_v_loc_id == v_id
			    && sABS(kruhobot_collision->m_traversal_B.m_interval.m_lower - u_time) < s_EPSILON
			    && sABS(kruhobot_collision->m_traversal_B.m_interval.m_upper - v_time) < s_EPSILON);
		}
	    }
	    #endif
	    */
	    
	    m_solver_Encoder->cast_Mutexes(solver, variable_A_IDs, variable_B_IDs);	    
	}
    }


    void sRealSMTCBS::introduce_RealModelCostCardinalities(Glucose::Solver                       *solver,
							   RealContext                           &sUNUSED(context),
							   const sRealInstance                   &real_Instance,
							   KruhobotDecisionDiagrams_vector       &kruhobot_RDDs,
							   const KruhobotDecisionMappings_vector &sUNUSED(kruhobot_RDD_Mappings),
							   sDouble                                sUNUSED(cost_bound),
							   const std::vector<sDouble>            &sUNUSED(kruhobot_lower_cost_Bounds),
							   RealModel                             &sUNUSED(real_sat_Model),
							   const KruhobotExtraVariables_vector   &kruhobot_envelope_Variables) const
    {
	sASSERT(!kruhobot_RDDs.empty());

	VariableIDs_vector cost_cardinality_variable_IDs;

	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();

	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{	
	    if (!kruhobot_envelope_Variables[kruhobot_id].empty())
	    {
		cost_cardinality_variable_IDs.push_back(kruhobot_envelope_Variables[kruhobot_id].begin()->second);
	    }
	}
	m_solver_Encoder->cast_BigMutex(solver, cost_cardinality_variable_IDs);
	
	/*
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();
	
	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
	    for (sInt_32 dec_id = 0; dec_id < kruhobot_RDDs[kruhobot_id].size(); ++dec_id)
	    {
		if (kruhobot_RDDs[kruhobot_id][dec_id].m_time > kruhobot_lower_cost_Bounds[kruhobot_id])
		{
		    printf("Overlapping kruhobot: %d, decision: %d, time: %.3f [lover time: %.3f]\n", kruhobot_id, dec_id, kruhobot_RDDs[kruhobot_id][dec_id].m_time, kruhobot_lower_cost_Bounds[kruhobot_id]);
		    sInt_32 variable_id = real_sat_Model.m_vertex_occupancy[kruhobot_id][dec_id];		    
		    printf("... this is variable: %d\n", variable_id);
		}
	    }
	}
	*/
    }


    void sRealSMTCBS::introduce_RealModelCostCardinalities(Glucose::Solver                       *solver,
							   RealContext                           &sUNUSED(context),
							   const sRealInstance                   &real_Instance,
							   KruhobotDecisionDiagrams_vector       &kruhobot_RDDs,
							   const KruhobotDecisionMappings_vector &sUNUSED(kruhobot_RDD_Mappings),
							   sDouble                                sUNUSED(cost_bound),
							   const std::vector<sDouble>            &sUNUSED(kruhobot_lower_cost_Bounds),
							   RealModel                             &sUNUSED(real_sat_Model),
							   const KruhobotExtraVariables_vector   &kruhobot_envelope_Variables,
							   const KruhobotExtraVariables_vector   &kruhobot_domus_Variables) const
    {
	sASSERT(!kruhobot_RDDs.empty());
	
	VariableIDs_2vector kruhobot_cost_cardinality_variable_IDs;

	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();
	
	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
	    if (!kruhobot_envelope_Variables[kruhobot_id].empty())
	    {
		VariableIDs_vector cost_cardinality_variable_IDs;
		for (ExtraVariables_mmap::const_iterator extra_variable = kruhobot_domus_Variables[kruhobot_id].begin(); extra_variable != kruhobot_domus_Variables[kruhobot_id].end(); ++extra_variable)
		{
		    cost_cardinality_variable_IDs.push_back(extra_variable->second);		    
		}
		kruhobot_cost_cardinality_variable_IDs.push_back(cost_cardinality_variable_IDs);
	    }
	}
	m_solver_Encoder->cast_DomularMutex(solver, kruhobot_cost_cardinality_variable_IDs);
	
	/*
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();
	
	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
	    for (sInt_32 dec_id = 0; dec_id < kruhobot_RDDs[kruhobot_id].size(); ++dec_id)
	    {
		if (kruhobot_RDDs[kruhobot_id][dec_id].m_time > kruhobot_lower_cost_Bounds[kruhobot_id])
		{
		    printf("Overlapping kruhobot: %d, decision: %d, time: %.3f [lover time: %.3f]\n", kruhobot_id, dec_id, kruhobot_RDDs[kruhobot_id][dec_id].m_time, kruhobot_lower_cost_Bounds[kruhobot_id]);
		    sInt_32 variable_id = real_sat_Model.m_vertex_occupancy[kruhobot_id][dec_id];		    
		    printf("... this is variable: %d\n", variable_id);
		}
	    }
	}
	*/
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
/*
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
*/	
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
			{
			    /*
  		            #ifdef sDEBUG
			    {
			        printf("---__ Sinked into goal kruhobot:%d, loc:%d, time:%.3f, dec:%d\n", kruhobot_id, location_id, time, decision_id);
			    }
		            #endif			
			    */
                        }
		    }
		}
		else
		{
//		    printf("Transit: %d, %ld, %ld\n", variable_ID, real_sat_Model.m_variable_mapping.size(), real_sat_Model.m_transition_mapping.size());
		    /*
		    if (variable_ID < real_sat_Model.m_variable_mapping.size() + real_sat_Model.m_transition_mapping.size())
		    {
			const RealBicoordinate &bicoordinate = real_sat_Model.m_transition_mapping[variable_ID - real_sat_Model.m_variable_mapping.size()];
			
			sInt_32 kruhobot_id = bicoordinate.m_kruhobot_id;
			sInt_32 decision_id = bicoordinate.m_decision_id;
			sInt_32 next_id = bicoordinate.m_next_id;		    

			if (next_id > 0)
			{
			    sInt_32 u_id = kruhobot_RDDs[kruhobot_id][decision_id].m_location_id;
			    sInt_32 v_id = kruhobot_RDDs[kruhobot_id][next_id].m_location_id;
			    
			    sDouble u_time = kruhobot_RDDs[kruhobot_id][decision_id].m_time;
			    sDouble v_time = kruhobot_RDDs[kruhobot_id][next_id].m_time;

			    if (u_id != real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id] || v_id != real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id])				
			    {			    
				printf("[%d] %d --> %d (%.3f,%.3f) %d\n", kruhobot_id, u_id, v_id, u_time, v_time, variable_ID);
			    }
			}
		    }
		    */
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


    void sRealSMTCBS::decode_PathModel(Glucose::Solver                       *solver,
				       const sRealInstance                   &real_Instance,
				       const KruhobotDecisionDiagrams_vector &kruhobot_RDDs,
				       const RealModel                       &real_sat_Model,
				       sDouble                                sUNUSED(cost_bound),
				       const std::vector<sDouble>            &kruhobot_lower_cost_Bounds,
				       KruhobotSchedules_vector              &kruhobot_Schedules,
				       KruhobotExtraVariables_vector         &kruhobot_set_extra_Variables,
				       KruhobotExtraVariables_vector         &kruhobot_all_extra_Variables) const
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
/*
		    if (location_id != real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id])
		    {
			if (time > kruhobot_lower_cost_Bounds[kruhobot_id])
			{
			    kruhobot_all_extra_Variables[kruhobot_id].insert(ExtraVariables_map::value_type(time, variable_ID));
			    kruhobot_set_extra_Variables[kruhobot_id].insert(ExtraVariables_map::value_type(time, variable_ID));
			}		    			
		    }
*/		    
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
		else
		{
		    if (variable_ID < real_sat_Model.m_variable_mapping.size() + real_sat_Model.m_transition_mapping.size())
		    {
			const RealBicoordinate &bicoordinate = real_sat_Model.m_transition_mapping[variable_ID - real_sat_Model.m_variable_mapping.size()];
			
			sInt_32 kruhobot_id = bicoordinate.m_kruhobot_id;
			sInt_32 decision_id = bicoordinate.m_decision_id;
			sInt_32 next_id = bicoordinate.m_next_id;		    

			if (next_id > 0)
			{
			    sInt_32 u_id = kruhobot_RDDs[kruhobot_id][decision_id].m_location_id;
			    sInt_32 v_id = kruhobot_RDDs[kruhobot_id][next_id].m_location_id;
			    
			    //sDouble u_time = kruhobot_RDDs[kruhobot_id][decision_id].m_time;
			    sDouble v_time = kruhobot_RDDs[kruhobot_id][next_id].m_time;

			    if (v_time > kruhobot_lower_cost_Bounds[kruhobot_id])
			    {
				if (u_id != real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id] || v_id != real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id])				
				{			    
				    kruhobot_all_extra_Variables[kruhobot_id].insert(ExtraVariables_mmap::value_type(v_time, variable_ID));
				    kruhobot_set_extra_Variables[kruhobot_id].insert(ExtraVariables_mmap::value_type(v_time, variable_ID));
				}
			    }
			}
		    }
		}
	    }
	    else
	    {
		sInt_32 variable_ID = sABS(literal);
		
		if (variable_ID < real_sat_Model.m_variable_mapping.size())
		{
		    /*
		    const RealCoordinate &coordinate = real_sat_Model.m_variable_mapping[variable_ID];

		    sInt_32 kruhobot_id = coordinate.m_kruhobot_id;
		    sInt_32 decision_id = coordinate.m_decision_id;
		    sInt_32 location_id = kruhobot_RDDs[kruhobot_id][decision_id].m_location_id;
		    sDouble time = kruhobot_RDDs[kruhobot_id][decision_id].m_time;

		    if (location_id != real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id])
		    {
			if (time > kruhobot_lower_cost_Bounds[kruhobot_id])
			{
			    kruhobot_all_extra_Variables[kruhobot_id].insert(ExtraVariables_map::value_type(time, variable_ID));			
			}
		    }
		    */
		}
		else
		{
		    if (variable_ID < real_sat_Model.m_variable_mapping.size() + real_sat_Model.m_transition_mapping.size())
		    {
			const RealBicoordinate &bicoordinate = real_sat_Model.m_transition_mapping[variable_ID - real_sat_Model.m_variable_mapping.size()];
			
			sInt_32 kruhobot_id = bicoordinate.m_kruhobot_id;
			sInt_32 decision_id = bicoordinate.m_decision_id;
			sInt_32 next_id = bicoordinate.m_next_id;		    

			if (next_id > 0)
			{
			    sInt_32 u_id = kruhobot_RDDs[kruhobot_id][decision_id].m_location_id;
			    sInt_32 v_id = kruhobot_RDDs[kruhobot_id][next_id].m_location_id;
			    
			    //sDouble u_time = kruhobot_RDDs[kruhobot_id][decision_id].m_time;
			    sDouble v_time = kruhobot_RDDs[kruhobot_id][next_id].m_time;

			    if (v_time > kruhobot_lower_cost_Bounds[kruhobot_id])
			    {
				if (u_id != real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id] || v_id != real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id])				
				{			    
				    kruhobot_all_extra_Variables[kruhobot_id].insert(ExtraVariables_mmap::value_type(v_time, variable_ID));
				}
			    }
			}
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


    void sRealSMTCBS::collect_GoalKruhobotDecisions(sInt_32 goal_location_id, const KruhobotDecisionDiagram_vector &kruhobot_RDD, DecisionIDs_vector &decision_IDs, sInt_32 &final_goal_decision_ID) const
    {
	sDouble final_goal_decision_time;
	final_goal_decision_ID = -1;
	
	for (KruhobotDecisionDiagram_vector::const_iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	{
	    if (decision->m_location_id == goal_location_id)
	    {
		decision_IDs.push_back(decision->m_dec_id);

		if (final_goal_decision_ID < 0)
		{
		    final_goal_decision_ID = decision->m_dec_id;
		    final_goal_decision_time = decision->m_time;
		}
		else
		{
		    if (decision->m_time > final_goal_decision_time)
		    {
			final_goal_decision_ID = decision->m_dec_id;
			final_goal_decision_time = decision->m_time;			
		    }
		}
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

} // namespace boOX
