/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             boOX 2-194_planck                              */
/*                                                                            */
/*                  (C) Copyright 2018 - 2022 Pavel Surynek                   */
/*                                                                            */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* smtcbsR_rdd.cpp / 2-194_planck                                             */
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
			if (transition_distance <= s_EPSILON)
			{
			    continue;
			}			    							    			
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
			if (transition_distance <= s_EPSILON)
			{
			    continue;
			}			    							    			
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
			if (transition_distance <= s_EPSILON)
			{
			    continue;
			}			    							    			
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
			if (transition_distance <= s_EPSILON)
			{
			    continue;
			}			    							    			
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
			if (transition_distance <= s_EPSILON)
			{
			    continue;
			}			    							    			
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
			if (transition_distance <= s_EPSILON)
			{
			    continue;
			}			    							    			
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
			    if (transition_distance <= s_EPSILON)
			    {
				continue;
			    }			    							    			    
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
				    if (bypass_transition_distance <= s_EPSILON)
				    {
					continue;
				    }			    							    				    
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
        //fingerprint_limit = -1;
	
	#ifdef sDEBUG
	{
	    /*
	    printf("RDD location conflicts:\n");
	    for (LocationConflicts_upper__umap::const_iterator location_Conflict = location_Conflicts.begin(); location_Conflict != location_Conflicts.end(); ++location_Conflict)
	    {
		for (LocationConflicts_upper_map::const_iterator location_conflict = location_Conflict->second.begin(); location_conflict != location_Conflict->second.end(); ++location_conflict)
		{
		    location_conflict->second.to_Screen();
		}
	    }
	    printf("RDD linear conflicts:\n");
	    for (LinearConflicts_upper__map::const_iterator linear_Conflict = linear_Conflicts.begin(); linear_Conflict != linear_Conflicts.end(); ++linear_Conflict)
	    {
		for (LinearConflicts_upper_map::const_iterator linear_conflict = linear_Conflict->second.begin(); linear_conflict != linear_Conflict->second.end(); ++linear_conflict)
		{
		    linear_conflict->second.to_Screen();
		}
	    }
	    printf("kru:%d (fing:%d), ind_bnd:%.3f: --------------------------------\n", kruhobot.m_id, fingerprint_limit, individual_makespan_bound);
	    */
	}
	#endif
	
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
		//sDouble effective_makespan_bound = sMIN(individual_makespan_bound + front_respectful_transition.m_waited, makespan_bound);
		sDouble effective_makespan_bound = individual_makespan_bound;

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
			    if (transition_distance <= s_EPSILON)
			    {
				continue;
			    }
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
					//sDouble effective_makespan_bound = sMIN(individual_makespan_bound + neighbor_respectful_transition.m_waited, makespan_bound);
					sDouble effective_makespan_bound = individual_makespan_bound;
					
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

			    sDouble transition_distance = map.m_straight_Distances[front_respectful_transition.m_location_id][neighbor_location_id];
			    if (transition_distance <= s_EPSILON)
			    {
				continue;
			    }			    
			    sDouble transition_delta_time = transition_distance / kruhobot.m_properties.m_linear_velo;			    
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
					sDouble possible_start_time = lower2_location_conflict->second.m_interval.m_upper - transition_delta_time;
					
					if (possible_start_time > front_respectful_transition.m_time + s_EPSILON)
					{
					    lower_location_conflict = lower2_location_conflict;					    
					    first_non_conf_location_time = possible_start_time;
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
					if (lower2_linear_conflict->second.m_interval.m_upper > front_respectful_transition.m_time + s_EPSILON)
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
					if (lower2_location_conflict->second.m_interval.m_upper > front_respectful_transition.m_time + s_EPSILON)
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
				sASSERT(linear_Conflict == linear_Conflicts.end());
				/*
				sDouble first_non_conf_linear_time = -1.0;
				sInt_32 first_linear_culprit_conflict_id = -1;			    
			    
				if (linear_Conflict != linear_Conflicts.end())
				{
				    UlinearConflicts_upper_map::const_iterator lower_linear_conflict = linear_Conflict->second.end();
				    
				    for (UlinearConflicts_upper_map::const_iterator lower2_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_respectful_transition.m_time,
																			  front_respectful_transition.m_time));
					 lower2_linear_conflict != linear_Conflict->second.end(); ++lower2_linear_conflict)
				    {
					if (lower2_linear_conflict->second.m_interval.m_upper > front_respectful_transition.m_time + s_EPSILON)
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
				*/
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
//			if (wait_finish_time >= front_respectful_transition.m_time)
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
						
			        /* bypass non-removed */
				#define BYPASS
				#ifdef BYPASS
				const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[front_respectful_transition.m_location_id].m_Neighbors;
				for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
				{
				    sInt_32 neighbor_location_id = (*neighbor)->m_target->m_id;
				    {
					sDouble bypass_transition_distance = map.m_straight_Distances[front_respectful_transition.m_location_id][neighbor_location_id];
					if (bypass_transition_distance <= s_EPSILON)
					{
					    continue;
					}			    					
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
				#endif
	
				/* waiting */
				if (!is_UnifiedlyVisited(front_respectful_transition.m_location_id, wait_finish_time, unified_Visits))
				{
				    #ifdef sDEBUG
				    {
					/*
					printf("  culp: %d [", wait_culprit_conflict_id);				    
					for(ConflictIDs_set::const_iterator fing_conflict_id = front_respectful_transition.m_conflict_fingerprint.m_conflict_IDs.begin(); fing_conflict_id != front_respectful_transition.m_conflict_fingerprint.m_conflict_IDs.end(); ++fing_conflict_id)
					{
					    printf("%d ", *fing_conflict_id);
					}
					printf("]\n");
					*/
				    }
				    #endif
				    
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
					    //sDouble effective_makespan_bound = sMIN(individual_makespan_bound + wait_respectful_transition.m_waited, makespan_bound);
					    sDouble effective_makespan_bound = individual_makespan_bound;
					    
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
						//sDouble effective_makespan_bound = sMIN(individual_makespan_bound + wait_respectful_transition.m_waited, makespan_bound);
						sDouble effective_makespan_bound = individual_makespan_bound;
						
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
		printf("%d: Fin queue: %ld (%ld)\n", kruhobot.m_id, front_respectful_transition_Queue.size(), unified_Visits.size());
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
	/*
	printf("Before interconnecting\n");
	to_Screen(kruhobot_RDD);
	*/
	
	interconnect_KruhobotRealDecisionDiagram_smart(kruhobot, map, kruhobot_RDD, kruhobot_RDD_mapping);
	/*
	printf("After interconnecting\n");
	to_Screen(kruhobot_RDD);
	*/
	
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

	/*
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
	*/
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


    #define sREAL_SMT_CBS_UPDATE_NEXT_COST_BOUND(time)                                        \
    {                                                                                         \
	if (next_cost_bound < 0.0)                                                            \
	{                                                                                     \
	    if (time > cost_bound + s_EPSILON)                                                \
	    {                                                                                 \
		next_cost_bound = time;                                                       \
	    }                                                                                 \
	}                                                                                     \
	else                                                                                  \
	{                                                                                     \
	    if (time > cost_bound + s_EPSILON && time + s_EPSILON < next_cost_bound)          \
	    {                                                                                 \
		next_cost_bound = time;                                                       \
	    }                                                                                 \
	}                                                                                     \
    }


    #define sREAL_SMT_CBS_UPDATE_NEXT_INDIVIDUAL_COST_BOUND(time)                             \
    {                                                                                         \
	if (time >= individual_cost_bound + s_EPSILON)                                        \
	{					                                              \
	    if (next_individual_cost_bound >= individual_cost_bound + s_EPSILON)              \
	    {                                                                                 \
		if (next_individual_cost_bound > time)                                        \
		{                                                                             \
		    next_individual_cost_bound = time;                                        \
		}                                                                             \
	    }                                                                                 \
	    else                                                                              \
	    {                                                                                 \
		next_individual_cost_bound = time;                                            \
	    }                                                                                 \
	}                                                                                     \
    }

    
    sDouble sRealSMTCBS::build_KruhobotCostRealDecisionDiagram_individualizedConflictRespectfulBucketing(const sKruhobot                      &kruhobot,
													 const s2DMap                         &map,
													 sInt_32                               source_loc_id,
													 sInt_32                               sink_loc_id,
													 const LocationConflicts_upper__umap  &location_Conflicts,
													 const UlinearConflicts_upper__map    &linear_Conflicts,
													 sDouble                               makespan_bound,
													 sDouble                               individual_cost_bound,
													 sInt_32                               fingerprint_limit,
													 KruhobotDecisionDiagram_vector       &kruhobot_RDD,
													 KruhobotDecisionMapping_map          &kruhobot_RDD_mapping) const
    {
        #ifdef sDEBUG
	bool sink_reached = false;
	sInt_32 processed_nodes = 0;
        #endif	
	
	sInt_32 last_transition_id = 0;
//	sDouble next_makespan_bound = -1.0;

	sDouble next_individual_cost_bound = individual_cost_bound;

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

		sASSERT(individual_cost_bound >= 0.0);
		sDouble effective_makespan_bound = sMIN(individual_cost_bound + front_respectful_transition.m_waited, makespan_bound);
		
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
			    if (transition_distance <= s_EPSILON)
			    {
				continue;
			    }			    
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
				    
				    sREAL_SMT_CBS_UPDATE_NEXT_INDIVIDUAL_COST_BOUND(neighbor_respectful_transition.m_time);
//				    unified_Visits[neighbor_respectful_transition.m_location_id].insert(neighbor_respectful_transition.m_time);

				    if (next_respectful_visit == respectful_exploration->second.end()) /* visiting for the first time in a given fingerprint */
				    {
					sDouble estimated_remaining = map.m_shortest_Distances[sink_loc_id][neighbor_location_id] / kruhobot.m_properties.m_linear_velo;
					sASSERT(individual_cost_bound >= 0.0);					
					sDouble effective_makespan_bound = sMIN(individual_cost_bound + neighbor_respectful_transition.m_waited, makespan_bound);

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
					    sREAL_SMT_CBS_UPDATE_NEXT_INDIVIDUAL_COST_BOUND(neighbor_respectful_transition.m_time + estimated_remaining);
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
					if (bypass_transition_distance <= s_EPSILON)
					{
					    continue;
					}			    					
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
						    sREAL_SMT_CBS_UPDATE_NEXT_INDIVIDUAL_COST_BOUND(bypass_respectful_transition.m_time);
						    
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
				
					sREAL_SMT_CBS_UPDATE_NEXT_INDIVIDUAL_COST_BOUND(wait_respectful_transition.m_time);					
					wait_respectful_transition.m_conflict_fingerprint = next_conflict_fingerprint;

					if (respectful_exploration == respectful_Explorations.end()) /* non-existent fingerprint */
					{
					    sDouble estimated_remaining = map.m_shortest_Distances[sink_loc_id][wait_respectful_transition.m_location_id] / kruhobot.m_properties.m_linear_velo;
					    
					    sASSERT(individual_cost_bound >= 0.0);
					    sDouble effective_makespan_bound = sMIN(individual_cost_bound + wait_respectful_transition.m_waited, makespan_bound);
					    
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
						sREAL_SMT_CBS_UPDATE_NEXT_INDIVIDUAL_COST_BOUND(wait_respectful_transition.m_time + estimated_remaining);
					    }
					}
					else /* existent fingerprint */
					{
					    RespectfulVisits_umap::iterator wait_respectful_visit = respectful_exploration->second.find(front_respectful_transition.m_location_id);
					    
					    if (wait_respectful_visit == respectful_exploration->second.end()) /* visiting for the first time */
					    {
						sDouble estimated_remaining = map.m_shortest_Distances[sink_loc_id][wait_respectful_transition.m_location_id] / kruhobot.m_properties.m_linear_velo;

						sASSERT(individual_cost_bound >= 0.0);
						sDouble effective_makespan_bound = sMIN(individual_cost_bound + wait_respectful_transition.m_waited, makespan_bound);
						
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
						    sREAL_SMT_CBS_UPDATE_NEXT_INDIVIDUAL_COST_BOUND(wait_respectful_transition.m_time + estimated_remaining);
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
	/*
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
	    next_individual_cost_bound = -1.0;
	}
	*/
//	individual_cost_bound = next_individual_cost_bound;

	/*
	#ifdef sDEBUG
	{
	    printf("RDD size: %ld [processed: %d, percentage:%.3f]\n", kruhobot_RDD.size(), processed_nodes, (sDouble)kruhobot_RDD.size() / processed_nodes);
	}
	#endif
	*/
	
	return next_individual_cost_bound;
    }

    
    sDouble sRealSMTCBS::build_KruhobotCostRealDecisionDiagram_individualizedConflictRespectfulBucketingLeaping(const sKruhobot                      &kruhobot,
														const s2DMap                         &map,
														sInt_32                               source_loc_id,
														sInt_32                               sink_loc_id,
														const LocationConflicts_upper__umap  &location_Conflicts,
														const UlinearConflicts_upper__map    &linear_Conflicts,
														sDouble                               makespan_bound,
														sDouble                               individual_cost_bound,
														sInt_32                               fingerprint_limit,
														KruhobotDecisionDiagram_vector       &kruhobot_RDD,
														KruhobotDecisionMapping_map          &kruhobot_RDD_mapping,
														sDouble                              leap,
														SinkATTs_set                         &sink_ATTs) const
    {
        #ifdef sDEBUG
	bool sink_reached = false;
	sInt_32 processed_nodes = 0;
        #endif	
	
	sInt_32 last_transition_id = 0;
//	sDouble next_makespan_bound = -1.0;

	sDouble next_individual_cost_bound = individual_cost_bound;

	RespectfulExplorations_map respectful_Explorations;
	RespectfulExplorations_map respectful_Bypasses;	

	BucketedRespectfulTransitions_mmap bucketed_respectful_transition_Queues;

	RespectfulTransition initial_transition(last_transition_id++, 0.0, 0.0, 0.0, 0.0, source_loc_id, -1);
	initial_transition.m_prev_corr_dec_id = -1;

	UnifiedVisits_umap unified_Visits;
	SinkReachabilities_mmap sink_Reachabilities;

	makespan_bound += leap;

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

		sASSERT(individual_cost_bound >= 0.0);
		sDouble effective_makespan_bound = sMIN(individual_cost_bound + leap + front_respectful_transition.m_waited, makespan_bound);
		
		if (front_respectful_transition.m_time + (map.m_shortest_Distances[sink_loc_id][front_respectful_transition.m_location_id] / kruhobot.m_properties.m_linear_velo) <= effective_makespan_bound + s_EPSILON)
		{
		    RespectfulExplorations_map::iterator respectful_exploration = respectful_Explorations.find(front_respectful_transition.m_conflict_fingerprint);
		    sASSERT(respectful_exploration != respectful_Explorations.end());
				    
		    if (sink_loc_id == front_respectful_transition.m_location_id)			
		    {
			if (front_respectful_transition.m_time >= individual_cost_bound - s_EPSILON)
			{
			    sink_ATTs.insert(front_respectful_transition.m_time);
			}			
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
			    if (transition_distance <= s_EPSILON)
			    {
				continue;
			    }			    
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
				    
				    sREAL_SMT_CBS_UPDATE_NEXT_INDIVIDUAL_COST_BOUND(neighbor_respectful_transition.m_time);
//				    unified_Visits[neighbor_respectful_transition.m_location_id].insert(neighbor_respectful_transition.m_time);

				    if (next_respectful_visit == respectful_exploration->second.end()) /* visiting for the first time in a given fingerprint */
				    {
					sDouble estimated_remaining = map.m_shortest_Distances[sink_loc_id][neighbor_location_id] / kruhobot.m_properties.m_linear_velo;
					sASSERT(individual_cost_bound >= 0.0);					
					sDouble effective_makespan_bound = sMIN(individual_cost_bound + leap + neighbor_respectful_transition.m_waited, makespan_bound);

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
					    sREAL_SMT_CBS_UPDATE_NEXT_INDIVIDUAL_COST_BOUND(neighbor_respectful_transition.m_time + estimated_remaining);
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
					if (bypass_transition_distance <= s_EPSILON)
					{
					    continue;
					}			    					
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
						    sREAL_SMT_CBS_UPDATE_NEXT_INDIVIDUAL_COST_BOUND(bypass_respectful_transition.m_time);
						    
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
				
					sREAL_SMT_CBS_UPDATE_NEXT_INDIVIDUAL_COST_BOUND(wait_respectful_transition.m_time);					
					wait_respectful_transition.m_conflict_fingerprint = next_conflict_fingerprint;

					if (respectful_exploration == respectful_Explorations.end()) /* non-existent fingerprint */
					{
					    sDouble estimated_remaining = map.m_shortest_Distances[sink_loc_id][wait_respectful_transition.m_location_id] / kruhobot.m_properties.m_linear_velo;
					    
					    sASSERT(individual_cost_bound >= 0.0);
					    sDouble effective_makespan_bound = sMIN(individual_cost_bound + leap + wait_respectful_transition.m_waited, makespan_bound);
					    
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
						sREAL_SMT_CBS_UPDATE_NEXT_INDIVIDUAL_COST_BOUND(wait_respectful_transition.m_time + estimated_remaining);
					    }
					}
					else /* existent fingerprint */
					{
					    RespectfulVisits_umap::iterator wait_respectful_visit = respectful_exploration->second.find(front_respectful_transition.m_location_id);
					    
					    if (wait_respectful_visit == respectful_exploration->second.end()) /* visiting for the first time */
					    {
						sDouble estimated_remaining = map.m_shortest_Distances[sink_loc_id][wait_respectful_transition.m_location_id] / kruhobot.m_properties.m_linear_velo;

						sASSERT(individual_cost_bound >= 0.0);
						sDouble effective_makespan_bound = sMIN(individual_cost_bound + leap + wait_respectful_transition.m_waited, makespan_bound);
						
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
						    sREAL_SMT_CBS_UPDATE_NEXT_INDIVIDUAL_COST_BOUND(wait_respectful_transition.m_time + estimated_remaining);
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
	
	return next_individual_cost_bound;
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
					       const LocationConflicts_upper__umap &sUNUSED(location_Conflicts),
					       const UlinearConflicts_upper__map    &linear_Conflicts,
					       const ConflictFingerprint           &conflict_Fingerprint) const
    {
	/*
	LocationConflicts_upper__umap::const_iterator location_Conflict = location_Conflicts.find(location_v_id);
	       
	if (location_Conflict != location_Conflicts.end())
	{
	    for (LocationConflicts_upper_map::const_iterator lower_location_conflict = location_Conflict->second.lower_bound(Interval(finish_time, finish_time));		
		 lower_location_conflict != location_Conflict->second.end(); ++lower_location_conflict)
	    {		
		if (lower_location_conflict->second.m_interval.m_lower < finish_time - s_EPSILON && lower_location_conflict->second.m_interval.m_upper > finish_time + s_EPSILON)
		{

		    #ifdef sDEBUG
		    {
			printf("  Conflicting due to location conflict: %.3f,%.3f\n", start_time, finish_time);
			lower_location_conflict->second.to_Screen("      ");
		    }
		    #endif		    
		    return true;		    
		    //break;
		}
	    }
	}
	*/

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
		//printf("%.3f, %.3f    %.3f,%.3f\n", start_time, finish_time, lower_linear_conflict->second.m_interval.m_lower, lower_linear_conflict->second.m_interval.m_upper);
		if (lower_linear_conflict->second.overlaps(Interval(start_time, finish_time)))
		{
		    if (conflict_Fingerprint.m_conflict_IDs.find(lower_linear_conflict->second.m_conflict_id) != conflict_Fingerprint.m_conflict_IDs.end())
		    {
			#ifdef sDEBUG
			{
			    /*
			    printf("  Conflicting due to linear conflict: %.3f,%.3f\n", start_time, finish_time);
			    lower_linear_conflict->second.to_Screen("      ");
			    */
			}
			#endif
			return true;
		    }
		}
		/*
		if (lower_linear_conflict->second.m_interval.m_lower > finish_time)
		{
		    break;
		}
		*/
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

	/*
	for (sInt_32 dec_id = 0; dec_id < sink_decision_id; ++dec_id)
	{
	    if (kruhobot_RDD[dec_id].m_location_id == sink_loc_id)
	    {
		kruhobot_RDD[dec_id].m_next_dec_IDs.push_back(sink_decision_id);
		kruhobot_RDD[sink_decision_id].m_prev_dec_IDs.insert(dec_id);
	    }
	}
	*/	
    }

    
    void sRealSMTCBS::interconnect_KruhobotRealDecisionDiagram(const sKruhobot                &kruhobot,
							       const s2DMap                   &map,
							       KruhobotDecisionDiagram_vector &kruhobot_RDD,
							       KruhobotDecisionMapping_map    &sUNUSED(kruhobot_RDD_mapping)) const
    {
	for (KruhobotDecisionDiagram_vector::iterator decision = kruhobot_RDD.begin(); decision != kruhobot_RDD.end(); ++decision)
	{
	    //KruhobotDecisionDiagram_vector::iterator wait_decision = kruhobot_RDD.end();	    
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
				decision->m_next_dec_IDs.push_back(next_decision->m_dec_id);
				next_decision->m_prev_dec_IDs.insert(decision->m_dec_id);
			    }

			    /*
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
			    */
			}
		    }
		}
	    }
	    /*
	    if (wait_decision != kruhobot_RDD.end())	    
	    {
		decision->m_next_dec_IDs.push_back(wait_decision->m_dec_id);
		wait_decision->m_prev_dec_IDs.insert(decision->m_dec_id);
	    }
	    */
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

		    if (next_decision.m_dec_id != decision->m_dec_id && decision->m_location_id == next_decision.m_location_id)
		    {
			if (next_decision.m_time > decision->m_time + s_EPSILON)
			{
			    decision->m_next_dec_IDs.push_back(next_decision.m_dec_id);
			    kruhobot_RDD[next_decision.m_dec_id].m_prev_dec_IDs.insert(decision->m_dec_id);
			}

			/*
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
			*/
		    }
		    ++vertex_decision;
		}
	    }
	    /*
	    if (wait_decision_id >= 0)	    
	    {
		decision->m_next_dec_IDs.push_back(wait_decision_id);
		kruhobot_RDD[wait_decision_id].m_prev_dec_IDs.insert(decision->m_dec_id);
	    }
	    */	    
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

} // namespace boOX
