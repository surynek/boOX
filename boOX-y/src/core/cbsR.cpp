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
/* cbsR.cpp / 0-279_zenon                                                     */
/*----------------------------------------------------------------------------*/
//
// Conflict based search for a semi-continuous version of MAPF.
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

#include "core/cbsR.h"
#include "util/statistics.h"


using namespace std;
using namespace boOX;


/*----------------------------------------------------------------------------*/


namespace boOX
{



    
/*----------------------------------------------------------------------------*/
// sRealCBSBase
    
    sRealCBSBase::sRealCBSBase(sRealInstance *real_Instance)
	: m_real_Instance(real_Instance)
	, m_timeout(-1.0)
    {
	// nothing
    }

    
    sRealCBSBase::sRealCBSBase(sRealInstance *real_Instance, sDouble timeout)
	: m_real_Instance(real_Instance)
	, m_timeout(timeout)
    {
	// nothing
    }

    
/*----------------------------------------------------------------------------*/

    sDouble sRealCBSBase::analyze_NonconflictingSchedules(const sRealInstance            &real_Instance,
							  const KruhobotSchedules_vector &kruhobot_Schedules,
							  KruhobotCollisions_mset        &kruhobot_Collisions) const
    {
	sDouble cummulative = 0.0;
	
	LocationCooccupations_umap location_Cooccupations;
	LinearCooccupations_map linear_Cooccupations;
	
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();

	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
	    const Schedule_vector &Schedule = kruhobot_Schedules[kruhobot_id];
	    for (Schedule_vector::const_iterator event = Schedule.begin(); event != Schedule.end(); ++event)
	    {
		if (event->m_from_loc_id == event->m_to_loc_id)
		{
		    location_Cooccupations[event->m_from_loc_id][Interval(event->m_start_time, event->m_finish_time)].insert(-kruhobot_id);
		}
		else
		{
/*
		    const sKruhobot &kruhobot = real_Instance.m_Kruhobots[kruhobot_id];
		    const s2DMap &map = *real_Instance.m_start_conjunction.m_Map;

		    sInt_32 nearest_from_location_id = map.find_NearestLocation(event->m_from_loc_id);
		    sInt_32 nearest_to_location_id = map.find_NearestLocation(event->m_to_loc_id);

		    sDouble from_transition_distance = map.m_Distances[event->m_from_loc_id][nearest_from_location_id];
		    sDouble to_transition_distance = map.m_Distances[event->m_to_loc_id][nearest_to_location_id];			
		    
		    sDouble nearest_from_delta_time = from_transition_distance / kruhobot.m_properties.m_linear_velo;
		    sDouble nearest_to_delta_time = to_transition_distance / kruhobot.m_properties.m_linear_velo;

		    printf("%.3f, %.3f\n", event->m_start_time, event->m_finish_time);
		    printf("%.3f, %.3f\n", event->m_start_time, event->m_start_time + nearest_from_delta_time);
		    printf("%.3f, %.3f\n", event->m_finish_time - nearest_to_delta_time, event->m_finish_time);
		    printf("\n");

		    location_Cooccupations[event->m_from_loc_id][Moment(event->m_start_time, event->m_start_time + nearest_from_delta_time)].insert(kruhobot_id);
		    location_Cooccupations[event->m_to_loc_id][Moment(event->m_finish_time - nearest_to_delta_time, event->m_finish_time)].insert(kruhobot_id);
*/		    
		    location_Cooccupations[event->m_from_loc_id][Interval(event->m_start_time, event->m_finish_time)].insert(kruhobot_id);
		    location_Cooccupations[event->m_to_loc_id][Interval(event->m_start_time, event->m_finish_time)].insert(kruhobot_id);		    		    

		    linear_Cooccupations[Uline(event->m_from_loc_id, event->m_to_loc_id)][Interval(event->m_start_time, event->m_finish_time)].insert(kruhobot_id);
		}
		cummulative = sMAX(cummulative, event->m_finish_time);
	    }
	}

	#ifdef sDEBUG
	{
	    /*
	    printf("Location\n");
	    for (LocationCooccupations_umap::const_iterator location_Coop = location_Cooccupations.begin(); location_Coop != location_Cooccupations.end(); ++location_Coop)
	    {
		printf("  loc: %d\n", location_Coop->first);

		const Cooccupations_map &cooccupations = location_Coop->second;
		for (Cooccupations_map::const_iterator coop = cooccupations.begin(); coop != cooccupations.end(); ++coop)
		{
		    printf("    [%.3f,%.3f]:", coop->first.m_lower, coop->first.m_upper);
		    
		    for (KruhobotIDs_uset::const_iterator kruhobot = coop->second.begin(); kruhobot != coop->second.end(); ++kruhobot)
		    {
			printf(" %d", *kruhobot);
		    }
		    printf("\n");
		}
	    }
	    printf("Linear\n");
	    for (LinearCooccupations_map::const_iterator linear_Coop = linear_Cooccupations.begin(); linear_Coop != linear_Cooccupations.end(); ++linear_Coop)
	    {
		printf("  lin: %d<->%d\n", linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id);

		const Cooccupations_map &cooccupations = linear_Coop->second;
		for (Cooccupations_map::const_iterator coop = cooccupations.begin(); coop != cooccupations.end(); ++coop)
		{
		    printf("    [%.3f,%.3f]:", coop->first.m_lower, coop->first.m_upper);
		    
		    for (KruhobotIDs_uset::const_iterator kruhobot = coop->second.begin(); kruhobot != coop->second.end(); ++kruhobot)
		    {
			printf(" %d", *kruhobot);
		    }
		    printf("\n");
		}		
	    }
	    */
	}
	#endif

	const s2DMap &map = *real_Instance.m_start_conjunction.m_Map;
	
	for (LocationCooccupations_umap::const_iterator location_Coop = location_Cooccupations.begin(); location_Coop != location_Cooccupations.end(); ++location_Coop)
	{
	    const Cooccupations_map &cooccupations = location_Coop->second;

	    for (Cooccupations_map::const_iterator coop = cooccupations.begin(); coop != cooccupations.end(); ++coop)
	    {
		sASSERT(!coop->second.empty());
		for (KruhobotIDs_uset::const_iterator kruhobot = coop->second.begin(); kruhobot != coop->second.end(); ++kruhobot)
		{
		    KruhobotIDs_uset::const_iterator next_kruhobot = kruhobot;
		    for (++next_kruhobot; next_kruhobot != coop->second.end(); ++next_kruhobot)
		    {
			sASSERT(*kruhobot != *next_kruhobot);
			
			Traversal traversal_A(*kruhobot, location_Coop->first, location_Coop->first, Interval(coop->first.m_lower, coop->first.m_upper));
			Traversal traversal_B(*next_kruhobot, location_Coop->first, location_Coop->first, Interval(coop->first.m_lower, coop->first.m_upper));

			sDouble importance = calc_KruhobotCollisionImportance(traversal_A, traversal_B, location_Cooccupations, linear_Cooccupations);
			
			kruhobot_Collisions.insert(KruhobotCollision(importance, traversal_A, traversal_B));
		    }
		
		    Cooccupations_map::const_iterator next_coop = coop;
		    for (++next_coop; next_coop != cooccupations.end(); ++next_coop)
		    {
			if (coop->first.overlaps(next_coop->first))
			{
			    for (KruhobotIDs_uset::const_iterator next_kruhobot = next_coop->second.begin(); next_kruhobot != next_coop->second.end(); ++next_kruhobot)
			    {
				Interval intersection = coop->first.intersect(next_coop->first);
				
				sASSERT(!intersection.empty());
				if (*kruhobot == *next_kruhobot)
				{
				    printf("%d x %d\n", *kruhobot, *next_kruhobot);
				    coop->first.to_Screen();
				    next_coop->first.to_Screen();
				    intersection.to_Screen();
				    sASSERT(false);
				}
			
				Traversal traversal_A(*kruhobot, location_Coop->first, location_Coop->first, Interval(intersection.m_lower, intersection.m_upper));
				Traversal traversal_B(*next_kruhobot, location_Coop->first, location_Coop->first, Interval(intersection.m_lower, intersection.m_upper));
				sDouble importance = calc_KruhobotCollisionImportance(traversal_A, traversal_B, location_Cooccupations, linear_Cooccupations);				
				kruhobot_Collisions.insert(KruhobotCollision(importance, traversal_A, traversal_B));
			    }
			}
			else
			{
			    break;
			}
		    }
		}
	    }
	}

	for (LinearCooccupations_map::const_iterator linear_Coop = linear_Cooccupations.begin(); linear_Coop != linear_Cooccupations.end(); ++linear_Coop)
	{	    
	    const Cooccupations_map &cooccupations = linear_Coop->second;
	    
	    for (Cooccupations_map::const_iterator coop = cooccupations.begin(); coop != cooccupations.end(); ++coop)
	    {
		sASSERT(!coop->second.empty());
		for (KruhobotIDs_uset::const_iterator kruhobot = coop->second.begin(); kruhobot != coop->second.end(); ++kruhobot)
		{
		    KruhobotIDs_uset::const_iterator next_kruhobot = kruhobot;
		    for (++next_kruhobot; next_kruhobot != coop->second.end(); ++next_kruhobot)
		    {
			sASSERT(*kruhobot != *next_kruhobot);
			
			Traversal traversal_A(*kruhobot, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, Interval(coop->first.m_lower, coop->first.m_upper));
			Traversal traversal_B(*next_kruhobot, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, Interval(coop->first.m_lower, coop->first.m_upper));
		       
			sDouble importance = calc_KruhobotCollisionImportance(traversal_A, traversal_B, location_Cooccupations, linear_Cooccupations);
			kruhobot_Collisions.insert(KruhobotCollision(importance, traversal_A, traversal_B));
		    }
		
		    Cooccupations_map::const_iterator next_coop = coop;
		    for (++next_coop; next_coop != cooccupations.end(); ++next_coop)
		    {
			if (coop->first.overlaps(next_coop->first))
			{
			    Interval intersection = coop->first.intersect(next_coop->first);
			    sASSERT(!intersection.empty());			    
							
			    for (KruhobotIDs_uset::const_iterator next_kruhobot = next_coop->second.begin(); next_kruhobot != next_coop->second.end(); ++next_kruhobot)
			    {
				sASSERT(*kruhobot != *next_kruhobot);
			
				Traversal traversal_A(*kruhobot, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, Interval(intersection.m_lower, intersection.m_upper));
				Traversal traversal_B(*next_kruhobot, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, Interval(intersection.m_lower, intersection.m_upper));

				sDouble importance = calc_KruhobotCollisionImportance(traversal_A, traversal_B, location_Cooccupations, linear_Cooccupations);
				kruhobot_Collisions.insert(KruhobotCollision(importance, traversal_A, traversal_B));
			    }
			}
			else
			{
			    break;
			}
		    }
		}

		LocationCooccupations_umap::const_iterator other_location_cooccupations_low = location_Cooccupations.find(linear_Coop->first.m_lower_id);
		if (other_location_cooccupations_low != location_Cooccupations.end())
		{
		    for (Cooccupations_map::const_iterator other_coop = other_location_cooccupations_low->second.begin(); other_coop != other_location_cooccupations_low->second.end(); ++other_coop)
		    {
			if (coop->first.overlaps(other_coop->first))
			{
			    Interval intersection = coop->first.intersect(other_coop->first);
			    sASSERT(!intersection.empty());			    
			    
			    for (KruhobotIDs_uset::const_iterator kruhobot = coop->second.begin(); kruhobot != coop->second.end(); ++kruhobot)
			    {
				for (KruhobotIDs_uset::const_iterator other_kruhobot = other_coop->second.begin(); other_kruhobot != other_coop->second.end(); ++other_kruhobot)
				{
				    if (*kruhobot != *other_kruhobot)
				    {
					Traversal traversal_A(*kruhobot, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, Interval(intersection.m_lower, intersection.m_upper));
					Traversal traversal_B(*other_kruhobot, linear_Coop->first.m_lower_id, linear_Coop->first.m_lower_id, Interval(intersection.m_lower, intersection.m_upper));

					sDouble importance = calc_KruhobotCollisionImportance(traversal_A, traversal_B, location_Cooccupations, linear_Cooccupations);
					kruhobot_Collisions.insert(KruhobotCollision(importance, traversal_A, traversal_B));
				    }
				}				
			    }
			}
		    }		    
		}
		
		LocationCooccupations_umap::const_iterator other_location_cooccupations_up = location_Cooccupations.find(linear_Coop->first.m_upper_id);
		if (other_location_cooccupations_up != location_Cooccupations.end())
		{
		    for (Cooccupations_map::const_iterator other_coop = other_location_cooccupations_up->second.begin(); other_coop != other_location_cooccupations_up->second.end(); ++other_coop)
		    {
			if (coop->first.overlaps(other_coop->first))
			{
			    Interval intersection = coop->first.intersect(other_coop->first);
			    sASSERT(!intersection.empty());
						    
			    for (KruhobotIDs_uset::const_iterator kruhobot = coop->second.begin(); kruhobot != coop->second.end(); ++kruhobot)
			    {
				for (KruhobotIDs_uset::const_iterator other_kruhobot = other_coop->second.begin(); other_kruhobot != other_coop->second.end(); ++other_kruhobot)
				{
				    if (*kruhobot != *other_kruhobot)
				    {
					Traversal traversal_A(*kruhobot, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, Interval(intersection.m_lower, intersection.m_upper));
					Traversal traversal_B(*other_kruhobot, linear_Coop->first.m_upper_id, linear_Coop->first.m_upper_id, Interval(intersection.m_lower, intersection.m_upper));

					sDouble importance = calc_KruhobotCollisionImportance(traversal_A, traversal_B, location_Cooccupations, linear_Cooccupations);
					kruhobot_Collisions.insert(KruhobotCollision(importance, traversal_A, traversal_B));
				    }
				}				
			    }
			}
		    }		    
		}

		// Lines vs. locations		
		for (LocationCooccupations_umap::const_iterator other_location_cooccupations = location_Cooccupations.begin(); other_location_cooccupations != location_Cooccupations.end(); ++other_location_cooccupations)
		{
		    for (Cooccupations_map::const_iterator other_coop = other_location_cooccupations->second.begin(); other_coop != other_location_cooccupations->second.end(); ++other_coop)
		    {
			sDouble point_distance = map.calc_PointDistance(linear_Coop->first.m_lower_id,
									linear_Coop->first.m_upper_id,
									other_location_cooccupations->first);
			
			if (coop->first.overlaps(other_coop->first))
			{
			    Interval intersection = coop->first.intersect(other_coop->first);			
			    sASSERT(!intersection.empty());

			    for (KruhobotIDs_uset::const_iterator kruhobot = coop->second.begin(); kruhobot != coop->second.end(); ++kruhobot)
			    {
				for (KruhobotIDs_uset::const_iterator other_kruhobot = other_coop->second.begin(); other_kruhobot != other_coop->second.end(); ++other_kruhobot)
				{
				    if (point_distance < real_Instance.m_Kruhobots[sABS(*kruhobot)].m_properties.m_radius + real_Instance.m_Kruhobots[sABS(*other_kruhobot)].m_properties.m_radius)
				    {				    
					if (sABS(*kruhobot) != sABS(*other_kruhobot))
					{
					    Traversal traversal_A(*kruhobot, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, Interval(intersection.m_lower, intersection.m_upper));
					    Traversal traversal_B(*other_kruhobot, other_location_cooccupations->first, other_location_cooccupations->first, Interval(intersection.m_lower, intersection.m_upper));

					    sDouble importance = calc_KruhobotCollisionImportance(traversal_A, traversal_B, location_Cooccupations, linear_Cooccupations);
					    kruhobot_Collisions.insert(KruhobotCollision(importance, traversal_A, traversal_B));
					}
				    }
				}
			    }			    
			}			
		    }
		}

		// Lines vs. lines
		LinearCooccupations_map::const_iterator other_linear_Coop = linear_Coop;		
		for (++other_linear_Coop; other_linear_Coop != linear_Cooccupations.end(); ++other_linear_Coop)
		{
		    sDouble line_distance = map.calc_LineDistance(linear_Coop->first.m_lower_id,
								  linear_Coop->first.m_upper_id,
								  other_linear_Coop->first.m_lower_id,
								  other_linear_Coop->first.m_upper_id);
		    const Cooccupations_map &other_cooccupations = other_linear_Coop->second;
		    // TODO: from lower bound
		    
		    for (Cooccupations_map::const_iterator other_coop = other_cooccupations.begin(); other_coop != other_cooccupations.end(); ++other_coop)
		    {
			if (coop->first.overlaps(other_coop->first))
			{
			    Interval intersection = coop->first.intersect(other_coop->first);
			    sASSERT(!intersection.empty());			    
			    
			    for (KruhobotIDs_uset::const_iterator kruhobot = coop->second.begin(); kruhobot != coop->second.end(); ++kruhobot)
			    {				    
				for (KruhobotIDs_uset::const_iterator other_kruhobot = other_coop->second.begin(); other_kruhobot != other_coop->second.end(); ++other_kruhobot)
				{
				    if (line_distance < real_Instance.m_Kruhobots[sABS(*kruhobot)].m_properties.m_radius + real_Instance.m_Kruhobots[sABS(*other_kruhobot)].m_properties.m_radius)
				    {
					if (sABS(*kruhobot) != sABS(*other_kruhobot))
					{					
					    Traversal traversal_A(*kruhobot, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, Interval(intersection.m_lower, intersection.m_upper));
					    Traversal traversal_B(*other_kruhobot, other_linear_Coop->first.m_lower_id, other_linear_Coop->first.m_upper_id, Interval(intersection.m_lower, intersection.m_upper));
					    
					    sDouble importance = calc_KruhobotCollisionImportance(traversal_A, traversal_B, location_Cooccupations, linear_Cooccupations);
					    kruhobot_Collisions.insert(KruhobotCollision(importance, traversal_A, traversal_B));
					}
				    }
				}
			    }
			}
		    }
		}
	    }
	}
	return cummulative;
    }


    void sRealCBSBase::introduce_KruhobotConflict(const Traversal                  &kruhobot_traversal,
						  KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
						  KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts,
						  bool                              infinity) const
    {
	sInt_32 kruhobot_id = sABS(kruhobot_traversal.m_kruhobot_id);

	if (kruhobot_traversal.m_u_loc_id != kruhobot_traversal.m_v_loc_id)
	{
	    LinearConflict linear_conflict(kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_v_loc_id, kruhobot_traversal.m_interval);
	    kruhobot_linear_Conflicts[kruhobot_id][Uline(kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_v_loc_id)].insert(LinearConflicts_map::value_type(kruhobot_traversal.m_interval, linear_conflict));
	}
	else
	{
	    LocationConflict location_conflict(kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_interval, infinity);
	    kruhobot_location_Conflicts[kruhobot_id][kruhobot_traversal.m_u_loc_id].insert(LocationConflicts_map::value_type(kruhobot_traversal.m_interval, location_conflict));
	}
    }

    
    void sRealCBSBase::introduce_KruhobotConflict(const Traversal                                &kruhobot_traversal,
						  KruhobotLocationConflicts_lexicographic_vector &kruhobot_location_Conflicts,
						  KruhobotLinearConflicts_lexicographic_vector   &kruhobot_linear_Conflicts,
						  bool                                            infinity) const
    {
	sInt_32 kruhobot_id = sABS(kruhobot_traversal.m_kruhobot_id);

	if (kruhobot_traversal.m_u_loc_id != kruhobot_traversal.m_v_loc_id)
	{
	    LinearConflict linear_conflict(kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_v_loc_id, kruhobot_traversal.m_interval);
	    kruhobot_linear_Conflicts[kruhobot_id][Uline(kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_v_loc_id)].insert(LinearConflicts_lexicographic_map::value_type(kruhobot_traversal.m_interval, linear_conflict));
	}
	else
	{
	    LocationConflict location_conflict(kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_interval, infinity);
	    kruhobot_location_Conflicts[kruhobot_id][kruhobot_traversal.m_u_loc_id].insert(LocationConflicts_lexicographic_map::value_type(kruhobot_traversal.m_interval, location_conflict));
	}	
    }

    
    void sRealCBSBase::introduce_KruhobotConflict(const Traversal                        &kruhobot_traversal,
						  KruhobotLocationConflicts_lower_vector &kruhobot_location_Conflicts,
						  KruhobotLinearConflicts_lower_vector   &kruhobot_linear_Conflicts,
						  bool                                    infinity) const
    {
	sInt_32 kruhobot_id = sABS(kruhobot_traversal.m_kruhobot_id);

	if (kruhobot_traversal.m_u_loc_id != kruhobot_traversal.m_v_loc_id)
	{
	    LinearConflict linear_conflict(kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_v_loc_id, kruhobot_traversal.m_interval);
	    kruhobot_linear_Conflicts[kruhobot_id][Uline(kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_v_loc_id)].insert(LinearConflicts_lower_map::value_type(kruhobot_traversal.m_interval, linear_conflict));
	}
	else
	{
	    LocationConflict location_conflict(kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_interval, infinity);
	    kruhobot_location_Conflicts[kruhobot_id][kruhobot_traversal.m_u_loc_id].insert(LocationConflicts_lower_map::value_type(kruhobot_traversal.m_interval, location_conflict));
	}	
    }

    
    void sRealCBSBase::introduce_KruhobotConflict(const Traversal                        &kruhobot_traversal,
						  KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
						  KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
						  bool                                    infinity) const
    {
	sInt_32 kruhobot_id = sABS(kruhobot_traversal.m_kruhobot_id);

	if (kruhobot_traversal.m_u_loc_id != kruhobot_traversal.m_v_loc_id)
	{
	    LinearConflict linear_conflict(kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_v_loc_id, kruhobot_traversal.m_interval);
	    kruhobot_linear_Conflicts[kruhobot_id][Uline(kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_v_loc_id)].insert(LinearConflicts_upper_map::value_type(kruhobot_traversal.m_interval, linear_conflict));
	}
	else
	{
	    LocationConflict location_conflict(kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_interval, infinity);
	    kruhobot_location_Conflicts[kruhobot_id][kruhobot_traversal.m_u_loc_id].insert(LocationConflicts_upper_map::value_type(kruhobot_traversal.m_interval, location_conflict));
	}	
    }

    
    sDouble sRealCBSBase::calc_KruhobotCollisionImportance(const Traversal            &traversal_A,
							   const Traversal            &traversal_B,
							   LocationCooccupations_umap &location_Cooccupations,
							   LinearCooccupations_map    &linear_Cooccupations) const
    {
	sDouble collision_importance =   calc_KruhobotCollisionImportance(traversal_A, location_Cooccupations, linear_Cooccupations)
	                               + calc_KruhobotCollisionImportance(traversal_B, location_Cooccupations, linear_Cooccupations);
	
	return collision_importance;
    }


    sDouble sRealCBSBase::calc_KruhobotCollisionImportance(const Traversal            &traversal,
							   LocationCooccupations_umap &location_Cooccupations,
							   LinearCooccupations_map    &linear_Cooccupations) const
    {
	sDouble traversal_importance = 0.0;
	
	LocationCooccupations_umap::const_iterator location_Coop = location_Cooccupations.find(traversal.m_u_loc_id);
	
	if (location_Coop != location_Cooccupations.end())
	{
	    const Cooccupations_map &cooccupations = location_Coop->second;
	    for (Cooccupations_map::const_iterator coop = cooccupations.begin(); coop != cooccupations.end(); ++coop)
	    {
		if (coop->first.overlaps(Interval(traversal.m_interval.m_lower, traversal.m_interval.m_upper)))
		{
		    traversal_importance += (traversal.m_interval.m_upper - traversal.m_interval.m_lower) * coop->second.size();
		}
	    }
	}

	location_Coop = location_Cooccupations.find(traversal.m_v_loc_id);
	
	if (location_Coop != location_Cooccupations.end())
	{
	    const Cooccupations_map &cooccupations = location_Coop->second;
	    for (Cooccupations_map::const_iterator coop = cooccupations.begin(); coop != cooccupations.end(); ++coop)
	    {
		if (coop->first.overlaps(Interval(traversal.m_interval.m_lower, traversal.m_interval.m_upper)))
		{
		    traversal_importance += (traversal.m_interval.m_upper - traversal.m_interval.m_lower) * coop->second.size();
		}
	    }
	}

	LinearCooccupations_map::const_iterator linear_Coop = linear_Cooccupations.find(Uline(traversal.m_u_loc_id, traversal.m_v_loc_id));
	
	if (linear_Coop != linear_Cooccupations.end())
	{
	    const Cooccupations_map &cooccupations = linear_Coop->second;
	    for (Cooccupations_map::const_iterator coop = cooccupations.begin(); coop != cooccupations.end(); ++coop)
	    {
		if (coop->first.overlaps(Interval(traversal.m_interval.m_lower, traversal.m_interval.m_upper)))
		{
		    traversal_importance += (traversal.m_interval.m_upper - traversal.m_interval.m_lower) * coop->second.size();
		}
	    }
	}

	return traversal_importance;
    }


/*----------------------------------------------------------------------------*/
    
    void sRealCBSBase::smooth_Schedule(const Schedule_vector &Schedule, Schedule_vector &smooth_Schedule)
    {
	sDouble wait_start = -1.0, wait_finish = -1.0;
	sInt_32 wait_loc_id = -1;

	for (Schedule_vector::const_iterator event = Schedule.begin(); event != Schedule.end(); ++event)
	{
	    if (event->m_from_loc_id == event->m_to_loc_id)
	    {
		if (wait_start < 0.0)
		{
		    wait_start = event->m_start_time;
		    wait_loc_id = event->m_from_loc_id;
		}
		wait_finish = event->m_finish_time;
	    }
	    else
	    {
		if (wait_loc_id >= 0)
		{
		    smooth_Schedule.push_back(Event(wait_loc_id, wait_loc_id, wait_start, wait_finish));

		    wait_loc_id = -1;
		    wait_start = wait_finish = -1.0;		    
		}
		smooth_Schedule.push_back(*event);
	    }
	}
	if (wait_loc_id >= 0)
	{
	    smooth_Schedule.push_back(Event(wait_loc_id, wait_loc_id, wait_start, wait_finish));
	}
    }


    sDouble sRealCBSBase::augment_Schedules(const sRealInstance &real_Instance, KruhobotSchedules_vector &kruhobot_Schedules)
    {	    
	sDouble final_time = 0.0;
	sInt_32 N_kruhobots_1 = kruhobot_Schedules.size();

	for (sInt_32 kruhobot_id = 1; kruhobot_id < N_kruhobots_1; ++kruhobot_id)
	{
	    if (!kruhobot_Schedules[kruhobot_id].empty())
	    {
		while (!kruhobot_Schedules[kruhobot_id].empty())
		{
		    Schedule_vector::const_reverse_iterator last_event = kruhobot_Schedules[kruhobot_id].rbegin();
				
		    if (last_event->m_from_loc_id == last_event->m_to_loc_id)
		    {
			kruhobot_Schedules[kruhobot_id].pop_back();
		    }
		    else
		    {
			break;
		    }
		}
	    }
	}	

	for (sInt_32 kruhobot_id = 1; kruhobot_id < N_kruhobots_1; ++kruhobot_id)
	{
	    if (!kruhobot_Schedules[kruhobot_id].empty())
	    {
		Schedule_vector::const_reverse_iterator last_event = kruhobot_Schedules[kruhobot_id].rbegin();

		while (last_event != kruhobot_Schedules[kruhobot_id].rend())
		{
		    if (last_event->m_from_loc_id != last_event->m_to_loc_id)
		    {
			final_time = sMAX(final_time, last_event->m_finish_time);
			break;
		    }
		    ++last_event;
		}
	    }
	}
	
	for (sInt_32 kruhobot_id = 1; kruhobot_id < N_kruhobots_1; ++kruhobot_id)
	{
	    if (!kruhobot_Schedules[kruhobot_id].empty())
	    {
		Schedule_vector::const_reverse_iterator last_event = kruhobot_Schedules[kruhobot_id].rbegin();

		if (last_event->m_finish_time < final_time)
		{
		    kruhobot_Schedules[kruhobot_id].push_back(Event(last_event->m_to_loc_id, last_event->m_to_loc_id, last_event->m_finish_time, final_time));
		}
	    }
	    else
	    {
		sInt_32 goal_loc_id = real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id];
		kruhobot_Schedules[kruhobot_id].push_back(Event(goal_loc_id, goal_loc_id, 0.0, final_time));		
	    }
	}
	for (sInt_32 kruhobot_id = 1; kruhobot_id < N_kruhobots_1; ++kruhobot_id)
	{
	    Schedule_vector smooth_kruhobot_Schedule;
	    smooth_Schedule(kruhobot_Schedules[kruhobot_id], smooth_kruhobot_Schedule);
	    kruhobot_Schedules[kruhobot_id] = smooth_kruhobot_Schedule;
	}
	
	return final_time;
    }

    
/*----------------------------------------------------------------------------*/

    void sRealCBSBase::to_Screen(const KruhobotSchedules_vector &kruhobot_Schedules, const sString &indent)
    {
	to_Stream(stdout, kruhobot_Schedules, indent);
    }


    void sRealCBSBase::to_Stream(FILE *fw, const KruhobotSchedules_vector &kruhobot_Schedules, const sString &indent)
    {
	sInt_32 N_kruhobots_1 = kruhobot_Schedules.size();
	fprintf(fw, "%sKruhobot schedules [\n", indent.c_str());	
	
	for (sInt_32 kruhobot_id = 1; kruhobot_id < N_kruhobots_1; ++kruhobot_id)
	{
	    fprintf(fw, "%s%sschedule [%d]: {\n", indent.c_str(), s_INDENT.c_str(), kruhobot_id);
	    
	    const Schedule_vector Schedule = kruhobot_Schedules[kruhobot_id];
	    for (Schedule_vector::const_iterator event = Schedule.begin(); event != Schedule.end(); ++event)
	    {
		fprintf(fw, "%s%s%d --> %d [%.3f, %.3f]\n", indent.c_str(), s2_INDENT.c_str(), event->m_from_loc_id, event->m_to_loc_id, event->m_start_time, event->m_finish_time);
	    }
	    fprintf(fw, "%s%s}\n", indent.c_str(), s_INDENT.c_str());
	}
	fprintf(fw, "%s]\n", indent.c_str());
    }
    


    
/*----------------------------------------------------------------------------*/
// sRealCBS

    sRealCBS::sRealCBS(sRealInstance *real_Instance)
	: sRealCBSBase(real_Instance)
    {
	// nothing
    }


    sRealCBS::sRealCBS(sRealInstance *real_Instance, sDouble timeout)
	: sRealCBSBase(real_Instance, timeout)
    {
	// nothing
    }    

    
/*----------------------------------------------------------------------------*/

    sDouble sRealCBS::find_ShortestNonconflictingSchedules(sRealSolution &real_Solution, sDouble cost_limit)
    {
	return find_ShortestNonconflictingSchedules(*m_real_Instance, real_Solution, cost_limit);	
    }

  
    sDouble sRealCBS::find_ShortestNonconflictingSchedules(const sRealInstance &real_Instance,
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


    sDouble sRealCBS::find_ShortestNonconflictingSchedules(KruhobotSchedules_vector &kruhobot_Schedules, sDouble cost_limit)
    {
	return find_ShortestNonconflictingSchedules(*m_real_Instance, kruhobot_Schedules, cost_limit);	
    }


    sDouble sRealCBS::find_ShortestNonconflictingSchedules(const sRealInstance      &sUNUSED(real_Instance),
							   KruhobotSchedules_vector &sUNUSED(kruhobot_Schedules),
							   sDouble                   sUNUSED(cost_limit))
    {
	return 0.0;
    }    

    
    sDouble sRealCBS::find_ShortestNonconflictingSchedules(KruhobotSchedules_vector &kruhobot_Schedules, sDouble cost_limit, sDouble extra_cost)
    {
	return find_ShortestNonconflictingSchedules(*m_real_Instance, kruhobot_Schedules, cost_limit, extra_cost);	
    }

    
    sDouble sRealCBS::find_ShortestNonconflictingSchedules(const sRealInstance      &real_Instance,
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

	    KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	    KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;

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

    sDouble sRealCBS::find_NonconflictingSchedules(const sRealInstance              &real_Instance,
						   KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
						   KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts,
						   KruhobotSchedules_vector         &kruhobot_Schedules,
						   sDouble                           cost_limit,
						   sDouble                           extra_cost)
    {
	sDouble cummulative;
	sInt_32 N_kruhobots = real_Instance.m_start_conjunction.get_KruhobotCount();

	sDouble start_time = sStatistics::get_CPU_Seconds();
	
	Node initial_node(0);
	initial_node.m_upper_node_id = -1;

	initial_node.m_update_kruhobot_id = -1;

	initial_node.m_left_node_id = -1;
	initial_node.m_right_node_id = -1;
	
	initial_node.m_kruhobot_location_Conflicts = kruhobot_location_Conflicts;
	initial_node.m_kruhobot_linear_Conflicts = kruhobot_linear_Conflicts;
	initial_node.m_kruhobot_Schedules.resize(N_kruhobots + 1);

	for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	{
	    sDouble kruhobot_cost;
	    if ((kruhobot_cost = find_KruhobotNonconflictingSchedule(real_Instance.m_Kruhobots[kruhobot_id],
								     *real_Instance.m_start_conjunction.m_Map,
								     real_Instance.m_start_conjunction.m_kruhobot_Locations[kruhobot_id],
								     real_Instance.m_goal_conjunction.m_kruhobot_Locations[kruhobot_id],
								     cost_limit,
								     extra_cost,					    
								     initial_node.m_kruhobot_location_Conflicts[kruhobot_id],
								     initial_node.m_kruhobot_linear_Conflicts[kruhobot_id],
								     -1.0,
								     initial_node.m_kruhobot_Schedules[kruhobot_id])) < 0.0)
	    {
		return -1.0;
	    }
	}
	
	augment_Schedules(real_Instance, initial_node.m_kruhobot_Schedules);
	
	initial_node.m_makespan = calc_ScheduleMakespan(real_Instance, initial_node.m_kruhobot_Schedules);
	initial_node.m_cost = calc_ScheduleCost(real_Instance, initial_node.m_kruhobot_Schedules);
	
	initial_node.m_conflicting = 0.0;

	#ifdef sDEBUG
	{
	    for (sInt_32 kruhobot_id = 1; kruhobot_id <= N_kruhobots; ++kruhobot_id)
	    {
		sASSERT(check_KruhobotNonconflictingSchedule(initial_node.m_kruhobot_Schedules[kruhobot_id],
							     initial_node.m_kruhobot_location_Conflicts[kruhobot_id],
							     initial_node.m_kruhobot_linear_Conflicts[kruhobot_id]));
	    }
	}
	#endif	
	Nodes_vector search_Store;
//	NodeReferences_vector search_Queue;
	NodeReferences_mset search_Queue;	
	
	search_Store.push_back(initial_node);
	search_Queue.insert(NodeReference(0, &search_Store));
//	search_Queue.push_back(NodeReference(0, &search_Store));

	NodeReference best_node = *search_Queue.begin();
	search_Queue.erase(search_Queue.begin());	

	if ((cummulative = examine_NonconflictingSchedules(real_Instance,
							   best_node.m_node_id,
							   search_Store[best_node.m_node_id].m_kruhobot_location_Conflicts,
							   search_Store[best_node.m_node_id].m_kruhobot_linear_Conflicts,
							   search_Store[best_node.m_node_id].m_kruhobot_Schedules,
							   cost_limit,
							   extra_cost,
							   search_Store,
							   search_Queue)) >= 0.0)
	{
	    kruhobot_Schedules = search_Store[best_node.m_node_id].m_kruhobot_Schedules;
	    return search_Store[best_node.m_node_id].m_makespan;	    
	}	

	while (!search_Queue.empty())
	{	    
	    if (m_timeout >= 0)
	    {
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > m_timeout)
		{
		    return -2.0;
		}
	    }
	       
            #ifdef sSTATISTICS
	    {
		++s_GlobalStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif

	    NodeReference best_node = *search_Queue.begin();
	    search_Queue.erase(search_Queue.begin());

            #ifdef sDEBUG
	    {
//		printf("Search queue size:%ld [%d] (%ld) - %.3f\n", search_Queue.size(), best_node.m_node_id, search_Store.size(), search_Store[best_node.m_node_id].m_cost);
/*
		printf("Search queue kruhobots:\n");
		for (NodeReferences_mset::const_iterator node = search_Queue.begin(); node != search_Queue.end(); ++node)
		{
		    printf("%d [%.3f,%.3f] ", search_Store[node->m_node_id].m_update_kruhobot_id, search_Store[node->m_node_id].m_makespan, search_Store[node->m_node_id].m_cost);
		}
		printf("\n");
		*/
//		search_Store[best_node.m_node_id].to_Screen();
	    }
	    #endif

	    #ifdef sVERBOSE	    
	    {
		static sDouble verbose_period = 1.0;
		
		sDouble end_time = sStatistics::get_CPU_Seconds();
		if (end_time - start_time > verbose_period)
		{
		    printf("Search steps: %lld (time: %.3f s, make %.3f, cost %.3f)\n",
			   s_GlobalStatistics.get_CurrentPhase().m_search_Steps,
			   end_time - start_time,
			   search_Store[best_node.m_node_id].m_makespan,
			   search_Store[best_node.m_node_id].m_cost);
		    verbose_period *= 1.5;
		}
	    }
	    #endif
	    sInt_32 update_kruhobot_id = search_Store[best_node.m_node_id].m_update_kruhobot_id;
	    sASSERT(update_kruhobot_id >= -1.0);
	    
	    if(update_kruhobot_id > 0)
	    {
		sDouble makespan_bound = search_Store[best_node.m_node_id].m_makespan;
/*
		printf("Before\n");
		search_Store[best_node.m_node_id].to_Screen();		
*/		
		if (update_NonconflictingSchedule(update_kruhobot_id,
						  real_Instance,
						  search_Store[best_node.m_node_id].m_kruhobot_location_Conflicts,
						  search_Store[best_node.m_node_id].m_kruhobot_linear_Conflicts,
						  search_Store[best_node.m_node_id].m_kruhobot_Schedules,
						  makespan_bound,
						  cost_limit,
						  extra_cost) < 0.0)
		{
		    continue;
		}
		augment_Schedules(real_Instance, search_Store[best_node.m_node_id].m_kruhobot_Schedules);
/*
		printf("After\n");		
		search_Store[best_node.m_node_id].to_Screen();		
*/		
		search_Store[best_node.m_node_id].m_makespan = calc_ScheduleMakespan(real_Instance, search_Store[best_node.m_node_id].m_kruhobot_Schedules);
		search_Store[best_node.m_node_id].m_cost = calc_ScheduleCost(real_Instance, search_Store[best_node.m_node_id].m_kruhobot_Schedules);
		
		sASSERT(check_KruhobotNonconflictingSchedule(search_Store[best_node.m_node_id].m_kruhobot_Schedules[update_kruhobot_id],
							     search_Store[best_node.m_node_id].m_kruhobot_location_Conflicts[update_kruhobot_id],
							     search_Store[best_node.m_node_id].m_kruhobot_linear_Conflicts[update_kruhobot_id]));
	    }
	    else
	    {
                #ifdef sVERBOSE	    
		{
		    sDouble end_time = sStatistics::get_CPU_Seconds();
		    printf("Final search steps: %lld (time: %.3f s)\n", s_GlobalStatistics.get_CurrentPhase().m_search_Steps, end_time - start_time);
		}
		#endif
		kruhobot_Schedules = search_Store[best_node.m_node_id].m_kruhobot_Schedules;
		return search_Store[best_node.m_node_id].m_makespan;
	    }

	    if ((cummulative = examine_NonconflictingSchedules(real_Instance,
							       best_node.m_node_id,
							       search_Store[best_node.m_node_id].m_kruhobot_location_Conflicts,
							       search_Store[best_node.m_node_id].m_kruhobot_linear_Conflicts,
							       search_Store[best_node.m_node_id].m_kruhobot_Schedules,
							       cost_limit,
							       extra_cost,
							       search_Store,
							       search_Queue)) >= 0.0)
	    {
		Node recheck_node = search_Store[best_node.m_node_id];
		recheck_node.m_node_id = search_Store.size();
		recheck_node.m_update_kruhobot_id = -1;
		
		search_Store.push_back(recheck_node);
		search_Queue.insert(NodeReference(recheck_node.m_node_id, &search_Store));		
	    }
	}

	return -1.0;
    }


    sDouble sRealCBS::update_NonconflictingSchedule(sInt_32                           upd_kruhobot_id,
						    const sRealInstance              &real_Instance,
						    KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
						    KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts,
						    KruhobotSchedules_vector         &kruhobot_Schedules,
						    sDouble                           makespan_bound,
						    sDouble                           cost_limit,
						    sDouble                           extra_cost) const
    {
	sDouble cummulative;

	cummulative = find_KruhobotNonconflictingSchedule(real_Instance.m_Kruhobots[upd_kruhobot_id],
							  *real_Instance.m_start_conjunction.m_Map,
							  real_Instance.m_start_conjunction.m_kruhobot_Locations[upd_kruhobot_id],
							  real_Instance.m_goal_conjunction.m_kruhobot_Locations[upd_kruhobot_id],
							  cost_limit,
							  extra_cost,
							  kruhobot_location_Conflicts[upd_kruhobot_id],
							  kruhobot_linear_Conflicts[upd_kruhobot_id],
							  makespan_bound,
							  kruhobot_Schedules[upd_kruhobot_id]);
/*
	generate_KruhobotImportantPoints(real_Instance.m_Kruhobots[upd_kruhobot_id],
					 *real_Instance.m_start_conjunction.m_Map,
					 real_Instance.m_start_conjunction.m_kruhobot_Locations[upd_kruhobot_id],
					 real_Instance.m_goal_conjunction.m_kruhobot_Locations[upd_kruhobot_id],
					 cost_limit,
					 extra_cost,
					 kruhobot_location_Conflicts[upd_kruhobot_id],
					 kruhobot_linear_Conflicts[upd_kruhobot_id],
					 makespan_bound);
*/
	return cummulative;
    }  


    sDouble sRealCBS::examine_NonconflictingSchedules(const sRealInstance              &real_Instance,
						      sInt_32                           upper_node_id,
						      KruhobotLocationConflicts_vector  kruhobot_location_Conflicts,
						      KruhobotLinearConflicts_vector    kruhobot_linear_Conflicts,
						      KruhobotSchedules_vector          kruhobot_Schedules,
						      sDouble                           sUNUSED(cost_limit),
						      sDouble                           sUNUSED(extra_cost),
						      Nodes_vector                     &search_Store,
						      NodeReferences_mset              &search_Queue) const
    {
	sDouble cummulative;
	KruhobotCollisions_mset kruhobot_Collisions;

	cummulative = analyze_NonconflictingSchedules(real_Instance,
						      kruhobot_Schedules,
						      kruhobot_Collisions);

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

	if (!kruhobot_Collisions.empty())
	{
	    sASSERT(upper_node_id >= 0);
	    
	    KruhobotCollision principal_collision = *kruhobot_Collisions.begin();
	    #ifdef sDEBUG
	    {
		/*
		to_Screen(kruhobot_Schedules);
		principal_collision.to_Screen();	    
		*/
	    }
	    #endif
	    	    
	    Node next_left_node(search_Store.size());
	    next_left_node.m_makespan = search_Store[upper_node_id].m_makespan;
	    next_left_node.m_cost = search_Store[upper_node_id].m_cost;
	    next_left_node.m_conflicting = kruhobot_Collisions.size();
	    
	    next_left_node.m_upper_node_id = upper_node_id;

	    next_left_node.m_kruhobot_location_Conflicts = kruhobot_location_Conflicts;
	    next_left_node.m_kruhobot_linear_Conflicts = kruhobot_linear_Conflicts;
	    next_left_node.m_kruhobot_Schedules = kruhobot_Schedules;

	    next_left_node.m_left_node_id = next_left_node.m_right_node_id = -1;	    

	    next_left_node.m_update_kruhobot_id = sABS(principal_collision.m_traversal_A.m_kruhobot_id);
	    search_Store[upper_node_id].m_left_node_id = next_left_node.m_node_id;

	    introduce_KruhobotConflict(principal_collision.m_traversal_A, next_left_node.m_kruhobot_location_Conflicts, next_left_node.m_kruhobot_linear_Conflicts, (principal_collision.m_traversal_A.m_kruhobot_id < 0));

	    search_Store.push_back(next_left_node);
	    search_Queue.insert(NodeReference(next_left_node.m_node_id, &search_Store));
//	    search_Queue.push_back(NodeReference(next_left_node.m_node_id, &search_Store));
	    
	    Node next_right_node(search_Store.size());
	    next_right_node.m_makespan = search_Store[upper_node_id].m_makespan;
	    next_right_node.m_cost = search_Store[upper_node_id].m_cost;	    
	    next_right_node.m_conflicting = kruhobot_Collisions.size();	    
	    next_right_node.m_upper_node_id = upper_node_id;

	    next_right_node.m_kruhobot_location_Conflicts = kruhobot_location_Conflicts;
	    next_right_node.m_kruhobot_linear_Conflicts = kruhobot_linear_Conflicts;
	    next_right_node.m_kruhobot_Schedules = kruhobot_Schedules;

	    next_right_node.m_left_node_id = next_right_node.m_right_node_id = -1;	    	    

	    next_right_node.m_update_kruhobot_id = sABS(principal_collision.m_traversal_B.m_kruhobot_id);
	    search_Store[upper_node_id].m_right_node_id = next_right_node.m_node_id;

	    introduce_KruhobotConflict(principal_collision.m_traversal_B, next_right_node.m_kruhobot_location_Conflicts, next_right_node.m_kruhobot_linear_Conflicts, (principal_collision.m_traversal_B.m_kruhobot_id < 0));
	    search_Store.push_back(next_right_node);
	    search_Queue.insert(NodeReference(next_right_node.m_node_id, &search_Store));
//	    search_Queue.push_back(NodeReference(next_right_node.m_node_id, &search_Store));

	    #ifdef sDEBUG
	    {
		/*
		printf("Left\n");
		next_left_node.to_Screen();
		printf("Right\n");	    
		next_right_node.to_Screen();
		*/
	    }
	    #endif
	    
	    return -1.0;
	}
	
	return cummulative;
    }


/*----------------------------------------------------------------------------*/
    
    sDouble sRealCBS::find_KruhobotNonconflictingSchedule(const sKruhobot               &kruhobot,
							  const s2DMap                  &map,
							  sInt_32                        source_loc_id,
							  sInt_32                        sink_loc_id,
							  sDouble                        cost_limit,
							  sDouble                        sUNUSED(extra_cost),
							  const LocationConflicts__umap &location_Conflicts,
							  const LinearConflicts__map    &linear_Conflicts,
							  sDouble                        makespan_bound,
							  Schedule_vector               &Schedule) const
    {
	Transitions_mmap transition_Queue;
	Transitions_map explored_Transitions;
	Transitions_vector transition_Store;

	LocationConflicts__umap::const_iterator initial_Conflict = location_Conflicts.find(source_loc_id);
	if (initial_Conflict != location_Conflicts.end())
	{
	    LocationConflicts_map::const_iterator lower_initial_conflict = initial_Conflict->second.lower_bound(Interval(0.0, 0.0));
	    if (lower_initial_conflict != initial_Conflict->second.end())
	    {
		if (lower_initial_conflict->second.overlaps(Interval(0.0, 0.0)))
		{
		    return -1.0;
		}
	    }
	}

	Transition initial_transition(transition_Store.size(), 0.0, 0.0, source_loc_id, -1);	
	transition_Queue.insert(Transitions_mmap::value_type(0.0, initial_transition));
	transition_Store.push_back(initial_transition);

	explored_Transitions.insert(Transitions_map::value_type(0.0, LocationIDs_uset()));
	explored_Transitions[0.0].insert(source_loc_id);

	LocationIDs_uset bound_explored_Transitions;

	while (!transition_Queue.empty())
	{
	    #ifdef sDEBUG
	    {
		/*
		printf("Transition queue: %ld (bnd:%ld)\n", transition_Queue.size(), bound_explored_Transitions.size());
		*/
	    }
	    #endif
	    const Transition &front_transition = transition_Queue.begin()->second;
	    
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
	    if (front_transition.m_time <= makespan_bound)
	    {
		Transitions_map::const_iterator explored_transition = explored_Transitions.find(front_transition.m_time);

		if (explored_transition == explored_Transitions.end())
		{
		    explored_Transitions.insert(Transitions_map::value_type(front_transition.m_time, LocationIDs_uset()));
		}
	    }
	    #ifdef sDEGBUG
	    {
		static sInt_32 cost_hit = 0;
		if (front_transition.m_time + map.m_Distances[front_transition.m_location_id][sink_loc_id] > cost_limit)
		{
		    printf("cost hit: %d\n", cost_hit);
		    ++cost_hit;
		}
	    }
	    #endif
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

//			printf("MOVE: %d --> %d (%.3f, %.3f)\n", front_transition.m_location_id, neighbor_location_id, front_transition.m_time, transition_finish_time);

			LocationIDs_uset *next_explored_Transitions;
			
			if (front_transition.m_time <= makespan_bound)
			{
			    Transitions_map::const_iterator explored_transition = explored_Transitions.find(transition_finish_time);			
			    if (explored_transition == explored_Transitions.end())
			    {
				explored_Transitions.insert(Transitions_map::value_type(transition_finish_time, LocationIDs_uset()));
			    }
			    next_explored_Transitions = &explored_Transitions[transition_finish_time];
			}
			else
			{
			    next_explored_Transitions = &bound_explored_Transitions;
			}

			// TODO: more efficient IFs
			if (next_explored_Transitions->find(neighbor_location_id) == next_explored_Transitions->end())
			{
			    bool location_conflicting = false;

			    LocationConflicts__umap::const_iterator location_Conflict = location_Conflicts.find(neighbor_location_id);

			    if (location_Conflict != location_Conflicts.end())
			    {
				LocationConflicts_map::const_iterator lower_location_conflict = location_Conflict->second.lower_bound(Interval(front_transition.m_time, transition_finish_time));
				
				if (lower_location_conflict != location_Conflict->second.end())
				{
				    if (lower_location_conflict->second.m_interval.overlaps(Interval(front_transition.m_time, transition_finish_time)))
				    {
					location_conflicting = true;
				    }
				}
				else
				{
				    LocationConflicts_map::const_reverse_iterator bound_location_conflict = location_Conflict->second.rbegin();
				    if (bound_location_conflict->second.m_infinity && bound_location_conflict->second.m_interval.m_upper >= makespan_bound)
				    {
					/*
					printf("Kruhobot X: %d\n", kruhobot.m_id);
					printf("movex: %d --> %d (%.3f, %.3f)\n", front_transition.m_location_id, neighbor_location_id, front_transition.m_time, transition_finish_time);
					printf("bnd: %.3f\n", makespan_bound);
					bound_location_conflict->second.m_interval.to_Screen();
					*/
					location_conflicting = true;					
				    }
				}
			    }

			    bool origin_location_conflicting = false;			    

			    LocationConflicts__umap::const_iterator origin_location_Conflict = location_Conflicts.find(front_transition.m_location_id);
				
			    if (origin_location_Conflict != location_Conflicts.end())
			    {
				LocationConflicts_map::const_iterator lower_origin_location_conflict = origin_location_Conflict->second.lower_bound(Interval(front_transition.m_time, transition_finish_time));
				
				if (lower_origin_location_conflict != origin_location_Conflict->second.end())
				{
				    if (lower_origin_location_conflict->second.m_interval.overlaps(Interval(front_transition.m_time, transition_finish_time)))
				    {
					origin_location_conflicting = true;
				    }
				}
				else
				{
				    LocationConflicts_map::const_reverse_iterator bound_location_conflict = origin_location_Conflict->second.rbegin();
				    if (bound_location_conflict->second.m_infinity && bound_location_conflict->second.m_interval.m_upper >= makespan_bound)
				    {
                                        /*
					printf("Kruhobot Y: %d\n", kruhobot.m_id);
					printf("movex: %d --> %d (%.3f, %.3f)\n", front_transition.m_location_id, neighbor_location_id, front_transition.m_time, transition_finish_time);
					printf("bnd: %.3f\n", makespan_bound);
					bound_location_conflict->second.m_interval.to_Screen();
				     	*/
					origin_location_conflicting = true;					
				    }
				}				
			    }

			    bool linear_conflicting = false;
			    LinearConflicts__map::const_iterator linear_Conflict = linear_Conflicts.find(Uline(front_transition.m_location_id, neighbor_location_id));

			    if (linear_Conflict != linear_Conflicts.end())
			    {
				LinearConflicts_map::const_iterator lower_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_transition.m_time, transition_finish_time));
				
				if (lower_linear_conflict != linear_Conflict->second.end())
				{
				    if (lower_linear_conflict->second.m_interval.overlaps(Interval(front_transition.m_time, transition_finish_time)))
				    {
					linear_conflicting = true;
				    }
				}
			    }
			    
			    if (!location_conflicting && !origin_location_conflicting && !linear_conflicting)
			    {
//				printf("    ++: %d --> %d (%.3f, %.3f)\n", front_transition.m_location_id, neighbor_location_id, front_transition.m_time, transition_finish_time);
							
				Transition neighbor_transition(transition_Store.size(), transition_finish_time, transition_finish_cost, neighbor_location_id, front_transition.m_trans_id);
				transition_Store.push_back(neighbor_transition);
				
				next_explored_Transitions->insert(neighbor_location_id);
//				transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_time, neighbor_transition));
				transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_cost, neighbor_transition));
			    }
			}
		    }
		}
		{ /* wait action */
		    sDouble wait_location_finish_time = -1.0;
		    sDouble wait_linear_finish_time = -1.0;

		    for (s2DMap::Locations_vector::const_iterator interacting_loc = map.m_Locations.begin(); interacting_loc != map.m_Locations.end(); ++interacting_loc)
		    {
			sInt_32 neighbor_location_id = interacting_loc->m_id;
			
			if (neighbor_location_id != front_transition.m_location_id && map.m_Network.is_Adjacent(neighbor_location_id, front_transition.m_location_id))
			{
			    sDouble transition_distance = map.m_straight_Distances[front_transition.m_location_id][neighbor_location_id];
			    sDouble transition_delta_time = transition_distance / kruhobot.m_properties.m_linear_velo;
			    sDouble transition_finish_time = front_transition.m_time + transition_delta_time;

			    LocationConflicts__umap::const_iterator location_Conflict = location_Conflicts.find(neighbor_location_id);
			    sDouble first_non_conf_location_time = -1.0;
			    
			    if (location_Conflict != location_Conflicts.end())
			    {
				LocationConflicts_map::const_iterator lower_location_conflict = location_Conflict->second.lower_bound(Interval(front_transition.m_time, transition_finish_time));

				while (lower_location_conflict != location_Conflict->second.end())
				{
				    if (lower_location_conflict->second.m_infinity && lower_location_conflict->second.m_interval.m_upper >= makespan_bound)
				    {
					break;
				    }
				    else
				    {
					bool future_conflicting = false;
					
					LocationConflicts_map::const_iterator future_location_conflict = lower_location_conflict;
					for (++future_location_conflict; future_location_conflict != location_Conflict->second.end(); ++future_location_conflict)
					{
					    if (future_location_conflict->second.overlaps(Interval(lower_location_conflict->second.m_interval.m_upper, lower_location_conflict->second.m_interval.m_upper + transition_delta_time)))
					    {
						future_conflicting = true;
						break;
					    }
					}
					if (!future_conflicting)
					{
					    first_non_conf_location_time = lower_location_conflict->second.m_interval.m_upper;
					    break;
					}
				    }
				    ++lower_location_conflict;
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
				LinearConflicts_map::const_iterator lower_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_transition.m_time, transition_finish_time));
				
				while (lower_linear_conflict != linear_Conflict->second.end())
				{
				    bool future_conflicting = false;
				    LinearConflicts_map::const_iterator future_linear_conflict = lower_linear_conflict;
				    for (++future_linear_conflict; future_linear_conflict != linear_Conflict->second.end(); ++future_linear_conflict)
				    {
					if (future_linear_conflict->second.overlaps(Interval(lower_linear_conflict->second.m_interval.m_upper, lower_linear_conflict->second.m_interval.m_upper + transition_delta_time)))
					{
					    future_conflicting = true;
					    break;
					}
				    }
				    if (!future_conflicting)
				    {
					first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
					break;					    
				    }
				    ++lower_linear_conflict;
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
		    sDouble wait_finish_time = sMAX(0.0, sMAX(wait_location_finish_time, wait_linear_finish_time));

//		    printf("WAIT: %d (%.3f, %.3f)\n", front_transition.m_location_id, front_transition.m_time, wait_finish_time);  
		    if (wait_finish_time > front_transition.m_time + s_EPSILON)
		    {
			bool free_to_wait = true;

			LocationConflicts__umap::const_iterator wait_location_Conflict = location_Conflicts.find(front_transition.m_location_id);
			if (wait_location_Conflict != location_Conflicts.end())
			{
			    LocationConflicts_map::const_iterator lower_wait_conflict = wait_location_Conflict->second.lower_bound(Interval(front_transition.m_time, wait_finish_time));
				
			    if (lower_wait_conflict->second.overlaps(Interval(front_transition.m_time, wait_finish_time)))
			    {
				free_to_wait = false;
			    }
			    else
			    {
				LocationConflicts_map::const_reverse_iterator bound_wait_conflict = wait_location_Conflict->second.rbegin();
				if (bound_wait_conflict->second.m_infinity && bound_wait_conflict->second.m_interval.m_upper >= makespan_bound)
				{
				    free_to_wait = false;
				}
			    }
			}
			if (free_to_wait)
			{
//			    printf("    --: %d (%.3f, %.3f)\n", front_transition.m_location_id, front_transition.m_time, wait_finish_time);  			    
			    LocationIDs_uset *wait_explored_Transitions;

			    if (front_transition.m_time <= makespan_bound)
			    {
				Transitions_map::const_iterator explored_transition = explored_Transitions.find(wait_finish_time);
				
				if (explored_transition == explored_Transitions.end())
				{
				    explored_Transitions.insert(Transitions_map::value_type(wait_finish_time, LocationIDs_uset()));
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
				
				wait_explored_Transitions->insert(front_transition.m_location_id);
//				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_time, wait_transition));
				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_cost, wait_transition));				
			    }
			}
		    }
		}				
	    }
	    transition_Queue.erase(transition_Queue.begin());
	}
	return -1.0;	
    }


    void sRealCBS::generate_KruhobotImportantPoints(const sKruhobot               &kruhobot,
						    const s2DMap                  &map,
						    sInt_32                        source_loc_id,
						    sInt_32                        sink_loc_id,
						    sDouble                        cost_limit,
						    sDouble                        extra_cost,
						    const LocationConflicts__umap &location_Conflicts,
						    const LinearConflicts__map    &linear_Conflicts,
						    sDouble                        makespan_bound) const
    {
	sInt_32 generated = 0;
	
	Transitions_mmap transition_Queue;
	Transitions_map explored_Transitions;
	Transitions_vector transition_Store;

	Transition initial_transition(transition_Store.size(), 0.0, 0.0, source_loc_id, -1);	
	transition_Queue.insert(Transitions_mmap::value_type(0.0, initial_transition));
	transition_Store.push_back(initial_transition);
	++generated;

	explored_Transitions.insert(Transitions_map::value_type(0.0, LocationIDs_uset()));
	explored_Transitions[0.0].insert(source_loc_id);

	LocationIDs_uset bound_explored_Transitions;

	#ifdef sDEBUG
	{
	    printf("Generating important points %d --> %d\n", source_loc_id, sink_loc_id);
	}
        #endif	

	while (!transition_Queue.empty())
	{
	    #ifdef sDEBUG
	    {
		//printf("Generation queue: %ld (bnd:%ld)\n", transition_Queue.size(), bound_explored_Transitions.size());
	    }
	    #endif
	    
	    const Transition &front_transition = transition_Queue.begin()->second;
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
		Transitions_map::const_iterator explored_transition = explored_Transitions.find(front_transition.m_time);

		if (explored_transition == explored_Transitions.end())
		{
		    explored_Transitions.insert(Transitions_map::value_type(front_transition.m_time, LocationIDs_uset()));
		}
	    }

	    if (front_transition.m_time + map.m_straight_Distances[front_transition.m_location_id][sink_loc_id] <= cost_limit + extra_cost)
	    {
                #ifdef sDEBUG
		{
		    printf("    Important point: %d at %.3f\n", front_transition.m_location_id, front_transition.m_time);
		}
                #endif

		for (s2DMap::Locations_vector::const_iterator location = map.m_Locations.begin(); location != map.m_Locations.end(); ++location)
		{
		    sInt_32 neighbor_location_id = location->m_id;

		    if (neighbor_location_id != front_transition.m_location_id && map.m_Network.is_Adjacent(neighbor_location_id, front_transition.m_location_id))
		    {
			sDouble transition_distance = map.m_straight_Distances[front_transition.m_location_id][neighbor_location_id];
			sDouble transition_delta_time = transition_distance / kruhobot.m_properties.m_linear_velo;
			sDouble transition_finish_time = front_transition.m_time + transition_delta_time;
			sDouble transition_finish_cost = front_transition.m_cost + transition_delta_time;

			LocationIDs_uset *next_explored_Transitions;
			
			if (front_transition.m_time <= makespan_bound)
			{
			    Transitions_map::const_iterator explored_transition = explored_Transitions.find(transition_finish_time);			
			    if (explored_transition == explored_Transitions.end())
			    {
				explored_Transitions.insert(Transitions_map::value_type(transition_finish_time, LocationIDs_uset()));
			    }
			    next_explored_Transitions = &explored_Transitions[transition_finish_time];
			}
			else
			{
			    next_explored_Transitions = &bound_explored_Transitions;
			}

			if (next_explored_Transitions->find(neighbor_location_id) == next_explored_Transitions->end())
			{
			    Transition neighbor_transition(transition_Store.size(), transition_finish_time, transition_finish_cost, neighbor_location_id, front_transition.m_trans_id);
			    transition_Store.push_back(neighbor_transition);
			    ++generated;
			    
			    next_explored_Transitions->insert(neighbor_location_id);
//			    transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_time, neighbor_transition));
			    transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_cost, neighbor_transition));
			}
		    }
		}
		{ /* wait action */
		    sDouble wait_location_finish_time = -1.0;
		    sDouble wait_linear_finish_time = -1.0;

		    for (s2DMap::Locations_vector::const_iterator interacting_loc = map.m_Locations.begin(); interacting_loc != map.m_Locations.end(); ++interacting_loc)
		    {
			sInt_32 neighbor_location_id = interacting_loc->m_id;
			
			if (neighbor_location_id != front_transition.m_location_id && map.m_Network.is_Adjacent(neighbor_location_id, front_transition.m_location_id))
			{
			    sDouble transition_distance = map.m_straight_Distances[front_transition.m_location_id][neighbor_location_id];
			    sDouble transition_delta_time = transition_distance / kruhobot.m_properties.m_linear_velo;
			    sDouble transition_finish_time = front_transition.m_time + transition_delta_time;

			    LocationConflicts__umap::const_iterator location_Conflict = location_Conflicts.find(neighbor_location_id);
			    sDouble first_non_conf_location_time = -1.0;
			    
			    if (location_Conflict != location_Conflicts.end())
			    {
				LocationConflicts_map::const_iterator lower_location_conflict = location_Conflict->second.lower_bound(Interval(front_transition.m_time, transition_finish_time));

				while (lower_location_conflict != location_Conflict->second.end())
				{
				    if (lower_location_conflict->second.m_infinity && lower_location_conflict->second.m_interval.m_upper >= makespan_bound)
				    {
					break;
				    }
				    else
				    {
					bool future_conflicting = false;
					
					LocationConflicts_map::const_iterator future_location_conflict = lower_location_conflict;
					for (++future_location_conflict; future_location_conflict != location_Conflict->second.end(); ++future_location_conflict)
					{
					    if (future_location_conflict->second.overlaps(Interval(lower_location_conflict->second.m_interval.m_upper, lower_location_conflict->second.m_interval.m_upper + transition_delta_time)))
					    {
						future_conflicting = true;
						break;
					    }
					}
					if (!future_conflicting)
					{
					    first_non_conf_location_time = lower_location_conflict->second.m_interval.m_upper;
					    break;
					}
				    }
				    ++lower_location_conflict;
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
				LinearConflicts_map::const_iterator lower_linear_conflict = linear_Conflict->second.lower_bound(Interval(front_transition.m_time, transition_finish_time));
				
				while (lower_linear_conflict != linear_Conflict->second.end())
				{
				    bool future_conflicting = false;
				    LinearConflicts_map::const_iterator future_linear_conflict = lower_linear_conflict;
				    for (++future_linear_conflict; future_linear_conflict != linear_Conflict->second.end(); ++future_linear_conflict)
				    {
					if (future_linear_conflict->second.overlaps(Interval(lower_linear_conflict->second.m_interval.m_upper, lower_linear_conflict->second.m_interval.m_upper + transition_delta_time)))
					{
					    future_conflicting = true;
					    break;
					}
				    }
				    if (!future_conflicting)
				    {
					first_non_conf_linear_time = lower_linear_conflict->second.m_interval.m_upper;
					break;					    
				    }
				    ++lower_linear_conflict;
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
		    sDouble wait_finish_time = sMAX(0.0, sMAX(wait_location_finish_time, wait_linear_finish_time));

//		    printf("WAIT: %d (%.3f, %.3f)\n", front_transition.m_location_id, front_transition.m_time, wait_finish_time);  
		    if (wait_finish_time > front_transition.m_time + s_EPSILON)
		    {
			bool free_to_wait = true;

			LocationConflicts__umap::const_iterator wait_location_Conflict = location_Conflicts.find(front_transition.m_location_id);
			if (wait_location_Conflict != location_Conflicts.end())
			{
			    LocationConflicts_map::const_iterator lower_wait_conflict = wait_location_Conflict->second.lower_bound(Interval(front_transition.m_time, wait_finish_time));
				
			    if (lower_wait_conflict->second.overlaps(Interval(front_transition.m_time, wait_finish_time)))
			    {
				free_to_wait = false;
			    }
			    else
			    {
				LocationConflicts_map::const_reverse_iterator bound_wait_conflict = wait_location_Conflict->second.rbegin();
				if (bound_wait_conflict->second.m_infinity && bound_wait_conflict->second.m_interval.m_upper >= makespan_bound)
				{
				    free_to_wait = false;
				}
			    }
			}
			if (free_to_wait)
			{
//			    printf("    --: %d (%.3f, %.3f)\n", front_transition.m_location_id, front_transition.m_time, wait_finish_time);  			    
			    LocationIDs_uset *wait_explored_Transitions;

			    if (front_transition.m_time <= makespan_bound)
			    {
				Transitions_map::const_iterator explored_transition = explored_Transitions.find(wait_finish_time);
				
				if (explored_transition == explored_Transitions.end())
				{
				    explored_Transitions.insert(Transitions_map::value_type(wait_finish_time, LocationIDs_uset()));
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
				++generated;				
				
				wait_explored_Transitions->insert(front_transition.m_location_id);
//				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_time, wait_transition));
				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_cost, wait_transition));				
			    }
			}
		    }
		}				
	    }
	    transition_Queue.erase(transition_Queue.begin());
	}
	printf("--> Generated: %d\n", generated);
	getchar();
    }    


    bool sRealCBS::check_KruhobotNonconflictingSchedule(const Schedule_vector         &Schedule,
							const LocationConflicts__umap &location_Conflicts,
							const LinearConflicts__map    &linear_Conflicts) const
    {	
	for (Schedule_vector::const_iterator event = Schedule.begin(); event != Schedule.end(); ++event)
	{
	    if (event->m_from_loc_id != event->m_to_loc_id)
	    {
		LocationConflicts__umap::const_iterator from_location_Conflict = location_Conflicts.find(event->m_from_loc_id);
		if (from_location_Conflict != location_Conflicts.end())
		{		    
		    for (LocationConflicts_map::const_iterator location_conflict = from_location_Conflict->second.begin(); location_conflict != from_location_Conflict->second.end(); ++location_conflict)
		    {
			if (location_conflict->first.overlaps(Interval(event->m_start_time, event->m_finish_time)))
			{
                            #ifdef sDEBUG
			    {
				event->to_Screen();
				location_conflict->second.to_Screen();
			    }
			    #endif
			    return false;
			}
		    }
		}
		
		LocationConflicts__umap::const_iterator to_location_Conflict = location_Conflicts.find(event->m_to_loc_id);
		if (to_location_Conflict != location_Conflicts.end())
		{
		    for (LocationConflicts_map::const_iterator location_conflict = to_location_Conflict->second.begin(); location_conflict != to_location_Conflict->second.end(); ++location_conflict)
		    {
			if (location_conflict->first.overlaps(Interval(event->m_start_time, event->m_finish_time)))
			{
			    #ifdef sDEBUG
			    {
				event->to_Screen();
				location_conflict->second.to_Screen();
			    }
			    #endif
			    return false;
			}
		    }
		}

		LinearConflicts__map::const_iterator linear_Conflict = linear_Conflicts.find(Uline(event->m_from_loc_id, event->m_to_loc_id));
		if (linear_Conflict != linear_Conflicts.end())
		{
		    for (LinearConflicts_map::const_iterator linear_conflict = linear_Conflict->second.begin(); linear_conflict != linear_Conflict->second.end(); ++linear_conflict)
		    {
			if (linear_conflict->first.overlaps(Interval(event->m_start_time, event->m_finish_time)))
			{
			    #ifdef sDEBUG
			    {
				event->to_Screen();
				linear_conflict->second.to_Screen();
			    }
			    #endif
			    return false;
			}
		    }
		}		
	    }
	    else
	    {
		LocationConflicts__umap::const_iterator location_Conflict = location_Conflicts.find(event->m_from_loc_id);
		if (location_Conflict != location_Conflicts.end())
		{
		    for (LocationConflicts_map::const_iterator location_conflict = location_Conflict->second.begin(); location_conflict != location_Conflict->second.end(); ++location_conflict)
		    {
			if (location_conflict->first.overlaps(Interval(event->m_start_time, event->m_finish_time)))
			{
			    #ifdef sDEBUG
			    {
				event->to_Screen();
				location_conflict->second.to_Screen();
			    }
			    #endif
			    return false;
			}
		    }
		}
	    }
	}
	return true;
    }    
	 

/*----------------------------------------------------------------------------*/

    sInt_32 sRealCBS::calc_NodeDepth(const Node &node, const Nodes_vector &nodes_Store)
    {
	sInt_32 depth = 0;
	sInt_32 node_id = node.m_node_id;
	
	while (node_id >= 0)
	{
	    node_id = nodes_Store[node_id].m_upper_node_id;
	    ++depth;
	}

	return depth;
    }


    sDouble sRealCBS::calc_ScheduleMakespan(const sRealInstance &sUNUSED(real_Instance), const KruhobotSchedules_vector &kruhobot_Schedules)
    {
	sDouble makespan = 0.0;
	sInt_32 N_kruhobots_1 = kruhobot_Schedules.size();

	for (sInt_32 kruhobot_id = 1; kruhobot_id < N_kruhobots_1; ++kruhobot_id)
	{
	    for (Schedule_vector::const_reverse_iterator event = kruhobot_Schedules[kruhobot_id].rbegin(); event != kruhobot_Schedules[kruhobot_id].rend(); ++event)
	    {
		if (event->m_from_loc_id != event->m_to_loc_id)
		{
		    makespan = sMAX(makespan, event->m_finish_time);
		    break;
		}
	    }
	}
	return makespan;
    }
    
    
    sDouble sRealCBS::calc_ScheduleCost(const sRealInstance &real_Instance, const KruhobotSchedules_vector &kruhobot_Schedules)
    {
	sDouble cost = 0.0;	
	sInt_32 N_kruhobots_1 = kruhobot_Schedules.size();	
	
	for (sInt_32 kruhobot_id = 1; kruhobot_id < N_kruhobots_1; ++kruhobot_id)
	{
	    if (!kruhobot_Schedules[kruhobot_id].empty())
	    {
		const sKruhobot &kruhobot = real_Instance.m_Kruhobots[kruhobot_id];
	    
		for (Schedule_vector::const_iterator event = kruhobot_Schedules[kruhobot_id].begin(); event != kruhobot_Schedules[kruhobot_id].end(); ++event)
		{
		    if (event->m_from_loc_id != event->m_to_loc_id)
		    {
			cost += event->m_finish_time - event->m_start_time;
		    }
		}
		Schedule_vector::const_reverse_iterator event = kruhobot_Schedules[kruhobot_id].rbegin();
		for (++event; event != kruhobot_Schedules[kruhobot_id].rend(); ++event)
		{
		    if (event->m_from_loc_id == event->m_to_loc_id)
		    {
			cost += (event->m_finish_time - event->m_start_time) * kruhobot.m_properties.m_wait_factor;
		    }
		}
	    }
	}

	return cost;	
    }    


/*----------------------------------------------------------------------------*/

    void sRealCBS::to_Screen(const Node &node, const Nodes_vector &nodes_Store, const sString &indent)
    {
	to_Stream(stdout, node, nodes_Store, indent);
    }

    
    void sRealCBS::to_Stream(FILE *fw, const Node &node, const Nodes_vector &nodes_Store, const sString &indent)
    {
	sInt_32 node_id = node.m_node_id;
	
	while (node_id >= 0)
	{
	    nodes_Store[node_id].to_Stream(fw, indent);
	    node_id = nodes_Store[node_id].m_upper_node_id;
	}
    }


    

/*----------------------------------------------------------------------------*/

} // namespace boOX


