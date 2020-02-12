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
/* cbsR.cpp / 1-224_leibniz                                                   */
/*----------------------------------------------------------------------------*/
//
// Conflict based search for a semi-continuous version of MAPF.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>

#include <map>

#include "config.h"
#include "compile.h"
#include "version.h"
#include "defs.h"
#include "result.h"

#include "common/types.h"
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
	UlinearCooccupations_map linear_Cooccupations;
	
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
//		    location_Cooccupations[event->m_from_loc_id][Interval(event->m_start_time, event->m_finish_time)].insert(kruhobot_id);
//		    location_Cooccupations[event->m_to_loc_id][Interval(event->m_start_time, event->m_finish_time)].insert(kruhobot_id);		    		    

		    linear_Cooccupations[Uline(event->m_from_loc_id, event->m_to_loc_id)][Interval(event->m_start_time, event->m_finish_time)].insert(kruhobot_id);
		    //printf("%d->%d [%.3f,%.3f)\n", event->m_from_loc_id, event->m_to_loc_id, event->m_start_time, event->m_finish_time);
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
	    printf("Ulinear\n");
	    for (UlinearCooccupations_map::const_iterator linear_Coop = linear_Cooccupations.begin(); linear_Coop != linear_Cooccupations.end(); ++linear_Coop)
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

	/* Colliding waitings in vertices - standard MAPF style collisions */
	for (LocationCooccupations_umap::const_iterator location_Coop = location_Cooccupations.begin(); location_Coop != location_Cooccupations.end(); ++location_Coop)
	{
	    const Cooccupations_map &cooccupations = location_Coop->second;

	    for (Cooccupations_map::const_iterator coop = cooccupations.begin(); coop != cooccupations.end(); ++coop)
	    {		
		sASSERT(!coop->second.empty());
		for (KruhobotIDs_uset::const_iterator kruhobot = coop->second.begin(); kruhobot != coop->second.end(); ++kruhobot)
		{
		    /* kruhobors share a vertex for the identical time interval */
		    KruhobotIDs_uset::const_iterator next_kruhobot = kruhobot;
		    for (++next_kruhobot; next_kruhobot != coop->second.end(); ++next_kruhobot)
		    {
			sASSERT(*kruhobot != *next_kruhobot);
			
			Traversal traversal_A(*kruhobot, location_Coop->first, location_Coop->first, Interval(coop->first.m_lower, coop->first.m_upper));
			Traversal traversal_B(*next_kruhobot, location_Coop->first, location_Coop->first, Interval(coop->first.m_lower, coop->first.m_upper));

			sDouble importance = calc_KruhobotCollisionImportance(traversal_A, traversal_B, location_Cooccupations, linear_Cooccupations);			
			kruhobot_Collisions.insert(KruhobotCollision(importance, traversal_A, traversal_B));
		    }

		    /* kruhobors share a vertex for overlapping time intervals */
		    Cooccupations_map::const_iterator next_coop = coop;
		    for (++next_coop; next_coop != cooccupations.end(); ++next_coop)
		    {
			if (coop->first.overlaps(next_coop->first))
			{
			    for (KruhobotIDs_uset::const_iterator next_kruhobot = next_coop->second.begin(); next_kruhobot != next_coop->second.end(); ++next_kruhobot)
			    {
				Interval intersection = coop->first.intersect(next_coop->first);
				
				sASSERT(!intersection.empty());
				/*
				if (*kruhobot == *next_kruhobot)
				{
				    printf("%d x %d\n", *kruhobot, *next_kruhobot);
				    coop->first.to_Screen();
				    next_coop->first.to_Screen();
				    intersection.to_Screen();
				    sASSERT(false);
				}
				*/
			
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

	for (UlinearCooccupations_map::const_iterator linear_Coop = linear_Cooccupations.begin(); linear_Coop != linear_Cooccupations.end(); ++linear_Coop)
	{	    
	    const Cooccupations_map &cooccupations = linear_Coop->second;

	    /* Line sharing collisions */
	    for (Cooccupations_map::const_iterator coop = cooccupations.begin(); coop != cooccupations.end(); ++coop)
	    {
		sASSERT(!coop->second.empty());

		for (KruhobotIDs_uset::const_iterator kruhobot = coop->second.begin(); kruhobot != coop->second.end(); ++kruhobot)
		{
		    /* kruhobots use the line for identical time intervals */
		    KruhobotIDs_uset::const_iterator next_kruhobot = kruhobot;
		    for (++next_kruhobot; next_kruhobot != coop->second.end(); ++next_kruhobot)
		    {
			sASSERT(*kruhobot != *next_kruhobot);
			
			Traversal traversal_A(*kruhobot, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, Interval(coop->first.m_lower, coop->first.m_upper));
			Traversal traversal_B(*next_kruhobot, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, Interval(coop->first.m_lower, coop->first.m_upper));
		       
			sDouble importance = calc_KruhobotCollisionImportance(traversal_A, traversal_B, location_Cooccupations, linear_Cooccupations);
			kruhobot_Collisions.insert(KruhobotCollision(importance, traversal_A, traversal_B));
		    }

		    /* kruhobots used the line for overlapping time intervals */
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

		/* Line vertex sharing collisions - lower end of the line */
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

		/* Line vertex sharing collisions - upper end of the line */
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

		// Lines vs. locations collisions
		for (LocationCooccupations_umap::const_iterator other_location_cooccupations = location_Cooccupations.begin(); other_location_cooccupations != location_Cooccupations.end(); ++other_location_cooccupations)
		{
		    sDouble point_distance = map.calc_PointDistance(linear_Coop->first.m_lower_id,
								    linear_Coop->first.m_upper_id,
								    other_location_cooccupations->first);


//		    for (Cooccupations_map::const_iterator other_coop = other_location_cooccupations->second.begin(); other_coop != other_location_cooccupations->second.end(); ++other_coop)
		    for (Cooccupations_map::const_iterator other_coop = other_location_cooccupations->second.lower_bound(Interval(coop->first.m_lower, coop->first.m_lower));
			 other_coop != other_location_cooccupations->second.end(); ++other_coop)		    
		    {
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

		// Lines vs. lines collisions
		UlinearCooccupations_map::const_iterator other_linear_Coop = linear_Coop;		
		for (++other_linear_Coop; other_linear_Coop != linear_Cooccupations.end(); ++other_linear_Coop)
		{
		    sDouble line_distance = map.calc_LineDistance(linear_Coop->first.m_lower_id,
								  linear_Coop->first.m_upper_id,
								  other_linear_Coop->first.m_lower_id,
								  other_linear_Coop->first.m_upper_id);
		    const Cooccupations_map &other_linear_cooccupations = other_linear_Coop->second;

//		    for (Cooccupations_map::const_iterator other_coop = other_linear_cooccupations.begin(); other_coop != other_linear_cooccupations.end(); ++other_coop)
		    for (Cooccupations_map::const_iterator other_coop = other_linear_cooccupations.lower_bound(Interval(coop->first.m_lower, coop->first.m_lower));
			 other_coop != other_linear_cooccupations.end(); ++other_coop)
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


    sDouble sRealCBSBase::analyze_NonconflictingSchedules_nonprioritized(const sRealInstance            &real_Instance,
									 const KruhobotSchedules_vector &kruhobot_Schedules,
									 KruhobotCollisions_mset        &kruhobot_Collisions) const
    {
	sDouble cummulative = 0.0;
	
	LocationCooccupations_umap location_Cooccupations;
	UlinearCooccupations_map linear_Cooccupations;
	
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
//		    location_Cooccupations[event->m_from_loc_id][Interval(event->m_start_time, event->m_finish_time)].insert(kruhobot_id);
//		    location_Cooccupations[event->m_to_loc_id][Interval(event->m_start_time, event->m_finish_time)].insert(kruhobot_id);		    		    

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
	    printf("Ulinear\n");
	    for (UlinearCooccupations_map::const_iterator linear_Coop = linear_Cooccupations.begin(); linear_Coop != linear_Cooccupations.end(); ++linear_Coop)
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

	/* Colliding waitings in vertices - standard MAPF style collisions */
	for (LocationCooccupations_umap::const_iterator location_Coop = location_Cooccupations.begin(); location_Coop != location_Cooccupations.end(); ++location_Coop)
	{
	    const Cooccupations_map &cooccupations = location_Coop->second;

	    for (Cooccupations_map::const_iterator coop = cooccupations.begin(); coop != cooccupations.end(); ++coop)
	    {		
		sASSERT(!coop->second.empty());
		for (KruhobotIDs_uset::const_iterator kruhobot = coop->second.begin(); kruhobot != coop->second.end(); ++kruhobot)
		{
		    /* kruhobors share a vertex for the identical time interval */
		    KruhobotIDs_uset::const_iterator next_kruhobot = kruhobot;
		    for (++next_kruhobot; next_kruhobot != coop->second.end(); ++next_kruhobot)
		    {
			sASSERT(*kruhobot != *next_kruhobot);
			
			Traversal traversal_A(*kruhobot, location_Coop->first, location_Coop->first, Interval(coop->first.m_lower, coop->first.m_upper));
			Traversal traversal_B(*next_kruhobot, location_Coop->first, location_Coop->first, Interval(coop->first.m_lower, coop->first.m_upper));

			/*
			printf("point [%d,%d]\n", *kruhobot, *next_kruhobot);
			printf("gamma 50: %d\n", location_Coop->first);			
			*/
			kruhobot_Collisions.insert(KruhobotCollision(traversal_A, traversal_B));
		    }

		    /* kruhobors share a vertex for overlapping time intervals */
		    Cooccupations_map::const_iterator next_coop = coop;
		    for (++next_coop; next_coop != cooccupations.end(); ++next_coop)
		    {
			if (coop->first.overlaps(next_coop->first))
			{
			    for (KruhobotIDs_uset::const_iterator next_kruhobot = next_coop->second.begin(); next_kruhobot != next_coop->second.end(); ++next_kruhobot)
			    {
				Interval intersection = coop->first.intersect(next_coop->first);
				
				sASSERT(!intersection.empty());
				/*
				if (*kruhobot == *next_kruhobot)
				{
				    printf("%d x %d\n", *kruhobot, *next_kruhobot);
				    coop->first.to_Screen();
				    next_coop->first.to_Screen();
				    intersection.to_Screen();
				    sASSERT(false);
				}
				*/
				
				Traversal traversal_A(*kruhobot, location_Coop->first, location_Coop->first, Interval(intersection.m_lower, intersection.m_upper));
				Traversal traversal_B(*next_kruhobot, location_Coop->first, location_Coop->first, Interval(intersection.m_lower, intersection.m_upper));
						
				kruhobot_Collisions.insert(KruhobotCollision(traversal_A, traversal_B));
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

	for (UlinearCooccupations_map::const_iterator linear_Coop = linear_Cooccupations.begin(); linear_Coop != linear_Cooccupations.end(); ++linear_Coop)
	{	    
	    const Cooccupations_map &cooccupations = linear_Coop->second;

	    /* Line sharing collisions */
	    for (Cooccupations_map::const_iterator coop = cooccupations.begin(); coop != cooccupations.end(); ++coop)
	    {
		sASSERT(!coop->second.empty());

		for (KruhobotIDs_uset::const_iterator kruhobot = coop->second.begin(); kruhobot != coop->second.end(); ++kruhobot)
		{
		    /* kruhobots use the line for identical time intervals */
		    KruhobotIDs_uset::const_iterator next_kruhobot = kruhobot;
		    for (++next_kruhobot; next_kruhobot != coop->second.end(); ++next_kruhobot)
		    {
			sASSERT(*kruhobot != *next_kruhobot);
			
			Traversal traversal_A(*kruhobot, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, Interval(coop->first.m_lower, coop->first.m_upper));
			Traversal traversal_B(*next_kruhobot, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, Interval(coop->first.m_lower, coop->first.m_upper));
		       
			kruhobot_Collisions.insert(KruhobotCollision(traversal_A, traversal_B));
		    }

		    /* kruhobots used the line for overlapping time intervals */
		    Cooccupations_map::const_iterator next_coop = coop;
		    for (++next_coop; next_coop != cooccupations.end(); ++next_coop)
		    {
			/*
			printf("G1\n");
			coop->first.to_Screen();
			next_coop->first.to_Screen();
			*/
			
			if (coop->first.overlaps(next_coop->first))
			{
			    Interval intersection = coop->first.intersect(next_coop->first);
			    sASSERT(!intersection.empty());			    
							
			    for (KruhobotIDs_uset::const_iterator next_kruhobot = next_coop->second.begin(); next_kruhobot != next_coop->second.end(); ++next_kruhobot)
			    {
				sASSERT(*kruhobot != *next_kruhobot);
			
				Traversal traversal_A(*kruhobot, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, Interval(intersection.m_lower, intersection.m_upper));
				Traversal traversal_B(*next_kruhobot, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, Interval(intersection.m_lower, intersection.m_upper));
				/*

				printf("    same line [%d,%d]\n", *kruhobot, *next_kruhobot);
				printf("    gamma 30: %d,%d,%d,%d\n", linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id);
 
				traversal_A.to_Screen("    ");
				traversal_B.to_Screen("    ");
				*/

				kruhobot_Collisions.insert(KruhobotCollision(traversal_A, traversal_B));
			    }
			}
			else
			{
			    break;
			}
		    }
		}

		/* Line vertex sharing collisions - lower end of the line */
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

					kruhobot_Collisions.insert(KruhobotCollision(traversal_A, traversal_B));
				    }
				}				
			    }
			}
		    }		    
		}

		/* Line vertex sharing collisions - upper end of the line */
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

					kruhobot_Collisions.insert(KruhobotCollision(traversal_A, traversal_B));
				    }
				}				
			    }
			}
		    }		    
		}

		// Lines vs. locations collisions
		for (LocationCooccupations_umap::const_iterator other_location_cooccupations = location_Cooccupations.begin(); other_location_cooccupations != location_Cooccupations.end(); ++other_location_cooccupations)
		{
		    sDouble point_distance = map.calc_PointDistance(linear_Coop->first.m_lower_id,
								    linear_Coop->first.m_upper_id,
								    other_location_cooccupations->first);


//		    for (Cooccupations_map::const_iterator other_coop = other_location_cooccupations->second.begin(); other_coop != other_location_cooccupations->second.end(); ++other_coop)
		    for (Cooccupations_map::const_iterator other_coop = other_location_cooccupations->second.lower_bound(Interval(coop->first.m_lower, coop->first.m_lower));
			 other_coop != other_location_cooccupations->second.end(); ++other_coop)		    
		    {
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

					    kruhobot_Collisions.insert(KruhobotCollision(traversal_A, traversal_B));
					}
				    }
				}
			    }			    
			}			
		    }
		}

		// Lines vs. lines collisions
		UlinearCooccupations_map::const_iterator other_linear_Coop = linear_Coop;		
		for (++other_linear_Coop; other_linear_Coop != linear_Cooccupations.end(); ++other_linear_Coop)
		{
		    sDouble line_distance = map.calc_LineDistance(linear_Coop->first.m_lower_id,
								  linear_Coop->first.m_upper_id,
								  other_linear_Coop->first.m_lower_id,
								  other_linear_Coop->first.m_upper_id);

		    if (linear_Coop->first.m_upper_id != other_linear_Coop->first.m_lower_id && other_linear_Coop->first.m_upper_id != linear_Coop->first.m_lower_id)
		    {
			const Cooccupations_map &other_linear_cooccupations = other_linear_Coop->second;

//		        for (Cooccupations_map::const_iterator other_coop = other_linear_cooccupations.begin(); other_coop != other_linear_cooccupations.end(); ++other_coop)
			for (Cooccupations_map::const_iterator other_coop = other_linear_cooccupations.lower_bound(Interval(coop->first.m_lower, coop->first.m_lower));
			     other_coop != other_linear_cooccupations.end(); ++other_coop)
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
						/*
						printf("G2\n");
						coop->first.to_Screen();
						other_coop->first.to_Screen();
						*/
						
						Traversal traversal_A(*kruhobot, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, Interval(intersection.m_lower, intersection.m_upper));
						Traversal traversal_B(*other_kruhobot, other_linear_Coop->first.m_lower_id, other_linear_Coop->first.m_upper_id, Interval(intersection.m_lower, intersection.m_upper));
                                                /*
						printf("    -> line distance [%d,%d]: %.3f\n", *kruhobot, *other_kruhobot, line_distance);
						printf("    -> gamma 2: %d,%d,%d,%d\n", linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, other_linear_Coop->first.m_lower_id, other_linear_Coop->first.m_upper_id);
                                                */				    
						kruhobot_Collisions.insert(KruhobotCollision(traversal_A, traversal_B));
					    }
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


    sDouble sRealCBSBase::analyze_NonconflictingSchedules_exactNonprioritized(const sRealInstance            &real_Instance,
									      const KruhobotSchedules_vector &kruhobot_Schedules,
									      KruhobotCollisions_mset        &kruhobot_Collisions) const
    {
	sDouble cummulative = 0.0;
	
	LocationCooccupations_umap location_Cooccupations;
	UlinearCooccupations__map linear_Cooccupations;
	
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
//		    location_Cooccupations[event->m_from_loc_id][Interval(event->m_start_time, event->m_finish_time)].insert(kruhobot_id);
//		    location_Cooccupations[event->m_to_loc_id][Interval(event->m_start_time, event->m_finish_time)].insert(kruhobot_id);		    		    

		    if (event->m_from_loc_id <= event->m_to_loc_id)
		    {
			linear_Cooccupations[Uline(event->m_from_loc_id, event->m_to_loc_id)][Interval(event->m_start_time, event->m_finish_time)].insert(KruhobotIDs_umap::value_type(kruhobot_id, 1));
		    }
		    else
		    {
			linear_Cooccupations[Uline(event->m_from_loc_id, event->m_to_loc_id)][Interval(event->m_start_time, event->m_finish_time)].insert(KruhobotIDs_umap::value_type(kruhobot_id, -1));
		    }
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
	    printf("Ulinear\n");
	    for (UlinearCooccupations_map::const_iterator linear_Coop = linear_Cooccupations.begin(); linear_Coop != linear_Cooccupations.end(); ++linear_Coop)
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

	/* Colliding waitings in vertices - standard MAPF style collisions */
	for (LocationCooccupations_umap::const_iterator location_Coop = location_Cooccupations.begin(); location_Coop != location_Cooccupations.end(); ++location_Coop)
	{
	    const Cooccupations_map &cooccupations = location_Coop->second;

	    for (Cooccupations_map::const_iterator coop = cooccupations.begin(); coop != cooccupations.end(); ++coop)
	    {		
		sASSERT(!coop->second.empty());
		for (KruhobotIDs_uset::const_iterator kruhobot = coop->second.begin(); kruhobot != coop->second.end(); ++kruhobot)
		{
		    /* kruhobors share a vertex for the identical time interval */
		    KruhobotIDs_uset::const_iterator next_kruhobot = kruhobot;
		    for (++next_kruhobot; next_kruhobot != coop->second.end(); ++next_kruhobot)
		    {
			sASSERT(*kruhobot != *next_kruhobot);
			
			Traversal traversal_A(*kruhobot, location_Coop->first, location_Coop->first, Interval(coop->first.m_lower, coop->first.m_upper));
			Traversal traversal_B(*next_kruhobot, location_Coop->first, location_Coop->first, Interval(coop->first.m_lower, coop->first.m_upper));

			/*
			printf("point [%d,%d]\n", *kruhobot, *next_kruhobot);
			printf("gamma 50: %d\n", location_Coop->first);			
			*/
			kruhobot_Collisions.insert(KruhobotCollision(traversal_A, traversal_B));
		    }

		    /* kruhobors share a vertex for overlapping time intervals */
		    Cooccupations_map::const_iterator next_coop = coop;
		    for (++next_coop; next_coop != cooccupations.end(); ++next_coop)
		    {
			if (coop->first.overlaps(next_coop->first))
			{
			    for (KruhobotIDs_uset::const_iterator next_kruhobot = next_coop->second.begin(); next_kruhobot != next_coop->second.end(); ++next_kruhobot)
			    {			
				Traversal traversal_A(*kruhobot, location_Coop->first, location_Coop->first, coop->first);
				Traversal traversal_B(*next_kruhobot, location_Coop->first, location_Coop->first, next_coop->first);
						
				kruhobot_Collisions.insert(KruhobotCollision(traversal_A, traversal_B));			
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

	for (UlinearCooccupations__map::const_iterator linear_Coop = linear_Cooccupations.begin(); linear_Coop != linear_Cooccupations.end(); ++linear_Coop)
	{	    
	    const Cooccupations__map &cooccupations = linear_Coop->second;

	    /* Line sharing collisions */
	    for (Cooccupations__map::const_iterator coop = cooccupations.begin(); coop != cooccupations.end(); ++coop)
	    {
		sASSERT(!coop->second.empty());

		for (KruhobotIDs_umap::const_iterator kruhobot = coop->second.begin(); kruhobot != coop->second.end(); ++kruhobot)
		{
		    /* kruhobots use the line for identical time intervals */
		    KruhobotIDs_umap::const_iterator next_kruhobot = kruhobot;
		    for (++next_kruhobot; next_kruhobot != coop->second.end(); ++next_kruhobot)
		    {
			sASSERT(*kruhobot != *next_kruhobot);

			Traversal traversal_A;

			if (kruhobot->second >= 0)
			{
			    traversal_A = Traversal(kruhobot->first, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, Interval(coop->first.m_lower, coop->first.m_upper));
			}
			else
			{
			    traversal_A = Traversal(kruhobot->first, linear_Coop->first.m_upper_id, linear_Coop->first.m_lower_id, Interval(coop->first.m_lower, coop->first.m_upper));			    
			}
			
			Traversal traversal_B;
			
			if (next_kruhobot->second >= 0)
			{
			    traversal_B = Traversal(next_kruhobot->first, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, Interval(coop->first.m_lower, coop->first.m_upper));
			}
			else
			{
			    traversal_B = Traversal(next_kruhobot->first, linear_Coop->first.m_upper_id, linear_Coop->first.m_lower_id, Interval(coop->first.m_lower, coop->first.m_upper));
			}			
			kruhobot_Collisions.insert(KruhobotCollision(traversal_A, traversal_B));
		    }

		    /* kruhobots used the line for overlapping time intervals */
		    Cooccupations__map::const_iterator next_coop = coop;
		    for (++next_coop; next_coop != cooccupations.end(); ++next_coop)
		    {			
			if (coop->first.overlaps(next_coop->first))
			{					
			    for (KruhobotIDs_umap::const_iterator next_kruhobot = next_coop->second.begin(); next_kruhobot != next_coop->second.end(); ++next_kruhobot)
			    {
				sASSERT(*kruhobot != *next_kruhobot);
			
				Traversal traversal_A;
				
				if (kruhobot->second >= 0)
				{
				    traversal_A = Traversal(kruhobot->first, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, Interval(coop->first.m_lower, coop->first.m_upper));
				}
				else
				{
				    traversal_A = Traversal(kruhobot->first, linear_Coop->first.m_upper_id, linear_Coop->first.m_lower_id, Interval(coop->first.m_lower, coop->first.m_upper));			    
				}
				
				Traversal traversal_B;
			
				if (next_kruhobot->second >= 0)
				{
				    traversal_B = Traversal(next_kruhobot->first, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, Interval(next_coop->first.m_lower, next_coop->first.m_upper));
				}
				else
				{
				    traversal_B = Traversal(next_kruhobot->first, linear_Coop->first.m_upper_id, linear_Coop->first.m_lower_id, Interval(next_coop->first.m_lower, next_coop->first.m_upper));
				}				
				kruhobot_Collisions.insert(KruhobotCollision(traversal_A, traversal_B));
			    }
			}
			else
			{
			    break;
			}
		    }
		}

		/* Line vertex sharing collisions - lower end of the line */
		LocationCooccupations_umap::const_iterator other_location_cooccupations_low = location_Cooccupations.find(linear_Coop->first.m_lower_id);
		if (other_location_cooccupations_low != location_Cooccupations.end())
		{
		    for (Cooccupations_map::const_iterator other_coop = other_location_cooccupations_low->second.begin(); other_coop != other_location_cooccupations_low->second.end(); ++other_coop)
		    {
			if (coop->first.overlaps(other_coop->first))
			{
			    for (KruhobotIDs_umap::const_iterator kruhobot = coop->second.begin(); kruhobot != coop->second.end(); ++kruhobot)
			    {
				for (KruhobotIDs_uset::const_iterator other_kruhobot = other_coop->second.begin(); other_kruhobot != other_coop->second.end(); ++other_kruhobot)
				{
				    if (kruhobot->first != *other_kruhobot)
				    {
					/*
					Traversal traversal_A(*kruhobot, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, coop->first);
					Traversal traversal_B(*other_kruhobot, linear_Coop->first.m_lower_id, linear_Coop->first.m_lower_id, other_coop->first);
					*/

					Traversal traversal_A;
					
					if (kruhobot->second >= 0)
					{
					    traversal_A = Traversal(kruhobot->first, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, Interval(coop->first.m_lower, coop->first.m_upper));
					}
					else
					{
					    traversal_A = Traversal(kruhobot->first, linear_Coop->first.m_upper_id, linear_Coop->first.m_lower_id, Interval(coop->first.m_lower, coop->first.m_upper));			    
					}
					Traversal traversal_B(*other_kruhobot, linear_Coop->first.m_lower_id, linear_Coop->first.m_lower_id, other_coop->first);
					
					kruhobot_Collisions.insert(KruhobotCollision(traversal_A, traversal_B));
				    }
				}				
			    }
			}
		    }		    
		}

		/* Line vertex sharing collisions - upper end of the line */
		LocationCooccupations_umap::const_iterator other_location_cooccupations_up = location_Cooccupations.find(linear_Coop->first.m_upper_id);
		if (other_location_cooccupations_up != location_Cooccupations.end())
		{
		    for (Cooccupations_map::const_iterator other_coop = other_location_cooccupations_up->second.begin(); other_coop != other_location_cooccupations_up->second.end(); ++other_coop)
		    {
			if (coop->first.overlaps(other_coop->first))
			{					    
			    for (KruhobotIDs_umap::const_iterator kruhobot = coop->second.begin(); kruhobot != coop->second.end(); ++kruhobot)
			    {
				for (KruhobotIDs_uset::const_iterator other_kruhobot = other_coop->second.begin(); other_kruhobot != other_coop->second.end(); ++other_kruhobot)
				{
				    if (kruhobot->first != *other_kruhobot)
				    {
					/*
					Traversal traversal_A(*kruhobot, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, coop->first);
					Traversal traversal_B(*other_kruhobot, linear_Coop->first.m_upper_id, linear_Coop->first.m_upper_id, other_coop->first);
					*/
					Traversal traversal_A;
					
					if (kruhobot->second >= 0)
					{
					    traversal_A = Traversal(kruhobot->first, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, Interval(coop->first.m_lower, coop->first.m_upper));
					}
					else
					{
					    traversal_A = Traversal(kruhobot->first, linear_Coop->first.m_upper_id, linear_Coop->first.m_lower_id, Interval(coop->first.m_lower, coop->first.m_upper));			    
					}
					Traversal traversal_B(*other_kruhobot, linear_Coop->first.m_upper_id, linear_Coop->first.m_upper_id, other_coop->first);					

					kruhobot_Collisions.insert(KruhobotCollision(traversal_A, traversal_B));
				    }
				}				
			    }
			}
		    }		    
		}

		// Lines vs. locations collisions
		for (LocationCooccupations_umap::const_iterator other_location_cooccupations = location_Cooccupations.begin(); other_location_cooccupations != location_Cooccupations.end(); ++other_location_cooccupations)
		{
		    sDouble point_distance = map.calc_PointDistance(linear_Coop->first.m_lower_id,
								    linear_Coop->first.m_upper_id,
								    other_location_cooccupations->first);

//		    for (Cooccupations_map::const_iterator other_coop = other_location_cooccupations->second.begin(); other_coop != other_location_cooccupations->second.end(); ++other_coop)
		    for (Cooccupations_map::const_iterator other_coop = other_location_cooccupations->second.lower_bound(Interval(coop->first.m_lower, coop->first.m_lower));
			 other_coop != other_location_cooccupations->second.end(); ++other_coop)		    
		    {
			if (coop->first.overlaps(other_coop->first))
			{			    
			    for (KruhobotIDs_umap::const_iterator kruhobot = coop->second.begin(); kruhobot != coop->second.end(); ++kruhobot)
			    {
				for (KruhobotIDs_uset::const_iterator other_kruhobot = other_coop->second.begin(); other_kruhobot != other_coop->second.end(); ++other_kruhobot)
				{
				    if (point_distance < real_Instance.m_Kruhobots[sABS(kruhobot->first)].m_properties.m_radius + real_Instance.m_Kruhobots[sABS(*other_kruhobot)].m_properties.m_radius)
				    {				    
					if (sABS(kruhobot->first) != sABS(*other_kruhobot))
					{
					    Traversal traversal_A;
					    
					    if (kruhobot->second >= 0)
					    {
						traversal_A = Traversal(kruhobot->first, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, Interval(coop->first.m_lower, coop->first.m_upper));
					    }
					    else
					    {
						traversal_A = Traversal(kruhobot->first, linear_Coop->first.m_upper_id, linear_Coop->first.m_lower_id, Interval(coop->first.m_lower, coop->first.m_upper));			    
					    }					    
					    // Traversal traversal_A(*kruhobot, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, coop->first);
					    Traversal traversal_B(*other_kruhobot, other_location_cooccupations->first, other_location_cooccupations->first, other_coop->first);

					    kruhobot_Collisions.insert(KruhobotCollision(traversal_A, traversal_B));
					}
				    }
				}
			    }			    
			}			
		    }
		}

		// Lines vs. lines collisions
		UlinearCooccupations__map::const_iterator other_linear_Coop = linear_Coop;		
		for (++other_linear_Coop; other_linear_Coop != linear_Cooccupations.end(); ++other_linear_Coop)
		{
		    sDouble line_distance = map.calc_LineDistance(linear_Coop->first.m_lower_id,
								  linear_Coop->first.m_upper_id,
								  other_linear_Coop->first.m_lower_id,
								  other_linear_Coop->first.m_upper_id);

		    //if (linear_Coop->first.m_upper_id != other_linear_Coop->first.m_lower_id && other_linear_Coop->first.m_upper_id != linear_Coop->first.m_lower_id)
		    {		    
			const Cooccupations__map &other_linear_cooccupations = other_linear_Coop->second;

//		        for (Cooccupations_map::const_iterator other_coop = other_linear_cooccupations.begin(); other_coop != other_linear_cooccupations.end(); ++other_coop)
			for (Cooccupations__map::const_iterator other_coop = other_linear_cooccupations.lower_bound(Interval(coop->first.m_lower, coop->first.m_lower));
			     other_coop != other_linear_cooccupations.end(); ++other_coop)
			{
			    if (coop->first.overlaps(other_coop->first))
			    {
				for (KruhobotIDs_umap::const_iterator kruhobot = coop->second.begin(); kruhobot != coop->second.end(); ++kruhobot)
				{				    
				    for (KruhobotIDs_umap::const_iterator other_kruhobot = other_coop->second.begin(); other_kruhobot != other_coop->second.end(); ++other_kruhobot)
				    {
					if (line_distance < real_Instance.m_Kruhobots[sABS(kruhobot->first)].m_properties.m_radius + real_Instance.m_Kruhobots[sABS(other_kruhobot->first)].m_properties.m_radius)
					{
					    if (sABS(kruhobot->first) != sABS(other_kruhobot->first))
					    {

						Traversal traversal_A;
						
						if (kruhobot->second >= 0)
						{
						    traversal_A = Traversal(kruhobot->first, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, Interval(coop->first.m_lower, coop->first.m_upper));
						}
						else
						{
						    traversal_A = Traversal(kruhobot->first, linear_Coop->first.m_upper_id, linear_Coop->first.m_lower_id, Interval(coop->first.m_lower, coop->first.m_upper));			    
						}
				
						Traversal traversal_B;
						
						if (other_kruhobot->second >= 0)
						{
						    traversal_B = Traversal(other_kruhobot->first, other_linear_Coop->first.m_lower_id, other_linear_Coop->first.m_upper_id, Interval(other_coop->first.m_lower, other_coop->first.m_upper));
						}
						else
						{
						    traversal_B = Traversal(other_kruhobot->first, other_linear_Coop->first.m_upper_id, other_linear_Coop->first.m_lower_id, Interval(other_coop->first.m_lower, other_coop->first.m_upper));
						}
						/*
						Traversal traversal_A(*kruhobot, linear_Coop->first.m_lower_id, linear_Coop->first.m_upper_id, coop->first);
						Traversal traversal_B(*other_kruhobot, other_linear_Coop->first.m_lower_id, other_linear_Coop->first.m_upper_id, other_coop->first);
						*/
						kruhobot_Collisions.insert(KruhobotCollision(traversal_A, traversal_B));
					    }
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


/*----------------------------------------------------------------------------*/

    sRealCBSBase::KruhobotAffection_pair sRealCBSBase::resolve_KruhobotCollision(const sRealInstance              &real_Instance,
										 const Traversal                  &kruhobot_traversal_A,
										 const Traversal                  &kruhobot_traversal_B,				       
										 KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
										 KruhobotUlinearConflicts_vector   &kruhobot_linear_Conflicts,
										 sInt_32                          &last_conflict_id,
										 bool                              infinity) const
    {
	
	return resolve_KruhobotCollision<KruhobotLocationConflicts_vector, KruhobotUlinearConflicts_vector>(real_Instance,
													   kruhobot_traversal_A,
													   kruhobot_traversal_B,				       
													   kruhobot_location_Conflicts,
													   kruhobot_linear_Conflicts,
													   last_conflict_id,
													   infinity);
    }

    
    sRealCBSBase::KruhobotAffection_pair sRealCBSBase::resolve_KruhobotCollision(const sRealInstance                    &real_Instance,
										 const Traversal                        &kruhobot_traversal_A,
										 const Traversal                        &kruhobot_traversal_B,				       
										 KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
										 KruhobotUlinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
										 sInt_32                                &last_conflict_id,
										 bool                                    infinity) const
    {
	
	return resolve_KruhobotCollision<KruhobotLocationConflicts_upper_vector, KruhobotUlinearConflicts_upper_vector>(real_Instance,
														       kruhobot_traversal_A,
														       kruhobot_traversal_B,				       
														       kruhobot_location_Conflicts,
														       kruhobot_linear_Conflicts,
														       last_conflict_id,
														       infinity);
    }


    sRealCBSBase::KruhobotAffection_pair sRealCBSBase::resolve_KruhobotCollision(const sRealInstance                    &real_Instance,
										 const Traversal                        &kruhobot_traversal_A,
										 const Traversal                        &kruhobot_traversal_B,				       
										 KruhobotLocationConflicts_lower_vector &kruhobot_location_Conflicts,
										 KruhobotUlinearConflicts_lower_vector   &kruhobot_linear_Conflicts,
										 sInt_32                                &last_conflict_id,
										 bool                                    infinity) const
    {
	
	return resolve_KruhobotCollision<KruhobotLocationConflicts_lower_vector, KruhobotUlinearConflicts_lower_vector>(real_Instance,
														       kruhobot_traversal_A,
														       kruhobot_traversal_B,				       
														       kruhobot_location_Conflicts,
														       kruhobot_linear_Conflicts,
														       last_conflict_id,
														       infinity);
    }    


    sRealCBSBase::KruhobotAffection_pair sRealCBSBase::resolve_KruhobotCollision(const sRealInstance                            &real_Instance,
										 const Traversal                                &kruhobot_traversal_A,
										 const Traversal                                &kruhobot_traversal_B,				       
										 KruhobotLocationConflicts_lexicographic_vector &kruhobot_location_Conflicts,
										 KruhobotUlinearConflicts_lexicographic_vector   &kruhobot_linear_Conflicts,
										 sInt_32                                        &last_conflict_id,
										 bool                                            infinity) const
    {
	
	return resolve_KruhobotCollision<KruhobotLocationConflicts_lexicographic_vector, KruhobotUlinearConflicts_lexicographic_vector>(real_Instance,
																       kruhobot_traversal_A,
																       kruhobot_traversal_B,				       
																       kruhobot_location_Conflicts,
																       kruhobot_linear_Conflicts,
																       last_conflict_id,
																       infinity);
    }    


    template<class T1, class T2>
    sRealCBSBase::KruhobotAffection_pair sRealCBSBase::resolve_KruhobotCollision(const sRealInstance                    &real_Instance,
										 const Traversal                        &kruhobot_traversal_A,
										 const Traversal                        &kruhobot_traversal_B,				       
										 T1                                     &kruhobot_location_Conflicts,
										 T2                                     &kruhobot_linear_Conflicts,
										 sInt_32                                &last_conflict_id,
										 bool                                    infinity) const
    {
	if (kruhobot_traversal_A.m_u_loc_id == kruhobot_traversal_A.m_v_loc_id)
	{
	    if (kruhobot_traversal_B.m_u_loc_id == kruhobot_traversal_B.m_v_loc_id)
	    {
		return resolve_KruhobotCollision_location_X_location<T1, T2>(real_Instance,
									     kruhobot_traversal_A,
									     kruhobot_traversal_B,				       
									     kruhobot_location_Conflicts,
									     kruhobot_linear_Conflicts,
									     last_conflict_id,
								     infinity);
	    }
	    else
	    {
		return resolve_KruhobotCollision_location_X_linear<T1, T2>(real_Instance,
									   kruhobot_traversal_A,
									   kruhobot_traversal_B,				       
									   kruhobot_location_Conflicts,
									   kruhobot_linear_Conflicts,
									   last_conflict_id,
									   infinity);		
	    }
	}
	else	    
	{
	    if (kruhobot_traversal_B.m_u_loc_id == kruhobot_traversal_B.m_v_loc_id)
	    {
		return resolve_KruhobotCollision_linear_X_location<T1, T2>(real_Instance,
									   kruhobot_traversal_A,
									   kruhobot_traversal_B,				       
									   kruhobot_location_Conflicts,
									   kruhobot_linear_Conflicts,
									   last_conflict_id,
									   infinity);				
	    }
	    else
	    {
		return resolve_KruhobotCollision_linear_X_linear<T1, T2>(real_Instance,
									 kruhobot_traversal_A,
									 kruhobot_traversal_B,				       
									 kruhobot_location_Conflicts,
									 kruhobot_linear_Conflicts,
									 last_conflict_id,
									 infinity);		
	    }
	}
    }    


    template<class T1, class T2>
    sRealCBSBase::KruhobotAffection_pair sRealCBSBase::resolve_KruhobotCollision_location_X_location(const sRealInstance                    &real_Instance,
												     const Traversal                        &kruhobot_traversal_location_A,
												     const Traversal                        &kruhobot_traversal_location_B,				       
												     T1                                     &kruhobot_location_Conflicts,
												     T2                                     &sUNUSED(kruhobot_linear_Conflicts),
												     sInt_32                                &last_conflict_id,
												     bool                                    infinity) const
    {
	sASSERT(kruhobot_traversal_location_A.m_u_loc_id == kruhobot_traversal_location_A.m_v_loc_id);
	sASSERT(kruhobot_traversal_location_B.m_u_loc_id == kruhobot_traversal_location_B.m_v_loc_id);

	sInt_32 kruhobot_A_id = sABS(kruhobot_traversal_location_A.m_kruhobot_id);
	sInt_32 kruhobot_B_id = sABS(kruhobot_traversal_location_B.m_kruhobot_id);

	sDouble rA = real_Instance.m_Kruhobots[kruhobot_A_id].m_properties.m_radius;
	sDouble vA = real_Instance.m_Kruhobots[kruhobot_A_id].m_properties.m_linear_velo;
	
	sDouble rB = real_Instance.m_Kruhobots[kruhobot_B_id].m_properties.m_radius;
	sDouble vB = real_Instance.m_Kruhobots[kruhobot_B_id].m_properties.m_linear_velo;

	sDouble escape_time_A = (rA + rB) / vA;
	Interval avoid_interval_A(sMAX(0, kruhobot_traversal_location_B.m_interval.m_lower - escape_time_A), sMAX(0, kruhobot_traversal_location_B.m_interval.m_upper + escape_time_A));

	sInt_32 affection_A = 0;
	sInt_32 affection_B = 0;	

	if (avoid_interval_A.size() > s_DELTION)
	{
	    LocationConflict location_conflict_A(last_conflict_id++, kruhobot_traversal_location_A.m_u_loc_id, avoid_interval_A, infinity);
	    kruhobot_location_Conflicts[kruhobot_A_id][kruhobot_traversal_location_A.m_u_loc_id].insert(LocationConflicts_map::value_type(avoid_interval_A, location_conflict_A));
	    ++affection_A;
	}
	
	sDouble escape_time_B = (rA + rB) / vB;
	Interval avoid_interval_B(sMAX(0, kruhobot_traversal_location_A.m_interval.m_lower - escape_time_B), sMAX(0, kruhobot_traversal_location_A.m_interval.m_upper + escape_time_B));

	if (avoid_interval_B.size() > s_DELTION)
	{	
	    LocationConflict location_conflict_B(last_conflict_id++, kruhobot_traversal_location_B.m_u_loc_id, avoid_interval_B, infinity);
	    kruhobot_location_Conflicts[kruhobot_B_id][kruhobot_traversal_location_B.m_u_loc_id].insert(LocationConflicts_map::value_type(avoid_interval_B, location_conflict_B));
	    ++affection_B;	    
	}

	return KruhobotAffection_pair(affection_A, affection_B);
    }     


    template<class T1, class T2>    
    sRealCBSBase::KruhobotAffection_pair sRealCBSBase::resolve_KruhobotCollision_location_X_linear(const sRealInstance &real_Instance,
												   const Traversal     &kruhobot_traversal_A_location,
												   const Traversal     &kruhobot_traversal_B_linear,
												   T1                  &kruhobot_location_Conflicts,
												   T2                  &kruhobot_linear_Conflicts,
												   sInt_32             &last_conflict_id,
												   bool                 sUNUSED(infinity)) const
    {
	sASSERT(kruhobot_traversal_A_location.m_u_loc_id == kruhobot_traversal_A_location.m_v_loc_id);
	sASSERT(kruhobot_traversal_B_linear.m_u_loc_id != kruhobot_traversal_B_linear.m_v_loc_id);

	const s2DMap &map = *real_Instance.m_start_conjunction.m_Map;	

	sInt_32 kruhobot_A_id = sABS(kruhobot_traversal_A_location.m_kruhobot_id);
	sInt_32 kruhobot_B_id = sABS(kruhobot_traversal_B_linear.m_kruhobot_id);

	sDouble rA = real_Instance.m_Kruhobots[kruhobot_A_id].m_properties.m_radius;
	
	sDouble vB = real_Instance.m_Kruhobots[kruhobot_B_id].m_properties.m_linear_velo;
	sDouble rB = real_Instance.m_Kruhobots[kruhobot_B_id].m_properties.m_radius;
	
	sDouble x1A = map.m_Locations[kruhobot_traversal_A_location.m_u_loc_id].m_x;
	sDouble y1A = map.m_Locations[kruhobot_traversal_A_location.m_u_loc_id].m_y;
	
	sDouble x1B_ = map.m_Locations[kruhobot_traversal_B_linear.m_u_loc_id].m_x;
	sDouble x2B_ = map.m_Locations[kruhobot_traversal_B_linear.m_v_loc_id].m_x;
	sDouble y1B_ = map.m_Locations[kruhobot_traversal_B_linear.m_u_loc_id].m_y;
	sDouble y2B_ = map.m_Locations[kruhobot_traversal_B_linear.m_v_loc_id].m_y;

	sDouble dxB = x2B_ - x1B_;
	sDouble dyB = y2B_ - y1B_;

	sDouble d2B = dxB * dxB + dyB * dyB;
	sDouble dB = sqrt(d2B);		

	sDouble x1B = x1B_ + (vB * (-kruhobot_traversal_B_linear.m_interval.m_lower) * dxB) / dB;
	//sDouble x2B = x2B_ + (vB * (-kruhobot_traversal_B_linear.m_interval.m_lower) * dxB) / dB;
	sDouble y1B = y1B_ + (vB * (-kruhobot_traversal_B_linear.m_interval.m_lower) * dyB) / dB;
	//sDouble y2B = y2B_ + (vB * (-kruhobot_traversal_B_linear.m_interval.m_lower) * dyB) / dB;

	/*
	sDouble dxAB = x1B - x1A;
	sDouble dyAB = y1B - y1A;
	*/
	sDouble rAB = rA + rB;

	sDouble dxBA = x1B - x1A;
	sDouble dyBA = y1B - y1A;		
	
	sInt_32 affection_A = 0;
	sInt_32 affection_B = 0;	
	
	{
	    sDouble gamma = dxBA * dxBA + dyBA * dyBA - rAB * rAB;
	    sDouble beta = 2 * vB * (dxBA * dxB + dyBA * dyB) / dB;    
	    sDouble alpha = vB * vB;
	    
	    sDouble discriminant = beta * beta - 4 * alpha * gamma;
		
	    if (discriminant <= s_EPSILON)
	    {
		return KruhobotAffection_pair(0, 0);
	    }
	    else
	    {
		sDouble tau_1 = (-beta - sqrt(discriminant)) / (2 * alpha); 
		sDouble tau_2 = (-beta + sqrt(discriminant)) / (2 * alpha);

		Interval check_intersection_A = kruhobot_traversal_A_location.m_interval.intersect(Interval(tau_1, tau_2));
		Interval check_intersection_B = kruhobot_traversal_B_linear.m_interval.intersect(Interval(tau_1, tau_2));

		if (check_intersection_A.size() <= s_DELTION || check_intersection_B.size() <= s_DELTION)
		{
		    return KruhobotAffection_pair(0, 0);
		}
		else
		{
		    sDouble delta_tau_A = tau_2 - kruhobot_traversal_A_location.m_interval.m_lower;
		    sDouble delta_tau_B = kruhobot_traversal_A_location.m_interval.m_upper - tau_1;

		    Interval avoid_interval_A(kruhobot_traversal_A_location.m_interval.m_lower, kruhobot_traversal_A_location.m_interval.m_lower + delta_tau_A);
		    if (avoid_interval_A.size() > s_DELTION)
		    {
			Interval intersection_A = kruhobot_traversal_A_location.m_interval.intersect(avoid_interval_A);
			
			if (intersection_A.size() > s_DELTION)
			{
			    LocationConflict location_conflict_A(last_conflict_id++, kruhobot_traversal_A_location.m_u_loc_id, intersection_A);
			    kruhobot_location_Conflicts[kruhobot_A_id][kruhobot_traversal_A_location.m_u_loc_id].insert(LocationConflicts_map::value_type(intersection_A, location_conflict_A));
			    ++affection_A;
			}
		    }		    		    		    
		    
		    Interval avoid_interval_B(kruhobot_traversal_B_linear.m_interval.m_lower, kruhobot_traversal_B_linear.m_interval.m_lower + delta_tau_B);

		    if (avoid_interval_B.size() > s_DELTION)
		    {
			Interval intersection_B = kruhobot_traversal_B_linear.m_interval.intersect(avoid_interval_B);		       
			
			if (intersection_B.size() > s_DELTION)
			{
			    LinearConflict linear_conflict_B(last_conflict_id++, kruhobot_traversal_B_linear.m_u_loc_id, kruhobot_traversal_B_linear.m_v_loc_id, intersection_B);
			    kruhobot_linear_Conflicts[kruhobot_B_id][Uline(kruhobot_traversal_B_linear.m_u_loc_id, kruhobot_traversal_B_linear.m_v_loc_id)].insert(LinearConflicts_map::value_type(intersection_B, linear_conflict_B));
			    ++affection_B;
			}
		    }		    		    
		}
	    }	    
	}
	return KruhobotAffection_pair(affection_A, affection_B);	
    }


    template<class T1, class T2>    
    sRealCBSBase::KruhobotAffection_pair sRealCBSBase::resolve_KruhobotCollision_linear_X_location(const sRealInstance                    &real_Instance,
												   const Traversal                        &kruhobot_traversal_A_linear,
												   const Traversal                        &kruhobot_traversal_B_location,
												   T1                                     &kruhobot_location_Conflicts,
												   T2                                     &kruhobot_linear_Conflicts,
												   sInt_32                                &last_conflict_id,
												   bool                                    infinity) const
    {
	KruhobotAffection_pair kruhobot_affection = resolve_KruhobotCollision_location_X_linear<T1, T2>(real_Instance,
													kruhobot_traversal_B_location,
													kruhobot_traversal_A_linear,
													kruhobot_location_Conflicts,
													kruhobot_linear_Conflicts,
													last_conflict_id,
													infinity);

	return KruhobotAffection_pair(kruhobot_affection.second, kruhobot_affection.first);
    }


    template<class T1, class T2>
    sRealCBSBase::KruhobotAffection_pair sRealCBSBase::resolve_KruhobotCollision_linear_X_linear(const sRealInstance &real_Instance,
												 const Traversal     &kruhobot_traversal_A_linear,
												 const Traversal     &kruhobot_traversal_B_linear,
												 T1                  &sUNUSED(kruhobot_location_Conflicts),
												 T2                  &kruhobot_linear_Conflicts,
												 sInt_32             &last_conflict_id,
												 bool                 sUNUSED(infinity)) const
    {
	sASSERT(kruhobot_traversal_A_linear.m_u_loc_id != kruhobot_traversal_A_linear.m_v_loc_id);
	sASSERT(kruhobot_traversal_B_linear.m_u_loc_id != kruhobot_traversal_B_linear.m_v_loc_id);

	const s2DMap &map = *real_Instance.m_start_conjunction.m_Map;	

	sInt_32 kruhobot_A_id = sABS(kruhobot_traversal_A_linear.m_kruhobot_id);
	sInt_32 kruhobot_B_id = sABS(kruhobot_traversal_B_linear.m_kruhobot_id);

	sDouble vA = real_Instance.m_Kruhobots[kruhobot_A_id].m_properties.m_linear_velo;
	sDouble rA = real_Instance.m_Kruhobots[kruhobot_A_id].m_properties.m_radius;
	
	sDouble vB = real_Instance.m_Kruhobots[kruhobot_B_id].m_properties.m_linear_velo;
	sDouble rB = real_Instance.m_Kruhobots[kruhobot_B_id].m_properties.m_radius;

	sDouble x1A_ = map.m_Locations[kruhobot_traversal_A_linear.m_u_loc_id].m_x;
	sDouble x2A_ = map.m_Locations[kruhobot_traversal_A_linear.m_v_loc_id].m_x;
	sDouble y1A_ = map.m_Locations[kruhobot_traversal_A_linear.m_u_loc_id].m_y;
	sDouble y2A_ = map.m_Locations[kruhobot_traversal_A_linear.m_v_loc_id].m_y;
	
	sDouble x1B_ = map.m_Locations[kruhobot_traversal_B_linear.m_u_loc_id].m_x;
	sDouble x2B_ = map.m_Locations[kruhobot_traversal_B_linear.m_v_loc_id].m_x;
	sDouble y1B_ = map.m_Locations[kruhobot_traversal_B_linear.m_u_loc_id].m_y;
	sDouble y2B_ = map.m_Locations[kruhobot_traversal_B_linear.m_v_loc_id].m_y;

	sDouble dxA = x2A_ - x1A_;
	sDouble dyA = y2A_ - y1A_;

	sDouble dxB = x2B_ - x1B_;
	sDouble dyB = y2B_ - y1B_;

	sDouble d2A = dxA * dxA + dyA * dyA;
	sDouble dA = sqrt(d2A);

	sDouble d2B = dxB * dxB + dyB * dyB;
	sDouble dB = sqrt(d2B);

	sDouble x1A = x1A_ + (vA * (-kruhobot_traversal_A_linear.m_interval.m_lower) * dxA) / dA;
//	sDouble x2A = x2A_ + (vA * (-kruhobot_traversal_A_linear.m_interval.m_lower) * dxA) / dA;
	sDouble y1A = y1A_ + (vA * (-kruhobot_traversal_A_linear.m_interval.m_lower) * dyA) / dA;
//	sDouble y2A = y2A_ + (vA * (-kruhobot_traversal_A_linear.m_interval.m_lower) * dyA) / dA;

	sDouble x1B = x1B_ + (vB * (-kruhobot_traversal_B_linear.m_interval.m_lower) * dxB) / dB;
//	sDouble x2B = x2B_ + (vB * (-kruhobot_traversal_B_linear.m_interval.m_lower) * dxB) / dB;
	sDouble y1B = y1B_ + (vB * (-kruhobot_traversal_B_linear.m_interval.m_lower) * dyB) / dB;
//	sDouble y2B = y2B_ + (vB * (-kruhobot_traversal_B_linear.m_interval.m_lower) * dyB) / dB;	

	sDouble dxAB = x1A - x1B;
	sDouble dyAB = y1A - y1B;
	sDouble rAB = rA + rB;

	sInt_32 affection_A = 0;
	sInt_32 affection_B = 0;

	{
	    sDouble gamma = dxAB * dxAB + dyAB * dyAB - rAB * rAB;
	    sDouble beta_x = vA * dxA / dA - vB * dxB / dB;
	    sDouble beta_y = vA * dyA / dA - vB * dyB / dB;
	    sDouble beta = 2 * (dxAB * beta_x + dyAB * beta_y);
	    sDouble alpha = beta_x * beta_x + beta_y * beta_y;
	    
	    sDouble discriminant = beta * beta - 4 * alpha * gamma;
		
	    if (discriminant <= s_EPSILON)
	    {
		return KruhobotAffection_pair(0, 0);
	    }
	    else
	    {
		sDouble tau_1 = (-beta - sqrt(discriminant)) / (2 * alpha); 
		sDouble tau_2 = (-beta + sqrt(discriminant)) / (2 * alpha);

		Interval check_intersection_A = kruhobot_traversal_A_linear.m_interval.intersect(Interval(tau_1, tau_2));
		Interval check_intersection_B = kruhobot_traversal_B_linear.m_interval.intersect(Interval(tau_1, tau_2));

		if (check_intersection_A.size() <= s_DELTION || check_intersection_B.size() <= s_DELTION)
		{
		    return KruhobotAffection_pair(0, 0);
		}
	    }
	}
	    
	{
	    sDouble gamma_0 = dxAB * dxAB + dyAB * dyAB - rAB * rAB;
	    sDouble gamma_1 = 2 * (vA * (dxAB * dxA + dyAB * dyA)) / dA;	
	    sDouble gamma_2 = vA * vA;

	    sDouble beta_0 = 2 * ((vA * dxAB * dxA / dA) - ((vB * dxAB * dxB) / dB) + (vA * dyAB * dyA / dA) - ((vB * dyAB * dyB) / dB));
	    sDouble beta_x = (vA * dxA / dA) - (vB * dxB / dB);
	    sDouble beta_y = (vA * dyA / dA) - (vB * dyB / dB);	
	    sDouble beta_1 = 2 * ((vA * dxA * beta_x) / dA + (vA * dyA * beta_y) / dA);

	    sDouble alpha = beta_x * beta_x + beta_y * beta_y;
	    
	    sDouble lower_unsafe_A, upper_unsafe_A;
	
	    sDouble A = beta_1 * beta_1 - 4 * alpha * gamma_2;
	    sDouble B = 2 * beta_0 * beta_1 - 4 * alpha * gamma_1;
	    sDouble C = beta_0 * beta_0 - 4 * alpha * gamma_0;
	    
	    if (sABS(A) > s_EPSILON)
	    {
		sDouble D = B * B - 4 * A * C;

		if (D > s_EPSILON)
		{
		    sDouble t0_1 = (-B - sqrt(D)) / (2 * A); 
		    sDouble t0_2 = (-B + sqrt(D)) / (2 * A);

		    lower_unsafe_A = -t0_1 + kruhobot_traversal_A_linear.m_interval.m_lower;		    
		    upper_unsafe_A = -t0_2 + kruhobot_traversal_A_linear.m_interval.m_lower;
		    
		    Interval avoid_interval_A(sMAX(0, sMIN(lower_unsafe_A, upper_unsafe_A)), sMAX(0, sMAX(lower_unsafe_A, upper_unsafe_A)));

		    if (avoid_interval_A.size() > s_DELTION)
		    {
			Interval intersection_A = kruhobot_traversal_B_linear.m_interval.intersect(avoid_interval_A);
			
			if (intersection_A.size() > s_DELTION)
			{
			    LinearConflict linear_conflict_A(last_conflict_id++, kruhobot_traversal_A_linear.m_u_loc_id, kruhobot_traversal_A_linear.m_v_loc_id, intersection_A);
			    kruhobot_linear_Conflicts[kruhobot_A_id][Uline(kruhobot_traversal_A_linear.m_u_loc_id, kruhobot_traversal_A_linear.m_v_loc_id)].insert(LinearConflicts_map::value_type(intersection_A, linear_conflict_A));
			    ++affection_A;
			    /*
			    LinearConflict linear_conflict_B(last_conflict_id++, kruhobot_traversal_B_linear.m_u_loc_id, kruhobot_traversal_B_linear.m_v_loc_id, intersection_A);
			    kruhobot_linear_Conflicts[kruhobot_B_id][Uline(kruhobot_traversal_B_linear.m_u_loc_id, kruhobot_traversal_B_linear.m_v_loc_id)].insert(LinearConflicts_map::value_type(intersection_A, linear_conflict_B));
			    ++affection_B;
			    */
			}
		    }
		}
	    }
	    else
	    {
		/*
		sDouble dxAB2 = x1A_ - x2B_;
		sDouble dyAB2 = y1A_ - y2B_;

		sDouble gamma = dxAB2 * dxAB2 + dyAB2 * dyAB2 - rAB * rAB;
		sDouble beta = 2 * vA * (dxAB2 * dxA + dyAB2 * dyA) / dA;
		sDouble alpha = vA * vA;
		*/
		sDouble gamma = dxAB * dxAB + dyAB * dyAB - rAB * rAB;
		sDouble beta_x = vA * dxA / dA - vB * dxB / dB;
		sDouble beta_y = vA * dyA / dA - vB * dyB / dB;
		sDouble beta = 2 * (dxAB * beta_x + dyAB * beta_y);
		sDouble alpha = beta_x * beta_x + beta_y * beta_y;
		
		sDouble disc = beta * beta - 4 * alpha * gamma;

		if (disc > s_EPSILON)
		{
		    sDouble time0_1 = (-beta - sqrt(disc)) / (2 * alpha); 
		    sDouble time0_2 = (-beta + sqrt(disc)) / (2 * alpha);

		    //lower_unsafe_A = kruhobot_traversal_B_linear.m_interval.m_upper - time0_1;
		    //upper_unsafe_A = kruhobot_traversal_B_linear.m_interval.m_upper - time0_2;

		    lower_unsafe_A = time0_1;
		    upper_unsafe_A = time0_2;

		    Interval avoid_interval_A(sMAX(0, sMIN(lower_unsafe_A, upper_unsafe_A)), sMAX(0, sMAX(lower_unsafe_A, upper_unsafe_A)));

		    if (avoid_interval_A.size() > s_DELTION)
		    {
			Interval intersection_A = kruhobot_traversal_A_linear.m_interval.intersect(avoid_interval_A);
		    
			if (intersection_A.size() > s_DELTION)
			{
			    sDouble AA = vA * vA + vB * vB - 2 * vA * vB * (dxA * dxB + dyA * dyB) / (dA * dB);
			    sDouble BB0 = 2 * (vA * (dxAB * dxA + dyAB * dyA) / dA - vB * (dxAB * dxB + dyAB * dyB) / dB);
			    sDouble BB1 = 2 * vA * vA - 2 * vA * vB * (dxA * dxB + dyA * dyB) / (dA * dB);
			    sDouble CC0 = dxAB * dxAB + dyAB * dyAB - rAB * rAB;
			    sDouble CC1 = 2 * vA * (dxAB * dxA + dyAB * dyA) / dA;
			    sDouble CC2 = vA * vA;

			    sDouble tB2 = kruhobot_traversal_B_linear.m_interval.m_upper;

			    sDouble A = AA * CC2;
			    sDouble B = AA * BB1 * tB2 + AA * CC1;
			    sDouble C = AA * BB0 * tB2 + AA * CC0 + AA * AA * tB2 * tB2;

			    sDouble DD = B * B - 4 * A * C;

			    if (DD > s_EPSILON)
			    {
				sDouble T0_1 = (-B - sqrt(DD)) / (2 * A); 
				sDouble T0_2 = (-B + sqrt(DD)) / (2 * A);
				
				sDouble lower_unsafe_A_ = kruhobot_traversal_A_linear.m_interval.m_lower - sMIN(0, T0_1);
				sDouble upper_unsafe_A_ = kruhobot_traversal_A_linear.m_interval.m_lower - sMIN(0, T0_2);

				Interval avoid_interval_A_(sMAX(0, sMIN(lower_unsafe_A_, upper_unsafe_A_)), sMAX(0, sMAX(lower_unsafe_A_, upper_unsafe_A_)));
				
				if (avoid_interval_A_.size() > s_DELTION)
				{
				    Interval intersection_A_ = kruhobot_traversal_A_linear.m_interval.intersect(avoid_interval_A_);
				    
				    if (intersection_A_.size() > s_DELTION)
				    {
					LinearConflict linear_conflict_A(last_conflict_id++, kruhobot_traversal_A_linear.m_u_loc_id, kruhobot_traversal_A_linear.m_v_loc_id, intersection_A_);
					kruhobot_linear_Conflicts[kruhobot_A_id][Uline(kruhobot_traversal_A_linear.m_u_loc_id, kruhobot_traversal_A_linear.m_v_loc_id)].insert(LinearConflicts_map::value_type(intersection_A_, linear_conflict_A));
					++affection_A;
				    }
				}
			    }
			    else
			    {
				LinearConflict linear_conflict_A(last_conflict_id++, kruhobot_traversal_A_linear.m_u_loc_id, kruhobot_traversal_A_linear.m_v_loc_id, kruhobot_traversal_A_linear.m_interval);
				kruhobot_linear_Conflicts[kruhobot_A_id][Uline(kruhobot_traversal_A_linear.m_u_loc_id, kruhobot_traversal_A_linear.m_v_loc_id)].insert(LinearConflicts_map::value_type(kruhobot_traversal_A_linear.m_interval, linear_conflict_A));
				++affection_A;				
			    }
			}
		    }		    
		}		
	    }
	}

	sDouble dxBA = x1B - x1A;
	sDouble dyBA = y1B - y1A;

	{
	    sDouble gamma_0 = dxBA * dxBA + dyBA * dyBA - rAB * rAB;
	    sDouble gamma_1 = 2 * (vB * (dxBA * dxB + dyBA * dyB)) / dB;	
	    sDouble gamma_2 = vB * vB;

	    sDouble beta_0 = 2 * ((vB * dxBA * dxB / dB) - ((vA * dxBA * dxA) / dA) + (vB * dyBA * dyB / dB) - ((vA * dyBA * dyA) / dA));
	    sDouble beta_x = (vB * dxB / dB) - (vA * dxA / dA);
	    sDouble beta_y = (vB * dyB / dB) - (vA * dyA / dA);
	    sDouble beta_1 = 2 * ((vB * dxB * beta_x) / dB + (vB * dyB * beta_y) / dB);	    

	    sDouble alpha = beta_x * beta_x + beta_y * beta_y;
	    
	    sDouble lower_unsafe_B, upper_unsafe_B;
	
	    sDouble A = beta_1 * beta_1 - 4 * alpha * gamma_2;
	    sDouble B = 2 * beta_0 * beta_1 - 4 * alpha * gamma_1;
	    sDouble C = beta_0 * beta_0 - 4 * alpha * gamma_0;

	    if (sABS(A) > s_EPSILON)
	    {
		sDouble D = B * B - 4 * A * C;

		if (D > s_EPSILON)
		{
		    sDouble t0_1 = (-B - sqrt(D)) / (2 * A); 
		    sDouble t0_2 = (-B + sqrt(D)) / (2 * A);

		    lower_unsafe_B = -t0_1 + kruhobot_traversal_B_linear.m_interval.m_lower;
		    upper_unsafe_B = -t0_2 + kruhobot_traversal_B_linear.m_interval.m_lower;

		    Interval avoid_interval_B(sMAX(0, sMIN(lower_unsafe_B, upper_unsafe_B)), sMAX(0, sMAX(lower_unsafe_B, upper_unsafe_B)));

		    if (avoid_interval_B.size() > s_DELTION)
		    {
			Interval intersection_B = kruhobot_traversal_A_linear.m_interval.intersect(avoid_interval_B);
		    
			if (intersection_B.size() > s_DELTION)
			{
			    LinearConflict linear_conflict_B(last_conflict_id++, kruhobot_traversal_B_linear.m_u_loc_id, kruhobot_traversal_B_linear.m_v_loc_id, intersection_B);
			    kruhobot_linear_Conflicts[kruhobot_B_id][Uline(kruhobot_traversal_B_linear.m_u_loc_id, kruhobot_traversal_B_linear.m_v_loc_id)].insert(LinearConflicts_map::value_type(intersection_B, linear_conflict_B));
			    ++affection_B;
/*
			    LinearConflict linear_conflict_A(last_conflict_id++, kruhobot_traversal_A_linear.m_u_loc_id, kruhobot_traversal_A_linear.m_v_loc_id, intersection_B);
			    kruhobot_linear_Conflicts[kruhobot_A_id][Uline(kruhobot_traversal_A_linear.m_u_loc_id, kruhobot_traversal_A_linear.m_v_loc_id)].insert(LinearConflicts_map::value_type(intersection_B, linear_conflict_A));
			    ++affection_A;
*/
			}
		    }
		}
	    }
	    else
	    {
		/*
		sDouble dxBA2 = x1B_ - x2A_;
		sDouble dyBA2 = y1B_ - y2A_;

		sDouble gamma = dxBA2 * dxBA2 + dyBA2 * dyBA2 - rAB * rAB;
		sDouble beta = 2 * vB * (dxBA2 * dxB + dyBA2 * dyB) / dB;
		sDouble alpha = vB * vB;
		*/
		/*
		sDouble gamma = dxBA * dxBA + dyBA * dyBA - rAB * rAB;
		sDouble beta_x = vA * dxA / dA - vB * dxB / dB;
		sDouble beta_y = vA * dyA / dA - vB * dyB / dB;
		sDouble beta = 2 * (beta_x + beta_y);
		sDouble alpha = beta_x * beta_x + beta_y * beta_y;		
		*/
		sDouble gamma = dxBA * dxBA + dyBA * dyBA - rAB * rAB;
		sDouble beta_x = vB * dxB / dB - vA * dxA / dA;
		sDouble beta_y = vB * dyB / dB - vA * dyA / dA;
		sDouble beta = 2 * (dxBA * beta_x + dyBA * beta_y);
		sDouble alpha = beta_x * beta_x + beta_y * beta_y;		

		sDouble disc = beta * beta - 4 * alpha * gamma;

		if (disc > s_EPSILON)
		{
		    sDouble time0_1 = (-beta - sqrt(disc)) / (2 * alpha); 
		    sDouble time0_2 = (-beta + sqrt(disc)) / (2 * alpha);

		    lower_unsafe_B = time0_1;
		    upper_unsafe_B = time0_2;		    

		    Interval avoid_interval_B(sMAX(0, sMIN(lower_unsafe_B, upper_unsafe_B)), sMAX(0, sMAX(lower_unsafe_B, upper_unsafe_B)));

		    if (avoid_interval_B.size() > s_DELTION)
		    {
			Interval intersection_B = kruhobot_traversal_B_linear.m_interval.intersect(avoid_interval_B);
		    
			if (intersection_B.size() > s_DELTION)
			{
			    sDouble AA = vA * vA + vB * vB - 2 * vA * vB * (dxA * dxB + dyA * dyB) / (dA * dB);
			    sDouble BB0 = 2 * (vB * (dxBA * dxB + dyBA * dyB) / dB - vA * (dxBA * dxA + dyBA * dyA) / dA);
			    sDouble BB1 = 2 * vB * vB - 2 * vA * vB * (dxA * dxB + dyA * dyB) / (dA * dB);			    
			    sDouble CC0 = dxBA * dxBA + dyBA * dyBA - rAB * rAB;			    
			    sDouble CC1 = 2 * vB * (dxBA * dxB + dyBA * dyB) / dB;
			    sDouble CC2 = vB * vB;

			    sDouble tB2 = kruhobot_traversal_A_linear.m_interval.m_upper;

			    sDouble A = AA * CC2;
			    sDouble B = AA * BB1 * tB2 + AA * CC1;
			    sDouble C = AA * BB0 * tB2 + AA * CC0 + AA * AA * tB2 * tB2;

			    sDouble DD = B * B - 4 * A * C;

			    if (DD > s_EPSILON)
			    {
				sDouble T0_1 = (-B - sqrt(DD)) / (2 * A); 
				sDouble T0_2 = (-B + sqrt(DD)) / (2 * A);
			    
				sDouble lower_unsafe_B_ = kruhobot_traversal_B_linear.m_interval.m_lower - sMIN(0, T0_1);
				sDouble upper_unsafe_B_ = kruhobot_traversal_B_linear.m_interval.m_lower - sMIN(0, T0_2);
				
				Interval avoid_interval_B_(sMAX(0, sMIN(lower_unsafe_B_, upper_unsafe_B_)), sMAX(0, sMAX(lower_unsafe_B_, upper_unsafe_B_)));
				
				if (avoid_interval_B_.size() > s_DELTION)
				{
				    Interval intersection_B_ = kruhobot_traversal_A_linear.m_interval.intersect(avoid_interval_B_);
				    
				    if (intersection_B_.size() > s_DELTION)
				    {
					LinearConflict linear_conflict_B(last_conflict_id++, kruhobot_traversal_B_linear.m_u_loc_id, kruhobot_traversal_B_linear.m_v_loc_id, intersection_B_);
					kruhobot_linear_Conflicts[kruhobot_B_id][Uline(kruhobot_traversal_B_linear.m_u_loc_id, kruhobot_traversal_B_linear.m_v_loc_id)].insert(LinearConflicts_map::value_type(intersection_B_, linear_conflict_B));
					++affection_B;			    				    
				    }
				}
			    }
			    else
			    {
				LinearConflict linear_conflict_B(last_conflict_id++, kruhobot_traversal_B_linear.m_u_loc_id, kruhobot_traversal_B_linear.m_v_loc_id, kruhobot_traversal_B_linear.m_interval);
				kruhobot_linear_Conflicts[kruhobot_B_id][Uline(kruhobot_traversal_B_linear.m_u_loc_id, kruhobot_traversal_B_linear.m_v_loc_id)].insert(LinearConflicts_map::value_type(kruhobot_traversal_B_linear.m_interval, linear_conflict_B));
				++affection_B;			    				    				
			    }
			}
		    }		    
		}		
	    }
	}
	return KruhobotAffection_pair(affection_A, affection_B);
    } 

    
/*----------------------------------------------------------------------------*/
    
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


/*----------------------------------------------------------------------------*/
    
    void sRealCBSBase::introduce_KruhobotConflict(const Traversal                  &kruhobot_traversal,
						  KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
						  KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts,
						  sInt_32                          &last_conflict_id,						  
						  bool                              infinity) const
    {
	sInt_32 kruhobot_id = sABS(kruhobot_traversal.m_kruhobot_id);

	if (kruhobot_traversal.m_u_loc_id != kruhobot_traversal.m_v_loc_id)
	{
	    LinearConflict linear_conflict(last_conflict_id++, kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_v_loc_id, kruhobot_traversal.m_interval);
	    kruhobot_linear_Conflicts[kruhobot_id][Uline(kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_v_loc_id)].insert(LinearConflicts_map::value_type(kruhobot_traversal.m_interval, linear_conflict));
	}
	else
	{
	    LocationConflict location_conflict(last_conflict_id++, kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_interval, infinity);
	    kruhobot_location_Conflicts[kruhobot_id][kruhobot_traversal.m_u_loc_id].insert(LocationConflicts_map::value_type(kruhobot_traversal.m_interval, location_conflict));
	}
    }

    
    void sRealCBSBase::introduce_KruhobotConflict(const Traversal                                &kruhobot_traversal,
						  KruhobotLocationConflicts_lexicographic_vector &kruhobot_location_Conflicts,
						  KruhobotLinearConflicts_lexicographic_vector   &kruhobot_linear_Conflicts,
						  sInt_32                                        &last_conflict_id,						  
						  bool                                            infinity) const
    {
	sInt_32 kruhobot_id = sABS(kruhobot_traversal.m_kruhobot_id);

	if (kruhobot_traversal.m_u_loc_id != kruhobot_traversal.m_v_loc_id)
	{
	    LinearConflict linear_conflict(last_conflict_id++, kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_v_loc_id, kruhobot_traversal.m_interval);
	    kruhobot_linear_Conflicts[kruhobot_id][Uline(kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_v_loc_id)].insert(LinearConflicts_lexicographic_map::value_type(kruhobot_traversal.m_interval, linear_conflict));
	}
	else
	{
	    LocationConflict location_conflict(last_conflict_id++, kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_interval, infinity);
	    kruhobot_location_Conflicts[kruhobot_id][kruhobot_traversal.m_u_loc_id].insert(LocationConflicts_lexicographic_map::value_type(kruhobot_traversal.m_interval, location_conflict));
	}	
    }

    
    void sRealCBSBase::introduce_KruhobotConflict(const Traversal                        &kruhobot_traversal,
						  KruhobotLocationConflicts_lower_vector &kruhobot_location_Conflicts,
						  KruhobotLinearConflicts_lower_vector   &kruhobot_linear_Conflicts,
						  sInt_32                                &last_conflict_id,						  
						  bool                                    infinity) const
    {
	sInt_32 kruhobot_id = sABS(kruhobot_traversal.m_kruhobot_id);

	if (kruhobot_traversal.m_u_loc_id != kruhobot_traversal.m_v_loc_id)
	{
	    LinearConflict linear_conflict(last_conflict_id++, kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_v_loc_id, kruhobot_traversal.m_interval);
	    kruhobot_linear_Conflicts[kruhobot_id][Uline(kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_v_loc_id)].insert(LinearConflicts_lower_map::value_type(kruhobot_traversal.m_interval, linear_conflict));
	}
	else
	{
	    LocationConflict location_conflict(last_conflict_id++, kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_interval, infinity);
	    kruhobot_location_Conflicts[kruhobot_id][kruhobot_traversal.m_u_loc_id].insert(LocationConflicts_lower_map::value_type(kruhobot_traversal.m_interval, location_conflict));
	}	
    }

    
    void sRealCBSBase::introduce_KruhobotConflict(const Traversal                        &kruhobot_traversal,
						  KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
						  KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
						  sInt_32                                &last_conflict_id,						  
						  bool                                    infinity) const
    {
	sInt_32 kruhobot_id = sABS(kruhobot_traversal.m_kruhobot_id);

	if (kruhobot_traversal.m_u_loc_id != kruhobot_traversal.m_v_loc_id)
	{
	    LinearConflict linear_conflict(last_conflict_id++, kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_v_loc_id, kruhobot_traversal.m_interval);
	    kruhobot_linear_Conflicts[kruhobot_id][Uline(kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_v_loc_id)].insert(LinearConflicts_upper_map::value_type(kruhobot_traversal.m_interval, linear_conflict));
	}
	else
	{
	    LocationConflict location_conflict(last_conflict_id++, kruhobot_traversal.m_u_loc_id, kruhobot_traversal.m_interval, infinity);
	    kruhobot_location_Conflicts[kruhobot_id][kruhobot_traversal.m_u_loc_id].insert(LocationConflicts_upper_map::value_type(kruhobot_traversal.m_interval, location_conflict));
	}	
    }

    
/*----------------------------------------------------------------------------*/
   
    sDouble sRealCBSBase::calc_KruhobotCollisionImportance(const Traversal            &traversal_A,
							   const Traversal            &traversal_B,
							   LocationCooccupations_umap &location_Cooccupations,
							   UlinearCooccupations_map    &linear_Cooccupations) const
    {
	sDouble collision_importance =   calc_KruhobotCollisionImportance(traversal_A, location_Cooccupations, linear_Cooccupations)
	                               + calc_KruhobotCollisionImportance(traversal_B, location_Cooccupations, linear_Cooccupations);
	
	return collision_importance;
    }


    sDouble sRealCBSBase::calc_KruhobotCollisionImportance(const Traversal            &traversal,
							   LocationCooccupations_umap &location_Cooccupations,
							   UlinearCooccupations_map    &linear_Cooccupations) const
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

	UlinearCooccupations_map::const_iterator linear_Coop = linear_Cooccupations.find(Uline(traversal.m_u_loc_id, traversal.m_v_loc_id));
	
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


    sResult sRealCBSBase::to_File(const sString &filename, const KruhobotSchedules_vector &kruhobot_Schedules, const sString &indent)
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sREAL_CBS_OPEN_ERROR;
	}
	
	to_Stream(fw, kruhobot_Schedules, indent);
	fclose(fw);

	return sRESULT_SUCCESS;	
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


    sDouble sRealCBS::find_ShortestNonconflictingSchedules(const sRealInstance      &real_Instance,
							   KruhobotSchedules_vector &kruhobot_Schedules,
							   sDouble                   cost_limit)
    {
	return find_ShortestNonconflictingSchedules(real_Instance, kruhobot_Schedules, cost_limit, 0.0);
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
	
	sDouble start_time = sStatistics::get_CPU_Seconds();
	
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
		    return -2.0;
		}
	    }
	}

	return -1.0;	
    }


/*----------------------------------------------------------------------------*/
    
    sDouble sRealCBS::find_ShortestNonconflictingSchedules_smart(sRealSolution &real_Solution, sDouble cost_limit)
    {
	return find_ShortestNonconflictingSchedules_smart(*m_real_Instance, real_Solution, cost_limit);	
    }

  
    sDouble sRealCBS::find_ShortestNonconflictingSchedules_smart(const sRealInstance &real_Instance,
								   sRealSolution       &sUNUSED(real_Solution),
								   sDouble              cost_limit)
    {
	sInt_32 cost;
	KruhobotSchedules_vector kruhobot_Schedules;

	if ((cost = find_ShortestNonconflictingSchedules_smart(real_Instance, kruhobot_Schedules, cost_limit)) < 0)
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


    sDouble sRealCBS::find_ShortestNonconflictingSchedules_smart(KruhobotSchedules_vector &kruhobot_Schedules, sDouble cost_limit)
    {
	return find_ShortestNonconflictingSchedules_smart(*m_real_Instance, kruhobot_Schedules, cost_limit);	
    }


    sDouble sRealCBS::find_ShortestNonconflictingSchedules_smart(const sRealInstance      &real_Instance,
								   KruhobotSchedules_vector &kruhobot_Schedules,
								   sDouble                   cost_limit)
    {
	return find_ShortestNonconflictingSchedules_smart(real_Instance, kruhobot_Schedules, cost_limit, 0.0);
    }    

    
    sDouble sRealCBS::find_ShortestNonconflictingSchedules_smart(KruhobotSchedules_vector &kruhobot_Schedules, sDouble cost_limit, sDouble extra_cost)
    {
	return find_ShortestNonconflictingSchedules_smart(*m_real_Instance, kruhobot_Schedules, cost_limit, extra_cost);	
    }

    
    sDouble sRealCBS::find_ShortestNonconflictingSchedules_smart(const sRealInstance      &real_Instance,
								   KruhobotSchedules_vector &kruhobot_Schedules,
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

	    KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	    KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;

	    kruhobot_location_Conflicts.resize(N_kruhobots + 1);
	    kruhobot_linear_Conflicts.resize(N_kruhobots + 1);
	    kruhobot_Schedules.resize(N_kruhobots + 1);  
	    
	    if ((solution_cost = find_NonconflictingSchedules_smart(real_Instance,
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
		    return -2.0;
		}
	    }
	}

	return -1.0;	
    }


/*----------------------------------------------------------------------------*/
    
    sDouble sRealCBS::find_ShortestNonconflictingSchedules_strong(sRealSolution &real_Solution, sDouble cost_limit)
    {
	return find_ShortestNonconflictingSchedules_strong(*m_real_Instance, real_Solution, cost_limit);	
    }

  
    sDouble sRealCBS::find_ShortestNonconflictingSchedules_strong(const sRealInstance &real_Instance,
								   sRealSolution       &sUNUSED(real_Solution),
								   sDouble              cost_limit)
    {
	sInt_32 cost;
	KruhobotSchedules_vector kruhobot_Schedules;

	if ((cost = find_ShortestNonconflictingSchedules_strong(real_Instance, kruhobot_Schedules, cost_limit)) < 0)
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


    sDouble sRealCBS::find_ShortestNonconflictingSchedules_strong(KruhobotSchedules_vector &kruhobot_Schedules, sDouble cost_limit)
    {
	return find_ShortestNonconflictingSchedules_strong(*m_real_Instance, kruhobot_Schedules, cost_limit);	
    }


    sDouble sRealCBS::find_ShortestNonconflictingSchedules_strong(const sRealInstance      &real_Instance,
								  KruhobotSchedules_vector &kruhobot_Schedules,
								  sDouble                   cost_limit)
    {
	return find_ShortestNonconflictingSchedules_strong(real_Instance, kruhobot_Schedules, cost_limit, 0.0);
    }    

    
    sDouble sRealCBS::find_ShortestNonconflictingSchedules_strong(KruhobotSchedules_vector &kruhobot_Schedules, sDouble cost_limit, sDouble extra_cost)
    {
	return find_ShortestNonconflictingSchedules_strong(*m_real_Instance, kruhobot_Schedules, cost_limit, extra_cost);	
    }

    
    sDouble sRealCBS::find_ShortestNonconflictingSchedules_strong(const sRealInstance      &real_Instance,
								  KruhobotSchedules_vector &kruhobot_Schedules,
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

	    KruhobotLocationConflicts_vector kruhobot_location_Conflicts;
	    KruhobotLinearConflicts_vector kruhobot_linear_Conflicts;

	    kruhobot_location_Conflicts.resize(N_kruhobots + 1);
	    kruhobot_linear_Conflicts.resize(N_kruhobots + 1);
	    kruhobot_Schedules.resize(N_kruhobots + 1);  
	    
	    if ((solution_cost = find_NonconflictingSchedules_strong(real_Instance,
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
		    return -2.0;
		}
	    }
	}

	return -1.0;	
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


    sDouble sRealCBS::find_NonconflictingSchedules_smart(const sRealInstance              &real_Instance,
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
	    if ((kruhobot_cost = find_KruhobotNonconflictingSchedule_smart(real_Instance.m_Kruhobots[kruhobot_id],
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
		if (update_NonconflictingSchedule_smart(update_kruhobot_id,
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


    sDouble sRealCBS::find_NonconflictingSchedules_strong(const sRealInstance              &real_Instance,
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
	    if ((kruhobot_cost = find_KruhobotNonconflictingSchedule_strong(real_Instance.m_Kruhobots[kruhobot_id],
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
		if (update_NonconflictingSchedule_strong(update_kruhobot_id,
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


    sDouble sRealCBS::update_NonconflictingSchedule_smart(sInt_32                           upd_kruhobot_id,
							  const sRealInstance              &real_Instance,
							  KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
							  KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts,
							  KruhobotSchedules_vector         &kruhobot_Schedules,
							  sDouble                           makespan_bound,
							  sDouble                           cost_limit,
							  sDouble                           extra_cost) const
    {
	sDouble cummulative;

	cummulative = find_KruhobotNonconflictingSchedule_smart(real_Instance.m_Kruhobots[upd_kruhobot_id],
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


    sDouble sRealCBS::update_NonconflictingSchedule_strong(sInt_32                           upd_kruhobot_id,
							   const sRealInstance              &real_Instance,
							   KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
							   KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts,
							   KruhobotSchedules_vector         &kruhobot_Schedules,
							   sDouble                           makespan_bound,
							   sDouble                           cost_limit,
							   sDouble                           extra_cost) const
    {
	sDouble cummulative;
	
	cummulative = find_KruhobotNonconflictingSchedule_strong(real_Instance.m_Kruhobots[upd_kruhobot_id],
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

	Transition initial_transition(transition_Store.size(), 0.0, 0.0, 0.0, source_loc_id, -1);	
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
		if (front_transition.m_time + (map.m_Distances[front_transition.m_location_id][sink_loc_id] / kruhobot.m_properties.m_linear_velo) > cost_limit)
		{
		    printf("cost hit: %d\n", cost_hit);
		    ++cost_hit;
		}
	    }
	    #endif
	    if (front_transition.m_time + (map.m_straight_Distances[sink_loc_id][front_transition.m_location_id] / kruhobot.m_properties.m_linear_velo) <= cost_limit)
//	    if (front_transition.m_time + map.m_straight_Distances[sink_loc_id][front_transition.m_location_id] <= makespan_bound)	    
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
			sDouble transition_finish_makespan = front_transition.m_makespan + transition_delta_time;			

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
							
				Transition neighbor_transition(transition_Store.size(), transition_finish_time, transition_finish_cost, transition_finish_makespan, neighbor_location_id, front_transition.m_trans_id);
				transition_Store.push_back(neighbor_transition);
				
				next_explored_Transitions->insert(neighbor_location_id);
//				transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_time, neighbor_transition));
//				transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_cost, neighbor_transition));
				transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_makespan, neighbor_transition));				
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
				Transition wait_transition(transition_Store.size(), wait_finish_time, front_transition.m_cost + wait_cost, wait_finish_time, front_transition.m_location_id, front_transition.m_trans_id);
				transition_Store.push_back(wait_transition);
				
				wait_explored_Transitions->insert(front_transition.m_location_id);
//				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_cost, wait_transition));				
				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_makespan, wait_transition));
			    }
			}
		    }
		}				
	    }
	    transition_Queue.erase(transition_Queue.begin());
	}
	return -1.0;	
    }


    sDouble sRealCBS::find_KruhobotNonconflictingSchedule_smart(const sKruhobot               &kruhobot,
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

	Transition initial_transition(transition_Store.size(), 0.0, 0.0, 0.0, source_loc_id, -1);	
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
		if (front_transition.m_time + (map.m_Distances[front_transition.m_location_id][sink_loc_id] / / kruhobot.m_properties.m_linear_velo) > cost_limit)
		{
		    printf("cost hit: %d\n", cost_hit);
		    ++cost_hit;
		}
	    }
	    #endif
//	    if (front_transition.m_time + map.m_straight_Distances[front_transition.m_location_id][sink_loc_id] <= makespan_bound + s_EPSILON)	    
	    if (front_transition.m_time + (map.m_straight_Distances[sink_loc_id][front_transition.m_location_id] / kruhobot.m_properties.m_linear_velo) <= cost_limit + s_EPSILON)
	    {

		const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[front_transition.m_location_id].m_Neighbors;
		for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
		{
		    sInt_32 neighbor_location_id = (*neighbor)->m_target->m_id;		    
/*		
		for (s2DMap::Locations_vector::const_iterator location = map.m_Locations.begin(); location != map.m_Locations.end(); ++location)
		{
		    sInt_32 neighbor_location_id = location->m_id;

		    if (neighbor_location_id != front_transition.m_location_id && map.m_Network.is_Adjacent(neighbor_location_id, front_transition.m_location_id))
*/
		    
		    {
			sDouble transition_distance = map.m_straight_Distances[front_transition.m_location_id][neighbor_location_id];
			sDouble transition_delta_time = transition_distance / kruhobot.m_properties.m_linear_velo;
			sDouble transition_finish_time = front_transition.m_time + transition_delta_time;
			sDouble transition_finish_cost = front_transition.m_cost + transition_delta_time;
			sDouble transition_finish_makespan = front_transition.m_makespan + transition_delta_time;

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
							
				Transition neighbor_transition(transition_Store.size(), transition_finish_time, transition_finish_cost, transition_finish_makespan, neighbor_location_id, front_transition.m_trans_id);
				transition_Store.push_back(neighbor_transition);
				
				next_explored_Transitions->insert(neighbor_location_id);
//				transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_time, neighbor_transition));
//				transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_cost, neighbor_transition));
				transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_makespan, neighbor_transition));				
			    }
			}
		    }
		}
		{ /* wait action */
		    sDouble wait_location_finish_time = -1.0;
		    sDouble wait_linear_finish_time = -1.0;

		    const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[front_transition.m_location_id].m_Neighbors;
		    for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
		    {
			sInt_32 neighbor_location_id = (*neighbor)->m_target->m_id;		    
/*		    
		    for (s2DMap::Locations_vector::const_iterator interacting_loc = map.m_Locations.begin(); interacting_loc != map.m_Locations.end(); ++interacting_loc)
		    {
			sInt_32 neighbor_location_id = interacting_loc->m_id;
			
			if (neighbor_location_id != front_transition.m_location_id && map.m_Network.is_Adjacent(neighbor_location_id, front_transition.m_location_id))
*/
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
				Transition wait_transition(transition_Store.size(), wait_finish_time, front_transition.m_cost + wait_cost, wait_finish_time, front_transition.m_location_id, front_transition.m_trans_id);
				transition_Store.push_back(wait_transition);
				
				wait_explored_Transitions->insert(front_transition.m_location_id);
//				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_time, wait_transition));
//				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_cost, wait_transition));
				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_makespan, wait_transition));
			    }
			}
		    }
		}				
	    }
	    transition_Queue.erase(transition_Queue.begin());
	}
	return -1.0;	
    }


    sDouble sRealCBS::find_KruhobotNonconflictingSchedule_strong(const sKruhobot               &kruhobot,
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

	Transition initial_transition(transition_Store.size(), 0.0, 0.0, 0.0, source_loc_id, -1);	
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
		if (front_transition.m_time + (map.m_Distances[front_transition.m_location_id][sink_loc_id] / kruhobot.m_properties.m_linear_velo) > cost_limit)
		{
		    printf("cost hit: %d\n", cost_hit);
		    ++cost_hit;
		}
	    }
	    #endif
//	    if (front_transition.m_time + map.m_straight_Distances[front_transition.m_location_id][sink_loc_id] <= makespan_bound + s_EPSILON)
	    if (front_transition.m_time + (map.m_shortest_Distances[sink_loc_id][front_transition.m_location_id] / kruhobot.m_properties.m_linear_velo) <= cost_limit + s_EPSILON)
	    {

		const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[front_transition.m_location_id].m_Neighbors;
		for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
		{
		    sInt_32 neighbor_location_id = (*neighbor)->m_target->m_id;		    
/*		
		for (s2DMap::Locations_vector::const_iterator location = map.m_Locations.begin(); location != map.m_Locations.end(); ++location)
		{
		    sInt_32 neighbor_location_id = location->m_id;

		    if (neighbor_location_id != front_transition.m_location_id && map.m_Network.is_Adjacent(neighbor_location_id, front_transition.m_location_id))
*/
		    
		    {
			sDouble transition_distance = map.m_straight_Distances[front_transition.m_location_id][neighbor_location_id];
			sDouble transition_delta_time = transition_distance / kruhobot.m_properties.m_linear_velo;
			sDouble transition_finish_time = front_transition.m_time + transition_delta_time;
			sDouble transition_finish_cost = front_transition.m_cost + transition_delta_time;
			sDouble transition_finish_makespan = front_transition.m_makespan + transition_delta_time;			

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
							
				Transition neighbor_transition(transition_Store.size(), transition_finish_time, transition_finish_cost, transition_finish_makespan, neighbor_location_id, front_transition.m_trans_id);
				transition_Store.push_back(neighbor_transition);
				
				next_explored_Transitions->insert(neighbor_location_id);
//				transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_time, neighbor_transition));
				transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_makespan, neighbor_transition));
			    }
			}
		    }
		}
		{ /* wait action */
		    sDouble wait_location_finish_time = -1.0;
		    sDouble wait_linear_finish_time = -1.0;

		    const sVertex::Neighbors_list &Neighbors = map.m_Network.m_Vertices[front_transition.m_location_id].m_Neighbors;
		    for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
		    {
			sInt_32 neighbor_location_id = (*neighbor)->m_target->m_id;		    
/*		    
		    for (s2DMap::Locations_vector::const_iterator interacting_loc = map.m_Locations.begin(); interacting_loc != map.m_Locations.end(); ++interacting_loc)
		    {
			sInt_32 neighbor_location_id = interacting_loc->m_id;
			
			if (neighbor_location_id != front_transition.m_location_id && map.m_Network.is_Adjacent(neighbor_location_id, front_transition.m_location_id))
*/
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
				Transition wait_transition(transition_Store.size(), wait_finish_time, front_transition.m_cost + wait_cost, wait_finish_time, front_transition.m_location_id, front_transition.m_trans_id);
				transition_Store.push_back(wait_transition);
				
				wait_explored_Transitions->insert(front_transition.m_location_id);
//				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_time, wait_transition));
//				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_cost, wait_transition));
				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_makespan, wait_transition));
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

	Transition initial_transition(transition_Store.size(), 0.0, 0.0, 0.0, source_loc_id, -1);	
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

	    if (front_transition.m_time + (map.m_straight_Distances[sink_loc_id][front_transition.m_location_id] / kruhobot.m_properties.m_linear_velo) <= cost_limit + extra_cost)
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
			sDouble transition_finish_makespan = front_transition.m_makespan + transition_delta_time;

			LocationIDs_uset *next_explored_Transitions;
			
			if (front_transition.m_time <= makespan_bound + s_EPSILON)
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
			    Transition neighbor_transition(transition_Store.size(), transition_finish_time, transition_finish_cost, transition_finish_makespan, neighbor_location_id, front_transition.m_trans_id);
			    transition_Store.push_back(neighbor_transition);
			    ++generated;
			    
			    next_explored_Transitions->insert(neighbor_location_id);
//			    transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_time, neighbor_transition));
//			    transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_cost, neighbor_transition));
			    transition_Queue.insert(Transitions_mmap::value_type(neighbor_transition.m_makespan, neighbor_transition));			    
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
				Transition wait_transition(transition_Store.size(), wait_finish_time, front_transition.m_cost + wait_cost, wait_finish_time, front_transition.m_location_id, front_transition.m_trans_id);
				transition_Store.push_back(wait_transition);
				++generated;				
				
				wait_explored_Transitions->insert(front_transition.m_location_id);
//				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_time, wait_transition));
//				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_cost, wait_transition));
				transition_Queue.insert(Transitions_mmap::value_type(wait_transition.m_makespan, wait_transition));				
			    }
			}
		    }
		}				
	    }
	    transition_Queue.erase(transition_Queue.begin());
	}
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


