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
/* smtcbsR.h / 1-224_leibniz                                                  */
/*----------------------------------------------------------------------------*/
//
// Conflict based search for a semi-continuous version of MAPF implemented
// on top of SAT-modulo theories.
//
/*----------------------------------------------------------------------------*/


#ifndef __SMTCBSR_H__
#define __SMTCBSR_H__

#include <vector>
#include <list>
#include <set>
#include <map>
#include <unordered_map>
#include <unordered_set>

#include "result.h"

#include "common/types.h"
#include "core/graph.h"
#include "core/agent.h"
#include "core/cbsR.h"
#include "core/smtcbs.h"


using namespace std;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/
// sRealSmtCBS

    class sRealSMTCBS
	: public sRealCBSBase
	, public sSMTCBSBase
    {
    public:
	typedef std::multimap<sDouble, sInt_32, std::less<sDouble> > NeighborIDs_mmap;
	typedef std::map<sInt_32, NeighborIDs_mmap, std::less<sInt_32> > NeighborMapping_map;
	
	struct KruhobotDecision
	{
	    KruhobotDecision() { /* nothing */ }
  	    KruhobotDecision(sInt_32 dec_id, sDouble time, sInt_32 location_id, sInt_32 prev_dec_id)
	    : m_dec_id(dec_id)
	    , m_time(time)
	    , m_location_id(location_id)
	    , m_sink_id(-1)
	    {
		m_prev_dec_IDs.insert(prev_dec_id);
	    }

	    void to_Screen(const sString &indent = "") const
	    {
		to_Stream(stdout, indent);
	    }
	    
	    void to_Stream(FILE *fw, const sString &indent = "") const
	    {
		fprintf(fw, "%s%d: %d [%.3f] (%d <-- { ", indent.c_str(), m_dec_id, m_location_id, m_time, m_dec_id);

		for (LocationIDs_uset::const_iterator prev = m_prev_dec_IDs.begin(); prev != m_prev_dec_IDs.end(); ++prev)
		{
		    printf("%d ", *prev);
		}      
		fprintf(fw, "}) --> { ");
		for (LocationIDs_vector::const_iterator next = m_next_dec_IDs.begin(); next != m_next_dec_IDs.end(); ++next)
		{
		    printf("%d ", *next);
		}
		if (m_sink_id >= 0)
		{
		    printf("} --_ %d\n", m_sink_id);
		}
		else
		{
		    fprintf(fw, "}\n");
		}		
	    }	    	    

	    sInt_32 m_dec_id;
	    
	    sDouble m_time;
	    sInt_32 m_location_id;
	    sInt_32 m_sink_id;

	    LocationIDs_uset m_prev_dec_IDs;
	    LocationIDs_vector m_next_dec_IDs;

	    NeighborMapping_map m_neighbor_mappping;
	};

	typedef std::set<sInt_32, std::less<sInt_32> > ConflictIDs_set;		
	
	struct ConflictFingerprint
	{
	    ConflictFingerprint() { /* nothing */ }

	    bool operator==(const ConflictFingerprint &conflict_fingerprint) const
	    {
		ConflictIDs_set::const_iterator conflict_id_A = m_conflict_IDs.begin();
		ConflictIDs_set::const_iterator conflict_id_B = conflict_fingerprint.m_conflict_IDs.begin();

		if (m_conflict_IDs.size() == conflict_fingerprint.m_conflict_IDs.size())
		{
		    while (conflict_id_A != m_conflict_IDs.end())
		    {
			sASSERT(conflict_id_B != conflict_fingerprint.m_conflict_IDs.end());

			if (*conflict_id_A != *conflict_id_B)
			{
			    return false;
			}		    
			++conflict_id_A;
			++conflict_id_B;		    
		    }
		    sASSERT(conflict_id_B == conflict_fingerprint.m_conflict_IDs.end());
		    
		    return true;
		}
		else
		{
		    return false;
		}
	    }
	    
	    bool operator<(const ConflictFingerprint &conflict_fingerprint) const
	    {
		/*
		if (m_conflict_IDs.size() < conflict_fingerprint.m_conflict_IDs.size())
		{
		    return true;
		}
		else
		{
		    if (m_conflict_IDs.size() > conflict_fingerprint.m_conflict_IDs.size())
		    {
			return false;
		    }
		    else
		*/
		    {
			ConflictIDs_set::const_iterator conflict_id_A = m_conflict_IDs.begin();
			ConflictIDs_set::const_iterator conflict_id_B = conflict_fingerprint.m_conflict_IDs.begin();

			while (conflict_id_A != m_conflict_IDs.end() && conflict_id_B != conflict_fingerprint.m_conflict_IDs.end())
			{		    
			    if (*conflict_id_A < *conflict_id_B)
			    {
				return true;
			    }
			    else
			    {
				if (*conflict_id_A > *conflict_id_B)
				{
				    return false;
				}			
			    }
			    ++conflict_id_A;
			    ++conflict_id_B;		    
			}
//			sASSERT(*this == conflict_fingerprint);
//			return false;

			if (conflict_id_A != m_conflict_IDs.end())
			{
			    sASSERT(conflict_id_B == conflict_fingerprint.m_conflict_IDs.end());
			    return false;
			}
			else
			{
			    sASSERT(conflict_id_A == m_conflict_IDs.end());
			    
			    if (conflict_id_B != conflict_fingerprint.m_conflict_IDs.end())
			    {
				return true;
			    }
			    else
			    {
				return false;
			    }
			}
//		    }
		}		     		
	    }

	    void to_Screen(const sString &indent = "") const
	    {
		to_Stream(stdout, indent);
	    }
	    
	    void to_Stream(FILE *fw, const sString &indent = "") const
	    {
		fprintf(fw, "%s{ ", indent.c_str());

		for (ConflictIDs_set::const_iterator conflict_id = m_conflict_IDs.begin(); conflict_id != m_conflict_IDs.end(); ++conflict_id)
		{
		    printf("%d ", *conflict_id);
		}      
		fprintf(fw, "}");

		if (indent != "")
		{
		    fprintf(fw, "\n");
		}
	    }	    	    	    

	    ConflictIDs_set m_conflict_IDs;
	};

	typedef std::vector<KruhobotDecision> KruhobotDecisionDiagram_vector;	
	typedef std::vector<KruhobotDecisionDiagram_vector> KruhobotDecisionDiagrams_vector;
	
	typedef std::vector<sInt_32> DecisionIDs_vector;
	typedef std::map<sDouble, sInt_32, std::less<sDouble> > Desisions_map;
	typedef std::vector<Desisions_map> KruhobotDesisions_vector;

	typedef std::map<sDouble, sInt_32, std::less<sDouble> > KruhobotDecisionIDs_map;
	typedef std::multimap<sDouble, sInt_32, std::less<sDouble> > KruhobotDecisionIDs_mmap;	
	
	typedef std::map<sInt_32, KruhobotDecisionIDs_mmap, std::less<sInt_32> > KruhobotDecisionMapping_map;
	typedef std::vector<KruhobotDecisionIDs_mmap> KruhobotDecisionMapping_vector;	    
	typedef std::vector<KruhobotDecisionMapping_map> KruhobotDecisionMappings_vector;

	typedef std::unordered_map<sInt_32, sInt_32> Explorations_umap;
	typedef std::map<sDouble, Explorations_umap> TransitionExplorations_map;

	typedef std::set<sDouble, std::less<sDouble> > DecisionTimes_set;
	typedef std::vector<DecisionTimes_set> LocationDecisionTimes_vector;

	struct RespectfulTransition
	{
	    RespectfulTransition() { /* nothing */ }
	    
	    RespectfulTransition(sInt_32 trans_id, sDouble time, sDouble cost, sDouble makespan, sInt_32 location_id, sInt_32 prev_trans_id)
	    : m_trans_id(trans_id)
	    , m_time(time)
	    , m_cost(cost)
	    , m_makespan(makespan)
	    , m_waited(0)
	    , m_location_id(location_id)
	    , m_prev_trans_id(prev_trans_id)
	    , m_prev_corr_dec_id(-1) { /* nothing */ }
	   
	    RespectfulTransition(sInt_32 trans_id, sDouble time, sDouble cost, sDouble makespan, sDouble waited, sInt_32 location_id, sInt_32 prev_trans_id)
	    : m_trans_id(trans_id)
	    , m_time(time)
	    , m_cost(cost)
	    , m_makespan(makespan)
	    , m_waited(waited)
	    , m_location_id(location_id)
	    , m_prev_trans_id(prev_trans_id)
	    , m_prev_corr_dec_id(-1) { /* nothing */ }
	    
	    void to_Screen(const sString &indent = "") const
	    {
		to_Stream(stdout, indent);
	    }
	    
	    void to_Stream(FILE *fw, const sString &indent = "") const
	    {
		fprintf(fw, "%s%d [%.3f {cost: %.3f, make: %.3f, wait:%.3f}] (%d <-- %d) @%d [\n", indent.c_str(), m_location_id, m_time, m_cost, m_makespan, m_waited, m_trans_id, m_prev_trans_id, m_prev_corr_dec_id);
		fprintf(fw, "%s%s conflict_fingerpring: ", indent.c_str(), s_INDENT.c_str());		
		m_conflict_fingerprint.to_Stream(fw);
		fprintf(fw, "%s]\n", indent.c_str());
	    }	    	    

	    sInt_32 m_trans_id;
	    
	    sDouble m_time;
	    sDouble m_cost;
	    sDouble m_makespan;
	    sDouble m_waited;
	    sInt_32 m_location_id;
	    sInt_32 m_prev_trans_id;
	    sInt_32 m_prev_corr_dec_id;

	    ConflictFingerprint m_conflict_fingerprint;	    
	};

	typedef std::map<sDouble, LocationIDs_uset> RespectfulTransitions_map;
	typedef std::multimap<sDouble, RespectfulTransition, std::less<sDouble> > RespectfulTransitions_mmap;
	typedef std::map<ConflictFingerprint, RespectfulTransitions_mmap, std::less<ConflictFingerprint> > BucketedRespectfulTransitions_mmap;
	typedef std::map<ConflictFingerprint, sDouble, std::less<ConflictFingerprint> > SinkReachabilities_mmap;
	
	typedef std::vector<RespectfulTransition> RespectfulTransitions_vector;

	struct RespectfulVisit
	{
	    RespectfulVisit()
	    { /* nothing */ }
	    
	    RespectfulVisit(sDouble time, sInt_32 trans_id)
	    : m_time(time)
	    , m_trans_id(trans_id) { /* nothing */ }
	    
	    sDouble m_time;
	    sInt_32 m_trans_id;

	    RespectfulTransitions_mmap::iterator m_queue_iter;
	};

	typedef std::unordered_map<sInt_32, RespectfulVisit> RespectfulVisits_umap;
	typedef std::map<ConflictFingerprint, RespectfulVisits_umap, std::less<ConflictFingerprint> > RespectfulExplorations_map;

	typedef std::map<ConflictFingerprint, RespectfulVisit, std::less<ConflictFingerprint> > ClimbingRespectfulVisits_map;
	typedef std::unordered_map<sInt_32, ClimbingRespectfulVisits_map> ClimbingRespectfulVisits_umap;
	typedef std::map<ConflictFingerprint, ClimbingRespectfulVisits_umap, std::less<ConflictFingerprint> > ClimbingRespectfulExplorations_map;

	typedef std::set<sDouble, std::less<sDouble> > VisitTimes_set;
	typedef std::unordered_map<sInt_32, VisitTimes_set> UnifiedVisits_umap;

	struct RealCoordinate
	{
	    RealCoordinate()
	    {
		// nothing
	    }
	    
	    RealCoordinate(sInt_32 kruhobot_id, sInt_32 decision_id)
	    : m_kruhobot_id(kruhobot_id)
	    , m_decision_id(decision_id)
	    , m_sink_id(-1)
	    {
		// nothing
	    }

	    RealCoordinate(sInt_32 kruhobot_id, sInt_32 decision_id, sInt_32 sink_id)
	    : m_kruhobot_id(kruhobot_id)
	    , m_decision_id(decision_id)
	    , m_sink_id(sink_id)
	    {
		// nothing
	    }
    
	    sInt_32 m_kruhobot_id;
	    sInt_32 m_decision_id;
	    sInt_32 m_sink_id;	    
	};	
	
	typedef std::vector<RealCoordinate> RealCoordinates_vector;	

	struct RealModel
	{
	    RealModel()
	    {
		// nothing
	    }
	    
	    VariableIDs_2vector m_vertex_occupancy;
	    VariableIDs_3vector m_edge_occupancy;
	    VariableIDs_2vector m_goal_sinking;	    
//	    VariableIDs_2vector m_layer_cardinality;

	    RealCoordinates_vector m_variable_mapping;
	};	

	struct RealContext
	{
	    RealContext(sDouble makespan_bound)
	    : m_makespan_bound(makespan_bound)
	    {
		// nothing
	    }

	    sDouble m_makespan_bound;
	};

	typedef std::vector<sDouble> Makespans_vector;
	
    public:
	sRealSMTCBS(sBoolEncoder *solver_Encoder, sRealInstance *real_Instance);
	sRealSMTCBS(sBoolEncoder *solver_Encoder, sRealInstance *real_Instance, sDouble timeout);
	/*----------------------------------------------------------------------------*/

	sDouble find_ShortestNonconflictingSchedules(sRealSolution &real_Solution, sDouble makespan_limit);
	sDouble find_ShortestNonconflictingSchedules(const  sRealInstance &real_Instance,
						     sRealSolution        &real_Solution,
						     sDouble               makespan_limit);
	
	sDouble find_ShortestNonconflictingSchedules(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit);	
	sDouble find_ShortestNonconflictingSchedules(const sRealInstance      &real_Instance,
						     KruhobotSchedules_vector &kruhobot_Schedules,
						     sDouble                   makespan_limit);

	sDouble find_ShortestNonconflictingSchedules(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit, sDouble extra_makespan);
	sDouble find_ShortestNonconflictingSchedules(const sRealInstance      &real_Instance,
						     KruhobotSchedules_vector &kruhobot_Schedules,
						     sDouble                   makespan_limit,
						     sDouble                   extra_makespan);
	/*----------------------------------------------------------------------------*/

	sDouble find_ShortestNonconflictingSchedules_pruningSmart(sRealSolution &real_Solution, sDouble makespan_limit);
	sDouble find_ShortestNonconflictingSchedules_pruningSmart(const  sRealInstance &real_Instance,
								  sRealSolution        &real_Solution,
								  sDouble               makespan_limit);
	
	sDouble find_ShortestNonconflictingSchedules_pruningSmart(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit);
	sDouble find_ShortestNonconflictingSchedules_pruningSmart(const sRealInstance      &real_Instance,
								  KruhobotSchedules_vector &kruhobot_Schedules,
								  sDouble                   makespan_limit);

	sDouble find_ShortestNonconflictingSchedules_pruningSmart(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit, sDouble extra_makespan);
	sDouble find_ShortestNonconflictingSchedules_pruningSmart(const sRealInstance      &real_Instance,
								  KruhobotSchedules_vector &kruhobot_Schedules,
								  sDouble                   makespan_limit,
								  sDouble                   extra_makespan);	
	/*----------------------------------------------------------------------------*/

	sDouble find_ShortestNonconflictingSchedules_pruningStrong(sRealSolution &real_Solution, sDouble makespan_limit);
	sDouble find_ShortestNonconflictingSchedules_pruningStrong(const  sRealInstance &real_Instance,
								  sRealSolution        &real_Solution,
								  sDouble               makespan_limit);
	
	sDouble find_ShortestNonconflictingSchedules_pruningStrong(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit);
	sDouble find_ShortestNonconflictingSchedules_pruningStrong(const sRealInstance      &real_Instance,
								  KruhobotSchedules_vector &kruhobot_Schedules,
								  sDouble                   makespan_limit);

	sDouble find_ShortestNonconflictingSchedules_pruningStrong(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit, sDouble extra_makespan);
	sDouble find_ShortestNonconflictingSchedules_pruningStrong(const sRealInstance      &real_Instance,
								  KruhobotSchedules_vector &kruhobot_Schedules,
								  sDouble                   makespan_limit,
								  sDouble                   extra_makespan);	
	/*----------------------------------------------------------------------------*/

	sDouble find_ShortestNonconflictingSchedules_conflictRespectful(sRealSolution &real_Solution, sDouble makespan_limit);
	sDouble find_ShortestNonconflictingSchedules_conflictRespectful(const  sRealInstance &real_Instance,
									sRealSolution        &real_Solution,
									sDouble               makespan_limit);
	
	sDouble find_ShortestNonconflictingSchedules_conflictRespectful(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit);
	sDouble find_ShortestNonconflictingSchedules_conflictRespectful(const sRealInstance      &real_Instance,
									KruhobotSchedules_vector &kruhobot_Schedules,
									sDouble                   makespan_limit);

	sDouble find_ShortestNonconflictingSchedules_conflictRespectful(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit, sDouble extra_makespan);
	sDouble find_ShortestNonconflictingSchedules_conflictRespectful(const sRealInstance      &real_Instance,
									KruhobotSchedules_vector &kruhobot_Schedules,
									sDouble                   makespan_limit,
									sDouble                   extra_makespan);	
	/*----------------------------------------------------------------------------*/

	sDouble find_ShortestNonconflictingSchedules_individualizedConflictRespectful(sRealSolution &real_Solution, sDouble makespan_limit);	
	sDouble find_ShortestNonconflictingSchedules_individualizedConflictRespectful(const sRealInstance  &real_Instance,
										      sRealSolution        &real_Solution,
										      sDouble               makespan_limit);
	
	sDouble find_ShortestNonconflictingSchedules_individualizedConflictRespectful(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit);
	sDouble find_ShortestNonconflictingSchedules_individualizedConflictRespectful(const sRealInstance      &real_Instance,
										      KruhobotSchedules_vector &kruhobot_Schedules,
										      sDouble                   makespan_limit);

	sDouble find_ShortestNonconflictingSchedules_individualizedConflictRespectful(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit, sDouble extra_makespan);
	sDouble find_ShortestNonconflictingSchedules_individualizedConflictRespectful(const sRealInstance      &real_Instance,
										      KruhobotSchedules_vector &kruhobot_Schedules,
										      sDouble                   makespan_limit,
										      sDouble                   extra_makespan);	
	/*----------------------------------------------------------------------------*/

	sDouble find_ExactShortestNonconflictingSchedules_individualizedConflictRespectful(sRealSolution &real_Solution, sDouble makespan_limit);	
	sDouble find_ExactShortestNonconflictingSchedules_individualizedConflictRespectful(const sRealInstance  &real_Instance,
											   sRealSolution        &real_Solution,
											   sDouble               makespan_limit);
	
	sDouble find_ExactShortestNonconflictingSchedules_individualizedConflictRespectful(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit);
	sDouble find_ExactShortestNonconflictingSchedules_individualizedConflictRespectful(const sRealInstance      &real_Instance,
											   KruhobotSchedules_vector &kruhobot_Schedules,
											   sDouble                   makespan_limit);

	sDouble find_ExactShortestNonconflictingSchedules_individualizedConflictRespectful(KruhobotSchedules_vector &kruhobot_Schedules, sDouble makespan_limit, sDouble extra_makespan);
	sDouble find_ExactShortestNonconflictingSchedules_individualizedConflictRespectful(const sRealInstance      &real_Instance,
											   KruhobotSchedules_vector &kruhobot_Schedules,
											   sDouble                   makespan_limit,
											   sDouble                   extra_makespan);	
	/*----------------------------------------------------------------------------*/		

	sDouble find_NonconflictingSchedules(const sRealInstance              &real_Instance,
					     KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
					     KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts,
					     KruhobotSchedules_vector         &kruhobot_Schedules,
					     sDouble                           makespan_limit,
					     sDouble                           extra_makespan);

	sDouble find_KruhobotIgnoringSchedule(const sKruhobot &kruhobot,
					      const s2DMap    &map,
					      sInt_32          source_loc_id,
					      sInt_32          sink_loc_id,
					      sDouble          makespan_limit,
					      sDouble          extra_makespan,
					      Schedule_vector &Schedule) const;

	sDouble find_KruhobotIgnoringSchedule_strong(const sKruhobot &kruhobot,
						     const s2DMap    &map,
						     sInt_32          source_loc_id,
						     sInt_32          sink_loc_id,
						     sDouble          makespan_limit,
						     sDouble          extra_makespan,
						     Schedule_vector &Schedule) const;
	/*----------------------------------------------------------------------------*/	

	void reflect_KruhobotCollision(const KruhobotCollision          &kruhobot_collision,
				       KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
				       KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts);
	
	void reflect_KruhobotCollisions(const KruhobotCollisions_mset    &kruhobot_Collisions,
					KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
					KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts);
	/*----------------------------------------------------------------------------*/

	void reflect_KruhobotCollision(const KruhobotCollision          &kruhobot_collision,
				       KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
				       KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts,
				       sInt_32                          &last_conflict_id);
	
	void reflect_KruhobotCollisions(const KruhobotCollisions_mset    &kruhobot_Collisions,
					KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
					KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts,
					sInt_32                          &last_conflict_id);
	/*----------------------------------------------------------------------------*/

	void reflect_KruhobotCollision(const KruhobotCollision          &kruhobot_collision,
				       KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
				       KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts,
				       KruhobotAffections_vector        &affected_Kruhobots,
				       sInt_32                          &last_conflict_id);
	
	void reflect_KruhobotCollisions(const KruhobotCollisions_mset    &kruhobot_Collisions,
					KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
					KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts,
					KruhobotAffections_vector        &affected_Kruhobots,
					sInt_32                          &last_conflict_id);
	/*----------------------------------------------------------------------------*/			

	sDouble find_NonconflictingSchedules(const sRealInstance                    &real_Instance,
					     KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
					     KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
					     KruhobotSchedules_vector               &kruhobot_Schedules,
					     sDouble                                 makespan_limit,
					     sDouble                                 extra_makespan);

	sDouble find_NonconflictingSchedules_pruningSmart(const sRealInstance                    &real_Instance,
							  KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
							  KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
							  KruhobotSchedules_vector               &kruhobot_Schedules,
							  sDouble                                 makespan_limit,
							  sDouble                                 extra_makespan);

	sDouble find_NonconflictingSchedules_pruningStrong(const sRealInstance                    &real_Instance,
							   KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
							   KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
							   KruhobotSchedules_vector               &kruhobot_Schedules,
							   sDouble                                 makespan_limit,
							   sDouble                                 extra_makespan);

	sDouble find_NonconflictingSchedules_conflictRespectful(const sRealInstance                    &real_Instance,
								KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
								KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
								KruhobotSchedules_vector               &kruhobot_Schedules,
								sDouble                                 makespan_limit,
								sDouble                                 extra_makespan);

	sDouble find_NonconflictingSchedules_individualizedConflictRespectful(const sRealInstance                    &real_Instance,
									      KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
									      KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
									      KruhobotSchedules_vector               &kruhobot_Schedules,
									      sDouble                                 makespan_limit,
									      sDouble                                 extra_makespan);

	sDouble find_ExactNonconflictingSchedules_individualizedConflictRespectful(const sRealInstance                    &real_Instance,
										   KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
										   KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
										   KruhobotSchedules_vector               &kruhobot_Schedules,
										   sDouble                                 makespan_limit,
										   sDouble                                 extra_makespan);		
	/*---------------------------------------------------------------------------*/			

	void reflect_KruhobotCollisions(const KruhobotCollisions_mset          &kruhobot_Collisions,
					KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
					KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts);

	void reflect_KruhobotCollision(const KruhobotCollision                &kruhobot_collision,
				       KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
				       KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts);		
	/*---------------------------------------------------------------------------*/
    
	void reflect_KruhobotCollisions(const KruhobotCollisions_mset          &kruhobot_Collisions,
					KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
					KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
					sInt_32                                &last_conflict_id);

	void reflect_KruhobotCollision(const KruhobotCollision                &kruhobot_collision,
				       KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
				       KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
				       sInt_32                                &last_conflict_id);
	/*---------------------------------------------------------------------------*/

	void reflect_KruhobotCollisions(const KruhobotCollisions_mset          &kruhobot_Collisions,
					KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
					KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
					KruhobotAffections_vector              &affected_Kruhobots,
					sInt_32                                &last_conflict_id);

	void reflect_KruhobotCollision(const KruhobotCollision                &kruhobot_collision,
				       KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
				       KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
				       KruhobotAffections_vector              &affected_Kruhobots,
				       sInt_32                                &last_conflict_id);
	/*---------------------------------------------------------------------------*/				

	bool find_InitialNonconflictingSchedules(Glucose::Solver                       *solver,
						 RealContext                           &context,					 
						 const sRealInstance                   &real_Instance,
						 KruhobotDecisionDiagrams_vector       &kruhobot_RDDs,
						 const KruhobotDecisionMappings_vector &kruhobot_RDD_Mappings,						 
						 RealModel                             &real_sat_Model,
						 KruhobotSchedules_vector              &kruhobot_Schedules) const;

	
	bool find_NextNonconflictingSchedules(Glucose::Solver                       *solver,
					      RealContext                           &context,
					      const sRealInstance                   &real_Instance,
					      KruhobotDecisionDiagrams_vector       &kruhobot_RDDs,
					      const KruhobotDecisionMappings_vector &kruhobot_RDD_Mappings,
					      const KruhobotCollisions_mset         &kruhobot_Collisions,
					      RealModel                             &real_sat_Model,
					      KruhobotSchedules_vector              &kruhobot_Schedules) const;

	bool find_NearNonconflictingSchedules(Glucose::Solver                       *solver,
					      RealContext                           &context,
					      const sRealInstance                   &real_Instance,
					      KruhobotDecisionDiagrams_vector       &kruhobot_RDDs,
					      const KruhobotDecisionMappings_vector &kruhobot_RDD_Mappings,
					      const KruhobotCollisions_mset         &kruhobot_Collisions,
					      RealModel                             &real_sat_Model,
					      KruhobotSchedules_vector              &kruhobot_Schedules) const;

	bool find_NearNonconflictingSchedules(Glucose::Solver                       *solver,
					      RealContext                           &context,
					      const sRealInstance                   &real_Instance,
					      KruhobotDecisionDiagrams_vector       &kruhobot_RDDs,
					      const KruhobotDecisionMappings_vector &kruhobot_RDD_Mappings,
					      const KruhobotCollisions_mset         &kruhobot_Collisions,
					      const KruhobotCollisions_mset         &next_kruhobot_Collisions,					      
					      RealModel                             &real_sat_Model,
					      KruhobotSchedules_vector              &kruhobot_Schedules) const;
	/*----------------------------------------------------------------------------*/		

	sDouble build_KruhobotRealDecisionDiagram(const sKruhobot                &kruhobot,
						  const s2DMap                   &map,
						  sInt_32                         source_loc_id,
						  sInt_32                         sink_loc_id,
						  const LocationConflicts__umap  &location_Conflicts,
						  const LinearConflicts__map     &linear_Conflicts,
						  sDouble                         makespan_bound,
						  KruhobotDecisionDiagram_vector &kruhobot_RDD,
						  KruhobotDecisionMapping_map    &kruhobot_RDD_mapping) const;

	sDouble build_KruhobotRealDecisionDiagram(const sKruhobot                     &kruhobot,
						  const s2DMap                        &map,
						  sInt_32                              source_loc_id,
						  sInt_32                              sink_loc_id,
						  const LocationConflicts_upper__umap &location_Conflicts,
						  const LinearConflicts_upper__map    &linear_Conflicts,
						  sDouble                              makespan_bound,
						  KruhobotDecisionDiagram_vector      &kruhobot_RDD,
						  KruhobotDecisionMapping_map         &kruhobot_RDD_mapping) const;

	sDouble build_KruhobotRealDecisionDiagram_pruning(const sKruhobot                     &kruhobot,
							  const s2DMap                        &map,
							  sInt_32                              source_loc_id,
							  sInt_32                              sink_loc_id,
							  const LocationConflicts_upper__umap &location_Conflicts,
							  const LinearConflicts_upper__map    &linear_Conflicts,
							  sDouble                              makespan_bound,
							  KruhobotDecisionDiagram_vector      &kruhobot_RDD,
							  KruhobotDecisionMapping_map         &kruhobot_RDD_mapping) const;

	sDouble build_KruhobotRealDecisionDiagram_pruningSmart(const sKruhobot                     &kruhobot,
							       const s2DMap                        &map,
							       sInt_32                              source_loc_id,
							       sInt_32                              sink_loc_id,
							       const LocationConflicts_upper__umap &location_Conflicts,
							       const LinearConflicts_upper__map    &linear_Conflicts,
							       sDouble                              makespan_bound,
							       KruhobotDecisionDiagram_vector      &kruhobot_RDD,
							       KruhobotDecisionMapping_map         &kruhobot_RDD_mapping) const;

	sDouble build_KruhobotRealDecisionDiagram_pruningStrong(const sKruhobot                     &kruhobot,
								const s2DMap                        &map,
								sInt_32                              source_loc_id,
								sInt_32                              sink_loc_id,
								const LocationConflicts_upper__umap &location_Conflicts,
								const LinearConflicts_upper__map    &linear_Conflicts,
								sDouble                              makespan_bound,
								KruhobotDecisionDiagram_vector      &kruhobot_RDD,
								KruhobotDecisionMapping_map         &kruhobot_RDD_mapping) const;

	sDouble build_KruhobotRealDecisionDiagram_conflictRespectful(const sKruhobot                     &kruhobot,
								     const s2DMap                        &map,
								     sInt_32                              source_loc_id,
								     sInt_32                              sink_loc_id,
								     const LocationConflicts_upper__umap &location_Conflicts,
								     const LinearConflicts_upper__map    &linear_Conflicts,
								     sDouble                              makespan_bound,
								     KruhobotDecisionDiagram_vector      &kruhobot_RDD,
								     KruhobotDecisionMapping_map         &kruhobot_RDD_mapping) const;

	sDouble build_KruhobotRealDecisionDiagram_conflictRespectfulBucketing(const sKruhobot                     &kruhobot,
									      const s2DMap                        &map,
									      sInt_32                              source_loc_id,
									      sInt_32                              sink_loc_id,
									      const LocationConflicts_upper__umap &location_Conflicts,
									      const LinearConflicts_upper__map    &linear_Conflicts,
									      sDouble                              makespan_bound,
									      KruhobotDecisionDiagram_vector      &kruhobot_RDD,
									      KruhobotDecisionMapping_map         &kruhobot_RDD_mapping) const;

	sDouble build_KruhobotRealDecisionDiagram_individualizedConflictRespectfulBucketing(const sKruhobot                     &kruhobot,
											    const s2DMap                        &map,
											    sInt_32                              source_loc_id,
											    sInt_32                              sink_loc_id,
											    const LocationConflicts_upper__umap &location_Conflicts,
											    const LinearConflicts_upper__map    &linear_Conflicts,
											    sDouble                              makespan_bound,
											    sDouble                             &individual_makespan_bound,
											    sInt_32                              fingerprint_limit,											    
											    KruhobotDecisionDiagram_vector      &kruhobot_RDD,
											    KruhobotDecisionMapping_map         &kruhobot_RDD_mapping) const;		
	
	Explorations_umap* obtain_ExploredTransitions(TransitionExplorations_map &explored_Transitions, sDouble time) const;
	bool is_UnifiedlyVisited(sInt_32 location_id, sDouble time, const UnifiedVisits_umap &unified_Visits) const;
	
	bool is_TransitionConflicting(sInt_32                              location_u_id,
				      sInt_32                              location_v_id,
				      sDouble                              start_time,
				      sDouble                              finish_time,
				      const LocationConflicts_upper__umap &location_Conflicts,
				      const LinearConflicts_upper__map    &linear_Conflicts,
				      const ConflictFingerprint           &conflict_Fingerprint) const;

	void determine_ClimbingStatus(sInt_32                              location_id,
				      sDouble                              time,
				      const LocationConflicts_upper__umap &location_Conflicts,
				      const LinearConflicts_upper__map    &linear_Conflicts,
				      const ConflictFingerprint           &conflict_Fingerprint) const;
	/*----------------------------------------------------------------------------*/	

	void augment_KruhobotRealDecisionDiagram(const sKruhobot                &kruhobot,
						 sInt_32                         source_loc_id,
						 sInt_32                         sink_loc_id,
						 sDouble                         makespan_bound,
						 KruhobotDecisionDiagram_vector &kruhobot_RDD,
						 KruhobotDecisionMapping_map    &kruhobot_RDD_mapping) const;
	
	void interconnect_KruhobotRealDecisionDiagram(const sKruhobot                &kruhobot,
						      const s2DMap                   &map,
						      KruhobotDecisionDiagram_vector &kruhobot_RDD,
						      KruhobotDecisionMapping_map    &kruhobot_RDD_mapping) const;

	void interconnect_KruhobotRealDecisionDiagram_smart(const sKruhobot                &kruhobot,
							    const s2DMap                   &map,
							    KruhobotDecisionDiagram_vector &kruhobot_RDD,
							    KruhobotDecisionMapping_map    &kruhobot_RDD_mapping) const;	

	void trim_KruhobotRealDecisionDiagram(sDouble makespan_bound, KruhobotDecisionDiagram_vector &kruhobot_RDD) const;
	void trim_KruhobotRealDecisionDiagrams(sDouble makespan_bound, KruhobotDecisionDiagrams_vector &kruhobot_RDDs) const;

	void collect_CharacteristicMakespans(const sRealInstance &real_Instance, const KruhobotDecisionDiagrams_vector &kruhobot_RDDs, Makespans_vector &characteristic_Makespans) const;
	bool compare_CharacteristicMakespans(const sRealInstance &real_Instance, const Makespans_vector &characteristic_Makespans, const Makespans_vector &next_characteristic_Makespans) const;
	/*----------------------------------------------------------------------------*/
	
	bool compare_KruhobotRealDecisionDiagrams(const s2DMap                         &map,
						  const KruhobotDecisionDiagram_vector &kruhobot_RDD,
						  const KruhobotDecisionDiagram_vector &next_kruhobot_RDD) const;
	bool compare_KruhobotRealDecisionDiagrams_smart(const s2DMap                         &map,
							const KruhobotDecisionDiagram_vector &kruhobot_RDD,
							const KruhobotDecisionDiagram_vector &next_kruhobot_RDD) const;

	void represent_DecisionTime(DecisionTimes_set &decision_Times, sDouble time) const;
	DecisionTimes_set::const_iterator obtain_DecisionTime(DecisionTimes_set &decision_Times, sDouble time) const;

	/*----------------------------------------------------------------------------*/

	sDouble calc_MakespanBound(const sRealInstance                    &real_Instance,
				   const KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
				   const KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts) const;
	sDouble calc_MakespanBound(const sRealInstance                          &real_Instance,
				   const KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
				   const KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts) const;	
	
	sDouble calc_KruhobotMakespanBound(const sKruhobot &kruhobot, const LocationConflicts__umap &location_Conflicts, const LinearConflicts__map &linear_Conflicts) const;
	sDouble calc_KruhobotMakespanBound(const sKruhobot &kruhobot, const LocationConflicts_upper__umap &location_Conflicts, const LinearConflicts_upper__map &linear_Conflicts) const;	
	/*----------------------------------------------------------------------------*/	

	sDouble calc_NextMakespanBound(sDouble                                 prev_makespan_bound,
				       const sRealInstance                    &real_Instance,
				       const KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
				       const KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts) const;
	sDouble calc_NextMakespanBound(sDouble                                       prev_makespan_bound,
				       const sRealInstance                          &real_Instance,
				       const KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
				       const KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts) const;	
	
	sDouble calc_KruhobotNextMakespanBound(sDouble                        prev_makespan_bound,
					       const sKruhobot               &kruhobot,
					       const LocationConflicts__umap &location_Conflicts,
					       const LinearConflicts__map    &linear_Conflicts) const;
	sDouble calc_KruhobotNextMakespanBound(sDouble                              prev_makespan_bound,
					       const sKruhobot                     &kruhobot,
					       const LocationConflicts_upper__umap &location_Conflicts,
					       const LinearConflicts_upper__map    &linear_Conflicts) const;
	/*----------------------------------------------------------------------------*/		

	sInt_32 build_RealModelVariables(Glucose::Solver                      *solver,
					 RealContext                           &context,					 
					 const sRealInstance                   &real_Instance,
					 KruhobotDecisionDiagrams_vector       &kruhobot_RDDs,
					 const KruhobotDecisionMappings_vector &kruhobot_RDD_Mappings,
					 RealModel                             &real_sat_Model) const;

	void build_RealModelConstraints(Glucose::Solver                      *solver,
					RealContext                           &context,					 
					const sRealInstance                   &real_Instance,
					const KruhobotDecisionDiagrams_vector &kruhobot_RDDs,
					const KruhobotDecisionMappings_vector &kruhobot_RDD_Mappings,
					RealModel                             &real_sat_Model) const;

	void refine_RealModelConstraints(Glucose::Solver                      *solver,
					 RealContext                           &context,					 
					 const sRealInstance                   &real_Instance,
					 const KruhobotDecisionDiagrams_vector &kruhobot_RDDs,
					 const KruhobotDecisionMappings_vector &kruhobot_RDD_Mappings,
					 const KruhobotCollisions_mset         &kruhobot_Collisions,
					 RealModel                             &real_sat_Model) const;
	/*----------------------------------------------------------------------------*/	

	sInt_32 match_CorrespondingDecision(const Traversal                      &traversal,
					    const KruhobotDecisionDiagram_vector &kruhobot_RDD,
					    const KruhobotDecisionMapping_map    &kruhobot_RDD_mapping) const;
	
	sInt_32 match_CorrespondingSink(const sRealInstance                  &real_Instance,
					sInt_32                               kruhobot_id,
					const Traversal                      &traversal,
					const KruhobotDecisionDiagram_vector &kruhobot_RDDs,
					const KruhobotDecisionMapping_map &kruhobot_RDD_mapping) const;

	sInt_32 match_CorrespondingSink(const sRealInstance                  &real_Instance,
					sInt_32                               kruhobot_id,
					sDouble                               makespan_bound,
					const Traversal                      &traversal,
					const KruhobotDecisionDiagram_vector &kruhobot_RDDs,
					const KruhobotDecisionMapping_map &kruhobot_RDD_mapping) const;	
	
	sInt_32 match_CorrespondingNeighbor(const Traversal                      &traversal,
					    const KruhobotDecisionDiagram_vector &kruhobot_RDD,
					    const KruhobotDecisionMapping_map    &kruhobot_RDD_mapping,
					    sInt_32                              &neighbor) const;

	sInt_32 match_CorrespondingDecisions(const Traversal                      &traversal,
					     const KruhobotDecisionDiagram_vector &kruhobot_RDD,
					     const KruhobotDecisionMapping_map    &kruhobot_RDD_mapping,
					     DecisionIDs_vector                   &decision_IDs) const;
	
	sInt_32 match_CorrespondingSinks(const sRealInstance                  &real_Instance,
					 sInt_32                               kruhobot_id,
					 const Traversal                      &traversal,
					 const KruhobotDecisionDiagram_vector &kruhobot_RDD,
					 const KruhobotDecisionMapping_map    &kruhobot_RDD_mapping,
					 std::vector<sInt_32>                 &sink_IDs) const;

	sInt_32 match_CorrespondingSinks(const sRealInstance                  &real_Instance,
					 sInt_32                               kruhobot_id,
					 sDouble                               makespan_bound,
					 const Traversal                      &traversal,
					 const KruhobotDecisionDiagram_vector &kruhobot_RDD,
					 const KruhobotDecisionMapping_map    &kruhobot_RDD_mapping,
					 std::vector<sInt_32>                 &sink_IDs) const;	
	
	sInt_32 match_CorrespondingNeighbors(const Traversal                      &traversal,
					     const KruhobotDecisionDiagram_vector &kruhobot_RDD,
					     const KruhobotDecisionMapping_map    &kruhobot_RDD_mapping,
					     DecisionIDs_vector                   &decision_IDs,
					     std::vector<sInt_32>                 &Neighbors) const;
	/*----------------------------------------------------------------------------*/	

	void decode_PathModel(Glucose::Solver                       *solver,
			      const sRealInstance                   &real_Instance,
			      const KruhobotDecisionDiagrams_vector &kruhobot_RDDs,
			      const RealModel                       &real_sat_Model,
			      KruhobotSchedules_vector              &kruhobot_Schedules) const;
	
	bool verify_KruhobotSchedules(const sRealInstance &real_Instance, const KruhobotSchedules_vector &kruhobot_Schedules) const;
	bool verify_KruhobotCollisionDuplicities(const KruhobotCollision &next_kruhobot_collision, const KruhobotCollisions_mset &kruhobot_Collisions) const;	
	bool verify_KruhobotCollisionDuplicities(const KruhobotCollisions_mset next_kruhobot_Collisions, const KruhobotCollisions_mset &kruhobot_Collisions) const;
	
	sInt_32 collect_StartKruhobotDecision(sInt_32 start_location_id, const KruhobotDecisionDiagram_vector &kruhobot_RDD) const;	
	sInt_32 collect_GoalKruhobotDecision(sInt_32 goal_location_id, const KruhobotDecisionDiagram_vector &kruhobot_RDD) const;
	sInt_32 collect_GoalKruhobotDecision(sInt_32 goal_location_id, sDouble makespan_bound, const KruhobotDecisionDiagram_vector &kruhobot_RDD) const;	

	void collect_StartKruhobotDecisions(sInt_32 start_location_id, const KruhobotDecisionDiagram_vector &kruhobot_RDD, DecisionIDs_vector &decision_IDs) const;
	void collect_GoalKruhobotDecisions(sInt_32 goal_location_id, const KruhobotDecisionDiagram_vector &kruhobot_RDD, DecisionIDs_vector &decision_IDs) const;	
	void collect_GoalKruhobotDecisions(sInt_32 goal_location_id, sDouble makespan_bound, const KruhobotDecisionDiagram_vector &kruhobot_RDD, DecisionIDs_vector &decision_IDs) const;

	void collect_KruhobotExcludedDecision(const sKruhobot                      &kruhobot,
					      const s2DMap                         &map,
					      sInt_32                               decision_id,
					      const KruhobotDecisionDiagram_vector &kruhobot_RDD,
					      DecisionIDs_vector                   &decision_IDs) const;
	/*----------------------------------------------------------------------------*/	
		
	static void to_Screen(const KruhobotDecisionDiagram_vector &kruhobot_RDD, const sString &indent = "");
	static void to_Stream(FILE *fw, const KruhobotDecisionDiagram_vector &kruhobot_RDD, const sString &indent = "");

	static void to_Screen(const KruhobotDecisionDiagrams_vector &kruhobot_RDDs, const sString &indent = "");
	static void to_Stream(FILE *fw, const KruhobotDecisionDiagrams_vector &kruhobot_RDDs, const sString &indent = "");		
    };


/*----------------------------------------------------------------------------*/

} // namespace boOX

#endif /* __SMTCBSR_H__ */
