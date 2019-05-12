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
/* smtcbsR.h / 0-279_zenon                                                    */
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

#include "types.h"
#include "result.h"

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

	typedef std::vector<KruhobotDecision> KruhobotDecisionDiagram_vector;	
	typedef std::vector<KruhobotDecisionDiagram_vector> KruhobotDecisionDiagrams_vector;
	
	typedef std::vector<sInt_32> DecisionIDs_vector;
	typedef std::map<sDouble, sInt_32, std::less<sDouble> > Desisions_map;
	typedef std::vector<Desisions_map> KruhobotDesisions_vector;

	typedef std::multimap<sDouble, sInt_32, std::less<sDouble> > KruhobotDecisionIDs_mmap;
	typedef std::map<sInt_32, KruhobotDecisionIDs_mmap, std::less<sInt_32> > KruhobotDecisionMapping_map;
	typedef std::vector<KruhobotDecisionMapping_map> KruhobotDecisionMappings_vector;

	typedef std::unordered_map<sInt_32, sInt_32> Explorations_umap;
	typedef std::map<sDouble, Explorations_umap> TransitionExplorations_map;

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

	sDouble find_ShortestNonconflictingSchedules(sRealSolution &real_Solution, sDouble cost_limit);
	sDouble find_ShortestNonconflictingSchedules(const  sRealInstance &real_Instance,
						     sRealSolution        &real_Solution,
						     sDouble               cost_limit);
	
	sDouble find_ShortestNonconflictingSchedules(KruhobotSchedules_vector &kruhobot_Schedules, sDouble cost_limit);	
	sDouble find_ShortestNonconflictingSchedules(const sRealInstance      &real_Instance,
						     KruhobotSchedules_vector &kruhobot_Schedules,
						     sDouble                   cost_limit);

	sDouble find_ShortestNonconflictingSchedules(KruhobotSchedules_vector &kruhobot_Schedules, sDouble cost_limit, sDouble extra_cost);
	sDouble find_ShortestNonconflictingSchedules(const sRealInstance      &real_Instance,
						     KruhobotSchedules_vector &kruhobot_Schedules,
						     sDouble                   cost_limit,
						     sDouble                   extra_cost);
	/*----------------------------------------------------------------------------*/	

	sDouble find_NonconflictingSchedules(const sRealInstance              &real_Instance,
					     KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
					     KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts,
					     KruhobotSchedules_vector         &kruhobot_Schedules,
					     sDouble                           cost_limit,
					     sDouble                           extra_cost);

	sDouble find_KruhobotIgnoringSchedule(const sKruhobot &kruhobot,
					      const s2DMap    &map,
					      sInt_32          source_loc_id,
					      sInt_32          sink_loc_id,
					      sDouble          cost_limit,
					      sDouble          extra_cost,
					      Schedule_vector &Schedule) const;	

	void reflect_KruhobotCollision(const KruhobotCollision          &kruhobot_collision,
				       KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
				       KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts);
	
	void reflect_KruhobotCollisions(const KruhobotCollisions_mset    &kruhobot_Collisions,
					KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
					KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts);       

	sDouble find_NonconflictingSchedules(const sRealInstance                    &real_Instance,
					     KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
					     KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
					     KruhobotSchedules_vector               &kruhobot_Schedules,
					     sDouble                                 cost_limit,
					     sDouble                                 extra_cost);

	void reflect_KruhobotCollisions(const KruhobotCollisions_mset          &kruhobot_Collisions,
					KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
					KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts);

	void reflect_KruhobotCollision(const KruhobotCollision                &kruhobot_collision,
				       KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
				       KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts);		
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
						  sDouble                         cost_limit,
						  sDouble                         extra_cost,
						  const LocationConflicts__umap  &location_Conflicts,
						  const LinearConflicts__map     &linear_Conflicts,
						  sDouble                         makespan_bound,					       
						  KruhobotDecisionDiagram_vector &kruhobot_RDD,
						  KruhobotDecisionMapping_map    &kruhobot_RDD_mapping) const;

	sDouble build_KruhobotRealDecisionDiagram(const sKruhobot                      &kruhobot,
						  const s2DMap                         &map,
						  sInt_32                               source_loc_id,
						  sInt_32                               sink_loc_id,
						  sDouble                               cost_limit,
						  sDouble                               extra_cost,
						  const LocationConflicts_upper__umap  &location_Conflicts,
						  const LinearConflicts_upper__map     &linear_Conflicts,
						  sDouble                               makespan_bound,					       
						  KruhobotDecisionDiagram_vector       &kruhobot_RDD,
						  KruhobotDecisionMapping_map          &kruhobot_RDD_mapping) const;

	void interconnect_KruhobotRealDecisionDiagram(const sKruhobot                &kruhobot,
						      const s2DMap                   &map,
						      KruhobotDecisionDiagram_vector &kruhobot_RDD,
						      KruhobotDecisionMapping_map    &kruhobot_RDD_mapping) const;

	void trim_KruhobotRealDecisionDiagram(sDouble makespan_bound, KruhobotDecisionDiagram_vector &kruhobot_RDD) const;
	void trim_KruhobotRealDecisionDiagrams(sDouble makespan_bound, KruhobotDecisionDiagrams_vector &kruhobot_RDDs) const;

	void collect_CharacteristicMakespans(const sRealInstance &real_Instance, const KruhobotDecisionDiagrams_vector &kruhobot_RDDs, Makespans_vector &characteristic_Makespans) const;
	bool compare_CharacteristicMakespans(const sRealInstance &real_Instance, const Makespans_vector &characteristic_Makespans, const Makespans_vector &next_characteristic_Makespans) const;	
	
	bool compare_KruhobotRealDecisionDiagrams(const KruhobotDecisionDiagram_vector &kruhobot_RDD, const KruhobotDecisionDiagram_vector &next_kruhobot_RDD) const;

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
