/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             boOX 2-212_planck                              */
/*                                                                            */
/*                  (C) Copyright 2018 - 2022 Pavel Surynek                   */
/*                                                                            */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* smtcbs.h / 2-212_planck                                                    */
/*----------------------------------------------------------------------------*/
//
// Conflict based search implemented using SAT-modulo theories
//
/*----------------------------------------------------------------------------*/


#ifndef __SMTCBS_H__
#define __SMTCBS_H__

#include <vector>
#include <list>
#include <set>
#include <map>
#include <unordered_map>

#include "glucose/System.h"
#include "glucose/ParseUtils.h"
#include "glucose/Options.h"
#include "glucose/Dimacs.h"
#include "glucose/Solver.h"

#include "result.h"

#include "common/types.h"

#include "core/graph.h"
#include "core/cbs.h"
#include "core/cnf.h"


using namespace std;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/
// sSMTCBSBase

    class sSMTCBSBase
    {
    public:
    	typedef std::vector<sInt_32> VariableIDs_vector;
	typedef VariableIDs_vector VariableIDs_1vector;
	
	typedef std::vector<VariableIDs_1vector> VariableIDs_2vector;
	typedef std::vector<VariableIDs_2vector> VariableIDs_3vector;
	typedef std::vector<VariableIDs_3vector> VariableIDs_4vector;
	typedef std::vector<VariableIDs_4vector> VariableIDs_5vector;

	struct Coordinate
	{
	    Coordinate()
	    {
		// nothing
	    }
	    
	    Coordinate(sInt_32 agent_id, sInt_32 vertex_id, sInt_32 layer)
	    : m_agent_id(agent_id)
	    , m_vertex_id(vertex_id)
	    , m_layer(layer)
	    {
		// nothing
	    }
	    sInt_32 m_agent_id;
	    sInt_32 m_vertex_id;
	    sInt_32 m_layer;
	};

	typedef std::vector<Coordinate> Coordinates_vector;

	struct Clause
	{
	    Clause(sInt_32 variable_ID_1, sInt_32 variable_ID_2)
	    : m_variable_ID_1(variable_ID_1)
	    , m_variable_ID_2(variable_ID_2)
	    {
		// nothing
	    }

	    sInt_32 m_variable_ID_1;
	    sInt_32 m_variable_ID_2;
	};

	typedef std::vector<Clause> Clauses_vector;
	
	struct Model
	{
	    Model()
	    {
		// nothing
	    }
	    
	    void reset(void)
	    {
		m_vertex_occupancy.clear();
		m_edge_occupancy.clear();
		m_layer_cardinality.clear();
		m_layer_fulfillment.clear();
		m_variable_mapping.clear();
	    }
	    
	    VariableIDs_3vector m_vertex_occupancy;
	    VariableIDs_4vector m_edge_occupancy;
	    VariableIDs_2vector m_layer_cardinality;
	    VariableIDs_2vector m_layer_fulfillment;	    

	    Coordinates_vector m_variable_mapping;
	};

    public:
	sSMTCBSBase(sBoolEncoder *solver_Encoder);
	/*----------------------------------------------------------------------------*/

	static sInt_32 alloc_VariableVector(sInt_32 first_variable_ID, VariableIDs_1vector &variable_vector, sInt_32 dim_1);
	static sInt_32 alloc_VariableVector(sInt_32 first_variable_ID, VariableIDs_2vector &variable_vector, sInt_32 dim_1, sInt_32 dim_2);
	static sInt_32 alloc_VariableVector(sInt_32 first_variable_ID, VariableIDs_3vector &variable_vector, sInt_32 dim_1, sInt_32 dim_2, sInt_32 dim_3);
	static sInt_32 alloc_VariableVector(sInt_32 first_variable_ID, VariableIDs_4vector &variable_vector, sInt_32 dim_1, sInt_32 dim_2, sInt_32 dim_3, sInt_32 dim_4);
	static sInt_32 alloc_VariableVector(sInt_32 first_variable_ID, VariableIDs_5vector &variable_vector, sInt_32 dim_1, sInt_32 dim_2, sInt_32 dim_3, sInt_32 dim_4, sInt_32 dim_5);
	/*----------------------------------------------------------------------------*/	
	
    private:
	sSMTCBSBase(const sSMTCBSBase &smtcbs_base);
	const sSMTCBSBase& operator=(const sSMTCBSBase &smtcbs_base);

    public:
	sBoolEncoder *m_solver_Encoder;	
};




/*----------------------------------------------------------------------------*/
// sSMTCBS
    
    class sSMTCBS
	: public sCBSBase
	, public sSMTCBSBase
    {
    public:
	typedef std::vector<Collision> Collisions_vector;
	typedef std::vector<EdgeCollision> EdgeCollisions_vector;	

	struct Context
	{
	    Context()
	    {
		// nothing
	    }
	    
	    Collisions_vector m_trans_Collisions;
	    EdgeCollisions_vector m_trans_edge_Collisions;
	    CapacitatedCollisions_vector m_trans_capacitated_Collisions;	    
	};
	
	typedef std::vector<sConfiguration> Configurations_vector;
	
    public:
	sSMTCBS(sBoolEncoder *solver_Encoder, sInstance *instance);
	sSMTCBS(sBoolEncoder *solver_Encoder, sInstance *instance, sDouble timeout);

	sSMTCBS(sBoolEncoder *solver_Encoder, sDouble subopt_weight, sInstance *instance);
	sSMTCBS(sBoolEncoder *solver_Encoder, sDouble subopt_weight, sInstance *instance, sDouble timeout);
	/*----------------------------------------------------------------------------*/	

	sSMTCBS(sBoolEncoder *solver_Encoder, sMission *mission);
	sSMTCBS(sBoolEncoder *solver_Encoder, sMission *mission, sDouble timeout);

	sSMTCBS(sBoolEncoder *solver_Encoder, sDouble subopt_weight, sMission *mission);
	sSMTCBS(sBoolEncoder *solver_Encoder, sDouble subopt_weight, sMission *mission, sDouble timeout);			
	/*----------------------------------------------------------------------------*/

	sInt_32 find_ShortestNonconflictingSwapping(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingSwapping(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const;

	sInt_32 find_ShortestNonconflictingSwapping(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingSwapping(sInstance         &instance,
						    AgentPaths_vector &agent_Paths,
						    sInt_32            cost_limit) const;

	sInt_32 find_ShortestNonconflictingSwappingInverse(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingSwappingInverse(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const;

	sInt_32 find_ShortestNonconflictingSwappingInverse(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingSwappingInverse(sInstance         &instance,
							   AgentPaths_vector &agent_Paths,
							   sInt_32            cost_limit) const;

	sInt_32 find_ShortestNonconflictingSwappingInverseDepleted(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingSwappingInverseDepleted(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const;

	sInt_32 find_ShortestNonconflictingSwappingInverseDepleted(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingSwappingInverseDepleted(sInstance         &instance,
								   AgentPaths_vector &agent_Paths,
								   sInt_32            cost_limit) const;

	sInt_32 find_ShortestNonconflictingSwappingInverseOmitted(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingSwappingInverseOmitted(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const;

	sInt_32 find_ShortestNonconflictingSwappingInverseOmitted(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingSwappingInverseOmitted(sInstance         &instance,
								  AgentPaths_vector &agent_Paths,
								  sInt_32            cost_limit) const;		
	/*----------------------------------------------------------------------------*/	

	sInt_32 find_ShortestNonconflictingPaths(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingPaths(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const;	
	
	sInt_32 find_ShortestNonconflictingPaths(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingPaths(sInstance         &instance,
						 AgentPaths_vector &agent_Paths,
						 sInt_32            cost_limit) const;

	sInt_32 find_ShortestNonconflictingPathsInverse(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingPathsInverse(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const;	
	
	sInt_32 find_ShortestNonconflictingPathsInverse(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingPathsInverse(sInstance         &instance,
							AgentPaths_vector &agent_Paths,
							sInt_32            cost_limit) const;

	sInt_32 find_ShortestNonconflictingPathsInverseSeparated(sSolution &solution, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingPathsInverseSeparated(sInstance &instance, sSolution &solution, sInt_32 cost_limit);	
	
	sInt_32 find_ShortestNonconflictingPathsInverseSeparated(AgentPaths_vector &agent_Paths, sInt_32 cost_limit);	
	sInt_32 find_ShortestNonconflictingPathsInverseSeparated(sInstance         &instance,
								 AgentPaths_vector &agent_Paths,
								 sInt_32            cost_limit);

	sInt_32 find_ShortestNonconflictingPathsInverseDepleted(sSolution &solution, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingPathsInverseDepleted(sInstance &instance, sSolution &solution, sInt_32 cost_limit);	
	
	sInt_32 find_ShortestNonconflictingPathsInverseDepleted(AgentPaths_vector &agent_Paths, sInt_32 cost_limit);	
	sInt_32 find_ShortestNonconflictingPathsInverseDepleted(sInstance         &instance,
								AgentPaths_vector &agent_Paths,
								sInt_32            cost_limit);

	sInt_32 find_ShortestNonconflictingPathsInverseOmitted(sSolution &solution, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingPathsInverseOmitted(sInstance &instance, sSolution &solution, sInt_32 cost_limit);	
	
	sInt_32 find_ShortestNonconflictingPathsInverseOmitted(AgentPaths_vector &agent_Paths, sInt_32 cost_limit);	
	sInt_32 find_ShortestNonconflictingPathsInverseOmitted(sInstance         &instance,
							       AgentPaths_vector &agent_Paths,
							       sInt_32            cost_limit);

	sInt_32 find_ShortestNonconflictingPathsInversePremodelOmitted(sSolution &solution, sInt_32 cost_limit);
	sInt_32 find_ShortestNonconflictingPathsInversePremodelOmitted(sInstance &instance, sSolution &solution, sInt_32 cost_limit);

	sInt_32 find_ShortestNonconflictingPathsInversePremodelOmitted(AgentPaths_vector &agent_Paths, sInt_32 cost_limit);	
	sInt_32 find_ShortestNonconflictingPathsInversePremodelOmitted(sInstance         &instance,
								       AgentPaths_vector &agent_Paths,
								       sInt_32            cost_limit);
	/*----------------------------------------------------------------------------*/	

	sInt_32 find_ShortestNonconflictingPermutation(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingPermutation(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const;

	sInt_32 find_ShortestNonconflictingPermutation(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingPermutation(sInstance         &instance,
						       AgentPaths_vector &agent_Paths,
						       sInt_32            cost_limit) const;

	sInt_32 find_ShortestNonconflictingPermutationInverse(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingPermutationInverse(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const;

	sInt_32 find_ShortestNonconflictingPermutationInverse(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingPermutationInverse(sInstance         &instance,
							      AgentPaths_vector &agent_Paths,
							      sInt_32            cost_limit) const;

	sInt_32 find_ShortestNonconflictingPermutationInverseDepleted(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingPermutationInverseDepleted(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const;

	sInt_32 find_ShortestNonconflictingPermutationInverseDepleted(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingPermutationInverseDepleted(sInstance         &instance,
								      AgentPaths_vector &agent_Paths,
								      sInt_32            cost_limit) const;

	sInt_32 find_ShortestNonconflictingPermutationInverseOmitted(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingPermutationInverseOmitted(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const;

	sInt_32 find_ShortestNonconflictingPermutationInverseOmitted(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingPermutationInverseOmitted(sInstance         &instance,
								      AgentPaths_vector &agent_Paths,
								      sInt_32            cost_limit) const;			
	/*----------------------------------------------------------------------------*/	

	sInt_32 find_ShortestNonconflictingRotation(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingRotation(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const;

	sInt_32 find_ShortestNonconflictingRotation(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingRotation(sInstance         &instance,
						    AgentPaths_vector &agent_Paths,
						    sInt_32            cost_limit) const;

	sInt_32 find_ShortestNonconflictingRotationInverse(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingRotationInverse(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const;

	sInt_32 find_ShortestNonconflictingRotationInverse(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingRotationInverse(sInstance         &instance,
							   AgentPaths_vector &agent_Paths,
							   sInt_32            cost_limit) const;

	sInt_32 find_ShortestNonconflictingRotationInverseDepleted(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingRotationInverseDepleted(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const;

	sInt_32 find_ShortestNonconflictingRotationInverseDepleted(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingRotationInverseDepleted(sInstance         &instance,
								   AgentPaths_vector &agent_Paths,
								   sInt_32            cost_limit) const;

	sInt_32 find_ShortestNonconflictingRotationInverseOmitted(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingRotationInverseOmitted(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const;

	sInt_32 find_ShortestNonconflictingRotationInverseOmitted(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingRotationInverseOmitted(sInstance         &instance,
								  AgentPaths_vector &agent_Paths,
								  sInt_32            cost_limit) const;	

	sInt_32 find_ShortestNonconflictingCapacitatedRotationInverseDepleted(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingCapacitatedRotationInverseDepleted(sInstance &instance, sSolution &solution, sInt_32 cost_limit) const;

	sInt_32 find_ShortestNonconflictingCapacitatedRotationInverseDepleted(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingCapacitatedRotationInverseDepleted(sInstance         &instance,
									      AgentPaths_vector &agent_Paths,
									      sInt_32            cost_limit) const;
	/*----------------------------------------------------------------------------*/

	sInt_32 find_ShortestNonconflictingHamiltonianInverseDepleted(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingHamiltonianInverseDepleted(sMission &mission, sSolution &solution, sInt_32 cost_limit) const;

	sInt_32 find_ShortestNonconflictingHamiltonianInverseDepleted(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingHamiltonianInverseDepleted(sMission          &mission,
								      AgentPaths_vector &agent_Paths,
								      sInt_32            cost_limit) const;

	sInt_32 find_ShortestNonconflictingHamiltonianInverseDepletedSpanning(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingHamiltonianInverseDepletedSpanning(sMission &mission, sSolution &solution, sInt_32 cost_limit) const;

	sInt_32 find_ShortestNonconflictingHamiltonianInverseDepletedSpanning(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingHamiltonianInverseDepletedSpanning(sMission          &mission,
									      AgentPaths_vector &agent_Paths,
									      sInt_32            cost_limit) const;
	
	sInt_32 find_ShortestNonconflictingHamiltonianInverseDepletedHamilton(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingHamiltonianInverseDepletedHamilton(sMission &mission, sSolution &solution, sInt_32 cost_limit) const;

	sInt_32 find_ShortestNonconflictingHamiltonianInverseDepletedHamilton(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingHamiltonianInverseDepletedHamilton(sMission          &mission,
									      AgentPaths_vector &agent_Paths,
									      sInt_32            cost_limit) const;

	sInt_32 find_ShortestNonconflictingHamiltonianInverseDepletedHamiltonPlus(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingHamiltonianInverseDepletedHamiltonPlus(sMission &mission, sSolution &solution, sInt_32 cost_limit) const;

	sInt_32 find_ShortestNonconflictingHamiltonianInverseDepletedHamiltonPlus(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingHamiltonianInverseDepletedHamiltonPlus(sMission          &mission,
										  AgentPaths_vector &agent_Paths,
										  sInt_32            cost_limit) const;	
	/*----------------------------------------------------------------------------*/	

	sInt_32 find_ShortestNonconflictingKarpianInverseDepleted(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingKarpianInverseDepleted(sMission &mission, sSolution &solution, sInt_32 cost_limit) const;

	sInt_32 find_ShortestNonconflictingKarpianInverseDepleted(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingKarpianInverseDepleted(sMission          &mission,
								      AgentPaths_vector &agent_Paths,
								      sInt_32            cost_limit) const;

	sInt_32 find_ShortestNonconflictingKarpianInverseDepletedSpanning(sSolution &solution, sInt_32 cost_limit) const;
	sInt_32 find_ShortestNonconflictingKarpianInverseDepletedSpanning(sMission &mission, sSolution &solution, sInt_32 cost_limit) const;

	sInt_32 find_ShortestNonconflictingKarpianInverseDepletedSpanning(AgentPaths_vector &agent_Paths, sInt_32 cost_limit) const;	
	sInt_32 find_ShortestNonconflictingKarpianInverseDepletedSpanning(sMission          &mission,
									  AgentPaths_vector &agent_Paths,
									  sInt_32            cost_limit) const;			
	/*----------------------------------------------------------------------------*/	

	sInt_32 find_NonconflictingSwapping(Context           &context,
					    AgentPaths_vector &agent_Paths,
					    sInt_32            cost_limit) const;	
	sInt_32 find_NonconflictingSwapping(Context           &context,
					    sInstance         &instance,
					    AgentPaths_vector &agent_Paths,
					    sInt_32            cost_limit) const;

	sInt_32 find_NonconflictingSwappingInverse(Context           &context,
						   AgentPaths_vector &agent_Paths,
						   sInt_32            cost_limit) const;	
	sInt_32 find_NonconflictingSwappingInverse(Context           &context,
						   sInstance         &instance,
						   AgentPaths_vector &agent_Paths,
						   sInt_32            cost_limit) const;

	sInt_32 find_NonconflictingSwappingInverseDepleted(Context           &context,
							   AgentPaths_vector &agent_Paths,
							   sInt_32            cost_limit) const;	
	sInt_32 find_NonconflictingSwappingInverseDepleted(Context           &context,
							   sInstance         &instance,
							   AgentPaths_vector &agent_Paths,
							   sInt_32            cost_limit) const;

	sInt_32 find_NonconflictingSwappingInverseOmitted(Context           &context,
							  AgentPaths_vector &agent_Paths,
							  sInt_32            cost_limit) const;	
	sInt_32 find_NonconflictingSwappingInverseOmitted(Context           &context,
							  sInstance         &instance,
							  AgentPaths_vector &agent_Paths,
							  sInt_32            cost_limit) const;		
	/*----------------------------------------------------------------------------*/	

	sInt_32 find_NonconflictingPaths(Context           &context,
					 AgentPaths_vector &agent_Paths,
					 sInt_32            cost_limit) const;	
	sInt_32 find_NonconflictingPaths(Context           &context,
					 sInstance         &instance,
					 AgentPaths_vector &agent_Paths,
					 sInt_32            cost_limit) const;

	sInt_32 find_NonconflictingPathsInverse(Context           &context,
						AgentPaths_vector &agent_Paths,
						sInt_32            cost_limit) const;	
	sInt_32 find_NonconflictingPathsInverse(Context           &context,
						sInstance         &instance,
						AgentPaths_vector &agent_Paths,
						sInt_32            cost_limit) const;

	sInt_32 find_NonconflictingPathsInverseSeparated(Context           &context,
							 AgentPaths_vector &agent_Paths,
							 sInt_32            cost_limit);	
	sInt_32 find_NonconflictingPathsInverseSeparated(Context           &context,
							 sInstance         &instance,
							 AgentPaths_vector &agent_Paths,
							 sInt_32            cost_limit);

	sInt_32 find_NonconflictingPathsInverseDepleted(Context           &context,
							AgentPaths_vector &agent_Paths,
							sInt_32            cost_limit);	
	sInt_32 find_NonconflictingPathsInverseDepleted(Context           &context,
							sInstance         &instance,
							AgentPaths_vector &agent_Paths,
							sInt_32            cost_limit);

	sInt_32 find_NonconflictingPathsInverseOmitted(Context           &context,
						       AgentPaths_vector &agent_Paths,
						       sInt_32            cost_limit);	
	sInt_32 find_NonconflictingPathsInverseOmitted(Context           &context,
						       sInstance         &instance,
						       AgentPaths_vector &agent_Paths,
						       sInt_32            cost_limit);

	sInt_32 find_NonconflictingPathsInversePremodelOmitted(Context           &context,
							       AgentPaths_vector &agent_Paths,
							       sInt_32            cost_limit);	
	sInt_32 find_NonconflictingPathsInversePremodelOmitted(Context           &context,
							       sInstance         &instance,
							       AgentPaths_vector &agent_Paths,
							       sInt_32            cost_limit);					
	/*----------------------------------------------------------------------------*/	

	sInt_32 find_NonconflictingPermutation(Context           &context,
					       AgentPaths_vector &agent_Paths,
					       sInt_32            cost_limit) const;	
	sInt_32 find_NonconflictingPermutation(Context           &context,
					       sInstance         &instance,
					       AgentPaths_vector &agent_Paths,
					       sInt_32            cost_limit) const;

	sInt_32 find_NonconflictingPermutationInverse(Context           &context,
						      AgentPaths_vector &agent_Paths,
						      sInt_32            cost_limit) const;	
	sInt_32 find_NonconflictingPermutationInverse(Context           &context,
						      sInstance         &instance,
						      AgentPaths_vector &agent_Paths,
						      sInt_32            cost_limit) const;

	sInt_32 find_NonconflictingPermutationInverseDepleted(Context           &context,
							      AgentPaths_vector &agent_Paths,
							      sInt_32            cost_limit) const;	
	sInt_32 find_NonconflictingPermutationInverseDepleted(Context           &context,
							      sInstance         &instance,
							      AgentPaths_vector &agent_Paths,
							      sInt_32            cost_limit) const;

	sInt_32 find_NonconflictingPermutationInverseOmitted(Context           &context,
							     AgentPaths_vector &agent_Paths,
							     sInt_32            cost_limit) const;	
	sInt_32 find_NonconflictingPermutationInverseOmitted(Context           &context,
							     sInstance         &instance,
							     AgentPaths_vector &agent_Paths,
							     sInt_32            cost_limit) const;		
	/*----------------------------------------------------------------------------*/ 	

	sInt_32 find_NonconflictingRotation(Context           &context,
					    AgentPaths_vector &agent_Paths,
					    sInt_32            cost_limit) const;	
	sInt_32 find_NonconflictingRotation(Context           &context,
					    sInstance         &instance,
					    AgentPaths_vector &agent_Paths,
					    sInt_32            cost_limit) const;

	sInt_32 find_NonconflictingRotationInverse(Context           &context,
						   AgentPaths_vector &agent_Paths,
						   sInt_32            cost_limit) const;	
	sInt_32 find_NonconflictingRotationInverse(Context           &context,
						   sInstance         &instance,
						   AgentPaths_vector &agent_Paths,
						   sInt_32            cost_limit) const;

	sInt_32 find_NonconflictingRotationInverseDepleted(Context           &context,
							   AgentPaths_vector &agent_Paths,
							   sInt_32            cost_limit) const;	
	sInt_32 find_NonconflictingRotationInverseDepleted(Context           &context,
							   sInstance         &instance,
							   AgentPaths_vector &agent_Paths,
							   sInt_32            cost_limit) const;

	sInt_32 find_NonconflictingRotationInverseOmitted(Context           &context,
							  AgentPaths_vector &agent_Paths,
							  sInt_32            cost_limit) const;	
	sInt_32 find_NonconflictingRotationInverseOmitted(Context           &context,
							  sInstance         &instance,
							  AgentPaths_vector &agent_Paths,
							  sInt_32            cost_limit) const;	

	sInt_32 find_NonconflictingCapacitatedRotationInverseDepleted(Context           &context,
								      AgentPaths_vector &agent_Paths,
								      sInt_32            cost_limit) const;
	sInt_32 find_NonconflictingCapacitatedRotationInverseDepleted(Context           &context,
								      sInstance         &instance,
								      AgentPaths_vector &agent_Paths,
								      sInt_32            cost_limit) const;
	/*----------------------------------------------------------------------------*/
	
	sInt_32 find_NonconflictingHamiltonianInverseDepleted(Context           &context,
							      AgentPaths_vector &agent_Paths,
							      sInt_32            cost_limit) const;	
	sInt_32 find_NonconflictingHamiltonianInverseDepleted(Context           &context,
							      sMission          &mission,
							      AgentPaths_vector &agent_Paths,
							      sInt_32            cost_limit) const;

	sInt_32 find_NonconflictingHamiltonianInverseDepletedSpanning(Context           &context,
								      AgentPaths_vector &agent_Paths,
								      sInt_32            cost_limit) const;	
	sInt_32 find_NonconflictingHamiltonianInverseDepletedSpanning(Context           &context,
								      sMission          &mission,
								      AgentPaths_vector &agent_Paths,
								      sInt_32            cost_limit) const;

	sInt_32 find_NonconflictingHamiltonianInverseDepletedHamilton(Context           &context,
								      AgentPaths_vector &agent_Paths,
								      sInt_32            cost_limit) const;	
	sInt_32 find_NonconflictingHamiltonianInverseDepletedHamilton(Context           &context,
								      sMission          &mission,
								      AgentPaths_vector &agent_Paths,
								      sInt_32            cost_limit) const;

	sInt_32 find_NonconflictingHamiltonianInverseDepletedHamiltonPlus(Context           &context,
									  AgentPaths_vector &agent_Paths,
									  sInt_32            cost_limit) const;	
	sInt_32 find_NonconflictingHamiltonianInverseDepletedHamiltonPlus(Context           &context,
									  sMission          &mission,
									  AgentPaths_vector &agent_Paths,
									  sInt_32            cost_limit) const;		

	sInt_32 find_NonconflictingKarpianInverseDepleted(Context           &context,
							      AgentPaths_vector &agent_Paths,
							      sInt_32            cost_limit) const;	
	sInt_32 find_NonconflictingKarpianInverseDepleted(Context           &context,
							      sMission          &mission,
							      AgentPaths_vector &agent_Paths,
							      sInt_32            cost_limit) const;

	sInt_32 find_NonconflictingKarpianInverseDepletedSpanning(Context           &context,
								      AgentPaths_vector &agent_Paths,
								      sInt_32            cost_limit) const;	
	sInt_32 find_NonconflictingKarpianInverseDepletedSpanning(Context           &context,
								      sMission          &mission,
								      AgentPaths_vector &agent_Paths,
								      sInt_32            cost_limit) const;				
	/*----------------------------------------------------------------------------*/

	sInt_32 find_NonconflictingPaths_GlucosePrincipal(const sInstance       &instance,
							  Context               &context,
							  sInstance::MDD_vector &MDD,
							  sInstance::MDD_vector &extra_MDD,
							  sInt_32                extra_cost,
							  AgentPaths_vector     &agent_Paths,
							  sInt_32                cost_limit) const;

	sInt_32 find_NonconflictingPaths_GlucoseCollisions(const sInstance       &instance,
							   Context               &context,
							   sInstance::MDD_vector &MDD,
							   sInstance::MDD_vector &extra_MDD,
							   sInt_32                extra_cost,
							   AgentPaths_vector     &agent_Paths,
							   sInt_32                cost_limit) const;

	sInt_32 find_NonconflictingPaths_GlucoseCollisionsInverse(const sInstance              &instance,
								  Context                      &context,
								  sInstance::MDD_vector        &MDD,
								  sInstance::MDD_vector        &extra_MDD,
								  sInstance::InverseMDD_vector &inverse_MDD,
								  sInt_32                       extra_cost,
								  AgentPaths_vector            &agent_Paths,
								  sInt_32                       cost_limit) const;

	sInt_32 find_NonconflictingPaths_GlucoseCollisionsInverseSeparated(const sInstance              &instance,
									   Context                      &context,
									   sInstance::MDD_vector        &MDD,
									   sInstance::MDD_vector        &extra_MDD,
									   sInstance::InverseMDD_vector &inverse_MDD,
									   sInt_32                       extra_cost,
									   AgentPaths_vector            &agent_Paths,
									   sInt_32                       cost_limit) const;

	sInt_32 find_NonconflictingPaths_GlucoseCollisionsInverseSemiseparated(const sInstance              &instance,
									       Context                      &context,
									       sInstance::MDD_vector        &MDD,
									       sInstance::MDD_vector        &extra_MDD,
									       sInstance::InverseMDD_vector &inverse_MDD,
									       sInt_32                       extra_cost,
									       AgentPaths_vector            &agent_Paths,
									       sInt_32                       cost_limit) const;

	sInt_32 find_NonconflictingPaths_GlucoseCollisionsInverseDepleted(const sInstance              &instance,
									  Context                      &context,
									  sInstance::MDD_vector        &MDD,
									  sInstance::MDD_vector        &extra_MDD,
									  sInstance::InverseMDD_vector &inverse_MDD,
									  sInt_32                       extra_cost,
									  AgentPaths_vector            &agent_Paths,
									  sInt_32                       cost_limit) const;

	sInt_32 find_NonconflictingPaths_GlucoseCollisionsInverseOmitted(const sInstance              &instance,
									 Context                      &context,
									 sInstance::MDD_vector        &MDD,
									 sInstance::MDD_vector        &extra_MDD,
									 sInstance::InverseMDD_vector &inverse_MDD,
									 sInt_32                       extra_cost,
									 AgentPaths_vector            &agent_Paths,
									 sInt_32                       cost_limit) const;
	
	sInt_32 find_NonconflictingPaths_GlucoseCollisionsInversePremodelOmitted(const sInstance              &instance,
										 Context                      &context,
										 sInstance::MDD_vector        &MDD,
										 sInstance::MDD_vector        &extra_MDD,
										 sInstance::InverseMDD_vector &inverse_MDD,
										 sInt_32                       mdd_depth,
										 sInt_32                       extra_cost,
										 AgentPaths_vector            &agent_Paths,
										 sInt_32                       cost_limit);
	/*----------------------------------------------------------------------------*/	

	sInt_32 find_NonconflictingSwapping_GlucosePrincipal(const sInstance       &instance,
							     Context               &context,
							     sInstance::MDD_vector &MDD,
							     sInstance::MDD_vector &extra_MDD,
							     sInt_32                extra_cost,
							     AgentPaths_vector     &agent_Paths,
							     sInt_32                cost_limit) const;

	sInt_32 find_NonconflictingSwapping_GlucoseCollisions(const sInstance       &instance,
							      Context               &context,
							      sInstance::MDD_vector &MDD,
							      sInstance::MDD_vector &extra_MDD,
							      sInt_32                extra_cost,
							      AgentPaths_vector     &agent_Paths,
							      sInt_32                cost_limit) const;

	sInt_32 find_NonconflictingSwapping_GlucoseCollisionsInverse(const sInstance              &instance,
								     Context                      &context,
								     sInstance::MDD_vector        &MDD,
								     sInstance::MDD_vector        &extra_MDD,
								     sInstance::InverseMDD_vector &inverse_MDD,
								     sInt_32                       extra_cost,
								     AgentPaths_vector            &agent_Paths,
								     sInt_32                       cost_limit) const;


	sInt_32 find_NonconflictingSwapping_GlucoseCollisionsInverseDepleted(const sInstance              &instance,
									     Context                      &context,
									     sInstance::MDD_vector        &MDD,
									     sInstance::MDD_vector        &extra_MDD,
									     sInstance::InverseMDD_vector &inverse_MDD,
									     sInt_32                       extra_cost,
									     AgentPaths_vector            &agent_Paths,
									     sInt_32                       cost_limit) const;

	sInt_32 find_NonconflictingSwapping_GlucoseCollisionsInverseOmitted(const sInstance              &instance,
									    Context                      &context,
									    sInstance::MDD_vector        &MDD,
									    sInstance::MDD_vector        &extra_MDD,
									    sInstance::InverseMDD_vector &inverse_MDD,
									    sInt_32                       extra_cost,
									    AgentPaths_vector            &agent_Paths,
									    sInt_32                       cost_limit) const;		
	/*----------------------------------------------------------------------------*/	

	sInt_32 find_NonconflictingPermutation_GlucosePrincipal(const sInstance       &instance,
								Context               &context,
								sInstance::MDD_vector &MDD,
								sInstance::MDD_vector &extra_MDD,
								sInt_32                extra_cost,
								AgentPaths_vector     &agent_Paths,
								sInt_32                cost_limit) const;

	sInt_32 find_NonconflictingPermutation_GlucoseCollisions(const sInstance       &instance,
								 Context               &context,
								 sInstance::MDD_vector &MDD,
								 sInstance::MDD_vector &extra_MDD,
								 sInt_32                extra_cost,
								 AgentPaths_vector     &agent_Paths,
								 sInt_32                cost_limit) const;

	sInt_32 find_NonconflictingPermutation_GlucoseCollisionsInverse(const sInstance              &instance,
									Context                      &context,
									sInstance::MDD_vector        &MDD,
									sInstance::MDD_vector        &extra_MDD,
									sInstance::InverseMDD_vector &inverse_MDD,
									sInt_32                       extra_cost,
									AgentPaths_vector            &agent_Paths,
									sInt_32                       cost_limit) const;

	sInt_32 find_NonconflictingPermutation_GlucoseCollisionsInverseDepleted(const sInstance              &instance,
										Context                      &context,
										sInstance::MDD_vector        &MDD,
										sInstance::MDD_vector        &extra_MDD,
										sInstance::InverseMDD_vector &inverse_MDD,
										sInt_32                       extra_cost,
										AgentPaths_vector            &agent_Paths,
										sInt_32                       cost_limit) const;

	sInt_32 find_NonconflictingPermutation_GlucoseCollisionsInverseOmitted(const sInstance              &instance,
									       Context                      &context,
									       sInstance::MDD_vector        &MDD,
									       sInstance::MDD_vector        &extra_MDD,
									       sInstance::InverseMDD_vector &inverse_MDD,
									       sInt_32                       extra_cost,
									       AgentPaths_vector            &agent_Paths,
									       sInt_32                       cost_limit) const;			
	/*----------------------------------------------------------------------------*/		

	sInt_32 find_NonconflictingRotation_GlucosePrincipal(const sInstance       &instance,
							     Context               &context,
							     sInstance::MDD_vector &MDD,
							     sInstance::MDD_vector &extra_MDD,
							     sInt_32                extra_cost,
							     AgentPaths_vector     &agent_Paths,
							     sInt_32                cost_limit) const;

	sInt_32 find_NonconflictingRotation_GlucoseCollisions(const sInstance       &instance,
							      Context               &context,
							      sInstance::MDD_vector &MDD,
							      sInstance::MDD_vector &extra_MDD,
							      sInt_32                extra_cost,
							      AgentPaths_vector     &agent_Paths,
							      sInt_32                cost_limit) const;

	sInt_32 find_NonconflictingRotation_GlucoseCollisionsInverse(const sInstance              &instance,
								     Context                      &context,
								     sInstance::MDD_vector        &MDD,
								     sInstance::MDD_vector        &extra_MDD,
								     sInstance::InverseMDD_vector &inverse_MDD,
								     sInt_32                       extra_cost,
								     AgentPaths_vector            &agent_Paths,
								     sInt_32                       cost_limit) const;

	sInt_32 find_NonconflictingRotation_GlucoseCollisionsInverseDepleted(const sInstance              &instance,
									     Context                      &context,
									     sInstance::MDD_vector        &MDD,
									     sInstance::MDD_vector        &extra_MDD,
									     sInstance::InverseMDD_vector &inverse_MDD,
									     sInt_32                       extra_cost,
									     AgentPaths_vector            &agent_Paths,
									     sInt_32                       cost_limit) const;

	sInt_32 find_NonconflictingRotation_GlucoseCollisionsInverseOmitted(const sInstance              &instance,
									    Context                      &context,
									    sInstance::MDD_vector        &MDD,
									    sInstance::MDD_vector        &extra_MDD,
									    sInstance::InverseMDD_vector &inverse_MDD,
									    sInt_32                       extra_cost,
									    AgentPaths_vector            &agent_Paths,
									    sInt_32                       cost_limit) const;	

	sInt_32 find_NonconflictingCapacitatedRotation_GlucoseCollisionsInverseDepleted(const sInstance              &instance,
											Context                      &context,
											sInstance::MDD_vector        &MDD,
											sInstance::MDD_vector        &extra_MDD,
											sInstance::InverseMDD_vector &inverse_MDD,
											sInt_32                       extra_cost,
											AgentPaths_vector            &agent_Paths,
											sInt_32                       cost_limit) const;
	/*----------------------------------------------------------------------------*/	

	sInt_32 find_NonconflictingHamiltonian_GlucoseCollisionsInverseDepleted(const sMission              &mission,
										Context                      &context,
										sInstance::MDD_vector        &MDD,
										sInstance::MDD_vector        &extra_MDD,
										sInstance::InverseMDD_vector &inverse_MDD,
										sInt_32                       extra_cost,
										AgentPaths_vector            &agent_Paths,
										sInt_32                       cost_limit) const;

	sInt_32 find_NonconflictingHamiltonian_GlucoseCollisionsInverseDepletedSpanning(const sMission              &mission,
											Context                      &context,
											sInstance::MDD_vector        &MDD,
											sInstance::MDD_vector        &extra_MDD,
											sInstance::InverseMDD_vector &inverse_MDD,
											sInt_32                       extra_cost,
											AgentPaths_vector            &agent_Paths,
											sInt_32                       cost_limit) const;

	sInt_32 find_NonconflictingKarpian_GlucoseCollisionsInverseDepleted(const sMission              &mission,
										Context                      &context,
										sInstance::MDD_vector        &MDD,
										sInstance::MDD_vector        &extra_MDD,
										sInstance::InverseMDD_vector &inverse_MDD,
										sInt_32                       extra_cost,
										AgentPaths_vector            &agent_Paths,
										sInt_32                       cost_limit) const;

	sInt_32 find_NonconflictingKarpian_GlucoseCollisionsInverseDepletedSpanning(const sMission              &mission,
										    Context                      &context,
										    sInstance::MDD_vector        &MDD,
										    sInstance::MDD_vector        &extra_MDD,
										    sInstance::InverseMDD_vector &inverse_MDD,
										    sInt_32                       extra_cost,
										    AgentPaths_vector            &agent_Paths,
										    sInt_32                       cost_limit) const;			
	/*----------------------------------------------------------------------------*/

	bool find_InitialNonconflictingPaths(Glucose::Solver       *solver,
					     Context               &context,
					     Model                 &sat_Model,
					     const sInstance       &instance,
					     sInstance::MDD_vector &MDD,
					     sInstance::MDD_vector &extra_MDD,
					     sInt_32                extra_cost,
					     sInt_32                cost_limit,
					     AgentPaths_vector     &agent_Paths) const;

	bool find_InitialNonconflictingPathsInverse(Glucose::Solver              *solver,
						    Context                      &context,
						    Model                        &sat_Model,
						    const sInstance              &instance,
						    sInstance::MDD_vector        &MDD,
						    sInstance::MDD_vector        &extra_MDD,
						    sInstance::InverseMDD_vector &inverse_MDD,
						    sInt_32                       extra_cost,
						    sInt_32                       cost_limit,
						    AgentPaths_vector            &agent_Paths) const;

	bool find_InitialNonconflictingPathsInverse_validity(Glucose::Solver              *solver,
							     Context                      &context,
							     Model                        &sat_Model,
							     const sInstance              &instance,
							     sInstance::MDD_vector        &MDD,
							     sInstance::MDD_vector        &extra_MDD,
							     sInstance::InverseMDD_vector &inverse_MDD,
							     sInt_32                       extra_cost,
							     sInt_32                       cost_limit,
							     AgentPaths_vector            &agent_Paths) const;		

	bool find_NextNonconflictingPaths(Glucose::Solver       *solver,
					  Context               &context,
					  Model                 &sat_Model,
					  const Collision       &principal_collision,
					  const sInstance       &instance,
					  sInstance::MDD_vector &MDD,
					  sInstance::MDD_vector &extra_MDD,
					  sInt_32                extra_cost,
					  sInt_32                cost_limit,
					  AgentPaths_vector     &agent_Paths) const;

	bool find_NextNonconflictingPaths(Glucose::Solver         *solver,
					  Context                 &context,
					  Model                   &sat_Model,
					  const Collisions_vector &Collisions,
					  const sInstance         &instance,
					  sInstance::MDD_vector   &MDD,
					  sInstance::MDD_vector   &extra_MDD,
					  sInt_32                  extra_cost,
					  sInt_32                  cost_limit,
					  AgentPaths_vector       &agent_Paths) const;

	bool find_NextNonconflictingPathsInverse(Glucose::Solver             *solver,
						 Context                      &context,
						 Model                        &sat_Model,
						 const Collisions_vector      &Collisions,
						 const sInstance              &instance,
						 sInstance::MDD_vector        &MDD,
						 sInstance::MDD_vector        &extra_MDD,
						 sInstance::InverseMDD_vector &inverse_MDD,
						 sInt_32                       extra_cost,
						 sInt_32                       cost_limit,
						 AgentPaths_vector            &agent_Paths) const;

	bool find_NextNonconflictingPathsInverse_initial(Glucose::Solver             *solver,
							 Context                      &context,
							 Model                        &sat_Model,
							 const Collisions_vector      &Collisions,
							 const sInstance              &instance,
							 sInstance::MDD_vector        &MDD,
							 sInstance::MDD_vector        &extra_MDD,
							 sInstance::InverseMDD_vector &inverse_MDD,
							 sInt_32                       extra_cost,
							 sInt_32                       cost_limit,
							 AgentPaths_vector            &agent_Paths) const;	

	bool find_NextNonconflictingPathsInverse_validity(Glucose::Solver             *solver,
							  Context                      &context,
							  Model                        &sat_Model,
							  const Collisions_vector      &Collisions,
							  const sInstance              &instance,
							  sInstance::MDD_vector        &MDD,
							  sInstance::MDD_vector        &extra_MDD,
							  sInstance::InverseMDD_vector &inverse_MDD,
							  sInt_32                       extra_cost,
							  sInt_32                       cost_limit,
							  AgentPaths_vector            &agent_Paths) const;

	bool find_NextNonconflictingPathsInverse_cost(Glucose::Solver             *solver,
						      Context                      &context,
						      Model                        &sat_Model,
						      const Collisions_vector      &Collisions,
						      const sInstance              &instance,
						      sInstance::MDD_vector        &MDD,
						      sInstance::MDD_vector        &extra_MDD,
						      sInstance::InverseMDD_vector &inverse_MDD,
						      sInt_32                       extra_cost,
						      sInt_32                       cost_limit,
						      AgentPaths_vector            &agent_Paths) const;
	
	bool find_InitialNonconflictingPathsInverseDepleted(Glucose::Solver              *solver,
							    Context                      &context,
							    Model                        &sat_Model,
							    const sInstance              &instance,
							    sInstance::MDD_vector        &MDD,
							    sInstance::MDD_vector        &extra_MDD,
							    sInstance::InverseMDD_vector &inverse_MDD,
							    sInt_32                       extra_cost,
							    sInt_32                       cost_limit,
							    AgentPaths_vector            &agent_Paths) const;

	bool find_InitialNonconflictingPathsInverseDepleted_validity(Glucose::Solver              *solver,
								     Context                      &context,
								     Model                        &sat_Model,
								     const sInstance              &instance,
								     sInstance::MDD_vector        &MDD,
								     sInstance::MDD_vector        &extra_MDD,
								     sInstance::InverseMDD_vector &inverse_MDD,
								     sInt_32                       extra_cost,
								     sInt_32                       cost_limit,
								     AgentPaths_vector            &agent_Paths) const;		
	
	bool find_NextNonconflictingPathsInverseDepleted(Glucose::Solver             *solver,
							 Context                      &context,
							 Model                        &sat_Model,
							 const Collisions_vector      &Collisions,
							 const sInstance              &instance,
							 sInstance::MDD_vector        &MDD,
							 sInstance::MDD_vector        &extra_MDD,
							 sInstance::InverseMDD_vector &inverse_MDD,
							 sInt_32                       extra_cost,
							 sInt_32                       cost_limit,
							 AgentPaths_vector            &agent_Paths) const;

	bool find_NextNonconflictingPathsInverseDepleted_initial(Glucose::Solver             *solver,
								 Context                      &context,
								 Model                        &sat_Model,
								 const Collisions_vector      &Collisions,
								 const sInstance              &instance,
								 sInstance::MDD_vector        &MDD,
								 sInstance::MDD_vector        &extra_MDD,
								 sInstance::InverseMDD_vector &inverse_MDD,
								 sInt_32                       extra_cost,
								 sInt_32                       cost_limit,
								 AgentPaths_vector            &agent_Paths) const;	

	bool find_NextNonconflictingPathsInverseDepleted_validity(Glucose::Solver             *solver,
								  Context                      &context,
								  Model                        &sat_Model,
								  const Collisions_vector      &Collisions,
								  const sInstance              &instance,
								  sInstance::MDD_vector        &MDD,
								  sInstance::MDD_vector        &extra_MDD,
								  sInstance::InverseMDD_vector &inverse_MDD,
								  sInt_32                       extra_cost,
								  sInt_32                       cost_limit,
								  AgentPaths_vector            &agent_Paths) const;

	bool find_NextNonconflictingPathsInverseDepleted_cost(Glucose::Solver             *solver,
							      Context                      &context,
							      Model                        &sat_Model,
							      const Collisions_vector      &Collisions,
							      const sInstance              &instance,
							      sInstance::MDD_vector        &MDD,
							      sInstance::MDD_vector        &extra_MDD,
							      sInstance::InverseMDD_vector &inverse_MDD,
							      sInt_32                       extra_cost,
							      sInt_32                       cost_limit,
							      AgentPaths_vector            &agent_Paths) const;

	bool find_InitialNonconflictingPathsInverseOmitted(Glucose::Solver              *solver,
							   Context                      &context,
							   Model                        &sat_Model,
							   const sInstance              &instance,
							   sInstance::MDD_vector        &MDD,
							   sInstance::MDD_vector        &extra_MDD,
							   sInstance::InverseMDD_vector &inverse_MDD,
							   sInt_32                       extra_cost,
							   sInt_32                       cost_limit,
							   AgentPaths_vector            &agent_Paths,
							   AgentTrees_vector            &agent_Trees) const;

	bool find_InitialNonconflictingPathsInversePremodelOmitted(Glucose::Solver              *solver,
								   Context                      &context,
								   Model                        &sat_Model,
								   const sInstance              &instance,
								   sInstance::MDD_vector        &MDD,
								   sInstance::MDD_vector        &extra_MDD,
								   sInstance::InverseMDD_vector &inverse_MDD,
								   sInt_32                       mdd_depth,
								   sInt_32                       extra_cost,
								   sInt_32                       cost_limit,
								   AgentPaths_vector            &agent_Paths,
								   Glucose::vec<Glucose::Lit>   &goal_Assumptions);	

	bool find_InitialNonconflictingPathsInverseOmitted_validity(Glucose::Solver              *solver,
								    Context                      &context,
								    Model                        &sat_Model,
								    const sInstance              &instance,
								    sInstance::MDD_vector        &MDD,
								    sInstance::MDD_vector        &extra_MDD,
								    sInstance::InverseMDD_vector &inverse_MDD,
								    sInt_32                       extra_cost,
								    sInt_32                       cost_limit,
								    AgentPaths_vector            &agent_Paths) const;		
	
	bool find_NextNonconflictingPathsInverseOmitted(Glucose::Solver             *solver,
							Context                      &context,
							Model                        &sat_Model,
							const Collisions_vector      &Collisions,
							const sInstance              &instance,
							sInstance::MDD_vector        &MDD,
							sInstance::MDD_vector        &extra_MDD,
							sInstance::InverseMDD_vector &inverse_MDD,
							sInt_32                       extra_cost,
							sInt_32                       cost_limit,
							AgentPaths_vector            &agent_Paths,
							AgentTrees_vector            &agent_Trees) const;

	bool find_NextNonconflictingPathsInversePremodelOmitted(Glucose::Solver             *solver,
								Context                      &context,
								Model                        &sat_Model,
								const Collisions_vector      &Collisions,
								const sInstance              &instance,
								sInstance::MDD_vector        &MDD,
								sInstance::MDD_vector        &extra_MDD,
								sInstance::InverseMDD_vector &inverse_MDD,
								sInt_32                       mdd_depth,
								sInt_32                       extra_cost,
								sInt_32                       cost_limit,
								AgentPaths_vector            &agent_Paths,
								Glucose::vec<Glucose::Lit>   &goal_Assumptions) const;	

	bool find_NextNonconflictingPathsInverseOmitted_initial(Glucose::Solver             *solver,
								Context                      &context,
								Model                        &sat_Model,
								const Collisions_vector      &Collisions,
								const sInstance              &instance,
								sInstance::MDD_vector        &MDD,
								sInstance::MDD_vector        &extra_MDD,
								sInstance::InverseMDD_vector &inverse_MDD,
								sInt_32                       extra_cost,
								sInt_32                       cost_limit,
								AgentPaths_vector            &agent_Paths) const;	

	bool find_NextNonconflictingPathsInverseOmitted_validity(Glucose::Solver             *solver,
								 Context                      &context,
								 Model                        &sat_Model,
								 const Collisions_vector      &Collisions,
								 const sInstance              &instance,
								 sInstance::MDD_vector        &MDD,
								 sInstance::MDD_vector        &extra_MDD,
								 sInstance::InverseMDD_vector &inverse_MDD,
								 sInt_32                       extra_cost,
								 sInt_32                       cost_limit,
								 AgentPaths_vector            &agent_Paths) const;

	bool find_NextNonconflictingPathsInverseOmitted_cost(Glucose::Solver             *solver,
							     Context                      &context,
							     Model                        &sat_Model,
							     const Collisions_vector      &Collisions,
							     const sInstance              &instance,
							     sInstance::MDD_vector        &MDD,
							     sInstance::MDD_vector        &extra_MDD,
							     sInstance::InverseMDD_vector &inverse_MDD,
							     sInt_32                       extra_cost,
							     sInt_32                       cost_limit,
							     AgentPaths_vector            &agent_Paths) const;		

	sInt_32 check_NonconflictingPaths(const sInstance         &instance,
					  const AgentPaths_vector &agent_Paths,
					  Collision               &principal_collision) const;

	sInt_32 check_NonconflictingPaths(const sInstance         &instance,
					  const AgentPaths_vector &agent_Paths,
					  Collisions_vector       &Collisions) const;

	sInt_32 check_NonconflictingPaths(const sInstance         &instance,
					  const AgentPaths_vector &agent_Paths,
					  const AgentTrees_vector &agent_Trees,					  
					  Collisions_vector       &Collisions) const;	
	/*----------------------------------------------------------------------------*/

	bool find_InitialNonconflictingSwapping(Glucose::Solver       *solver,
						Context               &context,
						Model                 &sat_Model,
						const sInstance       &instance,
						sInstance::MDD_vector &MDD,
						sInstance::MDD_vector &extra_MDD,
						sInt_32                extra_cost,
						sInt_32                cost_limit,
						AgentPaths_vector     &agent_Paths) const;

	bool find_InitialNonconflictingSwappingInverse(Glucose::Solver             *solver,
						       Context                      &context,
						       Model                        &sat_Model,
						       const sInstance              &instance,
						       sInstance::MDD_vector        &MDD,
						       sInstance::MDD_vector        &extra_MDD,
						       sInstance::InverseMDD_vector &inverse_MDD,
						       sInt_32                       extra_cost,
						       sInt_32                       cost_limit,
						       AgentPaths_vector            &agent_Paths) const;	

	bool find_NextNonconflictingSwapping(Glucose::Solver       *solver,
					     Context               &context,
					     Model                 &sat_Model,
					     const Collision       &principal_collision,
					     const sInstance       &instance,
					     sInstance::MDD_vector &MDD,
					     sInstance::MDD_vector &extra_MDD,
					     sInt_32                extra_cost,
					     sInt_32                cost_limit,
					     AgentPaths_vector     &agent_Paths) const;

	bool find_NextNonconflictingSwapping(Glucose::Solver             *solver,
					     Context                     &context,
					     Model                       &sat_Model,
					     const Collisions_vector     &Collisions,
					     const EdgeCollisions_vector &edge_Collisions,
					     const sInstance             &instance,
					     sInstance::MDD_vector       &MDD,
					     sInstance::MDD_vector       &extra_MDD,
					     sInt_32                      extra_cost,
					     sInt_32                      cost_limit,
					     AgentPaths_vector           &agent_Paths) const;

	bool find_NextNonconflictingSwappingInverse(Glucose::Solver              *solver,
						    Context                      &context,
						    Model                        &sat_Model,
						    const Collisions_vector      &Collisions,
						    const EdgeCollisions_vector  &edge_Collisions,
						    const sInstance              &instance,
						    sInstance::MDD_vector        &MDD,
						    sInstance::MDD_vector        &extra_MDD,
						    sInstance::InverseMDD_vector &inverse_MDD,
						    sInt_32                      extra_cost,
						    sInt_32                      cost_limit,
						    AgentPaths_vector           &agent_Paths) const;

	bool find_InitialNonconflictingSwappingInverseDepleted(Glucose::Solver             *solver,
							       Context                      &context,
							       Model                        &sat_Model,
							       const sInstance              &instance,
							       sInstance::MDD_vector        &MDD,
							       sInstance::MDD_vector        &extra_MDD,
							       sInstance::InverseMDD_vector &inverse_MDD,
							       sInt_32                       extra_cost,
							       sInt_32                       cost_limit,
							       AgentPaths_vector            &agent_Paths) const;	

	bool find_NextNonconflictingSwappingInverseDepleted(Glucose::Solver              *solver,
							    Context                      &context,
							    Model                        &sat_Model,
							    const Collisions_vector      &Collisions,
							    const EdgeCollisions_vector  &edge_Collisions,
							    const sInstance              &instance,
							    sInstance::MDD_vector        &MDD,
							    sInstance::MDD_vector        &extra_MDD,
							    sInstance::InverseMDD_vector &inverse_MDD,
							    sInt_32                      extra_cost,
							    sInt_32                      cost_limit,
							    AgentPaths_vector           &agent_Paths) const;

	bool find_InitialNonconflictingSwappingInverseOmitted(Glucose::Solver             *solver,
							      Context                      &context,
							      Model                        &sat_Model,
							      const sInstance              &instance,
							      sInstance::MDD_vector        &MDD,
							      sInstance::MDD_vector        &extra_MDD,
							      sInstance::InverseMDD_vector &inverse_MDD,
							      sInt_32                       extra_cost,
							      sInt_32                       cost_limit,
							      AgentPaths_vector            &agent_Paths) const;	

	bool find_NextNonconflictingSwappingInverseOmitted(Glucose::Solver              *solver,
							   Context                      &context,
							   Model                        &sat_Model,
							   const Collisions_vector      &Collisions,
							   const EdgeCollisions_vector  &edge_Collisions,
							   const sInstance              &instance,
							   sInstance::MDD_vector        &MDD,
							   sInstance::MDD_vector        &extra_MDD,
							   sInstance::InverseMDD_vector &inverse_MDD,
							   sInt_32                      extra_cost,
							   sInt_32                      cost_limit,
							   AgentPaths_vector           &agent_Paths) const;		

	sInt_32 check_NonconflictingSwapping(const sInstance         &instance,
					     const AgentPaths_vector &agent_Paths,
					     Collision               &principal_collision) const;
	
	sInt_32 check_NonconflictingSwapping(const sInstance         &instance,
					     const AgentPaths_vector &agent_Paths,
					     Collisions_vector       &Collisions,
					     EdgeCollisions_vector   &edge_Collisions) const;
	/*----------------------------------------------------------------------------*/

	bool find_InitialNonconflictingPermutation(Glucose::Solver       *solver,
						   Context               &context,
						   Model                 &sat_Model,
						   const sInstance       &instance,
						   sInstance::MDD_vector &MDD,
						   sInstance::MDD_vector &extra_MDD,
						   sInt_32                extra_cost,
						   sInt_32                cost_limit,
						   AgentPaths_vector     &agent_Paths) const;

	bool find_InitialNonconflictingPermutationInverse(Glucose::Solver              *solver,
							  Context                      &context,
							  Model                        &sat_Model,
							  const sInstance              &instance,
							  sInstance::MDD_vector        &MDD,
							  sInstance::MDD_vector        &extra_MDD,
							  sInstance::InverseMDD_vector &inverse_MDD,
							  sInt_32                       extra_cost,
							  sInt_32                       cost_limit,
							  AgentPaths_vector            &agent_Paths) const;	

	bool find_NextNonconflictingPermutation(Glucose::Solver       *solver,
						Context               &context,
						Model                 &sat_Model,
						const Collision       &principal_collision,
						const sInstance       &instance,
						sInstance::MDD_vector &MDD,
						sInstance::MDD_vector &extra_MDD,
						sInt_32                extra_cost,
						sInt_32                cost_limit,
						AgentPaths_vector     &agent_Paths) const;
	
	bool find_NextNonconflictingPermutation(Glucose::Solver         *solver,
						Context                 &context,
						Model                   &sat_Model,
						const Collisions_vector &Collisions,
						const sInstance         &instance,
						sInstance::MDD_vector   &MDD,
						sInstance::MDD_vector   &extra_MDD,
						sInt_32                  extra_cost,
						sInt_32                  cost_limit,
						AgentPaths_vector       &agent_Paths) const;

	bool find_NextNonconflictingPermutationInverse(Glucose::Solver              *solver,
						       Context                      &context,
						       Model                        &sat_Model,
						       const Collisions_vector      &Collisions,
						       const sInstance              &instance,
						       sInstance::MDD_vector        &MDD,
						       sInstance::MDD_vector        &extra_MDD,
						       sInstance::InverseMDD_vector &inverse_MDD,
						       sInt_32                       extra_cost,
						       sInt_32                       cost_limit,
						       AgentPaths_vector            &agent_Paths) const;

	bool find_InitialNonconflictingPermutationInverseDepleted(Glucose::Solver              *solver,
								  Context                      &context,
								  Model                        &sat_Model,
								  const sInstance              &instance,
								  sInstance::MDD_vector        &MDD,
								  sInstance::MDD_vector        &extra_MDD,
								  sInstance::InverseMDD_vector &inverse_MDD,
								  sInt_32                       extra_cost,
								  sInt_32                       cost_limit,
								  AgentPaths_vector            &agent_Paths) const;	

	bool find_NextNonconflictingPermutationInverseDepleted(Glucose::Solver              *solver,
							       Context                      &context,
							       Model                        &sat_Model,
							       const Collisions_vector      &Collisions,
							       const sInstance              &instance,
							       sInstance::MDD_vector        &MDD,
							       sInstance::MDD_vector        &extra_MDD,
							       sInstance::InverseMDD_vector &inverse_MDD,
							       sInt_32                       extra_cost,
							       sInt_32                       cost_limit,
							       AgentPaths_vector            &agent_Paths) const;

	bool find_InitialNonconflictingPermutationInverseOmitted(Glucose::Solver              *solver,
								 Context                      &context,
								 Model                        &sat_Model,
								 const sInstance              &instance,
								 sInstance::MDD_vector        &MDD,
								 sInstance::MDD_vector        &extra_MDD,
								 sInstance::InverseMDD_vector &inverse_MDD,
								 sInt_32                       extra_cost,
								 sInt_32                       cost_limit,
								 AgentPaths_vector            &agent_Paths) const;	

	bool find_NextNonconflictingPermutationInverseOmitted(Glucose::Solver              *solver,
							      Context                      &context,
							      Model                        &sat_Model,
							      const Collisions_vector      &Collisions,
							      const sInstance              &instance,
							      sInstance::MDD_vector        &MDD,
							      sInstance::MDD_vector        &extra_MDD,
							      sInstance::InverseMDD_vector &inverse_MDD,
							      sInt_32                       extra_cost,
							      sInt_32                       cost_limit,
							      AgentPaths_vector            &agent_Paths) const;			
	
	sInt_32 check_NonconflictingPermutation(const sInstance         &instance,
						const AgentPaths_vector &agent_Paths,
						Collision               &principal_collision) const;
	
	sInt_32 check_NonconflictingPermutation(const sInstance         &instance,
						const AgentPaths_vector &agent_Paths,
						Collisions_vector       &Collisions) const;
	/*----------------------------------------------------------------------------*/

	bool find_InitialNonconflictingRotation(Glucose::Solver       *solver,
						Context               &context,
						Model                 &sat_Model,
						const sInstance       &instance,
						sInstance::MDD_vector &MDD,
						sInstance::MDD_vector &extra_MDD,
						sInt_32                extra_cost,
						sInt_32                cost_limit,
						AgentPaths_vector     &agent_Paths) const;

	bool find_InitialNonconflictingRotationInverse(Glucose::Solver       *solver,
						       Context               &context,
						       Model                 &sat_Model,
						       const sInstance       &instance,
						       sInstance::MDD_vector &MDD,
						       sInstance::MDD_vector &extra_MDD,
						       sInstance::InverseMDD_vector &inverse_MDD,
						       sInt_32                extra_cost,
						       sInt_32                cost_limit,
						       AgentPaths_vector     &agent_Paths) const;	

	bool find_NextNonconflictingRotation(Glucose::Solver       *solver,
					     Context               &context,
					     Model                 &sat_Model,
					     const Collision       &principal_collision,
					     const sInstance       &instance,
					     sInstance::MDD_vector &MDD,
					     sInstance::MDD_vector &extra_MDD,
					     sInt_32                extra_cost,
					     sInt_32                cost_limit,
					     AgentPaths_vector     &agent_Paths) const;

	bool find_NextNonconflictingRotation(Glucose::Solver             *solver,
					     Context                     &context,
					     Model                       &sat_Model,
					     const Collisions_vector     &Collisions,
					     const EdgeCollisions_vector &edge_Collisions,
					     const sInstance             &instance,
					     sInstance::MDD_vector       &MDD,
					     sInstance::MDD_vector       &extra_MDD,
					     sInt_32                      extra_cost,
					     sInt_32                      cost_limit,
					     AgentPaths_vector           &agent_Paths) const;

	bool find_NextNonconflictingRotationInverse(Glucose::Solver              *solver,
						    Context                      &context,
						    Model                        &sat_Model,
						    const Collisions_vector      &Collisions,
						    const EdgeCollisions_vector  &edge_Collisions,
						    const sInstance              &instance,
						    sInstance::MDD_vector        &MDD,
						    sInstance::MDD_vector        &extra_MDD,
						    sInstance::InverseMDD_vector &inverse_MDD,
						    sInt_32                       extra_cost,
						    sInt_32                       cost_limit,
						    AgentPaths_vector            &agent_Paths) const;

	bool find_InitialNonconflictingRotationInverseDepleted(Glucose::Solver       *solver,
							       Context               &context,
							       Model                 &sat_Model,
							       const sInstance       &instance,
							       sInstance::MDD_vector &MDD,
							       sInstance::MDD_vector &extra_MDD,
							       sInstance::InverseMDD_vector &inverse_MDD,
							       sInt_32                extra_cost,
							       sInt_32                cost_limit,
							       AgentPaths_vector     &agent_Paths) const;	
	
	bool find_NextNonconflictingRotationInverseDepleted(Glucose::Solver              *solver,
							    Context                      &context,
							    Model                        &sat_Model,
							    const Collisions_vector      &Collisions,
							    const EdgeCollisions_vector  &edge_Collisions,
							    const sInstance              &instance,
							    sInstance::MDD_vector        &MDD,
							    sInstance::MDD_vector        &extra_MDD,
							    sInstance::InverseMDD_vector &inverse_MDD,
							    sInt_32                       extra_cost,
							    sInt_32                       cost_limit,
							    AgentPaths_vector            &agent_Paths) const;

	bool find_InitialNonconflictingRotationInverseOmitted(Glucose::Solver       *solver,
							      Context               &context,
							      Model                 &sat_Model,
							      const sInstance       &instance,
							      sInstance::MDD_vector &MDD,
							      sInstance::MDD_vector &extra_MDD,
							      sInstance::InverseMDD_vector &inverse_MDD,
							      sInt_32                extra_cost,
							      sInt_32                cost_limit,
							      AgentPaths_vector     &agent_Paths) const;	
	
	bool find_NextNonconflictingRotationInverseOmitted(Glucose::Solver              *solver,
							   Context                      &context,
							   Model                        &sat_Model,
							   const Collisions_vector      &Collisions,
							   const EdgeCollisions_vector  &edge_Collisions,
							   const sInstance              &instance,
							   sInstance::MDD_vector        &MDD,
							   sInstance::MDD_vector        &extra_MDD,
							   sInstance::InverseMDD_vector &inverse_MDD,
							   sInt_32                       extra_cost,
							   sInt_32                       cost_limit,
							   AgentPaths_vector            &agent_Paths) const;		

	bool find_InitialNonconflictingCapacitatedRotationInverseDepleted(Glucose::Solver       *solver,
									  Context               &context,
									  Model                 &sat_Model,
									  const sInstance       &instance,
									  sInstance::MDD_vector &MDD,
									  sInstance::MDD_vector &extra_MDD,
									  sInstance::InverseMDD_vector &inverse_MDD,
									  sInt_32                extra_cost,
									  sInt_32                cost_limit,
									  AgentPaths_vector     &agent_Paths) const;	
	
	bool find_NextNonconflictingCapacitatedRotationInverseDepleted(Glucose::Solver                   *solver,
								       Context                            &context,
								       Model                              &sat_Model,
								       const Collisions_vector            &Collisions,
								       const EdgeCollisions_vector        &edge_Collisions,
								       const CapacitatedCollisions_vector &capacitated_Collisions,
								       const sInstance                    &instance,
								       sInstance::MDD_vector              &MDD,
								       sInstance::MDD_vector              &extra_MDD,
								       sInstance::InverseMDD_vector       &inverse_MDD,
								       sInt_32                             extra_cost,
								       sInt_32                             cost_limit,
								       AgentPaths_vector                  &agent_Paths) const;

	sInt_32 check_NonconflictingRotation(const sInstance         &instance,
					     const AgentPaths_vector &agent_Paths,
					     Collision               &principal_collision) const;
	
	sInt_32 check_NonconflictingRotation(const sInstance         &instance,
					     const AgentPaths_vector &agent_Paths,
					     Collisions_vector       &Collisions,
					     EdgeCollisions_vector   &edge_Collisions) const;

	sInt_32 check_NonconflictingCapacitatedRotation(const sInstance         &instance,
							const AgentPaths_vector &agent_Paths,
							Collision               &principal_collision,
							CapacitatedCollision    &principal_capacitated_collision) const;
	
	sInt_32 check_NonconflictingCapacitatedRotation(const sInstance              &instance,
							const AgentPaths_vector      &agent_Paths,
							Collisions_vector            &Collisions,
							EdgeCollisions_vector        &edge_Collisions,
							CapacitatedCollisions_vector &capacitated_Collisions) const;		
	/*----------------------------------------------------------------------------*/	

	bool find_InitialNonconflictingHamiltonianInverseDepleted(Glucose::Solver      *solver,
								  Context              &context,
								  Model                &sat_Model,
								  const sMission       &mission,
								  sMission::MDD_vector &MDD,
								  sMission::MDD_vector &extra_MDD,
								  sMission::InverseMDD_vector &inverse_MDD,
								  sInt_32               extra_cost,
								  sInt_32               cost_limit,
								  AgentPaths_vector    &agent_Paths) const;	
	
	bool find_NextNonconflictingHamiltonianInverseDepleted(Glucose::Solver             *solver,
							       Context                     &context,
							       Model                       &sat_Model,
							       const Collisions_vector     &Collisions,
							       const EdgeCollisions_vector &edge_Collisions,
							       const sMission              &mission,
							       sMission::MDD_vector        &MDD,
							       sMission::MDD_vector        &extra_MDD,
							       sMission::InverseMDD_vector &inverse_MDD,
							       sInt_32                      extra_cost,
							       sInt_32                      cost_limit,
							       AgentPaths_vector           &agent_Paths) const;	

	sInt_32 check_NonconflictingHamiltonian(const sMission          &mission,
						const AgentPaths_vector &agent_Paths,
						Collision               &principal_collision) const;
	
	sInt_32 check_NonconflictingHamiltonian(const sMission          &mission,
						const AgentPaths_vector &agent_Paths,
						Collisions_vector       &Collisions,
						EdgeCollisions_vector   &edge_Collisions) const;

	sInt_32 calc_HamiltonianCost(const sMission &mission, const AgentPaths_vector &agent_Paths) const;	
	/*----------------------------------------------------------------------------*/	

	bool find_InitialNonconflictingKarpianInverseDepleted(Glucose::Solver      *solver,
							      Context              &context,
							      Model                &sat_Model,
							      const sMission       &mission,
							      sMission::MDD_vector &MDD,
							      sMission::MDD_vector &extra_MDD,
							      sMission::InverseMDD_vector &inverse_MDD,
							      sInt_32               extra_cost,
							      sInt_32               cost_limit,
							      AgentPaths_vector    &agent_Paths) const;	
	
	bool find_NextNonconflictingKarpianInverseDepleted(Glucose::Solver             *solver,
							   Context                     &context,
							   Model                       &sat_Model,
							   const Collisions_vector     &Collisions,
							   const EdgeCollisions_vector &edge_Collisions,
							   const sMission              &mission,
							   sMission::MDD_vector        &MDD,
							   sMission::MDD_vector        &extra_MDD,
							   sMission::InverseMDD_vector &inverse_MDD,
							   sInt_32                      extra_cost,
							   sInt_32                      cost_limit,
							   AgentPaths_vector           &agent_Paths) const;	

	sInt_32 check_NonconflictingKarpian(const sMission          &mission,
					    const AgentPaths_vector &agent_Paths,
					    Collision               &principal_collision) const;
	
	sInt_32 check_NonconflictingKarpian(const sMission          &mission,
					    const AgentPaths_vector &agent_Paths,
					    Collisions_vector       &Collisions,
					    EdgeCollisions_vector   &edge_Collisions) const;
	
	sInt_32 calc_KarpianCost(const sMission &mission, const AgentPaths_vector &agent_Paths) const;	
	/*----------------------------------------------------------------------------*/
	

	sInt_32 build_SwappingModelVariables(Glucose::Solver             *solver,
					     Context                     &context,					 
					     const sInstance             &instance,
					     const sInstance::MDD_vector &MDD,
					     const sInstance::MDD_vector &extra_MDD,
					     sInt_32                      cost_limit,					     
					     sInt_32                      extra_cost,
					     Model                       &sat_Model) const;


	sInt_32 build_SwappingModelVariablesInverse(Glucose::Solver                    *solver,
						    Context                            &context,
						    const sInstance                    &instance,
						    const sInstance::MDD_vector        &MDD,
						    const sInstance::MDD_vector        &extra_MDD,
						    const sInstance::InverseMDD_vector &inverse_MDD,
						    sInt_32                             cost_limit,						    
						    sInt_32                             extra_cost,
						    Model                              &sat_Model) const;	
	
	void build_SwappingModelConstraints(Glucose::Solver             *solver,
					    Context                     &context,
					    const sInstance             &instance,
					    const sInstance::MDD_vector &MDD,
					    const sInstance::MDD_vector &extra_MDD,
					    sInt_32                      cost_limit,					    
					    sInt_32                      extra_cost,
					    Model                       &sat_Model) const;

	void build_SwappingModelConstraintsInverse(Glucose::Solver                    *solver,
						   Context                            &context,
						   const sInstance                    &instance,
						   const sInstance::MDD_vector        &MDD,
						   const sInstance::MDD_vector        &extra_MDD,
						   const sInstance::InverseMDD_vector &inverse_MDD,
						   sInt_32                             cost_limit,						   
						   sInt_32                             extra_cost,
						   Model                              &sat_Model) const;
	
	void refine_SwappingModelCollision(Glucose::Solver             *solver,
					   const Collision             &collision,
					   const sInstance             &instance,
					   const sInstance::MDD_vector &MDD,
					   const sInstance::MDD_vector &extra_MDD,
					   sInt_32                      cost_limit,
					   sInt_32                      extra_cost,
					   Model                       &sat_Model) const;
	
	void refine_SwappingModelCollisions(Glucose::Solver             *solver,
					    const Collisions_vector     &Collisions,
					    const EdgeCollisions_vector &edge_Collisions,					    
					    const sInstance             &instance,
					    const sInstance::MDD_vector &MDD,
					    const sInstance::MDD_vector &extra_MDD,
					    sInt_32                      cost_limit,					    
					    sInt_32                      extra_cost,
					    Model                       &sat_Model) const;

	void refine_SwappingModelCollisionsInverse(Glucose::Solver                   *solver,
						   const Collisions_vector            &Collisions,
						   const EdgeCollisions_vector        &edge_Collisions,					    
						   const sInstance                    &instance,
						   const sInstance::MDD_vector        &MDD,
						   const sInstance::MDD_vector        &extra_MDD,
						   const sInstance::InverseMDD_vector &inverse_MDD,
						   sInt_32                             cost_limit,
						   sInt_32                             extra_cost,
						   Model                              &sat_Model) const;			

	void decode_SwappingModel(Glucose::Solver             *solver,
				  const sInstance             &instance,
				  const sInstance::MDD_vector &MDD,
				  const Model                 &sat_Model,
				  AgentPaths_vector           &agent_Paths) const;
	/*----------------------------------------------------------------------------*/
	
	sInt_32 build_SwappingSmallModelVariablesInverse(Glucose::Solver                    *solver,
							 Context                            &context,					 
							 const sInstance                    &instance,
							 const sInstance::MDD_vector        &MDD,
							 const sInstance::MDD_vector        &extra_MDD,
							 const sInstance::InverseMDD_vector &inverse_MDD,
							 sInt_32                             cost_limit,
							 sInt_32                             extra_cost,
							 Model                              &sat_Model) const;
	
	void build_SwappingSmallModelConstraintsInverse(Glucose::Solver                    *solver,
							Context                            &context,
							const sInstance                    &instance,
							const sInstance::MDD_vector        &MDD,
							const sInstance::MDD_vector        &extra_MDD,
							const sInstance::InverseMDD_vector &inverse_MDD,
							sInt_32                             cost_limit,
							sInt_32                             extra_cost,
							Model                              &sat_Model) const;

	void build_SwappingTinyModelConstraintsInverse(Glucose::Solver                    *solver,
						       Context                            &context,
						       const sInstance                    &instance,
						       const sInstance::MDD_vector        &MDD,
						       const sInstance::MDD_vector        &extra_MDD,
						       const sInstance::InverseMDD_vector &inverse_MDD,
						       sInt_32                             cost_limit,
						       sInt_32                             extra_cost,
						       Model                              &sat_Model) const;	
	
	void refine_SwappingSmallModelCollisionsInverse(Glucose::Solver                   *solver,
							const Collisions_vector            &Collisions,
							const EdgeCollisions_vector        &edge_Collisions,					    
							const sInstance                    &instance,
							const sInstance::MDD_vector        &MDD,
							const sInstance::MDD_vector        &extra_MDD,
							const sInstance::InverseMDD_vector &inverse_MDD,
							sInt_32                             cost_limit,
							sInt_32                             extra_cost,
							Model                              &sat_Model) const;			

	void decode_SwappingSmallModel(Glucose::Solver             *solver,
				       const sInstance             &instance,
				       const sInstance::MDD_vector &MDD,
				       const Model                 &sat_Model,
				       AgentPaths_vector           &agent_Paths) const;

	void decode_SwappingTreeModel(Glucose::Solver             *solver,
				      const sInstance             &instance,
				      const sInstance::MDD_vector &MDD,
				      const Model                 &sat_Model,
				      AgentPaths_vector           &agent_Paths) const;	
	/*----------------------------------------------------------------------------*/

	sInt_32 build_PathModelVariables(Glucose::Solver             *solver,
					 Context                     &context,					 
					 const sInstance             &instance,
					 const sInstance::MDD_vector &MDD,
					 const sInstance::MDD_vector &extra_MDD,
					 sInt_32                      cost_limit,
					 sInt_32                      extra_cost,
					 Model                       &sat_Model) const;

	sInt_32 build_PathModelVariablesInverse(Glucose::Solver                   *solver,
						Context                            &context,
						const sInstance                    &instance,
						const sInstance::MDD_vector        &MDD,
						const sInstance::MDD_vector        &extra_MDD,
						const sInstance::InverseMDD_vector &inverse_MDD,
						sInt_32                             cost_limit,						
						sInt_32                             extra_cost,
						Model                              &sat_Model) const;
	
	void build_PathModelConstraints(Glucose::Solver             *solver,
					Context                     &context,
					const sInstance             &instance,
					const sInstance::MDD_vector &MDD,
					const sInstance::MDD_vector &extra_MDD,
					sInt_32                      cost_limit,					
					sInt_32                      extra_cost,
					Model                       &sat_Model) const;

	void build_PathModelConstraintsInverse(Glucose::Solver                    *solver,
					       Context                            &context,
					       const sInstance                    &instance,
					       const sInstance::MDD_vector        &MDD,
					       const sInstance::MDD_vector        &extra_MDD,
					       const sInstance::InverseMDD_vector &inverse_MDD,
					       sInt_32                             cost_limit,					       
					       sInt_32                             extra_cost,
					       Model                              &sat_Model) const;

	void build_PathModelConstraintsInverse_validity(Glucose::Solver                    *solver,
							Context                            &context,
							const sInstance                    &instance,
							const sInstance::MDD_vector        &MDD,
							const sInstance::MDD_vector        &extra_MDD,
							const sInstance::InverseMDD_vector &inverse_MDD,
							sInt_32                             cost_limit,
							sInt_32                             extra_cost,
							Model                              &sat_Model) const;

	void build_PathModelConstraintsInverse_cost(Glucose::Solver                    *solver,
						    Context                            &context,
						    const sInstance                    &instance,
						    const sInstance::MDD_vector        &MDD,
						    const sInstance::MDD_vector        &extra_MDD,
						    const sInstance::InverseMDD_vector &inverse_MDD,
						    sInt_32                             cost_limit,
						    sInt_32                             extra_cost,
						    Model                              &sat_Model) const;

	void refine_PathModelCollision(Glucose::Solver             *solver,
				       const Collision             &collision,
				       const sInstance             &instance,
				       const sInstance::MDD_vector &MDD,
				       const sInstance::MDD_vector &extra_MDD,
				       sInt_32                      extra_cost,
				       Model                       &sat_Model) const;

	void refine_PathModelCollisions(Glucose::Solver             *solver,
					const Collisions_vector     &Collisions,
					const sInstance             &instance,
					const sInstance::MDD_vector &MDD,
					const sInstance::MDD_vector &extra_MDD,
					sInt_32                      extra_cost,
					Model                       &sat_Model) const;

	void refine_PathModelCollisionsInverse(Glucose::Solver                    *solver,
					       const Collisions_vector            &Collisions,
					       const sInstance                    &instance,
					       const sInstance::MDD_vector        &MDD,
					       const sInstance::MDD_vector        &extra_MDD,
					       const sInstance::InverseMDD_vector &inverse_MDD,
					       sInt_32                             extra_cost,
					       Model                              &sat_Model) const;	

	void decode_PathModel(Glucose::Solver             *solver,
			      const sInstance             &instance,
			      const sInstance::MDD_vector &MDD,
			      const Model                 &sat_Model,
			      AgentPaths_vector           &agent_Paths) const;
	/*----------------------------------------------------------------------------*/	

	sInt_32 build_PathSmallModelVariablesInverse(Glucose::Solver                   *solver,
						     Context                            &context,
						     const sInstance                    &instance,
						     const sInstance::MDD_vector        &MDD,
						     const sInstance::MDD_vector        &extra_MDD,
						     const sInstance::InverseMDD_vector &inverse_MDD,
						     sInt_32                             cost_limit,
						     sInt_32                             extra_cost,
						     Model                              &sat_Model) const;

	void build_PathSmallModelConstraintsInverse(Glucose::Solver                    *solver,
						    Context                            &context,
						    const sInstance                    &instance,
						    const sInstance::MDD_vector        &MDD,
						    const sInstance::MDD_vector        &extra_MDD,
						    const sInstance::InverseMDD_vector &inverse_MDD,
						    sInt_32                             cost_limit,
						    sInt_32                             extra_cost,
						    Model                              &sat_Model) const;	

	void build_PathSmallModelConstraintsInverse_validity(Glucose::Solver                    *solver,
							     Context                            &context,
							     const sInstance                    &instance,
							     const sInstance::MDD_vector        &MDD,
							     const sInstance::MDD_vector        &extra_MDD,
							     const sInstance::InverseMDD_vector &inverse_MDD,
							     sInt_32                             cost_limit,
							     sInt_32                             extra_cost,
							     Model                              &sat_Model) const;

	void build_PathSmallModelConstraintsInverse_cost(Glucose::Solver                    *solver,
							 Context                            &context,
							 const sInstance                    &instance,
							 const sInstance::MDD_vector        &MDD,
							 const sInstance::MDD_vector        &extra_MDD,
							 const sInstance::InverseMDD_vector &inverse_MDD,
							 sInt_32                             cost_limit,
							 sInt_32                             extra_cost,
							 Model                              &sat_Model) const;

	void build_PathTinyModelConstraintsInverse(Glucose::Solver                    *solver,
						   Context                            &context,
						   const sInstance                    &instance,
						   const sInstance::MDD_vector        &MDD,
						   const sInstance::MDD_vector        &extra_MDD,
						   const sInstance::InverseMDD_vector &inverse_MDD,
						   sInt_32                             cost_limit,
						   sInt_32                             extra_cost,
						   Model                              &sat_Model) const;	
	
	void build_PathTinyModelConstraintsInverse_validity(Glucose::Solver                    *solver,
							    Context                            &context,
							    const sInstance                    &instance,
							    const sInstance::MDD_vector        &MDD,
							    const sInstance::MDD_vector        &extra_MDD,
							    const sInstance::InverseMDD_vector &inverse_MDD,
							    sInt_32                             cost_limit,
							    sInt_32                             extra_cost,
							    Model                              &sat_Model) const;

	void build_PathTinyModelConstraintsInverse_cost(Glucose::Solver                    *solver,
							Context                            &context,
							const sInstance                    &instance,
							const sInstance::MDD_vector        &MDD,
							const sInstance::MDD_vector        &extra_MDD,
							const sInstance::InverseMDD_vector &inverse_MDD,
							sInt_32                             cost_limit,
							sInt_32                             extra_cost,
							Model                              &sat_Model) const;	       

	void refine_PathSmallModelCollisionsInverse(Glucose::Solver                    *solver,
						    const Collisions_vector            &Collisions,
						    const sInstance                    &instance,
						    const sInstance::MDD_vector        &MDD,
						    const sInstance::MDD_vector        &extra_MDD,
						    const sInstance::InverseMDD_vector &inverse_MDD,
						    sInt_32                             cost_limit,
						    sInt_32                             extra_cost,
						    Model                              &sat_Model) const;
	
	void build_PathTinyModelConstraintsInverse_assumed(Glucose::Solver                    *solver,
							   Context                            &context,
							   const sInstance                    &instance,
							   const sInstance::MDD_vector        &MDD,
							   const sInstance::MDD_vector        &extra_MDD,
							   const sInstance::InverseMDD_vector &inverse_MDD,
							   sInt_32                             mdd_depth,
							   sInt_32                             cost_limit,
							   sInt_32                             extra_cost,
							   Model                              &sat_Model,
							   Glucose::vec<Glucose::Lit>         &goal_Assumptions);	
		
	void decode_PathSmallModel(Glucose::Solver             *solver,
				   const sInstance             &instance,
				   const sInstance::MDD_vector &MDD,
				   const Model                 &sat_Model,
				   AgentPaths_vector           &agent_Paths) const;
	
	void decode_PathTreeModel(Glucose::Solver             *solver,
				  const sInstance             &instance,
				  const sInstance::MDD_vector &MDD,
				  const Model                 &sat_Model,
				  AgentPaths_vector           &agent_Paths) const;

	void decode_PathTreeModel(Glucose::Solver             *solver,
				  const sInstance             &instance,
				  const sInstance::MDD_vector &MDD,
				  const Model                 &sat_Model,
				  AgentPaths_vector           &agent_Paths,				  
				  AgentTrees_vector           &agent_Trees) const;	

	void decode_PathTreeModel(Glucose::Solver             *solver,
				  const sInstance             &instance,
				  const sInstance::MDD_vector &MDD,
				  sInt_32                      mdd_depth,
				  const Model                 &sat_Model,
				  AgentPaths_vector           &agent_Paths) const;	

	bool decode_Tree2Path(const sInstance         &instance,
			      const AgentTrees_vector &agent_Trees,
			      AgentPaths_vector       &agent_Paths) const;

	bool convert_Tree2Path(const sInstance         &instance,
			       sInt_32                  curr_vertex_id,
			       sInt_32                  goal_vertex_id,
			       sInt_32                  path_index,
			       const VertexIDs_2vector &agent_Tree,
			       VertexIDs_vector        &agent_Path) const;	
	
	/*----------------------------------------------------------------------------*/
	
	sInt_32 build_PermutationModelVariables(Glucose::Solver             *solver,
						Context                     &context,					 
						const sInstance             &instance,
						const sInstance::MDD_vector &MDD,
						const sInstance::MDD_vector &extra_MDD,
						sInt_32                      cost_limit,
						sInt_32                      extra_cost,
						Model                       &sat_Model) const;

	sInt_32 build_PermutationModelVariablesInverse(Glucose::Solver                   *solver,
						       Context                            &context,					 
						       const sInstance                    &instance,
						       const sInstance::MDD_vector        &MDD,
						       const sInstance::MDD_vector        &extra_MDD,
						       const sInstance::InverseMDD_vector &inverse_MDD,
						       sInt_32                             cost_limit,
						       sInt_32                             extra_cost,
						       Model                              &sat_Model) const;	
	
	void build_PermutationModelConstraints(Glucose::Solver             *solver,
					       Context                     &context,
					       const sInstance             &instance,
					       const sInstance::MDD_vector &MDD,
					       const sInstance::MDD_vector &extra_MDD,
					       sInt_32                      cost_limit,
					       sInt_32                      extra_cost,
					       Model                       &sat_Model) const;

	void build_PermutationModelConstraintsInverse(Glucose::Solver                    *solver,
						      Context                            &context,
						      const sInstance                    &instance,
						      const sInstance::MDD_vector        &MDD,
						      const sInstance::MDD_vector        &extra_MDD,
						      const sInstance::InverseMDD_vector &inverse_MDD,
						      sInt_32                             cost_limit,
						      sInt_32                             extra_cost,
						      Model                              &sat_Model) const;	
	
	void refine_PermutationModelCollision(Glucose::Solver             *solver,
					      const Collision             &collision,
					      const sInstance             &instance,
					      const sInstance::MDD_vector &MDD,
					      const sInstance::MDD_vector &extra_MDD,
					      sInt_32                      cost_limit,
					      sInt_32                      extra_cost,
					      Model                       &sat_Model) const;
	
	void refine_PermutationModelCollisions(Glucose::Solver             *solver,
					       const Collisions_vector     &Collisions,
					       const sInstance             &instance,
					       const sInstance::MDD_vector &MDD,
					       const sInstance::MDD_vector &extra_MDD,
					       sInt_32                      cost_limit,
					       sInt_32                      extra_cost,
					       Model                       &sat_Model) const;

	void refine_PermutationModelCollisionsInverse(Glucose::Solver                    *solver,
						      const Collisions_vector            &Collisions,
						      const sInstance                    &instance,
						      const sInstance::MDD_vector        &MDD,
						      const sInstance::MDD_vector        &extra_MDD,
						      const sInstance::InverseMDD_vector &inverse_MDD,
						      sInt_32                             cost_limit,
						      sInt_32                             extra_cost,
						      Model                              &sat_Model) const;	
	
	void decode_PermutationModel(Glucose::Solver             *solver,
				     const sInstance             &instance,
				     const sInstance::MDD_vector &MDD,
				     const Model                 &sat_Model,
				     AgentPaths_vector           &agent_Paths) const;
	/*----------------------------------------------------------------------------*/

	sInt_32 build_PermutationSmallModelVariablesInverse(Glucose::Solver                   *solver,
							    Context                            &context,					 
							    const sInstance                    &instance,
							    const sInstance::MDD_vector        &MDD,
							    const sInstance::MDD_vector        &extra_MDD,
							    const sInstance::InverseMDD_vector &inverse_MDD,
							    sInt_32                             cost_limit,
							    sInt_32                             extra_cost,
							    Model                              &sat_Model) const;	
	
	void build_PermutationSmallModelConstraintsInverse(Glucose::Solver                    *solver,
							   Context                            &context,
							   const sInstance                    &instance,
							   const sInstance::MDD_vector        &MDD,
							   const sInstance::MDD_vector        &extra_MDD,
							   const sInstance::InverseMDD_vector &inverse_MDD,
							   sInt_32                             cost_limit,
							   sInt_32                             extra_cost,
							   Model                              &sat_Model) const;

	void build_PermutationTinyModelConstraintsInverse(Glucose::Solver                    *solver,
							  Context                            &context,
							  const sInstance                    &instance,
							  const sInstance::MDD_vector        &MDD,
							  const sInstance::MDD_vector        &extra_MDD,
							  const sInstance::InverseMDD_vector &inverse_MDD,
							  sInt_32                             cost_limit,
							  sInt_32                             extra_cost,
							  Model                              &sat_Model) const;		
	
	void refine_PermutationSmallModelCollisionsInverse(Glucose::Solver                    *solver,
							   const Collisions_vector            &Collisions,
							   const sInstance                    &instance,
							   const sInstance::MDD_vector        &MDD,
							   const sInstance::MDD_vector        &extra_MDD,
							   const sInstance::InverseMDD_vector &inverse_MDD,
							   sInt_32                             cost_limit,
							   sInt_32                             extra_cost,
							   Model                              &sat_Model) const;	
	
	void decode_PermutationSmallModel(Glucose::Solver             *solver,
					  const sInstance             &instance,
					  const sInstance::MDD_vector &MDD,
					  const Model                 &sat_Model,
					  AgentPaths_vector           &agent_Paths) const;
	
	void decode_PermutationTreeModel(Glucose::Solver             *solver,
					 const sInstance             &instance,
					 const sInstance::MDD_vector &MDD,
					 const Model                 &sat_Model,
					 AgentPaths_vector           &agent_Paths) const;			
	/*----------------------------------------------------------------------------*/

	sInt_32 build_RotationModelVariables(Glucose::Solver                    *solver,
					     Context                            &context,
					     const sInstance                    &instance,
					     const sInstance::MDD_vector        &MDD,
					     const sInstance::MDD_vector        &extra_MDD,
					     sInt_32                             cost_limit,
					     sInt_32                             extra_cost,
					     Model                              &sat_Model) const;

	sInt_32 build_RotationModelVariablesInverse(Glucose::Solver             *solver,
						    Context                     &context,
						    const sInstance             &instance,
						    const sInstance::MDD_vector &MDD,
						    const sInstance::MDD_vector &extra_MDD,
						    const sInstance::InverseMDD_vector &inverse_MDD,
						    sInt_32                      cost_limit,
						    sInt_32                      extra_cost,
						    Model                       &sat_Model) const;
	
	void build_RotationModelConstraints(Glucose::Solver             *solver,
					    Context                     &context,
					    const sInstance             &instance,
					    const sInstance::MDD_vector &MDD,
					    const sInstance::MDD_vector &extra_MDD,
					    sInt_32                      cost_limit,
					    sInt_32                      extra_cost,
					    Model                       &sat_Model) const;

	void build_RotationModelConstraintsInverse(Glucose::Solver                   *solver,
						   Context                            &context,
						   const sInstance                    &instance,
						   const sInstance::MDD_vector        &MDD,
						   const sInstance::MDD_vector        &extra_MDD,
						   const sInstance::InverseMDD_vector &inverse_MDD,
						   sInt_32                             cost_limit,
						   sInt_32                             extra_cost,
						   Model                              &sat_Model) const;	
	
	void refine_RotationModelCollision(Glucose::Solver             *solver,
					   const Collision             &collision,
					   const sInstance             &instance,
					   const sInstance::MDD_vector &MDD,
					   const sInstance::MDD_vector &extra_MDD,
					   sInt_32                      cost_limit,
					   sInt_32                      extra_cost,
					   Model                       &sat_Model) const;
	
	void refine_RotationModelCollisions(Glucose::Solver             *solver,
					    const Collisions_vector     &Collisions,
					    const EdgeCollisions_vector &edge_Collisions,
					    const sInstance             &instance,
					    const sInstance::MDD_vector &MDD,
					    const sInstance::MDD_vector &extra_MDD,
					    sInt_32                      cost_limit,
					    sInt_32                      extra_cost,
					    Model                       &sat_Model) const;

	void refine_RotationModelCollisionsInverse(Glucose::Solver                   *solver,
						   const Collisions_vector            &Collisions,
						   const EdgeCollisions_vector        &edge_Collisions,
						   const sInstance                    &instance,
						   const sInstance::MDD_vector        &MDD,
						   const sInstance::MDD_vector        &extra_MDD,
						   const sInstance::InverseMDD_vector &inverse_MDD,
						   sInt_32                             cost_limit,
						   sInt_32                             extra_cost,
						   Model                              &sat_Model) const;			

	void decode_RotationModel(Glucose::Solver             *solver,
				  const sInstance             &instance,
				  const sInstance::MDD_vector &MDD,
				  const Model                 &sat_Model,
				  AgentPaths_vector           &agent_Paths) const;
	/*----------------------------------------------------------------------------*/

	sInt_32 build_RotationSmallModelVariablesInverse(Glucose::Solver             *solver,
							 Context                     &context,
							 const sInstance             &instance,
							 const sInstance::MDD_vector &MDD,
							 const sInstance::MDD_vector &extra_MDD,
							 const sInstance::InverseMDD_vector &inverse_MDD,
							 sInt_32                      cost_limit,
							 sInt_32                      extra_cost,
							 Model                       &sat_Model) const;

	void build_RotationSmallModelConstraintsInverse(Glucose::Solver                   *solver,
							Context                            &context,
							const sInstance                    &instance,
							const sInstance::MDD_vector        &MDD,
							const sInstance::MDD_vector        &extra_MDD,
							const sInstance::InverseMDD_vector &inverse_MDD,
							sInt_32                             cost_limit,
							sInt_32                             extra_cost,
							Model                              &sat_Model) const;

	void build_RotationTinyModelConstraintsInverse(Glucose::Solver                   *solver,
						       Context                            &context,
						       const sInstance                    &instance,
						       const sInstance::MDD_vector        &MDD,
						       const sInstance::MDD_vector        &extra_MDD,
						       const sInstance::InverseMDD_vector &inverse_MDD,
						       sInt_32                             cost_limit,
						       sInt_32                             extra_cost,
						       Model                              &sat_Model) const;		
	
	void refine_RotationSmallModelCollisionsInverse(Glucose::Solver                   *solver,
							const Collisions_vector            &Collisions,
							const EdgeCollisions_vector        &edge_Collisions,
							const sInstance                    &instance,
							const sInstance::MDD_vector        &MDD,
							const sInstance::MDD_vector        &extra_MDD,
							const sInstance::InverseMDD_vector &inverse_MDD,
							sInt_32                             cost_limit,
							sInt_32                             extra_cost,
							Model                              &sat_Model) const;			

	void decode_RotationSmallModel(Glucose::Solver             *solver,
				       const sInstance             &instance,
				       const sInstance::MDD_vector &MDD,
				       const Model                 &sat_Model,
				       AgentPaths_vector           &agent_Paths) const;

	void decode_RotationTreeModel(Glucose::Solver             *solver,
				      const sInstance             &instance,
				      const sInstance::MDD_vector &MDD,
				      const Model                 &sat_Model,
				      AgentPaths_vector           &agent_Paths) const;
	
	/*----------------------------------------------------------------------------*/

	sInt_32 build_HamiltonianSmallModelVariablesInverse(Glucose::Solver            *solver,
							    Context                    &context,
							    const sMission             &mission,
							    const sMission::MDD_vector &MDD,
							    const sMission::MDD_vector &extra_MDD,
							    const sMission::InverseMDD_vector &inverse_MDD,
							    sInt_32                     cost_limit,
							    sInt_32                     extra_cost,
							    Model                      &sat_Model) const;

	void build_HamiltonianSmallModelConstraintsInverse(Glucose::Solver                  *solver,
							   Context                           &context,
							   const sMission                    &mission,
							   const sMission::MDD_vector        &MDD,
							   const sMission::MDD_vector        &extra_MDD,
							   const sMission::InverseMDD_vector &inverse_MDD,
							   sInt_32                            cost_limit,
							   sInt_32                            extra_cost,
							   Model                             &sat_Model) const;	
	
	void refine_HamiltonianSmallModelCollisionsInverse(Glucose::Solver                  *solver,
							   const Collisions_vector           &Collisions,
							   const EdgeCollisions_vector       &edge_Collisions,
							   const sMission                    &mission,
							   const sMission::MDD_vector        &MDD,
							   const sMission::MDD_vector        &extra_MDD,
							   const sMission::InverseMDD_vector &inverse_MDD,
							   sInt_32                            cost_limit,
							   sInt_32                            extra_cost,
							   Model                             &sat_Model) const;			

	void decode_HamiltonianSmallModel(Glucose::Solver            *solver,
					  const sMission             &mission,
					  const sMission::MDD_vector &MDD,
					  const Model                &sat_Model,
					  AgentPaths_vector          &agent_Paths) const;
	/*----------------------------------------------------------------------------*/

	sInt_32 build_KarpianSmallModelVariablesInverse(Glucose::Solver            *solver,
							Context                    &context,
							const sMission             &mission,
							const sMission::MDD_vector &MDD,
							const sMission::MDD_vector &extra_MDD,
							const sMission::InverseMDD_vector &inverse_MDD,
							sInt_32                     cost_limit,
							sInt_32                     extra_cost,
							Model                      &sat_Model) const;

	void build_KarpianSmallModelConstraintsInverse(Glucose::Solver                  *solver,
						       Context                           &context,
						       const sMission                    &mission,
						       const sMission::MDD_vector        &MDD,
						       const sMission::MDD_vector        &extra_MDD,
						       const sMission::InverseMDD_vector &inverse_MDD,
						       sInt_32                            cost_limit,
						       sInt_32                            extra_cost,
						       Model                             &sat_Model) const;	
	
	void refine_KarpianSmallModelCollisionsInverse(Glucose::Solver                  *solver,
						       const Collisions_vector           &Collisions,
						       const EdgeCollisions_vector       &edge_Collisions,
						       const sMission                    &mission,
						       const sMission::MDD_vector        &MDD,
						       const sMission::MDD_vector        &extra_MDD,
						       const sMission::InverseMDD_vector &inverse_MDD,
						       sInt_32                            cost_limit,
						       sInt_32                            extra_cost,
						       Model                             &sat_Model) const;			

	void decode_KarpianSmallModel(Glucose::Solver            *solver,
				      const sMission             &mission,
				      const sMission::MDD_vector &MDD,
				      const Model                &sat_Model,
				      AgentPaths_vector          &agent_Paths) const;
	/*----------------------------------------------------------------------------*/		
	

	sInt_32 build_RotationSmallCapacitatedModelVariablesInverse(Glucose::Solver             *solver,
								    Context                     &context,
								    const sInstance             &instance,
								    const sInstance::MDD_vector &MDD,
								    const sInstance::MDD_vector &extra_MDD,
								    const sInstance::InverseMDD_vector &inverse_MDD,
								    sInt_32                      cost_limit,
								    sInt_32                      extra_cost,
								    Model                       &sat_Model) const;

	void build_RotationSmallCapacitatedModelConstraintsInverse(Glucose::Solver                   *solver,
								   Context                            &context,
								   const sInstance                    &instance,
								   const sInstance::MDD_vector        &MDD,
								   const sInstance::MDD_vector        &extra_MDD,
								   const sInstance::InverseMDD_vector &inverse_MDD,
								   sInt_32                             cost_limit,
								   sInt_32                             extra_cost,
								   Model                              &sat_Model) const;
	
	void refine_RotationSmallCapacitatedModelCollisionsInverse(Glucose::Solver                   *solver,
								   const Collisions_vector            &Collisions,
								   const EdgeCollisions_vector        &edge_Collisions,
								   const CapacitatedCollisions_vector &capacitated_Collisions,
								   const sInstance                    &instance,
								   const sInstance::MDD_vector        &MDD,
								   const sInstance::MDD_vector        &extra_MDD,
								   const sInstance::InverseMDD_vector &inverse_MDD,
								   sInt_32                             cost_limit,
								   sInt_32                             extra_cost,
								   Model                              &sat_Model) const;

	void decode_RotationSmallCapacitatedModel(Glucose::Solver             *solver,
						  const sInstance             &instance,
						  const sInstance::MDD_vector &MDD,
						  const Model                 &sat_Model,
						  AgentPaths_vector           &agent_Paths) const;
	/*----------------------------------------------------------------------------*/	
	
    private:
	sSMTCBS(const sSMTCBS &smt_cbs);
	const sSMTCBS& operator=(const sSMTCBS &smt_cbs);

    public:
	sDouble m_subopt_weight;
	
	sCBS m_cbs;
	sInstance m_cbs_instance;

	sInt_32 m_premodel_extra;
	sInt_32 m_premodel_cost;

	Model m_sat_Model;
	Glucose::Solver *m_solver;	
	sInstance::MDD_vector m_premodel_MDD, m_premodel_extra_MDD;
	sInstance::InverseMDD_vector m_premodel_inverse_MDD;
	sInt_32 m_cardinality_variable_ID;
 };


/*----------------------------------------------------------------------------*/

} // namespace boOX

#endif /* __SMTCBS_H__ */
