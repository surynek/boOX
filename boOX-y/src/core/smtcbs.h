/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                              boOX 0_iskra-149                              */
/*                                                                            */
/*                      (C) Copyright 2018 Pavel Surynek                      */
/*                http://www.surynek.com | <pavel@surynek.com>                */
/*                                                                            */
/*                                                                            */
/*============================================================================*/
/* smtcbs.h / 0_iskra-149                                                     */
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

#include "types.h"
#include "result.h"

#include "core/graph.h"
#include "core/cbs.h"
#include "core/cnf.h"


using namespace std;


/*----------------------------------------------------------------------------*/

namespace boOX
{


/*----------------------------------------------------------------------------*/
// sSMTCBS
    
    class sSMTCBS
	: public sCBSBase
    {
    public:
	typedef std::vector<Collision> Collisions_vector;
	typedef std::vector<EdgeCollision> EdgeCollisions_vector;	
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
	    
	    VariableIDs_3vector m_vertex_occupancy;
	    VariableIDs_4vector m_edge_occupancy;
	    VariableIDs_2vector m_layer_cardinality;

	    Coordinates_vector m_variable_mapping;
	};

	struct Context
	{
	    Context()
	    {
		// nothing
	    }
	    
	    Collisions_vector m_trans_Collisions;
	    EdgeCollisions_vector m_trans_edge_Collisions;
	};
	
	typedef std::vector<sConfiguration> Configurations_vector;
	
    public:
	sSMTCBS(sBoolEncoder *solver_Encoder, sInstance *instance);
	sSMTCBS(sBoolEncoder *solver_Encoder, sInstance *instance, sDouble timeout);	
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

	sInt_32 check_NonconflictingPaths(const sInstance         &instance,
					  const AgentPaths_vector &agent_Paths,
					  Collision               &principal_collision) const;

	sInt_32 check_NonconflictingPaths(const sInstance         &instance,
					  const AgentPaths_vector &agent_Paths,
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

	sInt_32 check_NonconflictingRotation(const sInstance         &instance,
					     const AgentPaths_vector &agent_Paths,
					     Collision               &principal_collision) const;
	
	sInt_32 check_NonconflictingRotation(const sInstance         &instance,
					     const AgentPaths_vector &agent_Paths,
					     Collisions_vector       &Collisions,
					     EdgeCollisions_vector   &edge_Collisions) const;
	/*----------------------------------------------------------------------------*/

	sInt_32 build_SwappingModelVariables(Glucose::Solver             *solver,
					     Context                     &context,					 
					     const sInstance             &instance,
					     const sInstance::MDD_vector &MDD,
					     const sInstance::MDD_vector &extra_MDD,
					     sInt_32                      extra_cost,
					     Model                       &sat_Model) const;


	sInt_32 build_SwappingModelVariablesInverse(Glucose::Solver                    *solver,
						    Context                            &context,					 
						    const sInstance                    &instance,
						    const sInstance::MDD_vector        &MDD,
						    const sInstance::MDD_vector        &extra_MDD,
						    const sInstance::InverseMDD_vector &inverse_MDD,
						    sInt_32                             extra_cost,
						    Model                              &sat_Model) const;	
	
	void build_SwappingModelConstraints(Glucose::Solver             *solver,
					    Context                     &context,
					    const sInstance             &instance,
					    const sInstance::MDD_vector &MDD,
					    const sInstance::MDD_vector &extra_MDD,
					    sInt_32                      extra_cost,
					    Model                       &sat_Model) const;

	void build_SwappingModelConstraintsInverse(Glucose::Solver                    *solver,
						   Context                            &context,
						   const sInstance                    &instance,
						   const sInstance::MDD_vector        &MDD,
						   const sInstance::MDD_vector        &extra_MDD,
						   const sInstance::InverseMDD_vector &inverse_MDD,
						   sInt_32                             extra_cost,
						   Model                              &sat_Model) const;
	
	void refine_SwappingModelCollision(Glucose::Solver             *solver,
					   const Collision             &collision,
					   const sInstance             &instance,
					   const sInstance::MDD_vector &MDD,
					   const sInstance::MDD_vector &extra_MDD,
					   sInt_32                      extra_cost,
					   Model                       &sat_Model) const;
	
	void refine_SwappingModelCollisions(Glucose::Solver             *solver,
					    const Collisions_vector     &Collisions,
					    const EdgeCollisions_vector &edge_Collisions,					    
					    const sInstance             &instance,
					    const sInstance::MDD_vector &MDD,
					    const sInstance::MDD_vector &extra_MDD,
					    sInt_32                      extra_cost,
					    Model                       &sat_Model) const;

	void refine_SwappingModelCollisionsInverse(Glucose::Solver                   *solver,
						   const Collisions_vector            &Collisions,
						   const EdgeCollisions_vector        &edge_Collisions,					    
						   const sInstance                    &instance,
						   const sInstance::MDD_vector        &MDD,
						   const sInstance::MDD_vector        &extra_MDD,
						   const sInstance::InverseMDD_vector &inverse_MDD,
						   sInt_32                             extra_cost,
						   Model                              &sat_Model) const;			

	void decode_SwappingModel(Glucose::Solver             *solver,
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
					 sInt_32                      extra_cost,
					 Model                       &sat_Model) const;

	sInt_32 build_PathModelVariablesInverse(Glucose::Solver                   *solver,
						Context                            &context,
						const sInstance                    &instance,
						const sInstance::MDD_vector        &MDD,
						const sInstance::MDD_vector        &extra_MDD,
						const sInstance::InverseMDD_vector &inverse_MDD,
						sInt_32                             extra_cost,
						Model                              &sat_Model) const;		
	void build_PathModelConstraints(Glucose::Solver             *solver,
					Context                     &context,
					const sInstance             &instance,
					const sInstance::MDD_vector &MDD,
					const sInstance::MDD_vector &extra_MDD,
					sInt_32                      extra_cost,
					Model                       &sat_Model) const;

	void build_PathModelConstraintsInverse(Glucose::Solver                    *solver,
					       Context                            &context,
					       const sInstance                    &instance,
					       const sInstance::MDD_vector        &MDD,
					       const sInstance::MDD_vector        &extra_MDD,
					       const sInstance::InverseMDD_vector &inverse_MDD,
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

	sInt_32 build_PermutationModelVariables(Glucose::Solver             *solver,
						Context                     &context,					 
						const sInstance             &instance,
						const sInstance::MDD_vector &MDD,
						const sInstance::MDD_vector &extra_MDD,
						sInt_32                      extra_cost,
						Model                       &sat_Model) const;

	sInt_32 build_PermutationModelVariablesInverse(Glucose::Solver                   *solver,
						       Context                            &context,					 
						       const sInstance                    &instance,
						       const sInstance::MDD_vector        &MDD,
						       const sInstance::MDD_vector        &extra_MDD,
						       const sInstance::InverseMDD_vector &inverse_MDD,
						       sInt_32                             extra_cost,
						       Model                              &sat_Model) const;	
	
	void build_PermutationModelConstraints(Glucose::Solver             *solver,
					       Context                     &context,
					       const sInstance             &instance,
					       const sInstance::MDD_vector &MDD,
					       const sInstance::MDD_vector &extra_MDD,
					       sInt_32                      extra_cost,
					       Model                       &sat_Model) const;

	void build_PermutationModelConstraintsInverse(Glucose::Solver                    *solver,
						      Context                            &context,
						      const sInstance                    &instance,
						      const sInstance::MDD_vector        &MDD,
						      const sInstance::MDD_vector        &extra_MDD,
						      const sInstance::InverseMDD_vector &inverse_MDD,
						      sInt_32                             extra_cost,
						      Model                              &sat_Model) const;	
	
	void refine_PermutationModelCollision(Glucose::Solver             *solver,
					      const Collision             &collision,
					      const sInstance             &instance,
					      const sInstance::MDD_vector &MDD,
					      const sInstance::MDD_vector &extra_MDD,
					      sInt_32                      extra_cost,
					      Model                       &sat_Model) const;
	
	void refine_PermutationModelCollisions(Glucose::Solver             *solver,
					       const Collisions_vector     &Collisions,
					       const sInstance             &instance,
					       const sInstance::MDD_vector &MDD,
					       const sInstance::MDD_vector &extra_MDD,
					       sInt_32                      extra_cost,
					       Model                       &sat_Model) const;

	void refine_PermutationModelCollisionsInverse(Glucose::Solver                    *solver,
						      const Collisions_vector            &Collisions,
						      const sInstance                    &instance,
						      const sInstance::MDD_vector        &MDD,
						      const sInstance::MDD_vector        &extra_MDD,
						      const sInstance::InverseMDD_vector &inverse_MDD,
						      sInt_32                             extra_cost,
						      Model                              &sat_Model) const;	
	
	void decode_PermutationModel(Glucose::Solver             *solver,
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
					     sInt_32                             extra_cost,
					     Model                              &sat_Model) const;

	sInt_32 build_RotationModelVariablesInverse(Glucose::Solver             *solver,
						    Context                     &context,
						    const sInstance             &instance,
						    const sInstance::MDD_vector &MDD,
						    const sInstance::MDD_vector &extra_MDD,
						    const sInstance::InverseMDD_vector &inverse_MDD,
						    sInt_32                      extra_cost,
						    Model                       &sat_Model) const;
	
	void build_RotationModelConstraints(Glucose::Solver             *solver,
					    Context                     &context,
					    const sInstance             &instance,
					    const sInstance::MDD_vector &MDD,
					    const sInstance::MDD_vector &extra_MDD,
					    sInt_32                      extra_cost,
					    Model                       &sat_Model) const;

	void build_RotationModelConstraintsInverse(Glucose::Solver                   *solver,
						   Context                            &context,
						   const sInstance                    &instance,
						   const sInstance::MDD_vector        &MDD,
						   const sInstance::MDD_vector        &extra_MDD,
						   const sInstance::InverseMDD_vector &inverse_MDD,
						   sInt_32                             extra_cost,
						   Model                              &sat_Model) const;	
	
	void refine_RotationModelCollision(Glucose::Solver             *solver,
					   const Collision             &collision,
					   const sInstance             &instance,
					   const sInstance::MDD_vector &MDD,
					   const sInstance::MDD_vector &extra_MDD,
					   sInt_32                      extra_cost,
					   Model                       &sat_Model) const;
	
	void refine_RotationModelCollisions(Glucose::Solver             *solver,
					    const Collisions_vector     &Collisions,
					    const EdgeCollisions_vector &edge_Collisions,
					    const sInstance             &instance,
					    const sInstance::MDD_vector &MDD,
					    const sInstance::MDD_vector &extra_MDD,
					    sInt_32                      extra_cost,
					    Model                       &sat_Model) const;

	void refine_RotationModelCollisionsInverse(Glucose::Solver                   *solver,
						   const Collisions_vector            &Collisions,
						   const EdgeCollisions_vector        &edge_Collisions,
						   const sInstance                    &instance,
						   const sInstance::MDD_vector        &MDD,
						   const sInstance::MDD_vector        &extra_MDD,
						   const sInstance::InverseMDD_vector &inverse_MDD,
						   sInt_32                             extra_cost,
						   Model                              &sat_Model) const;			

	void decode_RotationModel(Glucose::Solver             *solver,
				  const sInstance             &instance,
				  const sInstance::MDD_vector &MDD,
				  const Model                 &sat_Model,
				  AgentPaths_vector           &agent_Paths) const;
	/*----------------------------------------------------------------------------*/		
	
	
	static sInt_32 alloc_VariableVector(sInt_32 first_variable_ID, VariableIDs_1vector &variable_vector, sInt_32 dim_1);
	static sInt_32 alloc_VariableVector(sInt_32 first_variable_ID, VariableIDs_2vector &variable_vector, sInt_32 dim_1, sInt_32 dim_2);
	static sInt_32 alloc_VariableVector(sInt_32 first_variable_ID, VariableIDs_3vector &variable_vector, sInt_32 dim_1, sInt_32 dim_2, sInt_32 dim_3);
	static sInt_32 alloc_VariableVector(sInt_32 first_variable_ID, VariableIDs_4vector &variable_vector, sInt_32 dim_1, sInt_32 dim_2, sInt_32 dim_3, sInt_32 dim_4);
	static sInt_32 alloc_VariableVector(sInt_32 first_variable_ID, VariableIDs_5vector &variable_vector, sInt_32 dim_1, sInt_32 dim_2, sInt_32 dim_3, sInt_32 dim_4, sInt_32 dim_5);
	/*----------------------------------------------------------------------------*/	
    
    private:
	sSMTCBS(const sSMTCBS &smt_cbs);
	const sSMTCBS& operator=(const sSMTCBS &smt_cbs);
	
    public:
	sBoolEncoder *m_solver_Encoder;
 };


/*----------------------------------------------------------------------------*/

} // namespace boOX

#endif /* __SMTCBS_H__ */
